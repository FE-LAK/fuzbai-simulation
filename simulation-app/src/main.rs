use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
use fuzbai_simulator::mujoco_rs::util::LockUnpoison;

use std::{collections::VecDeque, sync::{Arc, LazyLock, Mutex}, time::{Instant}};
use std::panic::{AssertUnwindSafe, catch_unwind};

use tokio::runtime::Builder;
use tokio::sync::Notify;

mod http;

const COMPETITION_DURATION_SECS: u64 = 120;
const EXPIRED_BALL_POSITION: [f64; 3] = [605.0, -100.0, 100.0];

static COMPETITION_STATE: LazyLock<Mutex<CompetitionState>> = LazyLock::new(|| Mutex::new(CompetitionState::default()));

// Define global linkage for a MuJoCo error handler. This is redefined (from MuJoCo-rs's implementation)
// to allow C-unwind, so panics can be triggered inside the handler.

type ErrorCallback = ::std::option::Option<unsafe extern "C-unwind" fn(arg1: *const ::std::os::raw::c_char)>;
unsafe extern "C" {
    #[cfg(target_family = "unix")]
    static mut mju_user_error: ErrorCallback;

    #[cfg(target_os = "windows")]
    #[link_name = "__imp_mju_user_error"]
    static mut mju_user_error: *mut ErrorCallback;
}

/// Turns any MuJoCo errors into Rust panics, which can be caught and handled.
unsafe extern "C-unwind" fn handle_mujoco_error(c_error_message: *const std::os::raw::c_char) {
    if let Ok(e_msg) = unsafe { std::ffi::CStr::from_ptr(c_error_message).to_str() } {
        panic!("ERROR! MuJoCo's C library issued an error: {e_msg}");
    }
    else {
        panic!("ERROR! MuJoCo's C library issued an error (error parsing failed).");
    };
}

enum CompetitionPending {
    ResetScore,
    ResetSimulation,
}

#[derive(PartialEq, Clone)]
enum CompetitionStatus {
    Running(Instant),
    Expired,
    Free
}

struct CompetitionState {
    status: CompetitionStatus,
    pending: VecDeque<CompetitionPending>,
}

impl CompetitionState {
    /// Returns whether the competition time has ended.
    fn expired(&self) -> bool {
        self.status == CompetitionStatus::Expired
    }
}

impl Default for CompetitionState {
    fn default() -> Self {
        Self { status: CompetitionStatus::Expired, pending: VecDeque::new() }
    }
}

fn main() {
    let mut args = std::env::args();
    let _ = args.next().unwrap();  // program path;

    // Set the MuJoCo error handler (from C language) to catch internal MuJoCo crashes
    // and convert them to panics
    #[cfg(target_family = "unix")]
    unsafe { mju_user_error = Some(handle_mujoco_error) };

    // Dumb Microsoft logic: the variable is a pointer to the variable
    #[cfg(target_os = "windows")]
    unsafe {
        *mju_user_error = Some(handle_mujoco_error)
    };

    // Initialize the rest of Rust code
    let port_0 = args.next()
        .unwrap_or_else(|| "8080".into())
        .parse::<u16>().expect("passed team 1 port was invalid");
    let port_1 = args.next()
        .unwrap_or_else(|| "8081".into())
        .parse::<u16>().expect("passed team 2 port was invalid");

    /* Initialize states for each team */
    let states = [
        Arc::new(Mutex::new(http::ServerState {
            camera_state: http::CameraState::new(), port: port_0, team: PlayerTeam::Red, team_name: "Team 1".to_string(),
            score: 0, builtin: false, pending_commands: Vec::new()
        })),
        Arc::new(Mutex::new(http::ServerState {
            camera_state: http::CameraState::new(), port: port_1, team: PlayerTeam::Blue, team_name: "Team 2".to_string(),
            score: 0, builtin: false, pending_commands: Vec::new()
        }))
    ];

    /* Initialize tokio runtime and with it, the HTTP server */
    let states_clone = states.clone();

    // Notification to wake the HTTP task, so that it can trigger a shutdown.
    let shutdown_notify = Arc::new(Notify::new());
    let shutdown_notify_clone = shutdown_notify.clone();

    let tokio_handle = std::thread::Builder::new()
        .name("tokio thread".into())
        .spawn(move || {
            let runtime = Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap();

            runtime.block_on(http::http_task(states_clone, shutdown_notify_clone));
    }).unwrap();

    /* Initialize simulation */
    let sim_factory = |init_viewer| {
        FuzbAISimulator::new(
            1, // internal_step_factor: .step_simulation() = N * (2 ms)
            5, // sample_steps: save state to delay buffer every N * (2 ms). .delayed_observation() returns discrete samples every N * 2ms.
            true,
            0.055,
            None,
            VisualConfig::new(
                0, false,
                0, init_viewer
            ),
        )
    };

    /* Initialize simulation */
    // Create the simulation with viewer enabled on the main thread (due to OS limitations).
    // The viewer itself could be created completely independently from simulation (after creating the model),
    // however it is more convenient if this is created by the simulation on construction, rather than
    // separately.
    let mut sim = sim_factory(true);
    // Start physics in another thread
    let states_clone = states.clone();
    let sim_thread = std::thread::Builder::new()
        .name("simulation".into())
        .spawn(move || {
            // Loop forever unless viewer is closed.
            // This ensures the simulation is automatically restarted in case of a panic.
            loop { 
                // Catch potential panics. These should not occur if everything is done correctly,
                // but bugs happen and we are not Cloudflare :).
                let panic_result = catch_unwind(AssertUnwindSafe(||
                    simulation_thread(&mut sim, states_clone.clone())
                ));
                match panic_result {
                    Ok(()) => break,  // exited without a panic -> normal close
                    Err(_) => {
                        println!("ERROR! Physics thread panicked! Restarting");
                    }
                }

                // Save the simulation score and recreate, without reinitializing the viewer.
                let score = *sim.score();
                sim = sim_factory(false);
                sim.set_score(score);
            }
        })
        .unwrap();

    /* Create the viewer */
    let viewer = ViewerProxy;

    // Install image loaders for loading the LAK logo.
    viewer.with_egui_context(|ctx| {
        egui_extras::install_image_loaders(ctx);
    });

    // Add the competition UI as MuJoCo-rs's viewer callback.
    viewer.add_ui_callback_detached(move |ctx| {  // (egui::Context)
        use fuzbai_simulator::egui::{
            self,
            epaint::Vertex,
            Color32, Mesh, FontId, RichText, Align, Direction, Layout
        };

        // Split the total remaining time into minutes and seconds.
        // Additionally, if remaining time is zero, switch to expired state.
        // We change to the expired state here to lower mutex contention when
        // processing ui-related things and because placing it under any button processing
        // code would cause the status to not change when window is minimized,
        // as egui does not process hidden widgets.
        let (rem_min, rem_sec, status) = {
            let mut comp_state = COMPETITION_STATE.lock_unpoison();
            match &comp_state.status {
                CompetitionStatus::Running(timer) => {
                    let total_rem_s = COMPETITION_DURATION_SECS.saturating_sub(timer.elapsed().as_secs());
                    if total_rem_s == 0 {
                        comp_state.status = CompetitionStatus::Expired;
                    }
                    (total_rem_s / 60, total_rem_s % 60, comp_state.status.clone())
                },
                _ => (0, 0, comp_state.status.clone())
            }
        };

        // Control panel window
        egui::Window::new("FuzbAI Competition").show(ctx, |ui| {
            ui.heading("Teams");
            egui::Grid::new("team_grid").num_columns(4).show(ui, |ui| {
                ui.label("Port");
                ui.label("Team");
                ui.end_row();
                for state_mutex in &states {

                    // Create strings here to prevent unnecessary mutex holding. The locking itself takes a few nano-seconds.
                    let (team_string, port_string) = {
                        let team_state = state_mutex.lock_unpoison();
                        let team = &team_state.team;
                        let team_name = &team_state.team_name;

                        let port_string = team_state.port.to_string();
                        let team_string = format!("{team_name} ({team:?})");
                        (team_string, port_string)
                    };

                    ui.label(port_string);
                    ui.label(team_string);

                    // Read the builtin state into a new variable, as locking the twice mutex is
                    // much faster than keeping the lock active during the entire call.
                    let mut builtin = state_mutex.lock_unpoison().builtin;

                    ui.checkbox(&mut builtin, "built-in");
                    state_mutex.lock_unpoison().builtin = builtin;
                    ui.end_row();
                }
            });

            ui.separator();
            match status {
                CompetitionStatus::Expired | CompetitionStatus::Free => {
                    ui.horizontal(|ui| {
                        if ui.button("Start").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            competition_state.status = CompetitionStatus::Running(Instant::now());
                            competition_state.pending.push_back(CompetitionPending::ResetScore);
                            competition_state.pending.push_back(CompetitionPending::ResetSimulation);
                        }

                        if ui.button("Start free").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            competition_state.status = CompetitionStatus::Free;
                            competition_state.pending.push_back(CompetitionPending::ResetSimulation);
                        }

                        if ui.button("Reset score").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            competition_state.pending.push_back(CompetitionPending::ResetScore);
                        }
                    });
                }

                CompetitionStatus::Running(_) => {
                    ui.horizontal(|ui| {
                        if ui.button("Stop").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            competition_state.status = CompetitionStatus::Expired;
                        }
                    });
                }
            }
        });

        // Top status panel
        egui::TopBottomPanel::top("team_status")
            .exact_height(100.0)
        .show(ctx, |ui| {
            // Default to white
            ui.style_mut().visuals.override_text_color = Some(Color32::WHITE);

            // Paint gradient (red to blue)
            let painter = ui.painter();
            const LEFT_COLOR: Color32 = Color32::from_rgb(180, 60, 60);
            const RIGHT_COLOR: Color32 = Color32::from_rgb(60, 90, 180);

            let rect = ui.clip_rect();
            let mut mesh = Mesh::default();
            mesh.vertices.push(Vertex {
                pos: rect.left_top(),
                uv: Default::default(),
                color: LEFT_COLOR,
            });
            mesh.vertices.push(Vertex {
                pos: rect.right_top(),
                uv: Default::default(),
                color: RIGHT_COLOR,
            });
            mesh.vertices.push(Vertex {
                pos: rect.right_bottom(),
                uv: Default::default(),
                color: RIGHT_COLOR,
            });
            mesh.vertices.push(Vertex {
                pos: rect.left_bottom(),
                uv: Default::default(),
                color: LEFT_COLOR,
            });
            mesh.indices.extend_from_slice(&[0, 1, 2, 0, 2, 3]);

            painter.add(egui::Shape::mesh(mesh));

            // Three columns: red team, central state, blue team.
            ui.set_width(300.0);
            ui.columns(3, |uis| {
                let [red_ui, mid_ui, blue_ui] = uis else { unreachable!() };
                red_ui.vertical(|ui| {
                    // Prepare before passing to egui to reduce mutex contention.
                    let (team_name_rt, score_rt) = {
                        let red_lock = states[0].lock_unpoison();
                        (
                            RichText::new(&red_lock.team_name).strong().font(FontId::proportional(30.0)),
                            RichText::new(red_lock.score.to_string()).font(FontId::proportional(24.0))
                        )
                    };

                    ui.label(team_name_rt);
                    ui.label(score_rt);
                });

                mid_ui.centered_and_justified(|ui| {
                    if rem_min == 0 && rem_sec == 0 {
                        egui::Frame::new()
                            .inner_margin(10.0)
                            .show(ui, |ui| {
                                ui.add(egui::Image::new("https://lak.fe.uni-lj.si/wp-content/uploads/2022/03/Logo_LAK.png"));
                            });
                    } else {
                        ui.label(
                            RichText::new(format!("{rem_min:02}:{rem_sec:02}"))
                                .font(FontId::proportional(30.0))
                        );
                    }
                });

                blue_ui.with_layout(
                    Layout::from_main_dir_and_cross_align(
                        Direction::TopDown,
                        Align::Max, // right side
                    ),
                    |ui| {
                        // Prepare before passing to egui to reduce mutex contention.
                        let (team_name_rt, score_rt) = {
                            let blue_lock = states[1].lock_unpoison();
                            (
                                RichText::new(&blue_lock.team_name).strong().font(FontId::proportional(30.0)),
                                RichText::new(blue_lock.score.to_string()).font(FontId::proportional(24.0))
                            )
                        };

                        ui.label(team_name_rt);
                        ui.label(score_rt);
                    },
                );
            });
        });
    });

    viewer.render_loop();  // loop while viewer is running.

    /* Final cleanup */
    shutdown_notify.notify_one();
    let _ = sim_thread.join();
    let _ = tokio_handle.join();
}


fn simulation_thread(sim: &mut FuzbAISimulator, states: [Arc<Mutex<http::ServerState>>; 2]) {
    let mut command_buffer = Vec::with_capacity(4);
    while sim.viewer_running() {
        let competition_expired = {
            let mut comp_state = COMPETITION_STATE.lock_unpoison();
            while let Some(pending) = comp_state.pending.pop_front() {
                match pending {
                    CompetitionPending::ResetScore => sim.clear_score(),
                    CompetitionPending::ResetSimulation => sim.reset_simulation(),
                }
            }
            comp_state.expired()
        };

        if competition_expired {
            sim.reset_simulation();
            sim.serve_ball(Some(EXPIRED_BALL_POSITION));
        }

        // Step simulation and synchronize the state.
        if sim.terminated() || sim.truncated() {
            println!("Score (Red-Blue): {:?}", sim.score());
            sim.reset_simulation();
        }
        sim.step_simulation();

        /* Sync the simulation state with our competition state */
        let score = sim.score().clone();
        for state in &states {
            let mut state = state.lock_unpoison();
            let team = state.team.clone();
            let observation = sim.delayed_observation(team.clone(), None);
            let (
                mut ball_x, mut ball_y, ball_vx, ball_vy,
                rod_position_calib, rod_angle
            ) = observation;

            // When the ball is not detected on the real table, the system returns -1 for the position.
            if competition_expired {
                ball_x = -1.0;
                ball_y = -1.0;
            }

            let camera_data_0 = http::CameraData {
                cameraID: 0,
                ball_x, ball_y, ball_vx, ball_vy, ball_size: 35.0,
                rod_position_calib, rod_angle
            };

            // Assume the second camera gives the same results.
            let camera_data_1 = http::CameraData {cameraID: 1, ..camera_data_0};

            /* Update the state for the configured team */
            state.camera_state.camData = [camera_data_0, camera_data_1];
            state.score = score[team.clone() as usize];
            sim.set_external_mode(team.clone(), !state.builtin);

            command_buffer.clear();
            command_buffer.extend(state.pending_commands.iter().take(4).map(|c|
                (c.driveID, c.translationTargetPosition, c.rotationTargetPosition, c.translationVelocity, c.rotationVelocity)
            ));

            state.pending_commands.clear();
            sim.set_motor_command(&command_buffer, team);
        }
    }
}
