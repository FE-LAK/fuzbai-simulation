use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
use fuzbai_simulator::mujoco_rs::util::LockUnpoison;

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use tokio::runtime::Builder;
use tokio::sync::Notify;

use clap::Parser;

use crate::http::{
    DEFAULT_MANAGEMENT_HOST, DEFAULT_MANAGEMENT_PORT,
    DEFAULT_TEAM_HOST, DEFAULT_TEAM_1_PORT, DEFAULT_TEAM_2_PORT,
};
use crate::competition::{
    COMPETITION_DURATION_SECS, COMPETITION_WIN_GOALS, COMPETITION_STATE,
    CompetitionPending, CompetitionStatus,
};

mod competition;
mod http;

const EXPIRED_BALL_POSITION: [f64; 3] = [605.0, -100.0, 100.0];

/// Default value of observation delay.
const DEFAULT_DELAY_S: f64 = 0.055;

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

/// Timeout, in seconds, after which the frequency display should consider the connection
/// inactive.
const CONNECTION_FREQUENCY_DISPLAY_TIMEOUT_SECS: u64 = 1;

/// FuzbAI robotic foosball simulation application.
#[derive(Parser)]
#[command(about)]
struct Cli {
    /// Port for the first team server.
    #[arg(long, default_value_t = DEFAULT_TEAM_1_PORT)]
    team1_port: u16,

    /// Port for the second team server.
    #[arg(long, default_value_t = DEFAULT_TEAM_2_PORT)]
    team2_port: u16,

    /// Port for the management server.
    #[arg(long, default_value_t = DEFAULT_MANAGEMENT_PORT)]
    management_port: u16,

    /// Host address for team servers.
    #[arg(long, default_value = DEFAULT_TEAM_HOST)]
    team_host: String,

    /// Host address for the management server.
    #[arg(long, default_value = DEFAULT_MANAGEMENT_HOST)]
    management_host: String,

    /// Length of the observation delay in seconds.
    #[arg(long, default_value_t = DEFAULT_DELAY_S)]
    delay_s: f64
}

fn main() {
    let cli = Cli::parse();

    // Set the MuJoCo error handler (from C language) to catch internal MuJoCo crashes
    // and convert them to panics
    #[cfg(target_family = "unix")]
    unsafe { mju_user_error = Some(handle_mujoco_error) };

    // Dumb Microsoft logic: the variable is a pointer to the variable
    #[cfg(target_os = "windows")]
    unsafe {
        *mju_user_error = Some(handle_mujoco_error)
    };

    /* Extract CLI configuration */
    let port_0 = cli.team1_port;
    let port_1 = cli.team2_port;
    let port_management = cli.management_port;
    let team_host = cli.team_host;
    let management_host = cli.management_host;
    let delay_s = cli.delay_s;

    /* Initialize states for each team */
    let team_states = [
        Arc::new(Mutex::new(http::TeamState {
            camera_state: http::CameraState::new(), port: port_0, team: PlayerTeam::Red, team_name: "Team 1".to_string(),
            score: 0, builtin: false, pending_commands: Vec::new(),
            last_command_time: Instant::now(), frequency_smooth_hz: 0.0
        })),
        Arc::new(Mutex::new(http::TeamState {
            camera_state: http::CameraState::new(), port: port_1, team: PlayerTeam::Blue, team_name: "Team 2".to_string(),
            score: 0, builtin: false, pending_commands: Vec::new(),
            last_command_time: Instant::now(), frequency_smooth_hz: 0.0
        }))
    ];

    /* Initialize tokio runtime and with it, the HTTP server */
    let states_clone = team_states.clone();

    // Clone host strings and convert ports to strings before they are moved into the tokio thread.
    let team_host_ui = team_host.clone();
    let management_host_ui = management_host.clone();
    let management_port_str = port_management.to_string();
    let team_port_strings = [port_0.to_string(), port_1.to_string()];

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
            runtime.block_on(http::http_task(
                states_clone, port_management,
                &team_host, &management_host,
                shutdown_notify_clone
            ));
    }).unwrap();

    /* Define simulation factory */
    let sim_factory = move |init_viewer| {
        FuzbAISimulator::new(
            1, // internal_step_factor: .step_simulation() = N * (2 ms)
            5, // sample_steps: save state to delay buffer every N * (2 ms). .delayed_observation() returns discrete samples every N * 2ms.
            true,
            delay_s,
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
    let states_clone = team_states.clone();
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

        // LAK logo. Loaded statically into the binary at compile-time.
        const LAK_IMAGE_SRC: egui::ImageSource<'static> = egui::include_image!("../www/lak_logo.png");

        // Split the total remaining time into minutes and seconds (read-only).
        // The actual Running -> Finished transition happens in the simulation thread.
        let (rem_min, rem_sec, status) = {
            let comp_state = COMPETITION_STATE.lock_unpoison();
            match &comp_state.status {
                CompetitionStatus::Running(timer) => {
                    let total_rem_s = COMPETITION_DURATION_SECS.saturating_sub(timer.elapsed().as_secs());
                    (total_rem_s / 60, total_rem_s % 60, comp_state.status.clone())
                },
                CompetitionStatus::Paused(elapsed) | CompetitionStatus::Finished(elapsed) => {
                    let total_rem_s = COMPETITION_DURATION_SECS.saturating_sub(elapsed.as_secs());
                    (total_rem_s / 60, total_rem_s % 60, comp_state.status.clone())
                },
                _ => (0, 0, comp_state.status.clone())
            }
        };

        // Control panel window
        egui::Window::new("FuzbAI Competition").show(ctx, |ui| {
            ui.heading("Management");
            egui::Grid::new("management_grid").num_columns(2).show(ui, |ui| {
                ui.label("Host");
                ui.label("Port");
                ui.end_row();
                ui.label(&management_host_ui);
                ui.label(&management_port_str);
                ui.end_row();
            });

            ui.separator();
            ui.heading("Teams");
            egui::Grid::new("team_grid").num_columns(5).show(ui, |ui| {
                ui.label("Host");
                ui.label("Port");
                ui.label("Team");
                ui.end_row();
                for (state_mutex, port_str) in team_states.iter().zip(&team_port_strings) {

                    // Create strings here to prevent unnecessary mutex holding. The locking itself takes a few nano-seconds.
                    let (team_string, mut builtin, frequency_string) = {
                        let team_state = state_mutex.lock_unpoison();
                        let team = &team_state.team;
                        let team_name = &team_state.team_name;

                        let team_string = format!("{team_name} ({team:?})");
                        let frequency_string =
                        if team_state.last_command_time.elapsed().as_secs() < CONNECTION_FREQUENCY_DISPLAY_TIMEOUT_SECS {
                            format!("{:.2} Hz", team_state.frequency_smooth_hz)
                        } else { "Inactive".to_string() };
                        (team_string, team_state.builtin, frequency_string)
                    };

                    ui.label(&team_host_ui);
                    ui.label(port_str);
                    ui.label(team_string);
                    if ui.checkbox(&mut builtin, "built-in").clicked() {
                        state_mutex.lock_unpoison().builtin = builtin;
                    };
                    ui.label(frequency_string);
                    ui.end_row();
                }
            });

            ui.separator();
            match status {
                CompetitionStatus::Free | CompetitionStatus::Waiting => {
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
                },
                CompetitionStatus::Running(timer) => {
                    ui.horizontal(|ui| {
                        if ui.button("Stop").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            competition_state.status = CompetitionStatus::Waiting;
                        }
                        if ui.button("Pause").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            competition_state.status = CompetitionStatus::Paused(timer.elapsed());
                        }
                    });
                },
                CompetitionStatus::Paused(elapsed) => {
                    ui.horizontal(|ui| {
                        if ui.button("Resume").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            let maybe_instant = Instant::now().checked_sub(elapsed);
                            // A "just-in-case" check, which shouldn't be possible realistically, however we have no panic guards here
                            if let Some(instant) = maybe_instant {
                                competition_state.status = CompetitionStatus::Running(instant);
                                competition_state.pending.push_back(CompetitionPending::ResetSimulation);
                            }
                        }
                        if ui.button("Stop").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            competition_state.status = CompetitionStatus::Waiting;
                        }
                    });
                },
                CompetitionStatus::Finished(_) => {
                    ui.horizontal(|ui| {
                        if ui.button("Stop").clicked() {
                            let mut competition_state = COMPETITION_STATE.lock_unpoison();
                            competition_state.status = CompetitionStatus::Waiting;
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

            // Team_states[0] is always Red, [1] is always Blue.
            let (mut red_name, red_score) = {
                let lock = team_states[0].lock_unpoison();
                (lock.team_name.clone(), lock.score)
            };
            let (mut blue_name, blue_score) = {
                let lock = team_states[1].lock_unpoison();
                (lock.team_name.clone(), lock.score)
            };

            ui.columns(3, |uis| {
                let [red_ui, mid_ui, blue_ui] = uis else { unreachable!() };

                red_ui.vertical(|ui| {
                    if ui.add(
                        egui::TextEdit::singleline(&mut red_name)
                            .background_color(Color32::TRANSPARENT)
                            .font(FontId::proportional(30.0))
                    ).changed() {
                        team_states[0].lock_unpoison().team_name = red_name;
                    }

                    ui.label(RichText::new(red_score.to_string()).font(FontId::proportional(24.0)));
                });

                mid_ui.centered_and_justified(|ui| {
                    if rem_min == 0 && rem_sec == 0 {
                        egui::Frame::new()
                            .inner_margin(10.0)
                            .show(ui, |ui| {
                                ui.image(LAK_IMAGE_SRC);
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
                        if ui.add(
                            egui::TextEdit::singleline(&mut blue_name)
                                .background_color(Color32::TRANSPARENT)
                                .font(FontId::proportional(30.0))
                                .horizontal_align(Align::Max)
                        ).changed() {
                            team_states[1].lock_unpoison().team_name = blue_name;
                        }
                        ui.label(RichText::new(blue_score.to_string()).font(FontId::proportional(24.0)));
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


fn simulation_thread(sim: &mut FuzbAISimulator, team_states: [Arc<Mutex<http::TeamState>>; 2]) {
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

            // Check timer expiry and 5-goal win condition (Running -> Finished)
            if let CompetitionStatus::Running(timer) = &comp_state.status {
                let elapsed = timer.elapsed();
                let score = sim.score();
                if elapsed.as_secs() >= COMPETITION_DURATION_SECS
                    || score[0] >= COMPETITION_WIN_GOALS
                    || score[1] >= COMPETITION_WIN_GOALS
                {
                    comp_state.status = CompetitionStatus::Finished(elapsed);
                }
            }

            matches!(comp_state.status, CompetitionStatus::Paused(_)) ||
                matches!(comp_state.status, CompetitionStatus::Finished(_)) ||
                matches!(comp_state.status, CompetitionStatus::Waiting)
        };

        if competition_expired {
            sim.reset_simulation();
            sim.serve_ball(Some(EXPIRED_BALL_POSITION));
        }

        // Step simulation and synchronize the state.
        if sim.terminated() || sim.truncated() {
            let score = sim.score();
            let red_name = team_states[0].lock_unpoison().team_name.clone();
            let blue_name = team_states[1].lock_unpoison().team_name.clone();
            println!("Score ({} - {}): [{}, {}]", red_name, blue_name, score[0], score[1]);
            sim.reset_simulation();
        }
        sim.step_simulation();

        /* Sync the simulation state with our competition state */
        let score = *sim.score();
        for team_state in &team_states {
            let mut team_state_lock = team_state.lock_unpoison();
            let team = team_state_lock.team;
            let observation = sim.delayed_observation(team, None);
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
            team_state_lock.camera_state.camData = [camera_data_0, camera_data_1];
            team_state_lock.score = score[team as usize];
            sim.set_external_mode(team, !team_state_lock.builtin);

            command_buffer.clear();
            command_buffer.extend(team_state_lock.pending_commands.iter().take(4).map(|c|
                (c.driveID, c.translationTargetPosition, c.rotationTargetPosition, c.translationVelocity, c.rotationVelocity)
            ));

            team_state_lock.pending_commands.clear();
            sim.set_motor_command(&command_buffer, team);
        }
    }
}
