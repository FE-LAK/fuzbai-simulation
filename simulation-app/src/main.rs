use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
use fuzbai_simulator::mujoco_rs::mujoco_c;

use std::{collections::VecDeque, sync::{Arc, LazyLock, Mutex}, time::{Instant}};
use std::panic::{catch_unwind, AssertUnwindSafe};

use tokio::runtime::Builder;
use tokio::sync::Notify;

use traits::LockOrUnpoison;

const NUM_TOKIO_THREADS: usize = 4;
const COMPETITION_DURATION_SECS: u64 = 120;
const EXPIRED_BALL_BALL_POSITION: [f64; 3] = [605.0, -100.0, 100.0];

static COMPETITION_STATE: LazyLock<Mutex<CompetitionState>> = LazyLock::new(|| Mutex::new(CompetitionState::default()));

mod traits;
mod http;


unsafe extern "C" fn handle_mujoco_error(c_error_message: *const std::os::raw::c_char) {
    unsafe {
        if let Ok(e_msg) = std::ffi::CStr::from_ptr(c_error_message).to_str() {
            println!("ERROR! MuJoCo's C library issued an error: {e_msg} !!! Resetting simulation state and model !!!");
        }
    }

    let mut lock = COMPETITION_STATE.lock_no_poison();
    lock.pending.push_back(CompetitionPending::ReloadSimulation);
    println!("Reloading simulation state due to internal MuJoCo error");
}


enum CompetitionPending {
    ResetScore,
    ResetSimulation,
    ReloadSimulation
}

#[derive(PartialEq)]
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
    unsafe { mujoco_c::mju_user_error = Some(handle_mujoco_error) };

    // Initialize the rest of Rust code
    let port_0 = args.next()
        .unwrap_or_else(|| "8080".into())
        .parse::<u16>().expect("passed team 1 port passed was invalid");
    let port_1 = args.next()
        .unwrap_or_else(|| "8081".into())
        .parse::<u16>().expect("passed team 2 port passed was invalid");

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
                .worker_threads(NUM_TOKIO_THREADS)
                .enable_all()
                .build()
                .unwrap();

            runtime.block_on(http::http_task(states_clone, shutdown_notify_clone));
    }).unwrap();

    /* Initialize simulation */
    let mut sim = FuzbAISimulator::new(
        1, 5,
        true,
        0.050,
        None,
        VisualConfig::new(
            0, false,
            0, true
        ),
    );

    sim.set_external_mode(PlayerTeam::Red, true);
    sim.set_external_mode(PlayerTeam::Blue, true);

    /* Start physics in another thread */
    let states_clone = states.clone();
    let sim_thread = std::thread::Builder::new()
        .name("simulation".into())
        .spawn(move || simulation_thread(sim, states_clone))
        .unwrap();

    /* Create the viewer */
    let viewer = ViewerProxy;

    // Add UI callbacks. Note that internally, viewer's passive state (used for synchronization)
    // is locked (Mutex). This callbacks acquires a lock to the individual server state, which
    // is a POTENTIAL DANGER REGARDING DEADLOCKS. If The server state gets locked at any other location,
    // make sure YOU DO NOT CALL ANY VIEWER METHODS unless you unlock the server state (i.e., release its Mutex).
    viewer.add_ui_callback(move |ctx, _| {  // (egui::Context, mujoco_rs::MjData [Note that the latter is locked by the passive state Mutex])
        use fuzbai_simulator::egui;
        egui::Window::new("FuzbAI Competition").show(ctx, |ui| {
            ui.heading("Teams");
            egui::Grid::new("team_grid").num_columns(4).show(ui, |ui| {
                ui.label("Port");
                ui.label("Team");
                ui.label("Score");
                ui.end_row();
                for state in &states {
                    let mut state = state.lock_no_poison();
                    let team = &state.team;
                    let team_name = &state.team_name;
                    let port = state.port;
                    let score = state.score;
                    ui.label(port.to_string());
                    ui.label(format!("{team_name} ({team:?})"));
                    ui.label(score.to_string());
                    ui.checkbox(&mut state.builtin, "built-in");
                    ui.end_row();
                }
            });

            ui.separator();
            let mut state = COMPETITION_STATE.lock_no_poison();
            match state.status {
                CompetitionStatus::Expired | CompetitionStatus::Free => {
                    ui.horizontal(|ui| {
                        if ui.button("Start").clicked() {
                            state.status = CompetitionStatus::Running(Instant::now());
                            state.pending.push_back(CompetitionPending::ResetScore);
                            state.pending.push_back(CompetitionPending::ResetSimulation);
                        }

                        if ui.button("Start free").clicked() {
                            state.status = CompetitionStatus::Free;
                            state.pending.push_back(CompetitionPending::ResetSimulation);
                        }

                        if ui.button("Swap teams").clicked() {
                            let team_0 = &mut states[0].lock_no_poison().team;
                            let team_1 = &mut states[1].lock_no_poison().team;
                            let tmp = team_0.clone();
                            *team_0 = team_1.clone();
                            *team_1 = tmp;
                        }

                        if ui.button("Reset score").clicked() {
                            state.pending.push_back(CompetitionPending::ResetScore);
                        }
                    });

                    if let CompetitionStatus::Expired = state.status {
                        ui.label("Competition time expired");
                    }
                }

                CompetitionStatus::Running(timer) => {
                    ui.horizontal(|ui| {
                        if ui.button("Stop").clicked() {
                            state.status = CompetitionStatus::Expired;
                        }

                        let rem_total_seconds = COMPETITION_DURATION_SECS - timer.elapsed().as_secs().min(COMPETITION_DURATION_SECS);
                        let rem_minutes = rem_total_seconds / 60;
                        let rem_seconds = rem_total_seconds % 60;
                        ui.label(format!("Remaining: {rem_minutes:02}:{rem_seconds:02}",));

                        if rem_total_seconds == 0 {
                            state.status = CompetitionStatus::Expired;
                        }
                    });
                }
            }
        });
    });

    viewer.render_loop();  // loop while viewer is running.

    /* Final cleanup */
    shutdown_notify.notify_one();
    let _ = sim_thread.join();
    let _ = tokio_handle.join();
}


fn simulation_thread(mut sim: FuzbAISimulator, states: [Arc<Mutex<http::ServerState>>; 2]) {
    while sim.viewer_running() {      
        let mut comp_state = COMPETITION_STATE.lock_no_poison();
        while let Some(pending) = comp_state.pending.pop_front() {
            match pending {
                CompetitionPending::ResetScore => sim.clear_score(),
                CompetitionPending::ResetSimulation => sim.reset_simulation(),
                CompetitionPending::ReloadSimulation => sim.reload_simulation()
            }
        }

        let expired = comp_state.expired();
        if expired  {
            sim.reset_simulation();
            sim.serve_ball(Some(EXPIRED_BALL_BALL_POSITION));
        }

        drop(comp_state);

        // Step simulation and synchronize the state.
        // Catch any potential panics (just in case, none were detected in testing) --- we aren't  :).
        let result = catch_unwind(AssertUnwindSafe(|| {
            if sim.terminated() || sim.truncated() {
                println!("Score (Red-Blue): {:?}", sim.score());
                sim.reset_simulation();
            }
            sim.step_simulation();

            /* Sync the simulation state with our competition state */
            let score = sim.score().clone();
            for state in &states {
                let mut state = state.lock_no_poison();
                let team = state.team.clone();
                let observation = sim.delayed_observation(team.clone(), None);
                let (
                    mut ball_x, mut ball_y, ball_vx, ball_vy,
                    rod_position_calib, rod_angle
                ) = observation;

                // When the ball is not detected on the real table, the system returns -1 for the position.
                if expired {
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

                let commands: Box<_> = state.pending_commands.iter().map(|c|
                    (c.driveID, c.translationTargetPosition, c.rotationTargetPosition, c.translationVelocity, c.rotationVelocity)
                ).collect();

                sim.set_motor_command(&commands, team);
            }
        }));
        ////////////////////// END catch unwind

        // HOT RELOAD
        // In case the code panicked for whatever reason, the simulation must be reset without resetting the score.
        if result.is_err() {
            let score = *sim.score();
            sim = FuzbAISimulator::new(
                1, 5,
                true,
                0.050,
                None,
                VisualConfig::new(
                    0, false,
                    0, false
                ),
            );
            sim.set_score(score);
        }
    }
}
