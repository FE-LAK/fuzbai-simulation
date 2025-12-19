
use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use serde::{Serialize, Deserialize};
use tokio::runtime::Builder;
use actix_files::{Files};
use actix_web::web;

use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
use std::{collections::VecDeque, sync::{Arc, LazyLock, Mutex}, time::Instant};


const NUM_TOKIO_THREADS: usize = 4;

static COMPETITION_STATE: LazyLock<Mutex<CompetitionState>> = LazyLock::new(|| Mutex::new(CompetitionState::default()));

mod http;



enum CompetitionPending {
    ResetScore
}

struct CompetitionState {
    timer: Instant,
    pending: VecDeque<CompetitionPending>
}

impl Default for CompetitionState {
    fn default() -> Self {
        Self { timer: Instant::now(), pending: VecDeque::new() }
    }
}


fn main() {
    let mut args = std::env::args();
    let _ = args.next().unwrap();  // program path;

    let port_0 = args.next()
        .unwrap_or("8080".into())
        .parse::<u16>().expect("passed team 1 port passed was invalid");
    let port_1 = args.next()
        .unwrap_or("8081".into())
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
    let tokio_handle = std::thread::spawn(move || {
        let runtime = Builder::new_current_thread()
            .worker_threads(NUM_TOKIO_THREADS)
            .enable_all()
            .build()
            .unwrap();

        runtime.block_on(http::http_task(states_clone));
    });

    /* Initialize simulation */
    let mut sim = FuzbAISimulator::new(
        1, 5,
        true,
        0.055,
        None,
        VisualConfig::new(
            10, true,
            0, true
        ),
    );

    sim.set_external_mode(PlayerTeam::Red, true);
    sim.set_external_mode(PlayerTeam::Blue, true);

    /* Start physics in another thread */
    let states_clone = states.clone();
    let sim_thread = std::thread::spawn(move || {
        simulation_thread(sim, states_clone);
    });

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
                    let mut state = state.lock().unwrap();
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

                if ui.button("Swap teams").clicked() {
                    let team_0 = &mut states[0].lock().unwrap().team;
                    let team_1 = &mut states[1].lock().unwrap().team;
                    let tmp = team_0.clone();
                    *team_0 = team_1.clone();
                    *team_1 = tmp;
                }

                if ui.button("Reset score").clicked() {
                    COMPETITION_STATE.lock().unwrap().pending.push_back(CompetitionPending::ResetScore);
                }
            });
        });
    });

    viewer.render_loop();  // loop while viewer is running.

    /* Final cleanup */
    let _ = sim_thread.join();
    let _ = tokio_handle.join();
}


fn simulation_thread(mut sim: FuzbAISimulator, states: [Arc<Mutex<http::ServerState>>; 2]) {
    while sim.viewer_running() {
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation();
        }
        sim.step_simulation();

        let mut comp_state = COMPETITION_STATE.lock().unwrap();
        while let Some(pending) = comp_state.pending.pop_front() {
            match pending {
                CompetitionPending::ResetScore => { sim.clear_score(); }
            }
        }
        drop(comp_state);

        /* Sync the simulation state with our competition state */
        let score = sim.score().clone();
        for state in &states {
            let mut state = state.lock().unwrap();
            let team = state.team.clone();
            let observation = sim.delayed_observation(team.clone(), None);
            let (
                ball_x, ball_y, ball_vx, ball_vy,
                rod_position_calib, rod_angle
            ) = observation;

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
    }
}
