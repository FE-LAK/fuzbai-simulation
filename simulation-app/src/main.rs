
use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use tokio::runtime::Builder;
use actix_files::{Files};
use serde::Serialize;
use actix_web::web;

use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
use std::{collections::VecDeque, sync::{Arc, LazyLock, Mutex}, time::Instant};


const NUM_TOKIO_THREADS: usize = 4;

static COMPETITION_STATE: LazyLock<Mutex<CompetitionState>> = LazyLock::new(|| Mutex::new(CompetitionState::default()));


/// Data received from a single camera.
#[derive(Serialize, Default)]
#[allow(non_snake_case)]
struct CameraData {
    cameraID: u32,
    ball_x: f64,
    ball_y: f64,
    ball_vx: f64,
    ball_vy: f64,
    ball_size: f64,
    rod_position_calib: [f64; 8],
    rod_angle: [f64; 8],
}

impl CameraData {
    fn new(id: u32) -> Self {
        Self {
            cameraID: id,
            ..Default::default()
        }
    }
}


/// Return type for [`camera_state`].
/// Contains state for both cameras.
#[derive(Serialize)]
#[allow(non_snake_case)]
struct CameraState {
    camData: [CameraData; 2],
    camDataOK: [bool; 2],
}

impl CameraState {
    fn new() -> Self {
        Self {
            camData: [CameraData::new(0), CameraData::new(1)],
            camDataOK: [true, true]
        }
    }    
}


/// Shared state between HTTP server and the simulation.
struct ServerState {
    camera_state: CameraState,
    port: u16,
    score: u16,
    team: PlayerTeam,
    team_name: String,
    builtin: bool
}

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
    /* Initialize states for each team */
    let states = [
        Arc::new(Mutex::new(ServerState {
            camera_state: CameraState::new(), port: 8080, team: PlayerTeam::Red, team_name: "Team 1".to_string(),
            score: 0, builtin: false
        })),
        Arc::new(Mutex::new(ServerState {
            camera_state: CameraState::new(), port: 8081, team: PlayerTeam::Blue, team_name: "Team 2".to_string(),
            score: 0, builtin: false
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

        runtime.block_on(http_task(states_clone));
    });

    /* Initialize simulation */
    let mut sim = FuzbAISimulator::new(
        5, 5,
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

    /* Loop forever within Rust in the main thread and also without GIL */
    let viewer = ViewerProxy;
    viewer.add_ui_callback(move |ctx| {
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

    viewer.render_loop();

    /* Final cleanup */
    let _ = sim_thread.join();
    let _ = tokio_handle.join();
}


fn simulation_thread(mut sim: FuzbAISimulator, states: [Arc<Mutex<ServerState>>; 2]) {
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

            let camera_data_0 = CameraData {
                cameraID: 0,
                ball_x, ball_y, ball_vx, ball_vy, ball_size: 35.0,
                rod_position_calib, rod_angle
            };

            // Assume the second camera gives the same results.
            let camera_data_1 = CameraData {cameraID: 1, ..camera_data_0};

            /* Update the state for the configured team */
            state.camera_state.camData = [camera_data_0, camera_data_1];
            state.score = score[team.clone() as usize];
            sim.set_external_mode(team, !state.builtin);
        }
    }
}


async fn http_task(states: [Arc<Mutex<ServerState>>; 2]) {
    let factory = |state: Arc<Mutex<ServerState>>| {
            let port = state.lock().unwrap().port;
            HttpServer::new(move || {
            App::new()
                .app_data(web::Data::new(state.clone()))
                .service(camera_state)
                .service(send_command)
                .service(competition)
                .service(
                    Files::new("/", "./www/")
                    .index_file("Render.html")
                )
        })
        .bind(("127.0.0.1", port)).unwrap()
        .run()
    };

    let mut join_set = tokio::task::JoinSet::new();
    for state in states {
        join_set.spawn(factory(state.clone()));
    }

    while let Some(result) = join_set.join_next().await {
        match result {
            Ok(r) => {
                if let Err(err) = r {
                    eprintln!("task errored: {err}");
                }
            },
            Err(e) => {
                eprintln!("join error: {e}");
            }
        }
    }
}


#[get("/Camera/State")]
async fn camera_state(data: web::Data<Arc<Mutex<ServerState>>>) -> impl Responder {
    HttpResponse::Ok().json(&data.lock().unwrap().camera_state)
}

#[get("/Competition")]
async fn competition(data: web::Data<Arc<Mutex<ServerState>>>) -> impl Responder {
    let name = &data.lock().unwrap().team_name;
    let json: serde_json::Value = serde_json::json! {
        {
            "state": 2, "time": 0, "playerName": name,
            "scorePlayer": "", "scoreFuzbAI": "", "level": 0, "results": []
        }
    };

    HttpResponse::Ok().json(json)
}

#[post("/Motors/SendCommand")]
async fn send_command() -> impl Responder {
    HttpResponse::Ok()
}
