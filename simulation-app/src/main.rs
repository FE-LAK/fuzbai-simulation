
use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use serde::{Serialize, Deserialize};
use tokio::runtime::Builder;
use actix_files::{Files};
use actix_web::web;

use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
use std::{collections::VecDeque, sync::{Arc, LazyLock, Mutex}, time::Instant};


const NUM_TOKIO_THREADS: usize = 4;

static COMPETITION_STATE: LazyLock<Mutex<CompetitionState>> = LazyLock::new(|| Mutex::new(CompetitionState::default()));


/// Data received from a single camera.
#[derive(Serialize, Default, Clone)]
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
#[derive(Serialize, Clone)]
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


#[derive(Deserialize, Clone)]
#[allow(non_snake_case)]
struct MotorCommand {
    driveID: usize,
    rotationTargetPosition: f64,
    rotationVelocity: f64,
    translationTargetPosition: f64,
    translationVelocity: f64
}

#[derive(Deserialize)]
#[allow(non_snake_case)]
struct MotorCommands {
    commands: Vec<MotorCommand>
}


/// Shared state between HTTP server and the simulation.
struct ServerState {
    /* Synced from simulation to server (state) */
    camera_state: CameraState,
    score: u16,
    pending_commands: Vec<MotorCommand>,

    /* Synced from server (state) to simulation or part of the server (state) */
    port: u16,
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
        Arc::new(Mutex::new(ServerState {
            camera_state: CameraState::new(), port: port_0, team: PlayerTeam::Red, team_name: "Team 1".to_string(),
            score: 0, builtin: false, pending_commands: Vec::new()
        })),
        Arc::new(Mutex::new(ServerState {
            camera_state: CameraState::new(), port: port_1, team: PlayerTeam::Blue, team_name: "Team 2".to_string(),
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

        runtime.block_on(http_task(states_clone));
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
            sim.set_external_mode(team.clone(), !state.builtin);

            let commands: Box<_> = state.pending_commands.iter().map(|c|
                (c.driveID, c.translationTargetPosition, c.rotationTargetPosition, c.translationVelocity, c.rotationVelocity)
            ).collect();

            sim.set_motor_command(&commands, team);
        }
    }
}


async fn http_task(states: [Arc<Mutex<ServerState>>; 2]) {
    let factory = |state: Arc<Mutex<ServerState>>| {
            let port = state.lock().unwrap().port;
            HttpServer::new(move || {
            App::new()
                .app_data(web::Data::new(state.clone()))
                .service(get_camera_state)
                .service(send_command)
                .service(get_competition)
                .service(get_state)
                .service(reset_rotations)
                .service(
                    Files::new("/", "./www/")
                    .index_file("Render.html")
                )
        })
        .workers(1)
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
async fn get_camera_state(data: web::Data<Arc<Mutex<ServerState>>>) -> impl Responder {
    HttpResponse::Ok().json(&data.lock().unwrap().camera_state)
}

#[get("/Competition")]
async fn get_competition(data: web::Data<Arc<Mutex<ServerState>>>) -> impl Responder {
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
async fn send_command(data: web::Data<Arc<Mutex<ServerState>>>, commands: web::Json<MotorCommands>) -> impl Responder {
    data.lock().unwrap().pending_commands = commands.into_inner().commands;
    HttpResponse::Ok()
}

#[get("/Motors/ResetRotations")]
async fn reset_rotations() -> impl Responder {
    HttpResponse::Ok().json(serde_json::json!( {"message": "Resetting rotations."} ))
}

#[derive(Serialize, Clone)]
#[allow(non_snake_case)]
struct StateCameraStateRod {
    Item1: f64,
    Item2: f64,
}

#[derive(Serialize, Clone)]
struct StateCameraState   {
    rods: [StateCameraStateRod; 8],
    ball_x: f64,
    ball_y: f64,
    ball_vx: f64,
    ball_vy: f64,
    ball_size: f64,
}


#[derive(Serialize)]
#[allow(non_snake_case)]
struct StateDrivesStateDrivesValue {
    axisEncoderPosition: f64,
}

#[derive(Serialize)]
#[allow(non_snake_case)]
struct StateDrivesStateDrive {
    translation: StateDrivesStateDrivesValue,
    rotation: StateDrivesStateDrivesValue
}

#[derive(Serialize)]
#[allow(non_snake_case)]
struct StateDrivesState {
    drives: [StateDrivesStateDrive; 4]
}

#[derive(Serialize)]
#[allow(non_snake_case)]
struct State {
    motorsReady: bool,
    camData: [StateCameraState; 2],
    drivesStates: StateDrivesState
}

#[get("/State")]
async fn get_state(data: web::Data<Arc<Mutex<ServerState>>>) -> impl Responder {
    let camera_state = data.lock().unwrap().camera_state.clone();
    let rp = camera_state.camData[0].rod_position_calib;
    let rr = camera_state.camData[0].rod_angle;

    let ball_x = camera_state.camData[0].ball_x;
    let ball_y = camera_state.camData[0].ball_y;
    let ball_vx = camera_state.camData[0].ball_vx;
    let ball_vy = camera_state.camData[0].ball_vy;
    let ball_size = camera_state.camData[0].ball_size;

    let state_cam_data = StateCameraState {
        rods: std::array::from_fn(|i|
            StateCameraStateRod { Item1: rp[i], Item2: rr[i] },
        ),
        ball_x, ball_y, ball_vx, ball_vy, ball_size
    };

    HttpResponse::Ok().json(
        State {
            motorsReady: true,
            camData: [
                state_cam_data.clone(),
                state_cam_data
            ],
            drivesStates: StateDrivesState { drives: [
                StateDrivesStateDrive {translation: StateDrivesStateDrivesValue { axisEncoderPosition: rp[0] }, rotation: StateDrivesStateDrivesValue { axisEncoderPosition: rr[0] }},
                StateDrivesStateDrive {translation: StateDrivesStateDrivesValue { axisEncoderPosition: rp[1] }, rotation: StateDrivesStateDrivesValue { axisEncoderPosition: rr[1] }},
                StateDrivesStateDrive {translation: StateDrivesStateDrivesValue { axisEncoderPosition: rp[3] }, rotation: StateDrivesStateDrivesValue { axisEncoderPosition: rr[3] }},
                StateDrivesStateDrive {translation: StateDrivesStateDrivesValue { axisEncoderPosition: rp[5] }, rotation: StateDrivesStateDrivesValue { axisEncoderPosition: rr[5] }},
            ] }
        }
    )
}
