

use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use serde::{Serialize, Deserialize};
use actix_files::{Files};
use actix_web::web;

use fuzbai_simulator::PlayerTeam;
use std::sync::{Arc, Mutex};
use tokio::sync::Notify;



/// How many (CPU) workers to create on each server (red and blue).
/// A single worker will contain a new single-threaded tokio runtime in
/// which tasks will be spawn.
const NUM_WORKERS_PER_SERVER: usize = 3;


/// Data received from a single camera.
#[derive(Serialize, Default, Clone)]
#[allow(non_snake_case)]
pub struct CameraData {
    pub cameraID: u32,
    pub ball_x: f64,
    pub ball_y: f64,
    pub ball_vx: f64,
    pub ball_vy: f64,
    pub ball_size: f64,
    pub rod_position_calib: [f64; 8],
    pub rod_angle: [f64; 8],
}

impl CameraData {
    pub fn new(id: u32) -> Self {
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
pub struct CameraState {
    pub camData: [CameraData; 2],
    pub camDataOK: [bool; 2],
}

impl CameraState {
    pub fn new() -> Self {
        Self {
            camData: [CameraData::new(0), CameraData::new(1)],
            camDataOK: [true, true]
        }
    }    
}


#[derive(Deserialize, Clone)]
#[allow(non_snake_case)]
pub struct MotorCommand {
    pub driveID: usize,
    pub rotationTargetPosition: f64,
    pub rotationVelocity: f64,
    pub translationTargetPosition: f64,
    pub translationVelocity: f64
}

#[derive(Deserialize)]
#[allow(non_snake_case)]
pub struct MotorCommands {
    pub commands: Vec<MotorCommand>
}


/// Shared state between HTTP server and the simulation.
pub struct ServerState {
    /* Synced from simulation to server (state) */
    pub camera_state: CameraState,
    pub score: u16,
    pub pending_commands: Vec<MotorCommand>,

    /* Synced from server (state) to simulation or part of the server (state) */
    pub port: u16,
    pub team: PlayerTeam,
    pub team_name: String,
    pub builtin: bool
}

#[derive(Serialize, Clone)]
#[allow(non_snake_case)]
pub struct StateCameraStateRod {
    pub Item1: f64,
    pub Item2: f64,
}

#[derive(Serialize, Clone)]
pub struct StateCameraState   {
    pub rods: [StateCameraStateRod; 8],
    pub ball_x: f64,
    pub ball_y: f64,
    pub ball_vx: f64,
    pub ball_vy: f64,
    pub ball_size: f64,
}


#[derive(Serialize)]
#[allow(non_snake_case)]
pub struct StateDrivesStateDrivesValue {
    pub axisEncoderPosition: f64,
}

#[derive(Serialize)]
#[allow(non_snake_case)]
pub struct StateDrivesStateDrive {
    pub translation: StateDrivesStateDrivesValue,
    pub rotation: StateDrivesStateDrivesValue
}

#[derive(Serialize)]
#[allow(non_snake_case)]
pub struct StateDrivesState {
    pub drives: [StateDrivesStateDrive; 4]
}

#[derive(Serialize)]
#[allow(non_snake_case)]
pub struct State {
    pub motorsReady: bool,
    pub camData: [StateCameraState; 2],
    pub drivesStates: StateDrivesState
}

pub async fn http_task(states: [Arc<Mutex<ServerState>>; 2], shutdown_notify: Arc<Notify>) {
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
        .workers(NUM_WORKERS_PER_SERVER)
        .max_connections(10)
        .bind(("127.0.0.1", port)).unwrap()
        .run()
    };

    let mut join_set = tokio::task::JoinSet::new();

    let server_handles = states.map(|state| {
        let server = factory(state.clone());
        let handle = server.handle();
        join_set.spawn(server);
        handle
    });

    // Wait for the viewer to exit.
    shutdown_notify.notified().await;
    for handle in server_handles {
        handle.stop(false).await;
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
