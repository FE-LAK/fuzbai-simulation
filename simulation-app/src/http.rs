

use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use serde::{Serialize, Deserialize};
use utoipa::{OpenApi, ToSchema};
use actix_files::{Files};
use actix_web::web;

use fuzbai_simulator::{PlayerTeam, mujoco_rs::util::LockUnpoison};
use std::sync::{Arc, Mutex};
use tokio::sync::Notify;



/// How many (CPU) workers to create on each server (red and blue).
/// A single worker will contain a new single-threaded tokio runtime in
/// which tasks will be spawned.
const NUM_WORKERS_PER_SERVER: usize = 3;


/// Data received from a single camera.
#[derive(Serialize, Default, Clone, ToSchema)]
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
#[derive(Serialize, Clone, ToSchema)]
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


#[derive(Deserialize, Clone, ToSchema)]
#[allow(non_snake_case)]
pub struct MotorCommand {
    pub driveID: usize,
    pub rotationTargetPosition: f64,
    pub rotationVelocity: f64,
    pub translationTargetPosition: f64,
    pub translationVelocity: f64
}

#[derive(Deserialize, ToSchema)]
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

#[derive(Serialize, Clone, ToSchema)]
#[allow(non_snake_case)]
pub struct StateCameraStateRod {
    pub Item1: f64,
    pub Item2: f64,
}

#[derive(Serialize, Clone, ToSchema)]
pub struct StateCameraState   {
    pub rods: [StateCameraStateRod; 8],
    pub ball_x: f64,
    pub ball_y: f64,
    pub ball_vx: f64,
    pub ball_vy: f64,
    pub ball_size: f64,
}


#[derive(Serialize, ToSchema)]
#[allow(non_snake_case)]
pub struct StateDrivesStateDrivesValue {
    pub axisEncoderPosition: f64,
}

#[derive(Serialize, ToSchema)]
#[allow(non_snake_case)]
pub struct StateDrivesStateDrive {
    pub translation: StateDrivesStateDrivesValue,
    pub rotation: StateDrivesStateDrivesValue
}

#[derive(Serialize, ToSchema)]
#[allow(non_snake_case)]
pub struct StateDrivesState {
    pub drives: [StateDrivesStateDrive; 4]
}

#[derive(Serialize, ToSchema)]
#[allow(non_snake_case)]
pub struct State {
    pub motorsReady: bool,
    pub camData: [StateCameraState; 2],
    pub drivesStates: StateDrivesState
}

#[derive(Serialize, ToSchema)]
#[allow(non_snake_case)]
pub struct CompetitionResponse {
    pub state: u8,
    pub time: u32,
    pub playerName: String,
    pub scorePlayer: String,
    pub scoreFuzbAI: String,
    pub level: u8,
    pub results: Vec<String>,
}

#[derive(Serialize, ToSchema)]
pub struct ResetResponse {
    pub message: String,
}

#[derive(OpenApi)]
#[openapi(
    paths(
        get_camera_state,
        send_command,
        reset_rotations,
        get_state,
    ),
    components(
        schemas(
            CameraState,
            MotorCommands,
            State,
            ResetResponse,
        )
    ),
    tags(
        (name = "FuzbAI (partial) API", description = "Some of the routes available on the FuzbAI system.")
    )
)]
pub struct ApiDoc;

pub async fn http_task(states: [Arc<Mutex<ServerState>>; 2], shutdown_notify: Arc<Notify>) {
    let factory = |state: Arc<Mutex<ServerState>>| {
            let port = state.lock_unpoison().port;
            HttpServer::new(move || {
            let json_config = web::JsonConfig::default().limit(5000);
            App::new()
                .app_data(json_config)
                .app_data(web::Data::new(state.clone()))
                .service(get_openapi)
                .service(get_docs)
                .service(get_camera_state)
                .service(send_command)
                .service(get_state)
                .service(reset_rotations)
                .service(get_competition)
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

#[get("/api-doc/openapi.json")]
async fn get_openapi() -> impl Responder {
    HttpResponse::Ok().json(ApiDoc::openapi())
}

#[get("/docs")]
async fn get_docs() -> impl Responder {
    let html = r#"<!DOCTYPE html>
<html lang="en">
    <head>
        <meta charset="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <meta name="description" content="SwaggerUI" />
        <title>FuzbAI Simulation API Docs</title>
        <link rel="stylesheet" href="https://unpkg.com/swagger-ui-dist@5.11.0/swagger-ui.css" />
    </head>
    <body>
        <div id="swagger-ui"></div>
        <script src="https://unpkg.com/swagger-ui-dist@5.11.0/swagger-ui-bundle.js" crossorigin></script>
        <script>
            window.onload = () => {
                if (!window.SwaggerUIBundle) {
                    document.body.insertAdjacentHTML(
                        'beforeend',
                        '<p>Failed to load Swagger UI. Check network access to unpkg.com.</p>'
                    );
                    return;
                }
                window.ui = SwaggerUIBundle({
                    url: '/api-doc/openapi.json',
                    dom_id: '#swagger-ui',
                });
            };
        </script>
    </body>
</html>
"#;

    HttpResponse::Ok()
            .content_type("text/html; charset=utf-8")
            .body(html)
}

#[utoipa::path(
    get,
    path = "/Camera/State",
    tag = "State information",
    responses(
        (status = 200, description = "Camera state", body = CameraState)
    )
)]
#[get("/Camera/State")]
async fn get_camera_state(data: web::Data<Arc<Mutex<ServerState>>>) -> impl Responder {
    HttpResponse::Ok().json(&data.lock_unpoison().camera_state)
}

#[utoipa::path(
    post,
    path = "/Motors/SendCommand",
    request_body = MotorCommands,
    tag = "Control",
    responses(
        (status = 200, description = "Accepted")
    )
)]
#[post("/Motors/SendCommand")]
async fn send_command(data: web::Data<Arc<Mutex<ServerState>>>, commands: web::Json<MotorCommands>) -> impl Responder {
    data.lock_unpoison().pending_commands = commands.into_inner().commands;
    HttpResponse::Ok()
}

#[utoipa::path(
    get,
    path = "/Motors/ResetRotations",
    tag = "Control",
    responses(
        (status = 200, description = "Reset message", body = ResetResponse)
    )
)]
#[get("/Motors/ResetRotations")]
async fn reset_rotations() -> impl Responder {
    HttpResponse::Ok().json(ResetResponse { message: "Resetting rotations.".to_string() })
}

#[utoipa::path(
    get,
    path = "/State",
    tag = "State information",
    responses(
        (status = 200, description = "Simulation state", body = State)
    )
)]
#[get("/State")]
async fn get_state(data: web::Data<Arc<Mutex<ServerState>>>) -> impl Responder {
    let camera_state = data.lock_unpoison().camera_state.clone();
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

#[get("/Competition")]
async fn get_competition(data: web::Data<Arc<Mutex<ServerState>>>) -> impl Responder {
    let name = &data.lock_unpoison().team_name;
    let response = CompetitionResponse {
        state: 2,
        time: 0,
        playerName: name.clone(),
        scorePlayer: "".to_string(),
        scoreFuzbAI: "".to_string(),
        level: 0,
        results: Vec::new(),
    };

    HttpResponse::Ok().json(response)
}
