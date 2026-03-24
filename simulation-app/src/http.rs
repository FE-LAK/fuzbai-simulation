use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use actix_web::middleware::{NormalizePath, TrailingSlash};
use actix_files::Files;
use actix_web::web;

use serde::{Serialize, Deserialize};
use utoipa::{OpenApi, ToSchema};

use fuzbai_simulator::mujoco_rs::util::LockUnpoison;
use fuzbai_simulator::PlayerTeam;

use std::sync::{Arc, Mutex};
use std::time::Instant;

use tokio::sync::Notify;

use crate::COMPETITION_STATE;
use crate::CompetitionStatus;


/// How many (CPU) workers to create on each server (red and blue).
/// A single worker will contain a new single-threaded tokio runtime in
/// which tasks will be spawned.
const NUM_WORKERS_PER_SERVER: usize = 3;
/// Maximum concurrent connections per team server.
const MAX_CONNECTIONS_PER_SERVER: usize = 10;

/// The default port for the (originally) red team.
pub(crate) const DEFAULT_TEAM_1_PORT: u16 = 8080;
/// The default port for the (originally) blue team.
pub(crate) const DEFAULT_TEAM_2_PORT: u16 = 8081;
/// The default host for team servers.
pub(crate) const DEFAULT_TEAM_HOST: &str = "0.0.0.0";

/// How many (CPU) workers to create on the management server.
const NUM_WORKERS_PER_MANAGEMENT: usize = 1;
/// Maximum concurrent connections on the management server.
const MAX_CONNECTIONS_MANAGEMENT: usize = 5;
/// The default port used for management of the competition.
pub(crate) const DEFAULT_MANAGEMENT_PORT: u16 = 8000;
/// The default host for the management server.
pub(crate) const DEFAULT_MANAGEMENT_HOST: &str = "127.0.0.1";

/// The smoothing factor for the action execution frequency (first order filter).
const CONNECTION_FREQUENCY_SMOOTHING: f32 = 0.1;

/// Maximum size of JSON payloads.
const MAX_JSON_PAYLOAD_BYTES: usize = 5000;


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
    #[schema(minimum = 1)]
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


/// Shared state between HTTP server and the simulation (per team).
pub struct TeamState {
    /* Synced from simulation to server (state) */
    pub camera_state: CameraState,
    pub score: u16,
    pub pending_commands: Vec<MotorCommand>,

    /* Synced from server (state) to simulation or part of the server (state) */
    pub port: u16,
    pub team: PlayerTeam,
    pub team_name: String,
    pub builtin: bool,

    /* Connection benchmarking */
    pub last_command_time: Instant,
    pub frequency_smooth_hz: f32
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

pub async fn http_task(
    team_states: [Arc<Mutex<TeamState>>; 2],
    management_port: u16,
    team_host: &str,
    management_host: &str,
    shutdown_notify: Arc<Notify>,
) {
    // Try to obtain the www/ path next to the executable.
    // When the www/ does not exist next to the executable,
    // search the current working directory.
    let www_dir = std::env::current_exe()
        .ok()
        .and_then(|path| {
            path.parent()
                .map(|p| p.join("www"))
                .filter(|p| p.exists())
        })
        .unwrap_or_else(|| std::path::PathBuf::from("./www/"));


    let mut join_set = tokio::task::JoinSet::new();

    /* Management HTTP server creation */
    let management_data = web::Data::new(team_states.clone());
    let management_server = HttpServer::new(move || {
        App::new()
            .wrap(NormalizePath::new(TrailingSlash::Trim))
            .app_data(web::JsonConfig::default().limit(MAX_JSON_PAYLOAD_BYTES))
            .app_data(management_data.clone())
            .service(start_game)
            .service(pause_game)
            .service(reset_game)
            .service(competition_status)
    })
    .workers(NUM_WORKERS_PER_MANAGEMENT)
    .max_connections(MAX_CONNECTIONS_MANAGEMENT)
    .bind((management_host, management_port)).unwrap()
    .run();

    let management_handle = management_server.handle();
    join_set.spawn(management_server);

    /* Team HTTP server creation */
    let [team_handle_0, team_handle_1] = team_states.map(|team_state| {
        let port = team_state.lock_unpoison().port;
        let www_dir = www_dir.clone();
        let team_data = web::Data::from(team_state);
        let server = HttpServer::new(move || {
            App::new()
                .wrap(NormalizePath::new(TrailingSlash::Trim))
                .app_data(web::JsonConfig::default().limit(MAX_JSON_PAYLOAD_BYTES))
                .app_data(team_data.clone())
                .service(get_openapi)
                .service(get_docs)
                .service(get_camera_state)
                .service(send_command)
                .service(get_state)
                .service(reset_rotations)
                .service(get_competition)
                .service(
                    Files::new("/", &www_dir)
                    .index_file("Render.html")
                )
        })
        .workers(NUM_WORKERS_PER_SERVER)
        .max_connections(MAX_CONNECTIONS_PER_SERVER)
        .bind((team_host, port)).unwrap()
        .run();

        let handle = server.handle();
        join_set.spawn(server);
        handle
    });

    let server_handles = [management_handle, team_handle_0, team_handle_1];

    /* Shutdown logic */
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
  <title>SwaggerUI</title>
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
async fn get_camera_state(data: web::Data<Mutex<TeamState>>) -> impl Responder {
    let camera_state = {
        let mut lock = data.lock_unpoison();
        update_team_frequency(&mut *lock);
        lock.camera_state.clone()
    };
    HttpResponse::Ok().json(&camera_state)
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
async fn send_command(data: web::Data<Mutex<TeamState>>, commands: web::Json<MotorCommands>) -> impl Responder {
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
async fn get_state(data: web::Data<Mutex<TeamState>>) -> impl Responder {
    let camera_state = {
        let mut lock = data.lock_unpoison();
        update_team_frequency(&mut *lock);
        lock.camera_state.clone()
    };

    let cam = &camera_state.camData[0];
    let rp = cam.rod_position_calib;
    let rr = cam.rod_angle;

    let state_cam_data = StateCameraState {
        rods: std::array::from_fn(|i| StateCameraStateRod { Item1: rp[i], Item2: rr[i] }),
        ball_x: cam.ball_x,
        ball_y: cam.ball_y,
        ball_vx: cam.ball_vx,
        ball_vy: cam.ball_vy,
        ball_size: cam.ball_size,
    };

    // Rod indices matching the 4 physical drives (goalkeeper, defense, midfield, attack)
    const DRIVE_ROD_INDICES: [usize; 4] = [0, 1, 3, 5];

    HttpResponse::Ok().json(State {
        motorsReady: true,
        camData: [state_cam_data.clone(), state_cam_data],
        drivesStates: StateDrivesState {
            drives: DRIVE_ROD_INDICES.map(|i| StateDrivesStateDrive {
                translation: StateDrivesStateDrivesValue { axisEncoderPosition: rp[i] },
                rotation: StateDrivesStateDrivesValue { axisEncoderPosition: rr[i] },
            }),
        },
    })
}

#[get("/Competition")]
async fn get_competition(data: web::Data<Mutex<TeamState>>) -> impl Responder {
    let name = data.lock_unpoison().team_name.clone();
    HttpResponse::Ok().json(CompetitionResponse {
        state: 2,
        time: 0,
        playerName: name,
        scorePlayer: String::new(),
        scoreFuzbAI: String::new(),
        level: 0,
        results: Vec::new(),
    })
}

/// Updates the frequency and the last command time for the given team.
fn update_team_frequency(lock: &mut TeamState) {
    lock.frequency_smooth_hz += CONNECTION_FREQUENCY_SMOOTHING * (
        1.0 / lock.last_command_time.elapsed().as_secs_f32() - lock.frequency_smooth_hz
    );
    lock.last_command_time = Instant::now();
}

/********************************************************************************************/
/*                                        MANAGEMENT                                        */
/********************************************************************************************/
// Handlers here are used only for controlling the simulation via a management dashboard.

#[post("/start")]
async fn start_game() -> impl Responder {
    HttpResponse::Ok().json(serde_json::json!({"status": "ok"}))
}

#[post("/pause")]
async fn pause_game() -> impl Responder {
    HttpResponse::Ok().json(serde_json::json!({"status": "ok"}))
}

#[post("/reset")]
async fn reset_game() -> impl Responder {
    HttpResponse::Ok().json(serde_json::json!({"status": "ok"}))
}


/// Returns the status of the game and the elapsed time (if the game is running).
#[get("/status")]
async fn competition_status(team_states: web::Data<[Arc<Mutex<TeamState>>; 2]>) -> impl Responder {

    #[derive(Serialize)]
    struct ResponseCompetitionStatus {
        status: &'static str,
        score: [u16; 2],
        gametime: f32
    }

    // Process global state
    let (status, gametime) = {
        let lock = COMPETITION_STATE.lock_unpoison();
        match &lock.status {
            CompetitionStatus::Running(timer) => {  // Match is running
                ("running", timer.elapsed().as_secs_f32())
            },
            CompetitionStatus::Free => {  // Match is not running, but control is enabled
                ("free", 0.0)
            },
            CompetitionStatus::Expired => {  // Match expired / stopped
                ("waiting", 0.0)
            }
        }
    };

    // Get scores for both teams. Due to the swapping feature, the index 0 may not be red.
    let (team_1_score, team1_color) = {
        let lock = team_states[0].lock_unpoison();
        (lock.score, lock.team)
    };

    let team_2_score = team_states[1].lock_unpoison().score;

    let score_red_blue = if team1_color == PlayerTeam::Red {
        [team_1_score, team_2_score]
    } else { [team_2_score, team_1_score] };

    HttpResponse::Ok().json(
        ResponseCompetitionStatus {
            status, gametime, score: score_red_blue
        }
    )
}
