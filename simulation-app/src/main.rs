use std::sync::{Arc, Mutex};

use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use actix_files::{Files};
use actix_web::web;
use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig, types::ObservationType};
use tokio::runtime::Builder;
use serde::Serialize;
use std::ops::Deref;


const NUM_TOKIO_THREADS: usize = 4;


/// Data received from a single camera.
#[derive(Serialize, Default)]
#[allow(non_snake_case)]
struct CameraData {
    cameraID: u32,
    ball_x: f64,
    ball_y: f64,
    ball_vx: f64,
    ball_vy: f64,
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
struct TeamState {
    camera_state: Mutex<CameraState>,
    team: PlayerTeam
}


fn main() {
    /* Initialize states for each team */
    let red_state = Arc::new(TeamState {camera_state: Mutex::new(CameraState::new()), team: PlayerTeam::RED});
    let blue_state = Arc::new(TeamState {camera_state: Mutex::new(CameraState::new()), team: PlayerTeam::BLUE});

    let red_state_cloned = red_state.clone();
    let blue_state_cloned = blue_state.clone();

    /* Initialize tokio runtime and with it, the HTTP server */
    let tokio_handle = std::thread::spawn(move || {
        let runtime = Builder::new_current_thread()
            .worker_threads(NUM_TOKIO_THREADS)
            .enable_all()
            .build()
            .unwrap();

        runtime.block_on(http_task(red_state_cloned, blue_state_cloned));
    });

    /* Initialize simulation */
    let sim = FuzbAISimulator::new(
        5, 5,
        true,
        0.055,
        None,
        VisualConfig::new(
            10, true,
            0, true
        )
    );

    /* Start physics in another thread */
    let red_state_cloned = red_state.clone();
    let blue_state_cloned = blue_state.clone();
    let sim_thread = std::thread::spawn(move || {
        simulation_thread(sim, red_state_cloned, blue_state_cloned);
    });

    /* Loop forever within Rust in the main thread and also without GIL */
    let viewer = ViewerProxy;
    viewer.render_loop();

    /* Final cleanup */
    let _ = sim_thread.join();
    let _ = tokio_handle.join();
}


fn simulation_thread(mut sim: FuzbAISimulator, red_state: Arc<TeamState>, blue_state: Arc<TeamState>) {
    sim.set_external_mode(PlayerTeam::RED,false);
    sim.set_external_mode(PlayerTeam::BLUE, true);
    while sim.viewer_running() {
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation();
        }
        sim.step_simulation();


        fn observation_to_camera_data(observation: ObservationType) -> CameraData {
            let (x, y, vx, vy, rp, rr) = observation;
            CameraData {cameraID: 0, ball_x: x, ball_y: y, ball_vx: vx, ball_vy: vy, rod_position_calib: rp, rod_angle: rr}
        }

        /* Copy red state */
        let camera_data_0 = observation_to_camera_data(sim.delayed_observation(PlayerTeam::RED, None));
        let camera_data_1 = CameraData { cameraID: 1, ..camera_data_0 };
        red_state.camera_state.lock().unwrap().camData = [camera_data_0, camera_data_1];

        /* Copy blue state */
        let camera_data_0 = observation_to_camera_data(sim.delayed_observation(PlayerTeam::BLUE, None));
        let camera_data_1 = CameraData { cameraID: 1, ..camera_data_0 };
        blue_state.camera_state.lock().unwrap().camData = [camera_data_0, camera_data_1];
    }
}


async fn http_task(red_state: Arc<TeamState>, blue_state: Arc<TeamState>) {
    let factory = |port, state: Arc<TeamState>| {
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
    join_set.spawn(factory(8080, red_state.clone()));
    join_set.spawn(factory(8081, blue_state.clone()));

    while let Some(_) = join_set.join_next().await {

    }
}


#[get("/Camera/State")]
async fn camera_state(data: web::Data<Arc<TeamState>>) -> impl Responder {
    HttpResponse::Ok().json(data.camera_state.lock().unwrap().deref())
}

#[get("/Competition")]
async fn competition() -> impl Responder {
    let json: serde_json::Value = serde_json::json! {
        {
            "state": 2, "time": 0, "playerName": "SimBlue",
            "scorePlayer": 0, "scoreFuzbAI": 0, "level": 0, "results": []
        }
    };

    HttpResponse::Ok().json(json)
}

#[post("/Motors/SendCommand")]
async fn send_command() -> impl Responder {
    HttpResponse::Ok()
}
