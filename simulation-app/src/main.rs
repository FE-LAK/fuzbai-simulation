use std::{cell::LazyCell, sync::{Arc, LazyLock, Mutex}};

use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use actix_files::{Files};
use actix_web::web;
use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
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
struct TeamState {
    camera_state: Mutex<CameraState>,
    team: Mutex<PlayerTeam>
}


fn main() {
    /* Initialize states for each team */
    let states = [
        Arc::new(TeamState {camera_state: Mutex::new(CameraState::new()), team: Mutex::new(PlayerTeam::RED)}),
        Arc::new(TeamState {camera_state: Mutex::new(CameraState::new()), team: Mutex::new(PlayerTeam::BLUE)})
    ];

    /* Initialize tokio runtime and with it, the HTTP server */
    let states_clone = states.clone();
    let tokio_handle = std::thread::spawn(move || {
        let runtime = Builder::new_current_thread()
            .worker_threads(NUM_TOKIO_THREADS)
            .enable_all()
            .build()
            .unwrap();

        runtime.block_on(http_task(&states_clone));
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
    let sim_thread = std::thread::spawn(move || {
        simulation_thread(sim, &states);
    });

    /* Loop forever within Rust in the main thread and also without GIL */
    let viewer = ViewerProxy;
    viewer.render_loop();

    /* Final cleanup */
    let _ = sim_thread.join();
    let _ = tokio_handle.join();
}


fn simulation_thread(mut sim: FuzbAISimulator, states: &[Arc<TeamState>]) {
    sim.set_external_mode(PlayerTeam::RED,false);
    sim.set_external_mode(PlayerTeam::BLUE, true);
    while sim.viewer_running() {
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation();
        }
        sim.step_simulation();

        for state in states {
            let team = state.team.lock().unwrap().deref().clone();
            let observation = sim.delayed_observation(team, None);
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
            state.camera_state.lock().unwrap().camData = [camera_data_0, camera_data_1];
        }
    }
}


async fn http_task(states: &[Arc<TeamState>]) {
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
    join_set.spawn(factory(8080, states[0].clone()));
    join_set.spawn(factory(8081, states[1].clone()));

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
