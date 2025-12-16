use actix_web::{App, HttpResponse, HttpServer, Responder, get, post};
use actix_files::{Files};
use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
use tokio::runtime::Builder;
use serde::Serialize;


const NUM_TOKIO_THREADS: usize = 4;


/// Data received from a single camera.
#[derive(Serialize)]
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

/// Return type for [`camera_state`].
/// Contains state for both cameras.
#[derive(Serialize)]
#[allow(non_snake_case)]
struct CameraState {
    camData: [CameraData; 2],
    camDataOK: [bool; 2],
}


fn main() {
    /* Initialize tokio runtime and with it, the HTTP server */
    let tokio_handle = std::thread::spawn(|| {
        let runtime = Builder::new_current_thread()
            .worker_threads(NUM_TOKIO_THREADS)
            .enable_all()
            .build()
            .unwrap();

        runtime.block_on(http_task()).unwrap();
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
        simulation_thread(sim);
    });

    /* Loop forever within Rust in the main thread and also without GIL */
    let viewer = ViewerProxy;
    viewer.render_loop();

    /* Final cleanup */
    let _ = sim_thread.join();
    let _ = tokio_handle.join();
}


fn simulation_thread(mut sim: FuzbAISimulator) {
    sim.set_external_mode(PlayerTeam::RED,false);
    sim.set_external_mode(PlayerTeam::BLUE, true);
    while sim.viewer_running() {
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation();
        }
        sim.step_simulation();
    }
}


async fn http_task() -> std::io::Result<()> {
    HttpServer::new(|| {
        App::new()
            .service(camera_state)
            .service(competition)
            .service(
                Files::new("/", "./www/")
                .index_file("Render.html")
            )
    })
    .bind(("127.0.0.1", 8080))?
    .run()
    .await
}


#[get("/Camera/State")]
async fn camera_state() -> impl Responder {
    HttpResponse::Ok().json(CameraState {
        camData: [
            CameraData {
                cameraID: 0, ball_x: 0.0, ball_y: 0.0, ball_vx: 0.0, ball_vy: 0.0,
                rod_position_calib: [0.0; 8], rod_angle: [0.0; 8]
            },
            CameraData {
                cameraID: 1, ball_x: 0.0, ball_y: 0.0, ball_vx: 0.0, ball_vy: 0.0,
                rod_position_calib: [0.0; 8], rod_angle:[0.0; 8]
            }
        ],
        camDataOK: [true, true]
    })
}

#[get("/Competition")]
async fn competition() -> impl Responder {
    let json = r"{state: 2, time: 0, playerName: 'SimBlue', scorePlayer: 0, scoreFuzbAI: 0, level: 0, results: [] }";
    HttpResponse::Ok().json(json)
}
