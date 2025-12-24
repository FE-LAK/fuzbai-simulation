//! Simple camera state reading example.
use std::hint::black_box;
use serde::{Serialize, Deserialize};
use std::time::{Instant, Duration};


/// Data received from a single camera.
#[derive(Deserialize, Serialize, Default, Clone)]
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
#[derive(Deserialize, Serialize, Clone)]
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


fn main() {
    requests_thread();
}


fn requests_thread() {
    let client = reqwest::blocking::Client::builder()
        .pool_idle_timeout(Duration::from_secs(90))
        .timeout(Duration::from_secs(5))
        .build().unwrap();

    let mut smoothed = 0.0;
    loop {
        let start = Instant::now();
        let state: CameraState = 
        client.get("http://127.0.0.1:8080/Camera/State").send().unwrap().json().unwrap();
        black_box(&state);
        let elapsed_micros = start.elapsed().as_micros();
        smoothed += 0.0001 * (elapsed_micros as f64 - smoothed);
        let CameraState { camData: [CameraData { ball_x, ball_y, ..}, _], .. } = state;
        println!("x={ball_x} y={ball_y} perf={smoothed:.3}");
    }
}
