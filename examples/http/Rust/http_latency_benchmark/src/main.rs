//! HTTP latency benchmark for camera state requests.
//! Measures round-trip time for GET requests to the `/Camera/State` endpoint
//! and tracks smoothed latency metrics over time.
use serde::{Serialize, Deserialize};
use std::time::{Instant, Duration};

/// Data received from a single camera.
#[derive(Deserialize, Default, Clone)]
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

/// Camera state response from the HTTP API.
/// Contains state for both cameras.
#[derive(Deserialize, Clone)]
#[allow(non_snake_case)]
pub struct CameraState {
    pub camData: [CameraData; 2],
    pub camDataOK: [bool; 2],
}

fn main() {
    requests_thread();
}

fn requests_thread() {
    let client = reqwest::blocking::Client::builder().build().unwrap();
    let mut smoothed = 0.0; // Exponential moving average of latency in microseconds
    loop {
        // Measure round-trip time for camera state request
        let start = Instant::now();
        let state: CameraState = client.get("http://127.0.0.1:8080/Camera/State")
            .send().unwrap()
            .json().unwrap();
        let elapsed_micros = start.elapsed().as_micros();

        // Update smoothed latency with exponential moving average (k=0.0001)
        smoothed += 0.0001 * (elapsed_micros as f64 - smoothed);
        let CameraState { camData: [CameraData { ball_x, ball_y, ..}, _], .. } = state;
        println!("x={ball_x:.3} y={ball_y:.3} perf={smoothed:.3}");
    }
}
