use serde::{Deserialize, Serialize};
use reqwest::blocking::Client;

use std::{thread, time::Duration};

const TEAM_URL: &str = "http://127.0.0.1:8080"; // Red team (8081 for blue)

#[derive(Deserialize, Debug)]
#[allow(non_snake_case)]
struct CameraData {
    pub cameraID: u32,
    pub ball_x: f64,
    pub ball_y: f64,
    pub rod_position_calib: [f64; 8],
    pub rod_angle: [f64; 8],
}

#[derive(Deserialize, Debug)]
#[allow(non_snake_case)]
struct CameraState {
    pub camData: [CameraData; 2],
    pub camDataOK: [bool; 2],
}

#[derive(Serialize, Debug)]
#[allow(non_snake_case)]
struct MotorCommand {
    pub driveID: usize,
    pub rotationTargetPosition: f64,
    pub rotationVelocity: f64,
    pub translationTargetPosition: f64,
    pub translationVelocity: f64,
}

#[derive(Serialize, Debug)]
struct MotorCommands {
    pub commands: Vec<MotorCommand>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = Client::new();
    println!("Connecting to {TEAM_URL}...");

    loop {
        match client.get(format!("{TEAM_URL}/Camera/State")).send() {
            // Successful HTTP request with 2xx status code
            Ok(response) if response.status().is_success() => {
                let state: CameraState = response.json()?;
                let ball_y = state.camData[0].ball_y;
                println!("ball_y={ball_y:.3}");

                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)?
                    .as_secs_f64();

                let actions = MotorCommands {
                    commands: vec![
                        MotorCommand {
                            driveID: 1, // Goalkeeper
                            translationTargetPosition: (f64::sin(2.0 * now) + 1.0) / 2.0,
                            translationVelocity: 1.0,
                            rotationTargetPosition: 0.05 * f64::sin(10.0 * now),
                            rotationVelocity: 1.0,
                        },
                        MotorCommand {
                            driveID: 4, // Attacker
                            translationTargetPosition: (f64::sin(2.0 * now * 2.0) + 1.0) / 2.0,
                            translationVelocity: 1.0,
                            rotationTargetPosition: 0.05 * f64::sin(20.0 * now),
                            rotationVelocity: 1.0,
                        },
                    ],
                };

                let _ = client
                    .post(format!("{TEAM_URL}/Motors/SendCommand"))
                    .json(&actions)
                    .send();

                thread::sleep(Duration::from_millis(16)); // ~60Hz
            }
            // HTTP request succeeded but server returned error status (4xx, 5xx)
            Ok(response) => {
                println!("Server error: {}", response.status());
            }
            // Network error: connection failed, timeout, etc.
            Err(e) => {
                println!("Error: {e}");
                thread::sleep(Duration::from_secs(1));
            }
        }
    }
}
