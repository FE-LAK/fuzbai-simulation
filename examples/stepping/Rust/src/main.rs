//!
//! Before running run this example, some dependencies must be set.
//! 
//! First, move to the root of the repository (not the example).
//! Then fetch the MuJoCo dependency:
//! - Linux: ./fetch_mujoco_linux.sh
//! - Windows: .\fetch_mujoco_windows.ps1
//! - MacOS: ./fetch_mujoco_macos.sh
//! 
//! Then export variables (needs to be re-run every time the terminal is closed):
//! - Linux: source setup_linux.sh
//! - Windows: .\setup_windows.ps1
//! - MacOS: source setup_macos.sh
//! 
//! You should now be able to run the example.
//! 
use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};
use std::time::Instant;
use std::thread;

fn physics(mut sim: FuzbAISimulator) {
    let start_time = Instant::now();
    while sim.viewer_running() {
        // Terminated: goal scored or ball outside the field.
        // Truncated: ball stopped moving.
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation(); // Reset physics state.
        }

        // Query delayed observation (actual measurement).
        // (ball_x, ball_y, ball_vx, ball_vy, rod_positions, rod_rotations)
        let obs = sim.delayed_observation(
            PlayerTeam::Red, // Team
            None             // Delay override. When None is passed,
                             // delay will be equal to the value of simulated_delay_s (passed in constructor).
        );

        // Set motor commands
        let current_time_s = start_time.elapsed().as_secs_f64();
        sim.set_motor_command(
            &std::array::from_fn::<_, 4, _>(|i|  // Construct 4 element array, based on indices
                (
                    i + 1,  // Player index (1 => goalkeeper)
                    0.5 * (f64::sin(current_time_s) + 1.0),  // Target translation (0-1)
                    0.5 * f64::sin(2.0 * current_time_s),    // Target rotation (-1, 1)
                    1.0,  // Full translational velocity
                    1.0   // Full rotational velocity
                )
            ),
            PlayerTeam::Red
        );

        // Move time forward in simulation.
        sim.step_simulation();

        // Draw some measurements or estimates (e.g., visualize your corrected values) inside the viewer.
        sim.show_estimates(
            // Show the measured ball position
            Some([obs.0, obs.1, 0.0]),
            // Show the measured rod position on the viewer
            // (rod_id, extension [0-1], rotation [-64, 64], mask)
            //       The mask is an 8 bit mask, where each bit corresponds to a figure on the rod.
            //       Enable individual bits to enable display for individual figure.
            Some(&[(0, obs.4[0], obs.5[0], 0xFF)])
        );
    }
}

fn main() {
    // Create a simulation instance.
    // Multiple of these can exist, just not with the viewer enabled.
    let mut sim = FuzbAISimulator::new(
        10, // internal_step_factor: .step_simulation() = N * (2 ms)
        5,  // sample_steps: save state to delay buffer every N * (2 ms). .delayed_observation() returns discrete samples every N * 2ms.
        true, // realtime
        0.055, // simulated_delay_s
        None, // model_path
        VisualConfig::new(
            0,     // trace_length
            false, // trace_ball
            0x0,   // trace_rod_mask
            true   // enable_viewer
        )
    );

    // Disable the built-in agent of the red side.
    sim.set_external_mode(PlayerTeam::Red, true);

    // Enable the built-in agent of the blue side.
    sim.set_external_mode(PlayerTeam::Blue, false);

    // Start the physics simulation.
    let physics_thread = thread::spawn(move || {
        physics(sim);
    });

    // Control the viewer in the main thread.
    let viewer = ViewerProxy;
    viewer.render_loop();
    physics_thread.join().unwrap();
}
