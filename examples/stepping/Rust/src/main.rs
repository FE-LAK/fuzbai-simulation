use std::thread;
use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};

fn physics(mut sim: FuzbAISimulator) {
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
        10, // internal_step_factor
        5,  // sample_steps
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

    // Disable the built-in agent on both sides.
    sim.set_external_mode(PlayerTeam::Red, false);
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
