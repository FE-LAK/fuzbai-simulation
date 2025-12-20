use std::time::Duration;

use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};


fn main() {
    let mut sim = FuzbAISimulator::new(
        5, 5,
        true,
        0.055,
        None,
        VisualConfig::new(
            10, true,
            0, true
        )
    );

    sim.set_external_mode(PlayerTeam::Red,false);
    sim.set_external_mode(PlayerTeam::Blue, true);

    let sim_thread = std::thread::spawn(move || {
        while sim.viewer_running() {
            if sim.terminated() || sim.truncated() {
                sim.reset_simulation();
            }
            sim.step_simulation();
        }
    });

    let viewer = ViewerProxy;
    viewer.render_loop();
    sim_thread.join().unwrap();
}
