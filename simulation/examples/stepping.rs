use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, ViewerProxy, VisualConfig};


fn main() {
    let mut sim = FuzbAISimulator::new(
        10, 0,
        true,
        0.055,
        None,
        VisualConfig::new(
            10, true,
            0,
        )
    );

    sim.set_external_mode(PlayerTeam::RED,false);
    sim.set_external_mode(PlayerTeam::BLUE, true);

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
