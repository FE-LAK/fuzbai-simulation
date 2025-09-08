use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, VisualConfig};
use std::time::Instant;



fn main() {
    let mut sim = FuzbAISimulator::new(
        10, 0, true, 0.0,
        None,
        VisualConfig::new(
            10, true,
            0xFF,
            5, None
        )
    );

    sim.set_external_mode(PlayerTeam::RED,false);
    sim.set_external_mode(PlayerTeam::BLUE, true);
    let mut t0 = Instant::now();
    let mut n = 0usize;

    sim.set_motor_command(vec![(2, 0.5, 0.0, 1.0, 1.0)], PlayerTeam::RED);
    while sim.viewer_running() {
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation();
        }
        sim.step_simulation();

        let mut xyz: [f64; 3] = sim.ball_true_state()[..3].try_into().unwrap();
        xyz[2] = 0.0;

        if sim.viewer_running() {
            sim.show_estimates(Some(xyz), None);
        }

        n += 1;
        if t0.elapsed().as_micros() > 1_000_000 {
            println!("{n}");
            t0 = Instant::now();
            n = 0;
        }
    }
}
