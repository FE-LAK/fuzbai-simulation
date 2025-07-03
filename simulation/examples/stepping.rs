use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, SimVisualConfig};
use std::time::Instant;

fn main() {
    let mut sim = FuzbAISimulator::new(
        10, 0, true, 0.0,
        None,
        SimVisualConfig::new(50, true, Some(vec![2]), 5, Some((1280, 720)))
    );

    sim.set_external_mode(PlayerTeam::RED,true);
    sim.set_external_mode(PlayerTeam::BLUE, false);
    let mut t0 = Instant::now();
    let mut n = 0usize;
    let mut k = 0usize;

    sim.set_motor_command(vec![(2, 0.5, 0.0, 1.0, 1.0)], PlayerTeam::RED);
    while sim.viewer_running() {
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation();
        }

        if k % 5 == 0 {
            sim.screenshot(None, None, Some(1), None, Some(format!("./tmp/{}.png", k / 5 % 10)),
                true, Some(vec![2])
            );
        }
        k += 1;

        sim.step_simulation();
        // sim.show_estimates(None, None);
        n += 1;
        if t0.elapsed().as_micros() > 1_000_000 {
            println!("{n}");
            t0 = Instant::now();
            n = 0;
        }
    }
}
