use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, SimVisualConfig};
use std::time::Instant;

fn main() {
    let mut sim = FuzbAISimulator::new(
        10, 0, false, 0.0,
        Some("/home/davidhozic/repo/FuzbAI/environment/models/miza.xml"),
        SimVisualConfig {trace_length: 100, refresh_steps: 5, screenshot_size: None},
    );

    sim.set_external_mode(PlayerTeam::RED,true);
    sim.set_external_mode(PlayerTeam::BLUE, false);
    let mut t0 = Instant::now();
    let mut n = 0usize;

    sim.set_motor_command(vec![(2, 0.5, 0.0, 1.0, 1.0)], PlayerTeam::RED);
    loop {
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation();
        }

        sim.step_simulation();
        n += 1;
        if t0.elapsed().as_micros() > 1_000_000 {
            println!("{n}");
            t0 = Instant::now();
            n = 0;
        }
    }
}
