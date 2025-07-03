use fuzbai_simulator::{FuzbAISimulator, PlayerTeam, VisualConfig};
use std::time::Instant;

fn main() {
    let mut sim = FuzbAISimulator::new(
        10, 0, true, 0.0,
        None,
        VisualConfig::new(
            30, true,
            VisualConfig::player_mask(5, vec![2, 5]),
            5, Some((1280, 720))
        )
    );

    sim.set_external_mode(PlayerTeam::RED,false);
    sim.set_external_mode(PlayerTeam::BLUE, true);
    let mut t0 = Instant::now();
    let mut n = 0usize;
    let mut k = 0usize;

    sim.set_motor_command(vec![(2, 0.5, 0.0, 1.0, 1.0)], PlayerTeam::RED);
    while sim.viewer_running() {
        if sim.terminated() || sim.truncated() {
            sim.reset_simulation();
        }

        sim.screenshot(None, None, None, Some("cam_blue_front".into()),
            Some(format!("./tmp/{}.png", k % 30))
        );
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
