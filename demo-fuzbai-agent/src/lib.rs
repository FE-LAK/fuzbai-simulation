mod timer;
mod rod;

use timer::ManualTimer;
use rod::*;

/// The main agent class.
pub struct Agent {
    rods: [Rod; 4],
    timeout_timer: ManualTimer
}

impl Agent {
    pub fn get_action(&mut self, ball_x: f64, ball_y: f64, mut ball_vx: f64, mut ball_vy: f64) -> [(usize, f64, f64, f64, f64); 4] {
        ball_vx *= 1000.0;
        ball_vy *= 1000.0;
        self.timeout_timer.step();

        // While ball is inside the field, reset timer, in order to make the rods
        // execute actions. Otherwise, the timer is running and eventually,
        // the rods move to the middle.
        if ball_y > 0.0 && ball_y < 700.0 {
            self.timeout_timer.reset();
        }

        if self.timeout_timer.elapsed() < 2.0 {
            std::array::from_fn(|i| self.rods[i].get_action(ball_x, ball_y, ball_vx, ball_vy))
        } else {
            std::array::from_fn(|i| (i + 1, 0.5, 0.0, 1.0, 1.0))
        }
    }
}

impl Agent {
    pub fn new(timestep: f64) -> Self {
        Self {
            rods: [
                // drive ID, num. players, offset, spacing, travel, x position
                Rod::new(timestep, 1, 1, 258.0, 0.0, 190.0, 80.0),
                Rod::new(timestep, 2, 2, 55.0, 238.0, 356.0, 230.0),
                Rod::new(timestep, 3, 5, 55.0, 118.0, 116.0, 530.0),
                Rod::new(timestep, 4, 3, 55.0, 208.0, 180.0, 830.0),
            ],
            timeout_timer: ManualTimer::new(timestep)
        }
    }
}
