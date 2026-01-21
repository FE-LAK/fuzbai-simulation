mod timer;
mod rod;

pub use rod::*;

/// The main agent class.
pub struct Agent {
    rods: [Rod; 4]
}

impl Agent {
    pub fn get_action(&mut self, ball_x: f64, ball_y: f64, mut ball_vx: f64, mut ball_vy: f64) -> [(usize, f64, f64, f64, f64); 4] {
        ball_vx *= 1000.0;
        ball_vy *= 1000.0;
        std::array::from_fn(|i| self.rods[i].get_action(ball_x, ball_y, ball_vx, ball_vy))
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
        }
    }
}
