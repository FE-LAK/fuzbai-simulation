//! # Timers
//! Module containing definitions of different timers used by the agent.

/// Timer based on manual stepping (e.g., for sped up simulations).
pub struct ManualTimer {
    timestep: f64,
    n_steps: usize
}

impl ManualTimer {
    pub fn new(timestep: f64) -> Self {
        Self {timestep, n_steps: 0}
    }

    pub fn reset(&mut self) {
        self.n_steps = 0;
    }

    pub fn elapsed(&self) -> f64 {
        self.timestep * self.n_steps as f64
    }

    pub fn step(&mut self) {
        self.n_steps += 1;
    }
}
