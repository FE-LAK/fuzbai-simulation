use std::time::{Duration, Instant};
use std::sync::{LazyLock, Mutex};
use std::collections::VecDeque;

/// Global competition state shared between the simulation thread, GUI, and HTTP handlers.
pub static COMPETITION_STATE: LazyLock<Mutex<CompetitionState>> = LazyLock::new(|| Mutex::new(CompetitionState::default()));
pub const COMPETITION_DURATION_SECS: u64 = 120;

/// Actions queued for the simulation thread to execute on the next tick.
pub enum CompetitionPending {
    ResetScore,
    ResetSimulation,
}

/// Current phase of the competition.
#[derive(PartialEq, Clone)]
pub enum CompetitionStatus {
    /// Timed match in progress since the given [`Instant`].
    Running(Instant),
    /// Match manually paused.
    /// Wraps the elapsed [`Duration`] when the pause occurred.
    Paused(Duration),
    /// Match time expired (timer ran out).
    /// Wraps the total elapsed [`Duration`].
    Finished(Duration),
    /// Untimed free play.
    Free,
    /// Waiting for the game to start.
    Waiting,
}

/// Mutable competition state protected by `COMPETITION_STATE`.
pub struct CompetitionState {
    pub status: CompetitionStatus,
    pub pending: VecDeque<CompetitionPending>,
}

impl Default for CompetitionState {
    fn default() -> Self {
        Self { status: CompetitionStatus::Waiting, pending: VecDeque::new() }
    }
}
