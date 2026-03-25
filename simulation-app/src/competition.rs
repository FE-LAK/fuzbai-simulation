use std::sync::{LazyLock, Mutex};
use std::collections::VecDeque;
use std::time::Instant;

/// Global competition state shared between the simulation thread, GUI, and HTTP handlers.
pub static COMPETITION_STATE: LazyLock<Mutex<CompetitionState>> = LazyLock::new(|| Mutex::new(CompetitionState::default()));
pub const COMPETITION_DURATION_SECS: u64 = 120;

/// Actions queued for the simulation thread to execute on the next tick.
pub enum CompetitionPending {
    ResetScore,
    ResetSimulation,
    SwapTeams,
}

/// Current phase of the competition.
#[derive(PartialEq, Clone)]
pub enum CompetitionStatus {
    /// Timed match in progress since the given `Instant`.
    Running(Instant),
    /// Match stopped or not yet started.
    /// Wraps the time when the game ended (in seconds from start).
    Expired(u64),
    /// Untimed free play.
    Free,
    // Waiting for the gane to start
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
