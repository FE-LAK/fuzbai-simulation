//! # Player rods
//! Module used for player rods that get
//! used by the agent.
use crate::timer::*;

/* Other parameters */
const ROD_VELOCITY_LINEAR: f64 = 1.0;
const ROD_VELOCITY_ROTATIONAL: f64 = 1.0;


#[derive(Debug, PartialEq, Clone, Copy)]
enum FSMState {
    TrackBackBall = 0,
    FreeField,
    AimForShoot,
    Shoot
}


pub struct Rod {
    // Rod specific data
    drive_id: usize,
    n_players: usize,
    offset: f64,
    spacing: f64,
    travel: f64,
    x_pos: f64,

    // Dynamic variables
    extension: f64,
    rotation: f64,
    fsm_state: FSMState,

    // Timers
    main_timer: ManualTimer,

    // Preallocated buffers (to avoid reallocation)
    yposes: Box<[f64]>,
}


/// Methods for class [`Rod`] that don't get exported to Python bindings.
impl Rod {
    pub fn new(
        timestep: f64,
        drive_id: usize,
        n_players: usize,
        offset: f64,
        spacing: f64,
        travel: f64,
        x_pos: f64,
    ) -> Self {
        Self {
            drive_id,
            n_players,
            offset,
            spacing,
            travel,
            x_pos,
            extension: 0.0,
            rotation: 0.0,
            main_timer: ManualTimer::new(timestep),
            fsm_state: FSMState::TrackBackBall,
            yposes: vec![0.0; n_players].into_boxed_slice(),
        }
    }

    /// Convert rod's extension into absolute units for all players.
    fn extension_to_ypos(&mut self) -> &[f64] {
        // Convert to absolute units.
        let extension_mm = self.extension * self.travel;

        // Calculate position of other players based on position of the first
        let eo = extension_mm + self.offset;  // Position of first the player

        // Update the buffer inplace, without allocations.
        for (player_i, ypos) in self.yposes.iter_mut().enumerate() {
            *ypos = self.spacing * player_i as f64 + eo;
        }

        &self.yposes
    }
}


/// Methods for class [`Rod`] that get exported to Python bindings.
impl Rod {
    /// Perform a single step in time based on the latest state (parameters).
    pub fn get_action(&mut self, ball_x: f64, ball_y: f64, ball_vx: f64, ball_vy: f64) -> (usize, f64, f64, f64, f64) {

        /* Calculate the needed rod extension needed to intercept the ball at the predicted position. */
        let mut error = 0.0;
        let mut min_abs = f64::INFINITY;
        for &ypos in self.extension_to_ypos() {
            let error_player = ball_y - ypos;
            let abs_error = error_player.abs();
            if abs_error < min_abs {
                min_abs = abs_error;
                error = error_player;
            }
        }

        let mut new_extension = self.extension + error / self.travel;

        // Clip as the above loop will give the extension to be infinite if no players can be selected.
        new_extension = new_extension.clamp(0.0, 1.0);

        // State machine

        let rotation;
        let old_state = self.fsm_state;
        match old_state {
            FSMState::TrackBackBall => {
                rotation = 0.0;
                if ball_x < self.x_pos - 100.0 {
                    self.fsm_state = FSMState::FreeField;
                } else if error.abs() < 10.0 && ball_x > self.x_pos - 50.0 && ball_x < self.x_pos + 50.0 {
                    if self.main_timer.elapsed() > 0.25 {
                        self.fsm_state = FSMState::AimForShoot;
                    }
                } else {
                    self.main_timer.reset();
                }
            }
            FSMState::FreeField => {
                rotation = 0.5;
                if ball_x > self.x_pos - 50.0 {
                    self.fsm_state = FSMState::TrackBackBall;
                }
            }
            FSMState::AimForShoot => {
                rotation = 0.5;
                if self.main_timer.elapsed() > 0.25 {
                    self.fsm_state = FSMState::Shoot;
                }
            }
            FSMState::Shoot => {
                rotation = -0.25;
                if self.main_timer.elapsed() > 0.25 {
                    self.fsm_state = FSMState::TrackBackBall;
                }
            }
        }

        // Always reset timer on state change
        if old_state != self.fsm_state {
            self.main_timer.reset();
        }

        // Save state and return
        self.extension = new_extension;
        self.rotation = rotation;

        // Step timers
        self.main_timer.step();

        (self.drive_id, new_extension, rotation, ROD_VELOCITY_LINEAR, ROD_VELOCITY_ROTATIONAL)
    }

    /// Resets the rod state variables.
    pub fn reset(&mut self) {
        self.fsm_state = FSMState::TrackBackBall;
        self.main_timer.reset();
    }
}
