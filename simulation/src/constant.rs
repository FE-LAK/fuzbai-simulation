/// Contains definitions of needed constants.

use crate::types::*;


/// Internal sub-step (MuJoCo engine) timestamp. The actual time that passes in each [`FuzbAISimulator::step_simulation`] call is [`LOW_TIMESTEP`] *
/// [`FuzbAISimulator::internal_step_factor`] (constructor parameter.)
pub const LOW_TIMESTEP: f64 = 0.002;
/// Maximum expected delay in seconds. This dictates how large the state buffer will be.
pub const MAX_DELAY_S: f64 = 0.100;
/// [`MAX_DELAY_S`] in units of [`LOW_TIMESTEP`], with added +1 element to accommodate for the latest (t_delay = 0.0) observation.
pub(crate) const MAX_DELAY_BUFFER_LEN: usize = (MAX_DELAY_S / LOW_TIMESTEP + 1.0) as usize;

/// Z-coordinate of the main table field, in global coordinates.
/// This can be used for local->global coordinate system transformations.
pub const Z_FIELD: f64 = 0.174 + 0.0175;
/// Upper Z-level below which the simulation is consider truncated (invalid state).
pub const TRUNCATION_Z_LEVEL: f64 = -0.5;

/// MuJoCo geom IDs in the format (min_idx, max_idx).
/// They map collision on each player to the rod index (Indices of this array represent rods).
pub(crate) const ROD_COLLISION_PLAYER_IDS: [(usize, usize); 8] = [(26, 27), (29, 32), (34, 39), (41, 50), (52, 61), (63, 68), (70, 73), (75, 76)];

/// Global positions of the bodies holding each player geom
pub(crate) const ROD_POSITIONS: [XYZType; 8] = [
    [0.195, 0.021, 0.2595],
    [0.345, 0.023, 0.2595],
    [0.495, 0.025, 0.2595],
    [0.645, 0.029, 0.2595],
    [0.795, 0.025, 0.2595],
    [0.945, 0.021, 0.2595],
    [1.095, 0.025, 0.2595],
    [1.245, 0.025, 0.2595],
];

// Some player constants
pub const ROD_TRAVELS: [f64; 8] = [0.190, 0.356, 0.180, 0.116, 0.116, 0.180, 0.356, 0.190];  /// Travel (in meters) of each rod.
pub(crate) const ROD_FIRST_OFFSET: [f64; 8] = [0.258, 0.055, 0.055, 0.055, 0.055, 0.055, 0.055, 0.258];
pub(crate) const ROD_SPACING: [f64; 8] = [0.0, 0.238, 0.208, 0.118, 0.118, 0.208, 0.238, 0.0];
pub(crate) const ROD_N_PLAYERS: [usize; 8] = [1, 2, 3, 5, 5, 3, 2, 1];

/// Represents the mesh ID of the player's upper body.
pub(crate) const ROD_MESH_UPPER_PLAYER_ID: i32 = 8;
/// Represents the mesh ID of the player's lower body.
pub(crate) const ROD_MESH_LOWER_PLAYER_ID: i32 = 9;

// The mesh is in a different coordinate system, than the one MuJoCo creates for the geom.
// Visualizing estimates requires giving the rotation matrix for rotation measured inside the MuJoCo's
// geom system, while the actual position must match the meshes original system.
pub(crate) const ROD_ESTIMATE_FRAME_UPPER_OFFSET: f64 = -0.033778597213357395;
pub(crate) const ROD_ESTIMATE_FRAME_LOWER_OFFSET: f64 = -0.06625961001214105;

/// The color of rod estimates.
pub(crate) const DEFAULT_ROD_ESTIMATE_RGBA: RGBAType = [0.0, 1.0, 0.0, 1.0];
/// The local frame of the bottom part of the player isn't aligned with the
/// global frame, but is needs to be rotated 90 degrees. This represents a rotational matrix
/// for rotation 90 degrees around X axis.
pub(crate) const ROD_BOTTOM_PRE_ROTATION_MAT: [f64; 9] = [1.0, 0.0,  0.0,
                                               0.0, 0.0, -1.0,
                                               0.0, 1.0, 0.0];

/// Translational actuator parameters
pub(crate) const ROD_TRANS_PARAMS: [(f64, f64, f64, f64); 8] = [
    // Kp      Kd      vmax   amax
    (519.86417448, 61.02380066, 0.83105261, 3.8500058),  // goal r
    (511.89695555, 17.24171106, 1.03166915, 3.41411132),  // def  r
    (603.35853608, 34.65564965, 0.80390965, 3.58047916),  // att  b
    (552.79589519, 101.2978418, 0.66642105, 4.10410804),  // off  r
    (552.79589519, 101.2978418, 0.66642105, 4.10410804),  // off  b
    (603.35853608, 34.65564965, 0.80390965, 3.58047916),  // att  r
    (511.89695555, 17.24171106, 1.03166915, 3.41411132),  // def  b
    (519.86417448, 61.02380066, 0.83105261, 3.8500058),  // goal b
];


/// Rotational actuator parameters
pub(crate) const ROD_ROT_PARAMS: [(f64, f64, f64, f64); 8] = [
    // Kp    Kd      vmax  amax
    (0.2, 0.05, 9.70400987e+01, 1.44952051e+03),  // goal r
    (2.94732122e-01, 0.05, 9.70400987e+01, 1.44952051e+03),  // def  r
    (2.59114612e-01, 0.05, 1.00839892e+02, 1.48222420e+03),  // att  b
    (3.69438137e-01, 0.05, 1.02937797e+02, 1.48038953e+03),  // off  r
    (3.69438137e-01, 0.05, 1.02937797e+02, 1.48038953e+03),  // off  b
    (2.59114612e-01, 0.05, 1.00839892e+02, 1.48222420e+03),  // att  r
    (2.94732122e-01, 0.05, 9.70400987e+01, 1.44952051e+03),  // def  b
    (0.2, 0.05, 9.70400987e+01, 1.44952051e+03),  // goal b
];


/// Gaussian noise standard deviation, added to the translational axis of the rods (in observation's units)
pub(crate) const ROD_TRANS_NOISE: f64 = 0.005;
/// Gaussian noise standard deviation, added to the rotational axis of the rods (in observation's units)
pub(crate) const ROD_ROT_NOISE: f64 = 1.0 * 32.0 / 180.0;


/// Creates a mapping of the individual player geom IDs to the correct rod index (from red to blue).
/// This table also contains invalid mappings, which are mapped as -1 or are above the map's length.
pub(crate) const fn create_geom_to_rod_map() -> [isize; ROD_COLLISION_PLAYER_IDS[ROD_COLLISION_PLAYER_IDS.len() - 1].1 + 1] {
    let mut out: [isize; 77] = [-1; ROD_COLLISION_PLAYER_IDS[ROD_COLLISION_PLAYER_IDS.len() - 1].1 + 1];
    let mut rod_id = 0isize;
    let mut geom_id;
    let mut geom_id_max;
    while rod_id < 8 {
        geom_id = ROD_COLLISION_PLAYER_IDS[rod_id as usize].0;
        geom_id_max = ROD_COLLISION_PLAYER_IDS[rod_id as usize].1;
        while geom_id <= geom_id_max {
            out[geom_id] = rod_id;
            geom_id += 1;
        }
        rod_id += 1;
    }
    out
}

/// Maps MuJoCo's geometry ID into rod index (starting from the red team).
pub(crate) const GEOM_TO_ROD_MAPPING: [isize; ROD_COLLISION_PLAYER_IDS[ROD_COLLISION_PLAYER_IDS.len() - 1].1 + 1] = create_geom_to_rod_map();

/// Indices of array data that extract the red team's information.
pub const RED_INDICES: [usize; 4] = [0, 1, 3, 5];
/// Indices of array data that extract the red blue's information.
pub const BLUE_INDICES: [usize; 4] = [7, 6, 4, 2];
                                     

/// Trace RGBA color displaying the newest timestamp. Other timestamps
/// will be displayed in the form of a continuous gradient between 
/// ``TRACE_RGBA_START`` and ``TRACE_RGBA_END``.
pub const TRACE_RGBA_END: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
/// Trace RGBA color displaying the oldest timestamp.
pub const TRACE_RGBA_START: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
pub(crate) const TRACE_RGBA_DIFF: [f32; 4] = [
    TRACE_RGBA_END[0] - TRACE_RGBA_START[0], TRACE_RGBA_END[1] - TRACE_RGBA_START[1],
    TRACE_RGBA_END[2] - TRACE_RGBA_START[2], TRACE_RGBA_END[3] - TRACE_RGBA_START[3]
];
/// The (capsule's) radius of the trace.
pub const TRACE_RADIUS: f64 = 0.003;
/// Maximum time the trace can be shown for.

/// Maximum t race length in units of high-level ([`LOW_TIMESTEP`] * [`FuzbAISimulator::internal_step_factor`]) steps.
pub const MAX_TRACE_BUFFER_LEN: usize = (10.0 / 0.020) as usize;  // 10 seconds / 20 ms

/// Maximum number of static geoms inside viewer's scene
/// This includes possible player geoms and the ball. Trace length is added to this
/// number to produce the final number of max scene geoms.
pub(crate) const VIEWER_MAX_STATIC_SCENE_GEOM: usize = 100;
pub(crate) const SCREENSHOT_MAX_ESTIMATE_SCENE_GEOM: usize = 125;
/// Number of geoms in a single timestep of a trace.
pub(crate) const TRACE_GEOM_LEN: usize = 45;  // 22 players on one side * 2 sides * geoms for each player + 1 ball

/// The position at which to spawn the ball.
pub const DEFAULT_BALL_POSITION: XYZType = [0.718, 0.65, 0.194];
/// Scale of the velocity at which to nudge the ball with [`FuzbAISimulator::nudge_ball`].
pub const DEFAULT_BALL_NUDGE_VELOCITY_SCALE: XYZType = [0.5, 0.5, 0.0];
/// The noise (in millimeters) added to the ball's position.
pub const BALL_POSITION_NOISE: f64 = 5.0;
/// The noise (in meters) added to the ball's velocity.
pub const BALL_VELOCITY_NOISE: f64 = 0.0125;
/// Minimal ball velocity at which the ball is considered to be moving.
pub const BALL_MOVING_VELOCITY: f64 = 0.05;
/// The time after which the simulation is considered truncated upon
/// the ball's velocity going below [`BALL_MOVING_VELOCITY`].
pub const BALL_MOVING_TIMEOUT_S: f64 = 3.0;
/// The radius of the ball, in meters.
pub const BALL_RADIUS_M: f64 = 0.0175;
/// The color of ball's estimate display.
pub const DEFAULT_BALL_ESTIMATE_RGBA: RGBAType = [1.0, 1.0, 0.0, 1.0];

// Error messages
pub const E_NOT_ENOUGH_ELEMENTS: &str = "not enough elements";
