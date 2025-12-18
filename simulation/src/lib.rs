use mujoco_rs::viewer::{MjViewer, ViewerSharedState};
use mujoco_rs::prelude::*;
use mujoco_rs;

// Re-export for convenience
pub use mujoco_rs::viewer::egui;

use agent_rust::Agent as BuiltInAgent;

use std::sync::{Arc, OnceLock, Mutex};
use std::{collections::VecDeque};
use std::cell::RefCell;
use std::time::Instant;
use std::fmt::Debug;
use core::f64;

use rand::distr::{Distribution, Uniform};

#[cfg(feature = "python-bindings")]
use pyo3::prelude::*;

use constant::*;
use render::*;
use motors::*;
use types::*;

pub mod constant;
pub mod render;
pub mod motors;
pub mod types;


/* Globals */
/// Compiled MuJoCo model. Useful when compiling as a Python wheel or a Rust crate.
const MJB_MODEL_DATA: &[u8] = include_bytes!("./miza.mjb");

/// Global MjModel instance shared across multiple FuzbAI simulators.
/// This has the benefit of consuming less memory (as the model is fixed).
/// It is also required due to PyO3's aggressive checks for thread-safety and
/// prohibition of Rust's lifetimes.
static G_MJ_MODEL: OnceLock<MjModel> = OnceLock::new();
thread_local! {
    /// Multiple viewers are not allowed (unless in a different process).
    /// This is a protection mechanism from accidentally launching multiple realtime
    /// simulations.
    static G_MJ_VIEWER: RefCell<Option<MjViewer<&'static MjModel>>> = RefCell::new(None);
}
static G_VIEWER_SHARED_STATE: OnceLock<Arc<Mutex<ViewerSharedState<&'static MjModel>>>> = OnceLock::new();


/* Enum definitions */
#[cfg_attr(feature = "python-bindings", pyclass(eq, eq_int, module = "fuzbai_simulator"))]
#[derive(PartialEq, Clone, Debug)]
#[repr(usize)] 
/// Enumerates the two possible teams by color.
pub enum PlayerTeam {
    Red = 0,
    Blue
}

#[cfg(feature = "python-bindings")]
#[pymethods]
/// Python methods for pickle support
impl PlayerTeam {
    /// Dummy constructor to allow pickle support
    #[new]
    fn new() -> Self {
        Self::RED
    }

    fn __getstate__(&self) -> PyResult<usize> {
        Ok(self.clone() as usize)
    }

    fn __setstate__(&mut self, state: usize) -> PyResult<()> {
        // SAFETY: this is save as long as it is called from Python (Pyo3 will only call on the valid enum).
        *self = unsafe { std::mem::transmute(state) };
        Ok(())
    }
}


/* Struct definitions */

/// Specifies visualization related parameters of 
/// the [`FuzbAISimulator`] struct.
#[cfg_attr(feature = "python-bindings", pyclass(module = "fuzbai_simulator"))]
#[derive(Clone)]
pub struct VisualConfig {
    pub trace_length: usize,
    pub trace_ball: bool,
    pub trace_rod_mask: u64,
    pub enable_viewer: bool,
}

#[cfg(feature = "python-bindings")]
#[pymethods]
impl VisualConfig {
    #[new]
    pub fn py_new(trace_length: usize, trace_ball: bool, trace_rod_mask: u64, enable_viewer: bool) -> Self {
        Self::new(trace_length, trace_ball, trace_rod_mask, enable_viewer)
    }

    #[staticmethod]
    #[pyo3(name = "player_mask")]
    pub fn py_player_mask(rod_index: usize, player_indices: Vec<usize>) -> u64 {
        Self::player_mask(rod_index, &player_indices)
    }
}

impl VisualConfig {
    pub fn new(
        trace_length: usize, trace_ball: bool, trace_rod_mask: u64, enable_viewer: bool
    ) -> Self {
        VisualConfig {trace_length, trace_ball, trace_rod_mask, enable_viewer}
    }

    /// Creates the mask needed for [`VisualConfig.new`].
    /// To visualize multiple rods use the OR operator:
    /// `player_mask(0, vec![0]) | player_mask(2, vec![0, 2])`.
    pub fn player_mask(rod_index: usize, player_indices: &[usize]) -> u64 {
        let mut mask = 0;
        for index in player_indices {
            mask |= 1 << index;
        }
        mask << (rod_index * 8)
    }
}


/// High level simulation wrapping the MuJoCo physical
/// simulation of the table football.
#[cfg_attr(feature = "python-bindings", pyclass(module = "fuzbai_simulator"))]
pub struct FuzbAISimulator {
    // Parameter data
    internal_step_factor: usize,
    sample_steps: usize,
    simulated_delay_s: f64,
    visual_config: VisualConfig,
    realtime: bool,

    // State data

    /* Status flags */
    /// The simulation episode has ended (goal was scored).
    terminated: bool,
    /// Simulation resulted in an invalid state (timeout, ball out of the field).
    truncated: bool,

    /* Timing */
    /// The current timestep in the form of continuous time
    current_time: f64,
    /// The current timestep in the form of low-level discrete steps
    current_ll_step: usize,

    /// The time (instant) at which the ball was last moving
    /// This is used to detect whether the ball has stopped moving.
    ball_last_moving_t: f64,

    /* Player-related */
    /// Controls whether the red team (left) is controlled outside of this simulation (`true`)
    /// or by the internally stored agent (`false`).
    external_team_red: bool,
    /// Controls whether the blue team (right) is controlled outside of this simulation (`true`)
    /// or by the internally stored agent (`false`).
    external_team_blue: bool,

    /// The score of each team (red, blue).
    score: [u16; 2],

    /* Miscellaneous */
    /// Holds past observations up to [`MAX_DELAY_S`].
    // delayed_memory: ArrayDeque<ObservationType, MAX_DELAY_BUFFER_LEN, Wrapping>,
    delayed_memory: VecDeque<ObservationType>, 

    /* MuJoCo data */
    /// Simulation state struct.
    /// # SAFETY
    /// DO NOT SHARE BETWEEN THREADS OR FREE THE RAW DATA.
    /// This is not meant to be used in multiple threads and is thus .
    /// Additionally, due to interaction with C code, special care must be taken into
    /// account.
    mj_data: MjData<&'static MjModel>,

    /// View to the ball's joint data of mj_data.
    /// # SAFETY
    /// This needs to be dropped before mj_data.
    mj_data_joint_ball: MjJointDataInfo,

    /// View to the ball's damping actuator.
    /// # SAFETY
    /// These need to be dropped before mj_data.
    mj_data_act_ball_damp_x: MjActuatorDataInfo,
    mj_data_act_ball_damp_y: MjActuatorDataInfo,

    /* Contact detection */
    mj_red_goal_geom_ids: [i32; 2],
    mj_blue_goal_geom_ids: [i32; 2],

    collision_forces: [[f64; 4]; 8],
    collision_indices: [isize; 8],

    /* Player control */
    pending_motor_cmd_red: Vec<MotorCommand>,
    pending_motor_cmd_blue: Vec<MotorCommand>,
    pub red_builtin_player: BuiltInAgent,
    pub blue_builtin_player: BuiltInAgent,
    trans_motor_ctrl: TrapezoidMotorSystem<&'static MjModel>,
    rot_motor_ctrl: TrapezoidMotorSystem<&'static MjModel>,

    /* Visualization */
    visualizer: Visualizer<&'static MjModel>
}


#[cfg_attr(feature = "python-bindings", pymethods)]
impl FuzbAISimulator {
    pub fn terminated(&self) -> bool {
        self.terminated
    }

    pub fn truncated(&self) -> bool {
        self.truncated
    }

    pub fn score(&self) -> &[u16; 2] {
        &self.score
    }

    /// Returns the collision forces (relative to the red team).
    pub fn collision_forces(&self) -> [[f64; 4]; 8] {
        self.collision_forces
    }

    /// Returns the collision indices (relative to the red team).
    pub fn collision_indices(&self) -> [isize; 8] {
        self.collision_indices
    }

    /// Returns the configured simulated delay (in seconds).
    pub fn simulated_delay_s(&self) -> f64 {
        self.simulated_delay_s
    }

    /// Checks if the viewier is still running.
    pub fn viewer_running(&self) -> bool {
        G_VIEWER_SHARED_STATE.get().map_or(
            false,  // if it doesn't exist, then it can't be running
            |state| state.lock().unwrap().running()
        )
    }

    /// Returns the ball's TRUE state (without noise) in the format
    /// (x, y, z, vx, vy)
    pub fn ball_true_state(&self) -> [f64; 6] {
        let ball_view = self.mj_data_joint_ball.view(&self.mj_data);
        let [x, y, z, ..] = *ball_view.qpos else {panic!("{}", E_NOT_ENOUGH_ELEMENTS)};
        let [vx, vy, vz, ..] = *ball_view.qvel else {panic!("{}", E_NOT_ENOUGH_ELEMENTS)};

        [1000.0 * x - 115.0, 727.0 - 1000.0 * y, 1000.0 * z, vx, -vy, vz]
    }

    pub fn rods_true_state(&self) -> ([f64; 8], [f64; 8], [f64; 8], [f64; 8]) {
        let mut rod_trans = [0.0; 8];
        let mut rod_rot = [0.0; 8];
        let mut rod_trans_vel = [0.0; 8];
        let mut rod_rot_vel = [0.0; 8];
        for i in 0..8 {
            rod_trans[i] = 1.0 - self.trans_motor_ctrl.qpos(&self.mj_data, i) / ROD_TRAVELS[i];
            rod_trans_vel[i] = -self.trans_motor_ctrl.qvel(&self.mj_data, i);
            rod_rot[i] = self.rot_motor_ctrl.qpos(&self.mj_data, i) / std::f64::consts::PI * 32.0;
            rod_rot_vel[i] = self.rot_motor_ctrl.qvel(&self.mj_data, i);
        }

        (rod_trans, rod_rot, rod_trans_vel, rod_rot_vel)
    }

    /// Returns the observation of the simulation.
    /// The observation represents real-world like state with added
    /// noise. It includes the ball's state and the state of each rod.
    /// The parameter `team` controls which team's coordinate system the data should be relative to.
    /// The returned data is in format (ball_state, rod_positions, rod_rotations)
    pub fn observation(&self, team: PlayerTeam) -> ObservationType {
        let observation = self.observation_red();
        match team {
            PlayerTeam::Red => observation,
            PlayerTeam::Blue => Self::transform_observation_red_to_blue(observation)
        }
    }

    /// Same as [`observation`](FuzbAISimulator::observation) but delayed.
    /// The delay can be optionally specified by `delay`, otherwise the internally set one is used.
    pub fn delayed_observation(&self, team: PlayerTeam, delay: Option<f64>)  -> ObservationType {
        let delay = delay.unwrap_or(self.simulated_delay_s);
        if delay == 0.0 || self.delayed_memory.is_empty() {
            return self.observation(team);
        }

        // Map the continuous-in-time delay into the buffer index.
        // Floor to preserve the original behavior where we would take the first obs
        // whose timestamp was greater than the delayed time (t_obs > (current_t - delay)).
        // We round to two decimal places to avoid issues like getting 3.99999999999999.
        let mut index = (f64::round(delay / self.sample_steps as f64 / LOW_TIMESTEP * 100.0) / 100.0) as usize;

        // The buffer doesn't have information about that far in the past.
        // In such event the oldest available is returned.
        index = index.min(self.delayed_memory.len() - 1);

        let obs_red = self.delayed_memory[index];
        match team {
            PlayerTeam::Red => obs_red,
            PlayerTeam::Blue => Self::transform_observation_red_to_blue(obs_red)
        }
    }

    /// Sets the simulated delay (in seconds) to `simulated_delay_s`.
    pub fn set_simulated_delay(&mut self, simulated_delay_s: f64) {
        self.simulated_delay_s = simulated_delay_s;
        self.red_builtin_player.set_delay(simulated_delay_s);
        self.blue_builtin_player.set_delay(simulated_delay_s);
    }

    pub fn set_ball_damping(&mut self, damping: XYType) {
        self.mj_data_act_ball_damp_x.view_mut(&mut self.mj_data).ctrl[0] = damping[0];
        self.mj_data_act_ball_damp_y.view_mut(&mut self.mj_data).ctrl[0] = damping[1];
    }

    /// Sets the joint state of individual rod to `positions` and `rotations`.
    /// The position is in range of: [0, 1], while the rotation in range [-1, 1].
    /// The ranges thus match actions of [`set_motor_command`](FuzbAISimulator::set_motor_command)
    /// Everything is done relative to the red's coordinate system.
    pub fn set_rod_states(&mut self, positions: Option<[f64; 8]>, rotations: Option<[f64; 8]>) {
        let mut pos;
        // position
        if let Some(positions) = positions {
            for i in 0..8 {
                pos = (1.0 - positions[i]) * ROD_TRAVELS[i];
                self.trans_motor_ctrl.set_qpos(&mut self.mj_data, i, pos);
                self.trans_motor_ctrl.force_stop(i, &mut self.mj_data);
            }
        }

        // rotation
        if let Some(rotations) = rotations {
            for i in 0..8 {
                pos = rotations[i] * 2.0 * std::f64::consts::PI;
                self.rot_motor_ctrl.set_qpos(&mut self.mj_data, i, pos);
                self.rot_motor_ctrl.force_stop(i, &mut self.mj_data);
            }
        }

        self.mj_data.step();
    }

    /// Configures whether the specific `team` should obtain commands externally (`enable` = `true`)
    /// or internally (`enable` = `false`, the agent is stored within the simulation).
    pub fn set_external_mode(&mut self, team: PlayerTeam, enable: bool) {
        if let PlayerTeam::Red = team {
            self.external_team_red = enable;
        }
        else {
            self.external_team_blue = enable;
        }
    }

    /// Fills the delay buffer, allowing information to be fully delayed from the beginning of reset.
    pub fn fill_delay_buffer(&mut self) {
        if self.sample_steps == 0 {
            return;
        }

        for _ in 0..(self.simulated_delay_s / self.sample_steps as f64 / LOW_TIMESTEP).ceil() as usize {
            self.current_time += LOW_TIMESTEP;
            self.sample_state();
        }
    }

    /// Servers the ball to a given `position`.
    pub fn serve_ball(&mut self, position: Option<XYZType>) {
        let mut ball_view = self.mj_data_joint_ball.view_mut(&mut self.mj_data);
        let position = if let Some(xyz) = position {
            Self::local_to_global_position(xyz)
        }
        else {
            DEFAULT_BALL_POSITION
        };
        ball_view.qpos[..3].copy_from_slice(&position);
        ball_view.qpos[3..].copy_from_slice(&[1.0, 0.0, 0.0, 0.0]); // 0 rotation
    }

    pub fn nudge_ball(&mut self, velocity: Option<XYZType>) {
        let mut ball_view = self.mj_data_joint_ball.view_mut(&mut self.mj_data);
        if let Some(v) = velocity {
            ball_view.qvel[..3].copy_from_slice(&Self::local_to_global_velocity(v));
        }
        else {
            let mut rng = rand::rng();
            let mut dist;
            for i in 0..DEFAULT_BALL_NUDGE_VELOCITY_SCALE.len() {
                if DEFAULT_BALL_NUDGE_VELOCITY_SCALE[i] == 0.0 {
                    ball_view.qvel[i] = 0.0;
                    continue;
                }

                dist = Uniform::new(-DEFAULT_BALL_NUDGE_VELOCITY_SCALE[i], DEFAULT_BALL_NUDGE_VELOCITY_SCALE[i]).unwrap();
                ball_view.qvel[i] = dist.sample(&mut rng);
            }
        }

        // Match the rotational velocity to achieve spinning (w = v / r)
        ball_view.qvel[3] = -ball_view.qvel[1] / BALL_RADIUS_M;
        ball_view.qvel[4] =  ball_view.qvel[0] / BALL_RADIUS_M;
        ball_view.qvel[5] =  0.0;

        // Reset other states
        ball_view.qacc_warmstart.fill(0.0);
        ball_view.qacc.fill(0.0);
        ball_view.qfrc_applied.fill(0.0);
    }

    pub fn clear_trace(&mut self) {
        self.visualizer.clear_trace();
    }

    pub fn clear_score(&mut self) {
        self.score.fill(0);
    }

    /// Syncs the simulation state with the viewer.
    /// When viewer is not running, this has no effect.
    pub fn sync_viewer(&mut self) {
        let maybe_state = G_VIEWER_SHARED_STATE.get();
        if let Some(state) = maybe_state {
            state.lock().unwrap().sync_data(&mut self.mj_data);
        }
    }

    pub fn reset_simulation(&mut self) {
        self.terminated = false;
        self.truncated = false;

        self.current_time = 0.0;
        self.current_ll_step = 0;
        self.ball_last_moving_t = 0.0;

        self.delayed_memory.clear();
        self.clear_trace();

        self.mj_data.reset();
        self.serve_ball(None);
        self.nudge_ball(None);
        self.mj_data.step1();
    }

    pub fn step_simulation(&mut self) {
        self.clear_collisions();
        self.apply_motor_commands();
        // low-level steps
        for _ in 0..self.internal_step_factor {
            let t_start = Instant::now();
            // Position tracking -> torque
            self.trans_motor_ctrl.step(&mut self.mj_data);
            self.rot_motor_ctrl.step(&mut self.mj_data);
            // self.trans_motor_ctrl.check_reference();  // force-stop when reference is set close to the current position
            self.mj_data.step2();
            self.mj_data.step1();

            self.current_ll_step += 1;
            self.current_time += LOW_TIMESTEP;

            // Store the state into the delayed buffer
            if self.sample_steps > 0 && self.current_ll_step % self.sample_steps == 0 {
                self.sample_state();
            }

            self.update_collisions();

            // If realtime, sync the viewer's state with out simulation state
            if let Some(viewer_state) = G_VIEWER_SHARED_STATE.get() {
                {
                    let mut lock = viewer_state.lock().unwrap();
                    lock.sync_data(&mut self.mj_data);
                }
            };

            if self.realtime {
                while t_start.elapsed().as_secs_f64() < LOW_TIMESTEP {}  // Accurate timing
            }
        }

        self.update_visuals();

        /* Update high-level state */
        // Termination (goal scored)
        let contacts = self.mj_data.contacts();
        let mut geom_id;
        for contact in contacts.iter() {
            geom_id = contact.geom2;

            // Blue scored a goal
            if self.mj_red_goal_geom_ids.iter().any(|&id| id == geom_id) {
                self.terminated = true;
                self.score[1] += 1;
                break;
            }

            // Red scored a goal
            else if self.mj_blue_goal_geom_ids.iter().any(|&id| id == geom_id) {
                self.terminated = true;
                self.score[0] += 1;
                break;
            }
        }

        // Truncation
        let ball_joint_view = self.mj_data_joint_ball.view(&self.mj_data);
        let ball_global_pos = ball_joint_view.qpos.as_ref();
        let ball_global_vel = ball_joint_view.qvel.as_ref();
        if ball_global_pos[2] < TERMINATION_Z_LEVEL {
            self.terminated = true;
        }

        // Check if ball is moving
        else if f64::sqrt(ball_global_vel[0].powi(2) + ball_global_vel[1].powi(2)) > BALL_MOVING_VELOCITY {
            self.ball_last_moving_t = self.current_time;
        }
        else if self.current_time - self.ball_last_moving_t > BALL_MOVING_TIMEOUT_S {
            self.truncated = true;
        }
    }

    /// Sets the actuator control parameters.
    /// ``actuator_id`` of [0-7] represents translational actuators, ``actuator_id`` of [8-15]
    /// represents rotational actuators.
    pub fn _set_actuator_parameters(&mut self, mut actuator_id: usize, kp: f64, kd: f64, max_vel: f64, max_acc: f64) {
        let controller = if actuator_id < 8 {  // translational
            &mut self.trans_motor_ctrl
        }
        else {  // rotational
            actuator_id -= 8;
            &mut self.rot_motor_ctrl
        };

        controller.set_params(actuator_id, kp, kd, max_vel, max_acc);
    }

    /*
    Python-specific methods that either can't be shared with Rust methods or
    would cause Rust code to be inefficient when used in Rust-only.
    */
    #[cfg(feature = "python-bindings")]
    #[new]
    fn py_new(
        internal_step_factor: usize,
        sample_steps: usize,
        realtime: bool,
        simulated_delay_s: f64,
        model_path: Option<&str>,
        visual_config: VisualConfig
    ) -> Self {
        Self::new(internal_step_factor, sample_steps, realtime, simulated_delay_s, model_path, visual_config)
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "show_estimates")]
    fn py_show_estimates(&mut self, ball_xyz: Option<XYZType>, rod_tr: Option<Vec<(usize, f64, f64, u8)>>) {
        self.show_estimates(ball_xyz, rod_tr.as_deref());
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "local_to_global_position")]
    #[staticmethod]
    fn py_local_to_global_position(position: XYZType) -> XYZType {
        Self::local_to_global_position(position)
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "local_to_global_velocity")]
    #[staticmethod]
    fn py_local_to_global_velocity(velocity: XYZType) -> XYZType {
        Self::local_to_global_velocity(velocity)
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "local_to_global")]
    #[staticmethod]
    fn py_local_to_global(position: Option<XYZType>, velocity: Option<XYZType>) -> (Option<XYZType>, Option<XYZType>) {
        Self::local_to_global(position, velocity)
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "transform_observation_red_to_blue")]
    #[staticmethod]
    fn py_transform_observation_red_to_blue(observation: ObservationType) -> ObservationType {
        Self::transform_observation_red_to_blue(observation)
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "set_motor_command")]
    fn py_set_motor_command(&mut self, commands: Vec<MotorCommand>, team: PlayerTeam) { 
        self.set_motor_command(&commands, team);
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "set_built_in_disabled_rods")]
    fn py_set_built_in_disabled_rods(&mut self, team: PlayerTeam, indices: Vec<usize>) {
        self.set_built_in_disabled_rods(team, &indices);
    }
}

/// Non-Python exposed methods
impl FuzbAISimulator {
    pub fn new(
        internal_step_factor: usize,
        sample_steps: usize,
        realtime: bool,
        simulated_delay_s: f64,
        model_path: Option<&str>,
        visual_config: VisualConfig,
    ) -> Self {
        assert!(simulated_delay_s <= MAX_DELAY_S, "simulated_delay_s can't be larget than {MAX_DELAY_S}");
        assert!(visual_config.trace_length <= MAX_TRACE_BUFFER_LEN, "trace_length must be smaller than {MAX_TRACE_BUFFER_LEN}");

        let model = G_MJ_MODEL.get_or_init(|| {
            if let Some(path) = model_path {
                MjModel::from_xml(path)
            }
            else {
                MjModel::from_buffer(MJB_MODEL_DATA)
            }
            .expect("could not load the MuJoCo model")
        });

        let mut mj_data = model.make_data();
        // Make sure we have forward kinematics calculated before doing any stepping (in case reset will not be called).
        // In the step_simulation method we call step2 first and step1 after to prevent non-state attributes of MjData
        // from lagging behind one low-level step.
        mj_data.step1();

        let trace_length = visual_config.trace_length;
        if visual_config.enable_viewer {
            G_MJ_VIEWER.with(|slot| {
                let mut borrow = slot.borrow_mut();
                if borrow.is_none() {
                    let v = mujoco_rs::viewer::MjViewer::builder()
                        .vsync(true)
                        .max_user_geoms(MAX_ESTIMATE_SCENE_USER_GEOM + trace_length * TRACE_GEOM_LEN)
                        .warn_non_realtime(true)
                        .window_name("FuzbAI Simulator")
                    .build_passive(model).unwrap();
                    G_VIEWER_SHARED_STATE.set(v.state().clone()).expect("viewer is already initialized");
                    *borrow = Some(v);
                }
                else {
                    panic!("multiple realtime (with-rendering) simulations requested!")
                }
            });
        };

        // Create views to MjData
        let mj_data_joint_ball = mj_data.joint("ball").unwrap();
        let mj_data_joint_rod_trans = std::array::from_fn(|i| mj_data.joint(&format!("p{}_slide", i+1)).unwrap());
        let mj_data_joint_rod_rot = std::array::from_fn(|i| mj_data.joint(&format!("p{}_revolute", i+1)).unwrap());
        let mj_data_act_rod_trans = std::array::from_fn(|i| mj_data.actuator(&format!("p{}_slide_ctrl", i+1)).unwrap());
        let mj_data_act_rod_rot= std::array::from_fn(|i| mj_data.actuator(&format!("p{}_revolute_ctrl", i+1)).unwrap());

        // Find geom IDs of the individual goals. This is used for detecting scored goals.
        let mj_red_goal_geom_ids = [
            model.name_to_id(MjtObj::mjOBJ_GEOM, "left-goal-hole"),
            model.name_to_id(MjtObj::mjOBJ_GEOM, "left-goal-back"),
            
        ];

        let mj_blue_goal_geom_ids = [
            model.name_to_id(MjtObj::mjOBJ_GEOM, "right-goal-hole"),
            model.name_to_id(MjtObj::mjOBJ_GEOM, "right-goal-back"),
        ];

        // Initialize internal player teams
        let mut red_builtin_player = BuiltInAgent::new(simulated_delay_s);
        let mut blue_builtin_player = BuiltInAgent::new(simulated_delay_s);

        // When the simulation operates in discrete time (speed-up simulation),
        // the built-in players must do the same.
        if !realtime {
            red_builtin_player.to_step_mode(LOW_TIMESTEP * internal_step_factor as f64);
            blue_builtin_player.to_step_mode(LOW_TIMESTEP * internal_step_factor as f64);
        }

        let mj_data_act_ball_damp_x = mj_data.actuator("ball_damp_x").unwrap();
        let mj_data_act_ball_damp_y = mj_data.actuator("ball_damp_y").unwrap();
        let trans_motor_ctrl = TrapezoidMotorSystem::new(
            std::array::from_fn(|index| ROD_TRANS_PARAMS[index].0),
            std::array::from_fn(|index| ROD_TRANS_PARAMS[index].1),
            std::array::from_fn(|index| ROD_TRANS_PARAMS[index].2),
            std::array::from_fn(|index| ROD_TRANS_PARAMS[index].3),
            0.005,
            std::array::from_fn(|index| ROD_TRAVELS[index] * 0.005),
            mj_data_joint_rod_trans, mj_data_act_rod_trans
        );
        let rot_motor_ctrl = TrapezoidMotorSystem::new(
            std::array::from_fn(|index| ROD_ROT_PARAMS[index].0),
            std::array::from_fn(|index| ROD_ROT_PARAMS[index].1),
            std::array::from_fn(|index| ROD_ROT_PARAMS[index].2),
            std::array::from_fn(|index| ROD_ROT_PARAMS[index].3),
            10.0f64.to_radians(),
            [0.0; 8],
            mj_data_joint_rod_rot, mj_data_act_rod_rot
        );

        Self {
            internal_step_factor, sample_steps, simulated_delay_s, visual_config, realtime,
            mj_data, mj_data_joint_ball, trans_motor_ctrl, rot_motor_ctrl,
            mj_red_goal_geom_ids, mj_blue_goal_geom_ids,
            delayed_memory: VecDeque::new(),
            ball_last_moving_t: 0.0,
            external_team_red: true, external_team_blue: false,
            terminated: true, truncated: false, score: [0; 2],
            current_time: 0.0, current_ll_step: 0,
            collision_forces: [[0.0, 0.0, 0.0, 0.0]; 8], collision_indices: [-1; 8],
            pending_motor_cmd_red: Vec::new(), pending_motor_cmd_blue: Vec::new(),
            red_builtin_player, blue_builtin_player,
            mj_data_act_ball_damp_x, mj_data_act_ball_damp_y,
            visualizer: Visualizer::new(trace_length)
        }
    }

    #[inline]
    pub fn local_to_global_position(position: XYZType) -> XYZType {
        [(position[0] + 115.0) / 1000.0, (727.0 - position[1]) / 1000.0, position[2] / 1000.0 + Z_FIELD]
    }

    #[inline]
    pub fn local_to_global_velocity(velocity: XYZType) -> XYZType {
        [velocity[0], -velocity[1], velocity[2]]
    }

    /// Transforms local coordinates (from the read team) into Mujoco's global coordinate system.
    /// Method assumes that `position` is in mm, whilst the output is in m (as expected by MuJoCo).
    /// The `velocity` is in m/s and so is the output.
    #[inline]
    pub fn local_to_global(mut position: Option<XYZType>, mut velocity: Option<XYZType>) -> (Option<XYZType>, Option<XYZType>) {
        // Transform position
        if let Some(p) = position {
            position = Some(Self::local_to_global_position(p))
        }

        // Transform velocity
        if let Some(v) = velocity {
            velocity = Some(Self::local_to_global_velocity(v));
        }
        (position, velocity)
    }

    /// Transforms the observation from the red's coordinate system into the blue's.
    pub fn transform_observation_red_to_blue(observation: ObservationType) -> ObservationType {
        let (
            mut ball_x, mut ball_y, mut ball_vx, mut ball_vy,
            rod_positions, rod_rotations
        ) = observation;
        ball_x = 1207.0 - ball_x;
        ball_y = 702.0 - ball_y;
        ball_vx = -ball_vx;
        ball_vy = -ball_vy;
        let mut rod_positions_out = [0.0; 8];
        let mut rod_rotations_out = [0.0; 8];

        for i in 0..8 {
            rod_positions_out[i] = 1.0 - rod_positions[7-i];
            rod_rotations_out[i] = -rod_rotations[7-i];
        }
        (ball_x, ball_y, ball_vx, ball_vy, rod_positions_out, rod_rotations_out)
    }

    /// Draws the ball's and the rods' estimated states to the viewer.
    /// The ``rod_tr`` parameter is an array of ``[translation, rotation]``,
    /// where the translation is in normalized units [0..1] and the rotation is in range of
    /// [-64, 64], representing an entire circle (360 deg) in any direction.
    pub fn show_estimates(&mut self, ball_xyz: Option<XYZType>, rod_tr: Option<&[(usize, f64, f64, u8)]>) {
        if let Some(state) = G_VIEWER_SHARED_STATE.get() {
            let mut lock = state.lock().unwrap();
            let scene = lock.user_scene_mut();

            if let Some(unwrapped_ball_xyz) = ball_xyz {
                Visualizer::render_ball_estimate(scene, &unwrapped_ball_xyz, None);
            }

            if let Some(unwrapped_rot_tr) = rod_tr {
                Visualizer::render_rods_estimates(scene, unwrapped_rot_tr, None);
            }
        };
    }

    /// Sets new motor (rod movement) commands for the specified `team`.
    /// The commands will be applied at the next call to [`step_simulation`](FuzbAISimulator::step_simulation)
    pub fn set_motor_command(&mut self, commands: &[MotorCommand], team: PlayerTeam) {
        let pending_cmds = if let PlayerTeam::Red = team {&mut self.pending_motor_cmd_red} else {&mut self.pending_motor_cmd_blue};
        pending_cmds.clear();
        pending_cmds.extend_from_slice(commands);
        pending_cmds.shrink_to_fit();
    }

    /// Proxy method to the built-in player's `set_disabled` method.
    pub fn set_built_in_disabled_rods(&mut self, team: PlayerTeam, indices: &[usize]) {
        if let PlayerTeam::Red = team {
            self.red_builtin_player.set_disabled(indices);
        }
        else {
            self.blue_builtin_player.set_disabled(indices);
        }
    }

    fn update_visuals(&mut self) {
        // Store trace. This is done regardless of the viewier's existence
        // to support screenshots outside an active viewer.
        if self.visual_config.trace_length > 0 {
            let xpos = &self.mj_data_joint_ball.view(&self.mj_data).qpos[..3];
            let (rod_trans, rod_rot, ..) = self.rods_true_state();
            let trace_state: TraceType = (
                std::array::from_fn(|idx| xpos[idx]),
                std::array::from_fn(|idx| rod_trans[idx]),
                std::array::from_fn(|idx| rod_rot[idx]),
            );
            self.visualizer.sample_trace(trace_state);
        }

        // Reset the viwer's scene and draw the trace.
        if let Some(state) = G_VIEWER_SHARED_STATE.get() {
            let mut lock = state.lock().unwrap();
            let scene = lock.user_scene_mut();
            scene.clear_geom();

            // Draw trace
            self.visualizer.render_trace(scene, self.visual_config.trace_ball, self.visual_config.trace_rod_mask);
        }
    }

    /// Applies either externally set motor commands ([`FuzbAISimulator::external_team_red`] = `true`) or fetches
    /// the commands from the build-in opponent.
    fn apply_motor_commands(&mut self) {
        let mut obs;
        let mut act_id;
        let mut target_angle;
        let mut target_trans;

        // Red team
        if !self.external_team_red {
            obs = self.delayed_observation(PlayerTeam::Red, None);
            let (x, y, vx, vy, ..) = obs;
            self.pending_motor_cmd_red = self.red_builtin_player.get_action(x, y, vx, vy);
        }

        for command in &self.pending_motor_cmd_red {
            act_id = RED_INDICES[command.0 - 1];
            target_trans = (1.0 - command.1) * ROD_TRAVELS[act_id];
            target_angle = 2.0 * std::f64::consts::PI * command.2;
            self.trans_motor_ctrl.set_target(act_id, target_trans);
            self.rot_motor_ctrl.set_target(act_id, target_angle);
        }
        self.pending_motor_cmd_red.clear();

        // Blue team
        if !self.external_team_blue {
            obs = self.delayed_observation(PlayerTeam::Blue, None);
            let (x, y, vx, vy, ..) = obs;
            self.pending_motor_cmd_blue = self.blue_builtin_player.get_action(x, y, vx, vy);
        }

        for command in &self.pending_motor_cmd_blue {
            act_id = BLUE_INDICES[command.0 - 1];
            target_trans = command.1 * ROD_TRAVELS[act_id];
            target_angle = -2.0 * std::f64::consts::PI * command.2;
            self.trans_motor_ctrl.set_target(act_id, target_trans);
            self.rot_motor_ctrl.set_target(act_id, target_angle);
        }
        self.pending_motor_cmd_blue.clear();
    }

    /// Stores the current state into the delayed state buffer
    fn sample_state(&mut self) {
        let obs = self.observation(PlayerTeam::Red);
        if self.delayed_memory.len() >= MAX_DELAY_BUFFER_LEN {
            self.delayed_memory.pop_back();
        }
        self.delayed_memory.push_front(obs);
    }

    /// Returns the observation relative to the red's team.
    /// This exists for performance reasons, users should use [`observation`](FuzbAISimulator::observation) instead.
    #[inline]
    fn observation_red(&self) -> ObservationType {
        let [mut ball_x, mut ball_y, _, mut ball_vx, mut ball_vy, _] = self.ball_true_state();
        let (mut rod_positions, mut rod_rotations, ..) = self.rods_true_state();

        let dist = Uniform::new(0.0, 1.0).unwrap();
        let mut rng = rand::rng();

        ball_x  += (dist.sample(&mut rng) - 0.5) * 2.0 * BALL_POSITION_NOISE;
        ball_y  += (dist.sample(&mut rng) - 0.5) * 2.0 * BALL_POSITION_NOISE;
        ball_vx += (dist.sample(&mut rng) - 0.5) * 2.0 * BALL_VELOCITY_NOISE;
        ball_vy += (dist.sample(&mut rng) - 0.5) * 2.0 * BALL_VELOCITY_NOISE;

        rod_positions = std::array::from_fn(|i| (rod_positions[i] + (dist.sample(&mut rng) - 0.5) * 2.0 * ROD_TRANS_NOISE).clamp(0.0, 1.0));
        rod_rotations = std::array::from_fn(|i| (rod_rotations[i] + (dist.sample(&mut rng) - 0.5) * 2.0 * ROD_ROT_NOISE).clamp(-64.0, 64.0));

        (ball_x, ball_y, ball_vx, ball_vy, rod_positions, rod_rotations)
    }

    /// Resets the saved collision forces to [0.0, 0.0, 0.0, 0.0] and resets indices to -1.
    fn clear_collisions(&mut self) {
        self.collision_forces.fill([0.0, 0.0, 0.0, 0.0]);
        self.collision_indices.fill(-1);
    }

    /// Read and remember collisions between rods and the ball.
    /// If a collision was detected in the previous call of this method,
    /// the force greater by its absolute value is kept.
    /// Collisions can be cleared by [`clear_collisions`](FuzbAISimulator::clear_collisions).
    fn update_collisions(&mut self) {
        // Obtain a raw copy here as Rust will not allow us to use the data
        // while we are iterating the contacts (borrow checker). 
        for (contact_id, contact) in self.mj_data.contacts().iter().enumerate() {
            let geom_id = contact.geom2 as usize;  // geom1 is the ball geom's ID, geom2 is the other geom in contact
            let frame;
            let fx;
            let fy;
            let fz;
            let fx_no_z;
            let current_max;
            if geom_id >= GEOM_TO_ROD_MAPPING.len() {
                continue;
            }

            let rod_id = GEOM_TO_ROD_MAPPING[geom_id];
            if rod_id == -1 {
                continue;
            }

            let force = self.mj_data.contact_force(contact_id);
            frame = contact.frame;
            fx = -frame[0] * force[0];
            fy = -frame[1] * force[0];
            fz = -frame[2] * force[0];
            fx_no_z = fx + fz.max(-fx).min(0.0);
            current_max = &self.collision_forces[rod_id as usize];  // current max force
            if f64::sqrt(fx_no_z.powi(2) + fy.powi(2)) > f64::sqrt(current_max[3].powi(2) + current_max[1].powi(2)) {
                self.collision_forces[rod_id as usize] = [fx, fy, fz, fx_no_z];
                self.collision_indices[rod_id as usize] = geom_id as isize;
            }
            break;
        }
    }
}


/// Viewer proxy for controlling the viewer
/// in an object-oriented fashion.
/// It holds no storage as use of more than one viewer is not considered, nor
/// does it make sense in any way.
#[cfg_attr(feature = "python-bindings", pyclass(module = "fuzbai_simulator"))]
pub struct ViewerProxy;

#[cfg_attr(feature = "python-bindings", pymethods)]
impl ViewerProxy {
    #[cfg(feature = "python-bindings")]
    #[new]
    fn py_new() -> Self {
        Self
    }

    pub fn running(&self) -> bool {
        if let Some(state) = G_VIEWER_SHARED_STATE.get() && state.lock().unwrap().running() {
            true
        }
        else {
            false
        }
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "render")]
    fn py_render(&self, py: Python<'_>) {
        py.allow_threads(|| {
            self.render();
        })
    }

    #[cfg(feature = "python-bindings")]
    #[pyo3(name = "render_loop")]
    fn py_render_loop(&self, py: Python<'_>) {
        py.allow_threads(|| {
            self.render_loop();
        })
    }
}

/// Rust-only methods.
impl ViewerProxy {
    pub fn render(&self) {
        G_MJ_VIEWER.with_borrow_mut(|maybe_viewer| {
            if let Some(viewer) = maybe_viewer {
                viewer.render();
            }
        })
    }

    pub fn render_loop(&self) {
        G_MJ_VIEWER.with_borrow_mut(|maybe_viewer| {
            if let Some(viewer) = maybe_viewer {
                while viewer.running() {
                    viewer.render();
                }
            }
        })
    }

    pub fn add_ui_callback<F>(&self, callback: F)
    where
        F: FnMut(&egui::Context, &mut MjData<&MjModel>) + 'static
    {
        G_MJ_VIEWER.with_borrow_mut(|maybe_viewer| {
            if let Some(viewer) = maybe_viewer {
                viewer.add_ui_callback(callback);
            }
        });
    }
}


#[cfg(feature = "python-bindings")]
#[pymodule]
/// Python module definition
fn fuzbai_simulator(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<FuzbAISimulator>()?;
    m.add_class::<PlayerTeam>()?;
    m.add_class::<VisualConfig>()?;
    m.add_class::<ViewerProxy>()?;
    m.add("RED_INDICES", RED_INDICES)?;
    m.add("BLUE_INDICES", BLUE_INDICES)?;
    Ok(())
}
