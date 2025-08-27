//! Implementation of the FuzbAI's motor system
use mujoco_rs::wrappers::*;
use crate::constant::LOW_TIMESTEP;


pub(crate) struct TrapezoidMotorSystem {
    kp: [f64; 8],
    kd: [f64; 8],
    max_velocity: [f64; 8],
    max_acceleration: [f64; 8],

    joint_info: [MjJointInfo; 8],
    actuator_info: [MjActuatorInfo; 8],

    // State
    target_velocity: [f64; 8],

    // Control
    target_pos: [f64; 8],

    // Stability
    direction: [f64; 8],
    stop_threshold: f64,
    dead_band: [f64; 8]
}

impl TrapezoidMotorSystem {
    pub fn new(
        kp: [f64; 8], kd: [f64; 8], max_velocity: [f64; 8], max_acceleration: [f64; 8],
        stop_threshold: f64, dead_band: [f64; 8],
        joint_info: [MjJointInfo; 8],
        actuator_info: [MjActuatorInfo; 8]
    ) -> Self {
        let target_velocity: [f64; 8] = [0.0; 8];
        let target_pos = [0.0; 8];
        let direction = [0.0; 8];
        Self {
            kp, kd, max_velocity, max_acceleration, joint_info, actuator_info,
            target_velocity, target_pos, direction, stop_threshold, dead_band
        }
    }

    pub fn set_params(&mut self, act_id: usize, kp: f64, kd: f64, max_vel: f64, max_acc: f64) {
        self.kp[act_id] = kp;
        self.kd[act_id] = kd;
        self.max_velocity[act_id] = max_vel;
        self.max_acceleration[act_id] = max_acc;
    }

    /// Updates the actuator states and the output torque
    pub fn step(&mut self, data: &mut MjData) {
        for act_id in 0..8 {
            let joint_view = self.joint_info[act_id].view_mut(data);
            let qpos = joint_view.qpos[0];
            let qvel = joint_view.qvel[0];
            let grav_comp = joint_view.qfrc_bias[0];

            let pos_error = self.target_pos[act_id] - qpos;
            let stop_dist = self.target_velocity[act_id] * self.target_velocity[act_id] / (2.0 * self.max_acceleration[act_id]);

            // Accelerate with maximum acceleration if not below stopping distance.
            // When at stopping distance, decelerate with maximum -acceleration.
            let d_velocity = if pos_error.abs() > stop_dist {
                self.max_acceleration[act_id] * LOW_TIMESTEP * pos_error.signum()
            }
            else {
                -self.max_acceleration[act_id] * LOW_TIMESTEP * self.target_velocity[act_id].signum()
            };

            self.target_velocity[act_id] += d_velocity;
            self.target_velocity[act_id] = self.target_velocity[act_id].clamp(-self.max_velocity[act_id], self.max_velocity[act_id]);
            let vel_error: f64 = self.target_velocity[act_id] - qvel;
            let tanh_arg = (pos_error * 3.0 / self.stop_threshold).abs();

            // performance tuning
            let scale = if tanh_arg < 3.0 {
                f64::tanh(tanh_arg).powf(0.5)
            } else {
                1.0
            };

            let mut act_view = self.actuator_info[act_id].view_mut(data);
            act_view.ctrl[0] = scale * self.kp[act_id] * vel_error - self.kd[act_id] * qvel + grav_comp;
        }
    }

    /// Sets a new position target.
    #[inline]
    pub fn set_target(&mut self, act_id: usize, target: f64) {
        let diff = target - self.target_pos[act_id];
        let new_direction = diff.signum();
        // We are outside the dead-zone or we are moving in the same direction
        if diff.abs() > self.dead_band[act_id] || new_direction == self.direction[act_id] {
            self.target_pos[act_id] = target;
            self.direction[act_id] = new_direction;
        }
    }

    #[inline]
    pub fn set_qpos(&mut self, data: &mut MjData, act_id: usize, value: f64) {
        self.joint_info[act_id].view_mut(data).qpos[0] = value;
    }

    #[inline]
    #[allow(unused)]
    pub fn set_qvel(&mut self, data: &mut MjData, act_id: usize, value: f64) {
        self.joint_info[act_id].view_mut(data).qvel[0] = value;
    }

    #[inline]
    pub fn qpos(&self, data: &MjData, act_id: usize) -> f64 {
        self.joint_info[act_id].view(data).qpos[0]
    }

    #[inline]
    pub fn qvel(&self, data: &MjData, act_id: usize) -> f64 {
        self.joint_info[act_id].view(data).qvel[0]
    }

    /// Forcefully stops the actuator.
    #[inline]
    pub fn force_stop(&mut self, act_id: usize, data: &mut MjData) {
        let mut joint_view = self.joint_info[act_id].view_mut(data);
        joint_view.qvel[0] = 0.0;  // actual stop
        self.target_velocity[act_id] = 0.0;
        self.target_pos[act_id] = joint_view.qpos[0];
        self.actuator_info[act_id].view_mut(data).ctrl[0] = 0.0;
    }

    /// Check if the motor can stop exactly at the reference position with maximum
    /// deceleration. If that is not possible, forcefully stop the motor at the current position.
    #[inline]
    #[allow(unused)]
    pub fn check_reference(&mut self, data: &mut MjData) {
        for act_id in 0..8 {
            let qvel = self.qvel(data, act_id);
            let stop_dist = qvel * qvel / (2.0 * self.max_acceleration[act_id]);
            let qvel_abs = qvel.abs();
            if (self.target_pos[act_id] - self.qpos(data, act_id)).abs() < stop_dist && qvel_abs < 0.5 && qvel_abs > 0.2
            {
                self.force_stop(act_id, data);
            }
        }
    }

}
