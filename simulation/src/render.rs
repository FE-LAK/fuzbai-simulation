//! Rendering definitions
use std::collections::VecDeque;
use std::marker::PhantomData;
use std::ops::Deref;
use mujoco_rs::wrappers::*;
use mujoco_rs::mujoco_c::*;
use crate::constant::*;
use crate::types::*;
use core::f64;


/// Records and renders trace of XYZ data. Also allows rendering of ball and rod estimates.
pub struct Visualizer<M: Deref<Target = MjModel>> {
    trace_buffer: VecDeque<TraceType>,
    trace_length: usize,
    phantom: PhantomData<M>
}

impl<M: Deref<Target = MjModel>> Visualizer<M> {
    pub fn new(trace_length: usize) -> Self {
        Self {trace_buffer: VecDeque::new(), trace_length, phantom: PhantomData}
    }

    #[inline]
    pub fn sample_trace(&mut self, positions: TraceType) {
        if self.trace_buffer.len() >= self.trace_length {
            self.trace_buffer.pop_front();
        }
        self.trace_buffer.push_back(positions);
    }

    #[inline]
    pub fn clear_trace(&mut self) {
        self.trace_buffer.clear();
    }

    pub fn render_trace(&mut self, scene: &mut MjvScene<M>, ball_trace: bool, trace_rod_mask: u64) {
        let mut ball_rgba: [f32; 4];
        let mut rod_rgba: [f32; 4];
        let mut coeff;
        let mut n_iter = self.trace_buffer.len();
        if n_iter < 2 {
            return;
        }

        n_iter -= 1;

        for (i, (state_prev, state)) in self.trace_buffer.iter()
            .zip(self.trace_buffer.iter().skip(1)).enumerate()
        {
            // Gradient based on the marker age
            // Newer markers will be closer to TRACE_RGBA_END.
            coeff = i as f32 / n_iter as f32;

            // Render the trace of ball positions
            if ball_trace {
                ball_rgba = std::array::from_fn(
                    |idx| BALL_TRACE_RGBA_START[idx] + coeff * BALL_TRACE_RGBA_DIFF[idx]
                );

                // Position and orient the capsule in such way that it
                // connects the previous and current ball position
                scene.create_geom(
                    mjtGeom::mjGEOM_CAPSULE, None, None,
                    None, Some(ball_rgba)
                ).connect(BALL_TRACE_RADIUS, state_prev.0, state.0);
            }

            // Render rod geoms
            rod_rgba = std::array::from_fn(|idx|
                ROD_TRACE_RGBA_START[idx] + coeff * ROD_TRACE_RGBA_DIFF[idx]
            );
            Self::render_rods_estimates(
                scene,                   // Extract the mask indicating which players to draw for the specific rod.
                state.1.into_iter().zip(state.2).enumerate().map(|(rod_i, (t, r))|
                    (rod_i, t, r, ((trace_rod_mask >> rod_i * 8) & 0xFF) as u8)
                ),
                Some(rod_rgba)
            );
        }       
    }

    /// Renders to `scene` the `position` as the ball's estimate.
    pub fn render_ball_estimate(scene: &mut MjvScene<M>, position: &XYZType, color: Option<RGBAType>) {
        let color = color.unwrap_or(DEFAULT_BALL_ESTIMATE_RGBA);
        let position_global = [
            (position[0] + 115.0) / 1000.0,
            (727.0 - position[1]) / 1000.0,
            position[2] / 1000.0 + Z_FIELD
        ];

        scene.create_geom(
            mjtGeom::mjGEOM_SPHERE, Some([BALL_RADIUS_M, 0.0, 0.0]),
            Some(position_global), None, Some(color)
        );        
    }

    pub fn render_rods_estimates<T>(scene: &mut MjvScene<M>, pos_rot: T, color: Option<RGBAType>)
        where T: IntoIterator<Item=(usize, f64, f64, u8)>
    {
        let mut first_position;
        let mut pos_xyz: [f64; 3];
        let mut pos_trans;
        let mut quat = [0.0; 4];
        let mut mat = [0.0; 9];
        let mut mat_bottom;
        let mut offset_xyz;
        let mut vgeom;
        let mut dy;

        let color = color.unwrap_or(DEFAULT_ROD_ESTIMATE_RGBA);
        for (i, t, r, player_mask) in pos_rot {
            first_position = ROD_TRAVELS[i] * (1.0 - t) + ROD_FIRST_OFFSET[i];
            pos_xyz = ROD_POSITIONS[i];
            unsafe {
                mju_axisAngle2Quat(
                    quat.as_mut_ptr(),
                    [0.0, 1.0, 0.0].as_ptr(),
                    r * f64::consts::PI / 32.0
                );
                mju_quat2Mat(mat.as_mut_ptr(), quat.as_ptr());
            }

            for p in 0..ROD_N_PLAYERS[i] {
                // P'th player is not enabled for drawing.
                // p is subtracted from the maximum index because player_mask is in
                // red's local coordinate system.
                if (1 << (ROD_N_PLAYERS[i] - p - 1)) & player_mask == 0 {
                    continue;
                }

                dy = first_position + ROD_SPACING[i] * p as f64;
                pos_trans = pos_xyz;
                pos_trans[1] += dy;

                // Rotation will affect the geom relative to it's MuJoCo geom coordinate system, however
                // we want to rotate around the actual rod. We offset the geom away from the rod exactly
                // ``ROD_ESTIMATE_FRAME_UPPER_OFFSET`` in the rotated direction.
                offset_xyz = [0.0, 0.0, ROD_ESTIMATE_FRAME_UPPER_OFFSET];
                unsafe {mju_mulMatVec3(offset_xyz.as_mut_ptr(), mat.as_ptr(), offset_xyz.as_ptr())};
                pos_trans = std::array::from_fn(|i| pos_trans[i] + offset_xyz[i]);

                vgeom = scene.create_geom(
                    mjtGeom::mjGEOM_MESH, None, Some(pos_trans),
                    Some(mat), Some(color)
                );

                // According to mujoco/src/engine/engine_vis_visualize.c, actual data id is twice the mesh id...
                // ! Good thing that this isn't documented anywhere in the MuJoCo docs !.
                vgeom.dataid = ROD_MESH_UPPER_PLAYER_ID * 2;

                // Same for the bottom geom of the player
                pos_trans = pos_xyz;
                pos_trans[1] += dy;
                mat_bottom = [0.0; 9];

                // Rotate the bottom part 90 degrees around x first, then apply the joint rotation
                unsafe {mju_mulMatMat(
                    mat_bottom.as_mut_ptr(),
                    mat.as_ptr(), ROD_BOTTOM_PRE_ROTATION_MAT.as_ptr(),
                    3, 3, 3
                )};

                offset_xyz = [0.0, 0.0, ROD_ESTIMATE_FRAME_LOWER_OFFSET];
                unsafe{mju_mulMatVec3(offset_xyz.as_mut_ptr(), mat.as_ptr(), offset_xyz.as_ptr())};

                pos_trans = std::array::from_fn(|i| pos_trans[i] + offset_xyz[i]);
                vgeom = scene.create_geom(
                    mjtGeom::mjGEOM_MESH, None, Some(pos_trans),
                    Some(mat_bottom), Some(color)
                );
                vgeom.dataid = ROD_MESH_LOWER_PLAYER_ID * 2;
            }
        }
    }
}
