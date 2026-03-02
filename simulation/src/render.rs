//! Rendering definitions
use mujoco_rs::wrappers::fun::*;
use mujoco_rs::wrappers::*;

use std::collections::VecDeque;
use std::marker::PhantomData;
use std::borrow::Borrow;
use std::ops::Deref;
use core::f64;

use crate::constant::*;
use crate::types::*;


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

                // Position and orient the capsule in such a way that it
                // connects the previous and current ball position
                scene.create_geom(
                    MjtGeom::mjGEOM_CAPSULE, None, None,
                    None, Some(ball_rgba)
                ).connect(BALL_TRACE_RADIUS, state_prev.0, state.0);
            }

            // Render rod geoms
            rod_rgba = std::array::from_fn(|idx|
                ROD_TRACE_RGBA_START[idx] + coeff * ROD_TRACE_RGBA_DIFF[idx]
            );
            Self::render_rods_estimates(
                scene,
                // Extract the mask indicating which players to draw for the specific rod.
                state.1.into_iter().zip(state.2).enumerate().map(|(rod_i, (t, r))|
                    (rod_i, t, r, ((trace_rod_mask >> rod_i * 8) & 0xFF) as u8)
                ),
                Some(rod_rgba)
            );
        }       
    }

    /// Renders a single ball configuration zone to `scene` as a flat, semi-transparent box.
    ///
    /// The zone is specified as `(x_min, x_max, y_min, y_max)` in the **red team's local
    /// coordinate system** (x and y in millimeters). The box is drawn at the table field surface
    /// height ([`Z_FIELD`]).
    ///
    /// The optional `color` overrides the default [`DEFAULT_BALL_CONFIG_ZONE_RGBA`].
    /// The optional `label` is rendered as a 3D text label at the zone's center.
    pub fn render_zone(
        scene: &mut MjvScene<M>,
        zone: (f64, f64, f64, f64),
        color: Option<RGBAType>,
        label: Option<&str>,
    ) {
        let (x_min, x_max, y_min, y_max) = zone;
        let color = color.unwrap_or(DEFAULT_BALL_CONFIG_ZONE_RGBA);

        // Compute center in local mm and convert to global m via the same formula as local_to_global_position.
        let center_local = [(x_min + x_max) / 2.0, (y_min + y_max) / 2.0, 0.0];
        let pos = [
            (center_local[0] + 115.0) / 1000.0,
            (727.0 - center_local[1]) / 1000.0,
            Z_FIELD,
        ];

        // MuJoCo box size is half-extents in each axis.
        let size = [
            (x_max - x_min) / 2000.0,
            (y_max - y_min) / 2000.0,
            BALL_CONFIG_ZONE_HEIGHT / 2.0,
        ];

        let vgeom = scene.create_geom(
            MjtGeom::mjGEOM_BOX, Some(size), Some(pos), None, Some(color)
        );
        if let Some(text) = label {
            vgeom.set_label(text);
        }
    }

    /// Renders a ball configuration zone as a bordered rectangle (4 thin bars forming the outline)
    /// plus an almost-invisible fill used only to anchor the text `label`.
    ///
    /// Unlike [`render_zone`], overlapping zones remain visually distinguishable because only
    /// the border is translucent - the interior stays transparent. This method requires
    /// 5 user-geom slots in the scene (1 fill + 4 border bars).
    ///
    /// `border_mm` - border bar thickness in local millimetre.  
    /// `z_offset_m` - raises all geoms above [`Z_FIELD`] by the given metres. Use a larger value
    ///   for inner zones so a slightly angled camera reveals them floating above outer zones.
    /// `label_x_offset_mm` - shift the label's x position relative to the zone centre (local mm).
    pub fn render_zone_outline(
        scene: &mut MjvScene<M>,
        zone: (f64, f64, f64, f64),
        color: Option<RGBAType>,
        label: Option<&str>,
        border_mm: f64,
        z_offset_m: f64,
        label_x_offset_mm: f64,
    ) {
        let (x_min, x_max, y_min, y_max) = zone;
        let color = color.unwrap_or(DEFAULT_BALL_CONFIG_ZONE_RGBA);
        let hz = BALL_CONFIG_ZONE_HEIGHT / 2.0;
        let z  = Z_FIELD + z_offset_m;

        let to_global = |cx_mm: f64, cy_mm: f64| -> [f64; 3] {[
            (cx_mm + 115.0) / 1000.0,
            (727.0 - cy_mm) / 1000.0,
            z,
        ]};

        // Ghost fill - barely visible, anchors the label

        let fill_color = [color[0], color[1], color[2], 0.08f32];
        let fill_pos  = to_global((x_min + x_max) / 2.0 + label_x_offset_mm, (y_min + y_max) / 2.0);
        let fill_size = [(x_max - x_min) / 2000.0, (y_max - y_min) / 2000.0, hz];
        let fill_geom = scene.create_geom(
            MjtGeom::mjGEOM_BOX, Some(fill_size), Some(fill_pos), None, Some(fill_color)
        );
        if let Some(text) = label {
            fill_geom.set_label(text);
        }

        // Border bars (caller's alpha allows overlapping borders to blend)

        let border_color = color;
        let bh  = border_mm / 2.0;
        let hx  = (x_max - x_min) / 2.0;
        let cx  = (x_min + x_max) / 2.0;
        let cy  = (y_min + y_max) / 2.0;

        scene.create_geom(MjtGeom::mjGEOM_BOX,
            Some([hx / 1000.0, bh / 1000.0, hz]),
            Some(to_global(cx, y_min + bh)), None, Some(border_color));

        scene.create_geom(MjtGeom::mjGEOM_BOX,
            Some([hx / 1000.0, bh / 1000.0, hz]),
            Some(to_global(cx, y_max - bh)), None, Some(border_color));

        scene.create_geom(MjtGeom::mjGEOM_BOX,
            Some([bh / 1000.0, (y_max - y_min) / 2000.0, hz]),
            Some(to_global(x_min + bh, cy)), None, Some(border_color));

        scene.create_geom(MjtGeom::mjGEOM_BOX,
            Some([bh / 1000.0, (y_max - y_min) / 2000.0, hz]),
            Some(to_global(x_max - bh, cy)), None, Some(border_color));
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
            MjtGeom::mjGEOM_SPHERE, Some([BALL_RADIUS_M, 0.0, 0.0]),
            Some(position_global), None, Some(color)
        );        
    }

    pub fn render_rods_estimates<T>(scene: &mut MjvScene<M>, pos_rot: T, color: Option<RGBAType>)
        where T: IntoIterator, T::Item: Borrow<(usize, f64, f64, u8)>,
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
        for item in pos_rot {
            let (i, t, r, player_mask) = item.borrow();
            first_position = ROD_TRAVELS[*i] * (1.0 - t) + ROD_FIRST_OFFSET[*i];
            pos_xyz = ROD_POSITIONS[*i];

            // Calculate the rotation matrix based on the observed angle.
            mju_axis_angle_2_quat(&mut quat, &[0.0, 1.0, 0.0], r * f64::consts::PI / 32.0);
            mju_quat_2_mat(&mut mat, &quat);

            // Render for configured players on each rod.
            for p in 0..ROD_N_PLAYERS[*i] {
                // P'th player is not enabled for drawing.
                // p is subtracted from the maximum index because player_mask is in
                // red's local coordinate system.
                if (1 << (ROD_N_PLAYERS[*i] - p - 1)) & player_mask == 0 {
                    continue;
                }

                dy = first_position + ROD_SPACING[*i] * p as f64;
                pos_trans = pos_xyz;
                pos_trans[1] += dy;

                // Rotation will affect the geom relative to its MuJoCo geom coordinate system, however
                // we want to rotate around the actual rod. We offset the geom away from the rod exactly
                // `ROD_ESTIMATE_FRAME_UPPER_OFFSET` in the rotated direction.
                offset_xyz = [0.0; 3];
                mju_mul_mat_vec_3(&mut offset_xyz, &mat, &[0.0, 0.0, ROD_ESTIMATE_FRAME_UPPER_OFFSET]);
                pos_trans = std::array::from_fn(|i| pos_trans[i] + offset_xyz[i]);

                vgeom = scene.create_geom(
                    MjtGeom::mjGEOM_MESH, None, Some(pos_trans),
                    Some(mat), Some(color)
                );

                // MuJoCo's data id is twice the mesh id (from mujoco/src/engine/engine_vis_visualize.c).
                vgeom.dataid = ROD_MESH_UPPER_PLAYER_ID * 2;

                // Same for the bottom geom of the player
                pos_trans = pos_xyz;
                pos_trans[1] += dy;
                mat_bottom = [0.0; 9];

                // Rotate the bottom part 90 degrees around x first, then apply the joint rotation
                mju_mul_mat_mat(
                    &mut mat_bottom,
                    &mat, &ROD_BOTTOM_PRE_ROTATION_MAT,
                    3, 3, 3
                );

                offset_xyz = [0.0; 3];
                mju_mul_mat_vec_3(&mut offset_xyz, &mat, &[0.0, 0.0, ROD_ESTIMATE_FRAME_LOWER_OFFSET]);
                pos_trans = std::array::from_fn(|i| pos_trans[i] + offset_xyz[i]);
                vgeom = scene.create_geom(
                    MjtGeom::mjGEOM_MESH, None, Some(pos_trans),
                    Some(mat_bottom), Some(color)
                );
                vgeom.dataid = ROD_MESH_LOWER_PLAYER_ID * 2;
            }
        }
    }
}
