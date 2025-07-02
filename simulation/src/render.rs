//! Rendering definitions
use std::collections::VecDeque;
use mujoco_rs::wrappers::*;
use mujoco_rs::mujoco_c::*;
use crate::constant::*;
use std::ffi::CString;
use crate::types::*;
use core::f64;
use std::ptr;
use glfw;


/// Struct encapsulating screenshot required functionality.
pub struct Render {
    width: usize,
    height: usize,
    scene: MjvScene,
    scene_opt: MjvOption,
    rect: MjRectangle,
    ctx: MjContext,
    window: *mut glfw::ffi::GLFWwindow,
    owns_glfw: bool
}


impl Render {
    pub fn new(model: &MjModel, width: usize, height: usize, max_geom: usize) -> Self {
        let scene = MjvScene::new(model, max_geom);
        let mut ctx;
        let window;
        let mut options = MjvOption::default();
        let owns_glfw;

        unsafe {
            mjv_defaultOption(&mut options);
            let prev_ctx = glfw::ffi::glfwGetCurrentContext();
            if prev_ctx.is_null() {
                glfw::ffi::glfwInit();  // TODO: refactor simulate.cc to accept glfw separately
                owns_glfw = true;
            }
            else {
                owns_glfw = false;
            }

            glfw::ffi::glfwWindowHint(glfw::ffi::VISIBLE, 0);
            window = glfw::ffi::glfwCreateWindow(
                width as i32, height as i32, CString::new("empty window").unwrap().as_ptr(),
                ptr::null_mut(), ptr::null_mut()
            );

            glfw::ffi::glfwMakeContextCurrent(window);
            ctx = MjContext::new(model);
            mjr_setBuffer(mjtFramebuffer__mjFB_OFFSCREEN as i32, ctx.raw_mut());
        }

        Self {
            width, height,
            scene, scene_opt: options,
            rect: MjRectangle {left: 0, bottom: 0, width: width as i32, height: height as i32},
            ctx: ctx,
            window,
            owns_glfw
        }
    }

    #[inline]
    pub fn width(&self) -> usize {self.width}

    #[inline]
    pub fn height(&self) -> usize {self.height}

    pub fn scene_mut(&mut self) -> &mut MjvScene {
        &mut self.scene
    }

    pub fn render(&mut self) -> Vec<u16> {
        // [[0; RENDER_MAX_WIDTH]; RENDER_MAX_HEIGHT]
        let mut output = vec![0; self.width * self.height * 3];  // width * height * 3 (red, green, blue)
        
        unsafe {
            glfw::ffi::glfwMakeContextCurrent(self.window);
            let ctx_raw: *const mjrContext_ = self.ctx.raw();
            mjr_render(self.rect, self.scene.raw(), ctx_raw);
            mjr_readPixels(output.as_mut_ptr(), ptr::null_mut(), self.rect, ctx_raw);
        }

        // flip image upside-down
        let row_size = self.width * 3;
        for y in 0..(self.height / 2) {
            let top: usize = y * row_size;
            let bottom: usize = (self.height - 1 - y) * row_size;

            for x in 0..row_size {
                output.swap(top + x, bottom + x);
            }
        }

        output.iter().map(|x| *x as u16).collect()
    }

    pub fn update_scene(&mut self, data: &mut MjData, camera_id: Option<isize>, camera_name: Option<String>) {
        let model_raw = unsafe { data.model().raw() };

        let camera_id = if let Some(name) = camera_name {
            unsafe { mj_name2id(model_raw, mjtObj__mjOBJ_CAMERA as i32, CString::new(name).unwrap().as_ptr()) as isize }
        } else {
            camera_id.unwrap_or(-1)  // free camera
        };


        let mut camera = MjCamera::new(camera_id, data.model());
        unsafe {
            mjv_updateScene(
                model_raw, data.raw_mut(), &self.scene_opt, ptr::null(),
                camera.raw_mut(), mjtCatBit__mjCAT_ALL as i32, self.scene.raw()
            );
        }
    }
}

impl Drop for Render {
    fn drop(&mut self) {
        if self.owns_glfw {
            unsafe { glfw::ffi::glfwTerminate() };
        }
    }
}


/// SAFETY: The entire simulation is not thread-safe by default, responsibility is on the user.
unsafe impl Sync for Render {}
unsafe impl Send for Render {}



/// Records and renders trace of XYZ data. Also allows rendering of ball and rod estimates.
pub struct Visualizer {
    trace_buffer: VecDeque<TraceType>,
    trace_length: usize,
}

impl Visualizer {
    pub fn new(trace_length: usize) -> Self {
        Self {trace_buffer: VecDeque::new(), trace_length}
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

    pub fn render_trace(&mut self, scene: &mut MjvScene, ball_trace: bool, trace_rod_mask: u8) {
        let mut rgba: [f32; 4];
        let mut coeff;
        let mut n_iter = self.trace_buffer.len();
        if n_iter < 2 {
            return;
        }

        n_iter -= 1;
        let raw_scene = unsafe{ scene.raw() };
        assert!(raw_scene.ngeom as usize + (n_iter - 1) * TRACE_GEOM_LEN < raw_scene.maxgeom as usize);
        for (i, (state_prev, state)) in self.trace_buffer.iter()
            .zip(self.trace_buffer.iter().skip(1)).enumerate()
        {
            // Gradient based on the marker age
            // Newer markers will be closer to TRACE_RGBA_END.
            coeff = i as f32 / n_iter as f32;
            rgba = std::array::from_fn(|idx| TRACE_RGBA_START[idx] + coeff * TRACE_RGBA_DIFF[idx]);

            // Render the trace of ball positions
            if ball_trace {
                unsafe {
                    mujoco_rs::mujoco_c::mjv_initGeom(
                        raw_scene.geoms.add(raw_scene.ngeom as usize),
                        mjtGeom__mjGEOM_CAPSULE as i32,
                        [0.0;3].as_ptr(), [0.0;3].as_ptr(), [0.0;9].as_ptr(),
                        rgba.as_ptr()
                    );
                    // Position and orient the capsule in such way that it connects the previous and current ball position
                    mujoco_rs::mujoco_c::mjv_connector(
                        raw_scene.geoms.add(raw_scene.ngeom as usize),
                        mjtGeom__mjGEOM_CAPSULE as i32,
                        TRACE_RADIUS, state_prev.0.as_ptr(), state.0.as_ptr()
                    );
                    raw_scene.ngeom += 1
                }
            }

            // Render rod geoms
            for (rod_i, (((tp, rp), t), r)) in state_prev.1.iter().zip(state_prev.2)
                .zip(state.1).zip(state.2).enumerate()
            {
                if trace_rod_mask & (1 << rod_i) > 0 {
                    // Draw the trace for the middle player only.
                    let trace_offset = if ROD_N_PLAYERS[rod_i] % 2 == 0 {
                        // Take the mean between max and min player position.
                        (ROD_N_PLAYERS[rod_i] as f64 - 1.0) * ROD_SPACING[rod_i] / 2.0
                    }
                    else {
                        (ROD_N_PLAYERS[rod_i] / 2).min(ROD_N_PLAYERS[rod_i]) as f64 * ROD_SPACING[rod_i]
                    };

                    let pos0 = [
                        ROD_POSITIONS[rod_i][0] + ROD_ESTIMATE_FRAME_LOWER_OFFSET * rp.sin(),
                        ROD_POSITIONS[rod_i][1] + ROD_FIRST_OFFSET[rod_i] + ROD_TRAVELS[rod_i] * (1.0 - tp) +
                            trace_offset,
                        ROD_POSITIONS[rod_i][2] + ROD_ESTIMATE_FRAME_LOWER_OFFSET * rp.cos(),
                    ];
                    let pos1 = [
                        ROD_POSITIONS[rod_i][0] + ROD_ESTIMATE_FRAME_LOWER_OFFSET * r.sin(),
                        ROD_POSITIONS[rod_i][1] + ROD_FIRST_OFFSET[rod_i] + ROD_TRAVELS[rod_i] * (1.0 - t) +
                            trace_offset,
                        ROD_POSITIONS[rod_i][2] + ROD_ESTIMATE_FRAME_LOWER_OFFSET * r.cos(),
                    ];

                    unsafe {
                        mujoco_rs::mujoco_c::mjv_initGeom(
                            raw_scene.geoms.add(raw_scene.ngeom as usize),
                            mjtGeom__mjGEOM_CAPSULE as i32,
                            [0.0;3].as_ptr(), [0.0;3].as_ptr(), [0.0;9].as_ptr(),
                            rgba.as_ptr()
                        );
                        // Position and orient the capsule in such way that it connects
                        // the previous and current ball position
                        mujoco_rs::mujoco_c::mjv_connector(
                            raw_scene.geoms.add(raw_scene.ngeom as usize),
                            mjtGeom__mjGEOM_CAPSULE as i32, TRACE_RADIUS,
                            pos0.as_ptr(), pos1.as_ptr()
                        );
                        raw_scene.ngeom += 1
                    }
                }
            }
        }       
    }

    /// Renders to `scene` the `position` as the ball's estimate.
    #[inline]
    pub fn render_ball_estimate(scene: &mut MjvScene, position: &XYZType, color: Option<RGBAType>) {
        let color = color.unwrap_or(DEFAULT_BALL_ESTIMATE_RGBA);
        unsafe {
            let raw_scene = scene.raw();
            assert!(raw_scene.ngeom < raw_scene.maxgeom);
            mujoco_rs::mujoco_c::mjv_initGeom(
                raw_scene.geoms.add(raw_scene.ngeom as usize),
                mjtGeom__mjGEOM_SPHERE as i32,
                &BALL_RADIUS_M,
                position.as_ptr(),
                std::ptr::null(),
                color.as_ptr()
            );
            raw_scene.ngeom += 1;
        }
    }

    #[inline]
    pub fn render_rods_estimates(scene: &mut MjvScene, pos_rot: &[(usize, f64, f64)], color: Option<RGBAType>) {
        let raw_scene = unsafe { scene.raw() };
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
        for (i, t, r) in pos_rot.iter() {
            first_position = ROD_TRAVELS[*i] * (1.0 - t) + ROD_FIRST_OFFSET[*i];
            pos_xyz = ROD_POSITIONS[*i];
            unsafe {
                mju_axisAngle2Quat(quat.as_mut_ptr(), [0.0, 1.0, 0.0].as_ptr(), r * f64::consts::PI / 32.0);
                mju_quat2Mat(mat.as_mut_ptr(), quat.as_ptr());
            }

            for p in 0..ROD_N_PLAYERS[*i] {
                unsafe {
                    dy = first_position + ROD_SPACING[*i] * p as f64;
                    pos_trans = pos_xyz;
                    pos_trans[1] += dy;
                    vgeom = raw_scene.geoms.add(raw_scene.ngeom as usize).as_mut().unwrap();

                    // Rotation will affect the geom relative to it's MuJoCo geom coordinate system, however
                    // we want to rotate around the actual rod. We offset the geom away from the rod exactly
                    // ``ROD_ESTIMATE_FRAME_UPPER_OFFSET`` in the rotated direction.
                    offset_xyz = [0.0, 0.0, ROD_ESTIMATE_FRAME_UPPER_OFFSET];
                    mju_mulMatVec3(offset_xyz.as_mut_ptr(), mat.as_ptr(), offset_xyz.as_ptr());
                    pos_trans = std::array::from_fn(|i| pos_trans[i] + offset_xyz[i]);
                    mjv_initGeom(vgeom, mjtGeom__mjGEOM_MESH as i32, ptr::null(),
                                 pos_trans.as_ptr(), mat.as_ptr(), color.as_ptr());

                    // According to mujoco/src/engine/engine_vis_visualize.c, actual data id is twice the mesh id...
                    // ! Good thing that this isn't documented anywhere in the MuJoCo docs !.
                    vgeom.dataid =  ROD_MESH_UPPER_PLAYER_ID * 2;
                    raw_scene.ngeom += 1;

                    // Same for the bottom geom of the player
                    pos_trans = pos_xyz;
                    pos_trans[1] += dy;
                    vgeom = raw_scene.geoms.add(raw_scene.ngeom as usize).as_mut().unwrap();
                    mat_bottom = [0.0; 9];

                    // Rotate the bottom part 90 degrees around x first, then apply the joint rotation
                    mju_mulMatMat(mat_bottom.as_mut_ptr(), mat.as_ptr(), ROD_BOTTOM_PRE_ROTATION_MAT.as_ptr(), 3, 3, 3);

                    offset_xyz = [0.0, 0.0, ROD_ESTIMATE_FRAME_LOWER_OFFSET];
                    mju_mulMatVec3(offset_xyz.as_mut_ptr(), mat.as_ptr(), offset_xyz.as_ptr());
                    pos_trans = std::array::from_fn(|i| pos_trans[i] + offset_xyz[i]);
                    mjv_initGeom(vgeom, mjtGeom__mjGEOM_MESH as i32, ptr::null(),
                                 pos_trans.as_ptr(), mat_bottom.as_ptr(), color.as_ptr());                        
                    vgeom.dataid =  ROD_MESH_LOWER_PLAYER_ID * 2;
                    raw_scene.ngeom += 1;
                }
            }
        }
    }
}
