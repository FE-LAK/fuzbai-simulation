//! Rendering definitions
use std::collections::VecDeque;
use mujoco_rs::wrappers::*;
use mujoco_rs::mujoco_c::*;
use crate::constant::*;
use std::ffi::CString;
use crate::types::*;
use core::f64;
use std::ptr;


/// Struct encapsulating screenshot required functionality.
pub struct Render {
    width: usize,
    height: usize,
    scene: MjvScene,
    scene_opt: MjvOption,
    rect: MjrRectangle,
    ctx: MjrContext,
    window: *mut glfw::ffi::GLFWwindow,
    owns_glfw: bool
}

// We only use the raw pointer to check for previous OpenGL contexts to prevent
// destroying OpenGL data when the C++ code uses it. This is technically not thread-safe
// as OpenGL doesn't even allow usage outside the main thread (it seems like they tried to make the library unusable lol).
unsafe impl Send for Render {}
unsafe impl Sync for Render {}

impl Render {
    pub fn new(model: &MjModel, width: usize, height: usize, max_geom: usize) -> Self {
        let scene = MjvScene::new(model, max_geom);
        let mut ctx;
        let window;
        let options = MjvOption::default();
        let owns_glfw;

        unsafe {
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
            ctx = MjrContext::new(model);
            ctx.offscreen();
        }

        Self {
            width, height,
            scene, scene_opt: options,
            rect: MjrRectangle {left: 0, bottom: 0, width: width as i32, height: height as i32},
            ctx: ctx,
            window: window,
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
        let mut output;
        
        unsafe {
            glfw::ffi::glfwMakeContextCurrent(self.window);
        }

        output = self.scene.render(&self.rect, &self.ctx);

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
        let camera_id = if let Some(name) = camera_name {
            data.model().name2id(mjtObj__mjOBJ_CAMERA as i32, &name)
        } else {
            camera_id.unwrap_or(-1) as i32  // free camera
        };

        let mut camera = MjvCamera::new(camera_id as isize, data.model());
        self.scene.update(data, &self.scene_opt, &mut camera);
    }
}

impl Drop for Render {
    fn drop(&mut self) {
        if self.owns_glfw {
            unsafe { glfw::ffi::glfwTerminate() };
        }
    }
}


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

    pub fn render_trace(&mut self, scene: &mut MjvScene, ball_trace: bool, trace_rod_mask: u64) {
        let mut ball_rgba: [f32; 4];
        let mut rod_rgba: [f32; 4];
        let mut coeff;
        let mut n_iter = self.trace_buffer.len();
        if n_iter < 2 {
            return;
        }

        n_iter -= 1;

        assert!(scene.ngeom as usize + (n_iter - 1) * TRACE_GEOM_LEN < scene.maxgeom as usize);
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
                unsafe {
                    mujoco_rs::mujoco_c::mjv_initGeom(
                        scene.geoms.add(scene.ngeom as usize),
                        mjtGeom__mjGEOM_CAPSULE as i32,
                        [0.0;3].as_ptr(), [0.0;3].as_ptr(), [0.0;9].as_ptr(),
                        ball_rgba.as_ptr()
                    );
                    // Position and orient the capsule in such way that it
                    // connects the previous and current ball position
                    mujoco_rs::mujoco_c::mjv_connector(
                        scene.geoms.add(scene.ngeom as usize),
                        mjtGeom__mjGEOM_CAPSULE as i32,
                        BALL_TRACE_RADIUS, state_prev.0.as_ptr(), state.0.as_ptr()
                    );
                    scene.ngeom += 1
                }
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
    pub fn render_ball_estimate(scene: &mut MjvScene, position: &XYZType, color: Option<RGBAType>) {
        let color = color.unwrap_or(DEFAULT_BALL_ESTIMATE_RGBA);
        let position_global = [
            (position[0] + 115.0) / 1000.0,
            (727.0 - position[1]) / 1000.0,
            position[2] / 1000.0 + Z_FIELD
        ];
        unsafe {
            assert!(!scene.full());
            mujoco_rs::mujoco_c::mjv_initGeom(
                scene.geoms.add(scene.ngeom as usize),
                mjtGeom__mjGEOM_SPHERE as i32,
                &BALL_RADIUS_M,
                position_global.as_ptr(),
                std::ptr::null(),
                color.as_ptr()
            );
            scene.ngeom += 1;
        }
    }

    pub fn render_rods_estimates<T>(scene: &mut MjvScene, pos_rot: T, color: Option<RGBAType>)
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
            assert!(!scene.full());

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

                unsafe {
                    dy = first_position + ROD_SPACING[i] * p as f64;
                    pos_trans = pos_xyz;
                    pos_trans[1] += dy;
                    vgeom = scene.geoms.add(scene.ngeom as usize).as_mut().unwrap();

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
                    scene.ngeom += 1;

                    // Same for the bottom geom of the player
                    pos_trans = pos_xyz;
                    pos_trans[1] += dy;
                    vgeom = scene.geoms.add(scene.ngeom as usize).as_mut().unwrap();
                    mat_bottom = [0.0; 9];

                    // Rotate the bottom part 90 degrees around x first, then apply the joint rotation
                    mju_mulMatMat(
                        mat_bottom.as_mut_ptr(),
                        mat.as_ptr(), ROD_BOTTOM_PRE_ROTATION_MAT.as_ptr(),
                        3, 3, 3
                    );

                    offset_xyz = [0.0, 0.0, ROD_ESTIMATE_FRAME_LOWER_OFFSET];
                    mju_mulMatVec3(offset_xyz.as_mut_ptr(), mat.as_ptr(), offset_xyz.as_ptr());
                    pos_trans = std::array::from_fn(|i| pos_trans[i] + offset_xyz[i]);
                    mjv_initGeom(vgeom, mjtGeom__mjGEOM_MESH as i32, ptr::null(),
                                 pos_trans.as_ptr(), mat_bottom.as_ptr(), color.as_ptr());                        
                    vgeom.dataid =  ROD_MESH_LOWER_PLAYER_ID * 2;
                    scene.ngeom += 1;
                }
            }
        }
    }
}
