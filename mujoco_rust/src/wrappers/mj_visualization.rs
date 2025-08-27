//! Definitions related to visualization.
use std::ops::{Deref, DerefMut};
use std::default::Default;
use std::mem::MaybeUninit;
use std::ptr;

use glfw::ffi::{glfwGetCurrentContext, glfwGetWindowSize};

use super::mj_rendering::{MjrContext, MjrRectangle};
use super::mj_model::MjModel;
use super::mj_data::MjData;
use crate::mujoco_c::*;


/***********************************************************************************************************************
** MjvScene
***********************************************************************************************************************/
pub type MjvScene = mjvScene;
impl MjvScene {
    pub fn new(model: &MjModel, max_geom: usize) -> Self {
        let mut scn: mjvScene_ = Self::default();
        unsafe {
            mjv_makeScene(model.deref(), &mut scn, max_geom as i32);
            scn
        }
    }

    pub fn update(&mut self, data: &mut MjData, opt: &MjvOption, cam: &mut MjvCamera) {
        unsafe {
            mjv_updateScene(
                data.model().deref(), data.deref_mut(), opt, ptr::null(),
                cam, mjtCatBit__mjCAT_ALL as i32, self
            );
        }
    }

    /// Returns `true` when the scene is full (ngeom == maxgeom).
    pub fn full(&self) -> bool {
        self.ngeom == self.maxgeom
    }

    /// Creates a new [`MjvGeom`] inside the scene. A reference is returned for additional modification,
    /// however it must be dropped before any additional calls to this method or any other methods.
    /// The return reference's lifetime is bound to the lifetime of self.
    pub fn create_geom<'a>(
        &'a mut self, geom_type: mjtGeom, size: Option<[f64; 3]>,
        pos: Option<[f64; 3]>, mat: Option<[f64; 9]>, rgba: Option<[f32; 4]>
    ) -> &'a mut MjvGeom {
        assert!(self.ngeom < self.maxgeom);

        /* Gain raw pointers to data inside the Option enum (which is a C union) */
        let size_ptr = size.as_ref().map_or(ptr::null(), |x| x.as_ptr());
        let pos_ptr = pos.as_ref().map_or(ptr::null(), |x| x.as_ptr());
        let mat_ptr = mat.as_ref().map_or(ptr::null(), |x| x.as_ptr());
        let rgba_ptr = rgba.as_ref().map_or(ptr::null(), |x| x.as_ptr());

        let p_geom;
        unsafe {
            p_geom = self.geoms.add(self.ngeom as usize);
            mjv_initGeom(p_geom, geom_type as i32, size_ptr, pos_ptr, mat_ptr, rgba_ptr);
            self.ngeom += 1;
            p_geom.as_mut().unwrap()
        }
    }

    /// Clears the created geoms.
    pub fn clear_geom(&mut self) {
        self.ngeom = 0;
    }

    /// Renders the scene to the screen. This does not automatically make the OpenGL context current.
    pub fn render(&mut self, viewport: &MjrRectangle, context: &MjrContext) -> Vec<u8> {
        unsafe {
            /* Read window size */
            let window = glfwGetCurrentContext();
            let mut width  = 0;
            let mut height = 0;
            glfwGetWindowSize(window, &mut width, &mut height);

            let mut output = vec![0; width as usize * height as usize * 3];  // width * height * RGB

            mjr_render(viewport.clone(), self, context);
            self.read_pixels(&mut output, viewport, context);
            output
        }
    }


    /// Reads the render scene and writes the image data to `output`. The size of the `output` must 
    /// be `width * height * 3`, where `width` and `height` are the sizes of the current glfw window
    /// context.
    pub fn read_pixels(&self, output: &mut [u8], viewport: &MjrRectangle, context: &MjrContext) {
        unsafe {
            mjr_readPixels(output.as_mut_ptr(), ptr::null_mut(), viewport.clone(), context)
        }
    }
}

impl Default for MjvScene {
    fn default() -> Self {
        unsafe {
            let mut scn = MaybeUninit::uninit();
            mjv_defaultScene(scn.as_mut_ptr());
            scn.assume_init()
        }
    }
}

impl Drop for MjvScene {
    fn drop(&mut self) {
        unsafe {
            mjv_freeScene(self);
        }
    }
}

/***********************************************************************************************************************
** MjvCamera
***********************************************************************************************************************/
pub type MjvCamera = mjvCamera;
impl MjvCamera {
    pub fn new(camera_id: isize, model: &MjModel) -> Self {
        let mut camera = Self::default();

        camera.fixedcamid = camera_id as i32;
        if camera_id == -1 {  // free camera
            camera.type_ = mjtCamera__mjCAMERA_FREE as i32;
            unsafe { mjv_defaultFreeCamera(model.deref(), &mut camera); }
        }
        else {
            camera.type_ = mjtCamera__mjCAMERA_FIXED as i32;
        }

        camera
    }
}

impl Default for MjvCamera {
    fn default() -> Self {
        unsafe {
            let mut c = MaybeUninit::uninit();
            mjv_defaultCamera(c.as_mut_ptr());
            c.assume_init()
        }
    }
}

/***********************************************************************************************************************
** MjvPerturb
***********************************************************************************************************************/
pub type MjvPerturb = mjvPerturb;
impl Default for MjvPerturb {
    fn default() -> Self {
        unsafe {
            let mut pert = MaybeUninit::uninit();
            mjv_defaultPerturb(pert.as_mut_ptr());
            pert.assume_init()
        }
    }
}

/***********************************************************************************************************************
** mjvOption
***********************************************************************************************************************/
pub type MjvOption = mjvOption;
impl Default for MjvOption {
    fn default() -> Self {
        let mut opt = MaybeUninit::uninit();
        unsafe {
            mjv_defaultOption(opt.as_mut_ptr());
            opt.assume_init()
        }
    }
}

/***********************************************************************************************************************
** mjvGeom
***********************************************************************************************************************/
pub type MjvGeom = mjvGeom;
impl MjvGeom {
    /// Wrapper around the MuJoCo's mjv_connector function.
    pub fn connect(&mut self, width: f64, from: [f64; 3], to: [f64; 3]) {
        unsafe {
            mjv_connector(self, self.type_, width, from.as_ptr(), to.as_ptr());
        }
    }
}
