//! Definitions related to rendering.
use std::mem::MaybeUninit;
use crate::mujoco_c::*;

use super::mj_model::MjModel;


/***********************************************************************************************************************
** MjrContext
***********************************************************************************************************************/
pub type MjrContext = mjrContext;
impl MjrContext {
    pub fn new(model: &MjModel) -> Self {
        unsafe {
            let mut c = MaybeUninit::uninit();
            mjr_defaultContext(c.as_mut_ptr());
            mjr_makeContext(&**model, c.as_mut_ptr(), mjtFontScale__mjFONTSCALE_100 as i32);
            c.assume_init()
        }
    }

    /// Set OpenGL framebuffer for rendering to mjFB_OFFSCREEN.
    pub fn offscreen(&mut self) {
        unsafe {
            mjr_setBuffer(mjtFramebuffer__mjFB_OFFSCREEN as i32, self);
        }
    }

    /// Set OpenGL framebuffer for rendering to mjFB_WINDOW.
    pub fn window(&mut self) {
        unsafe {
            mjr_setBuffer(mjtFramebuffer__mjFB_WINDOW as i32, self);
        }
    }
}

impl Drop for MjrContext {
    fn drop(&mut self) {
        unsafe {
            mjr_freeContext(self);
        }
    }
}

/***********************************************************************************************************************
** MjrContext
***********************************************************************************************************************/
pub type MjrRectangle = mjrRect;
