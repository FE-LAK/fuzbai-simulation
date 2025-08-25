use crate::wrappers::*;
use crate::mujoco_c;

use std::ffi::CString;
use glfw;


/// Wrapper around the C++ implementation of MujoCo viewer
/// # SAFETY
/// Due to performance reasons and PyO3, this must be destroyed before
/// [`MjData`] and [`MjModel`] instances that are passed in the constructor.
/// Normally, we would include references to them but it's very inconvenient.

pub struct MjViewer {
    sim: *mut mujoco_c::mujoco_Simulate,
    running: bool,

    // Store these here since the C++ bindings save references to them.
    // We don't actually need them ourselves, at least not here.
    _cam: Box<mujoco_c::mjvCamera>,
    _opt: Box<mujoco_c::mjvOption>,
    _pert: Box<mujoco_c::mjvPerturb>,
    _user_scn: Box<MjvScene>,
    _glfw: glfw::Glfw
}




impl MjViewer {
    #[inline]
    pub fn running(&self) -> bool {
        self.running
    }

    #[inline]
    pub fn user_scn_mut(&mut self) -> &mut MjvScene {
        &mut self._user_scn
    }

    pub fn launch_passive(model: &MjModel, data: &mut MjData, scene_max_ngeom: usize) -> Self {
        let mut _glfw = glfw::init(glfw::fail_on_errors).unwrap();

        // Allocate on the heap as the data must not be moved due to C++ bindings
        let mut _cam = Box::new(mujoco_c::mjvCamera::default());
        let mut _opt: Box<mujoco_c::mjvOption_> = Box::new(mujoco_c::mjvOption::default());
        let mut _pert = Box::new(mujoco_c::mjvPerturb::default());
        let mut _user_scn = Box::new(MjvScene::new(&model, scene_max_ngeom));
        let sim;
        unsafe {
            mujoco_c::mjv_defaultCamera(&mut *_cam);
            mujoco_c::mjv_defaultOption(&mut *_opt);
            mujoco_c::mjv_defaultPerturb(&mut *_pert);
            sim = mujoco_c::new_simulate(&mut *_cam, &mut *_opt, &mut *_pert, &mut *_user_scn, true);
            (*sim).RenderInit();
            (*sim).Load(model.raw_mut(), data.raw_mut(), CString::new("file.xml").unwrap().as_ptr());
            (*sim).RenderStep(true);
        }

        Self {sim, running: true, _cam, _opt, _pert, _glfw, _user_scn}
    }

    /// Returns the underlying C++ binding object of the viewer.
    pub fn raw(&mut self) -> *mut mujoco_c::mujoco_Simulate {
        self.sim
    }

    /// Renders the simulation.
    /// `update_timer` flag species whether the time should be updated
    /// inside the viewer (for vsync purposes).
    /// # SAFETY
    /// This needs to be called periodically from the MAIN thread, otherwise
    /// GLFW stops working.
    pub fn render(&mut self, update_timer: bool) {
        unsafe {
            assert!(self.running, "render called after viewer has been closed!");
            self.running = (*self.sim).RenderStep(update_timer);
        }
    }

    pub fn sync(&mut self) {
        unsafe {
            (*self.sim).Sync();
        }
    }
}


impl Drop for MjViewer {
    fn drop(&mut self) {
        unsafe {
            (*self.sim).RenderCleanup();
            mujoco_c::free_simulate(self.sim);
        }
    }
}
