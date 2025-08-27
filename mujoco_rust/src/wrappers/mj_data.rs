use std::ops::{Deref, DerefMut};
use std::ffi::CString;

use super::mj_auxilary::MjContact;
use super::mj_model::MjModel;
use crate::mujoco_c::*;

use crate::util::PointerViewMut;
use crate::mj_slice_view;


pub struct MjData<'a> {
    data: *mut mjData,
    model: &'a MjModel
}

// Allow usage in threaded contexts as the data won't be shared anywhere outside Rust,
// except in the C++ code.
unsafe impl Send for MjData<'_> {}
unsafe impl Sync for MjData<'_> {}


impl<'a> MjData<'a> {
    /// Constructor for a new MjData. This should is called from MjModel.
    pub(crate) fn new(model: &'a MjModel) -> Self {
        unsafe {
            Self {
                data: mj_makeData(model.__raw()),
                model: model,
            }
        }
    }

    /// Returns the raw wrapped value.
    /// # SAFETY
    /// Once returned, the Rust's compiler will lose track of this reference.
    /// This is meant only for the compatibility with existing MuJoCo C++ code.
    pub(crate) unsafe fn __raw(&self) -> *mut mjData {
        self.data
    }

    /// Returns a slice of detected contacts.
    /// # SAFETY
    /// This is NOT THREAD-SAFE, nor lifetime safe.
    /// The returned slice must ne dropped before [`MjData`].
    pub fn contacts(&self) -> &[MjContact] {
        unsafe {
            let ll = self.data.as_ref().unwrap();
            let n_contacts = ll.ncon as usize;
            std::slice::from_raw_parts(ll.contact, n_contacts)
        }
    }

    pub fn joint(&self, name: &str) -> Option<MjDataViewJoint> {
        let id = unsafe { mj_name2id(self.model.deref(), mjtObj__mjOBJ_JOINT as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let qpos;
        let qvel;
        let qacc_warmstart;
        let qacc;
        let qfrc_applied;
        let qfrc_bias;
        unsafe {
            // ptr:expr, $id:expr, $addr_map:expr, $njnt:expr, $max_n:expr
            qpos = mj_slice_view!(self.qpos, id as usize, self.model.jnt_qposadr, self.model.njnt as usize, self.model.nq as usize).unwrap();
            qvel = mj_slice_view!(self.qvel, id as usize, self.model.jnt_dofadr, self.model.njnt as usize, self.model.nv as usize).unwrap();
            qacc_warmstart = mj_slice_view!(self.qacc_warmstart, id as usize, self.model.jnt_dofadr, self.model.njnt as usize, self.model.nv as usize).unwrap();
            qacc = mj_slice_view!(self.qacc, id as usize, self.model.jnt_dofadr, self.model.njnt as usize, self.model.nv as usize).unwrap();
            qfrc_applied = mj_slice_view!(self.qfrc_applied, id as usize, self.model.jnt_dofadr, self.model.njnt as usize, self.model.nv as usize).unwrap();
            qfrc_bias = mj_slice_view!(self.qfrc_bias, id as usize, self.model.jnt_dofadr, self.model.njnt as usize, self.model.nv as usize).unwrap();
        }

        Some(MjDataViewJoint {name: name.to_string(), id: id as usize, qpos, qvel, qacc_warmstart, qacc, qfrc_applied, qfrc_bias})
    }

    pub fn geom(&self, name: &str) -> Option<MjDataViewGeom> {
        const GEOM_XPOS_LEN: usize = 3;
        const GEOM_XMAT_LEN: usize = 9;

        let id = unsafe { mj_name2id(self.model.deref(), mjtObj__mjOBJ_GEOM as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let xpos;
        let xmat;
        unsafe {
            xpos = PointerViewMut::new(self.geom_xpos.add(GEOM_XPOS_LEN * id as usize), 3);
            xmat = PointerViewMut::new(self.geom_xpos.add(GEOM_XMAT_LEN * id as usize), 9);
        }

        Some(MjDataViewGeom { name: name.to_string(), id: id as usize, xmat, xpos })
    }

    pub fn actuator(&self, name: &str) -> Option<MjDataViewActuator> {
        let id = unsafe { mj_name2id(self.model.deref(), mjtObj__mjOBJ_ACTUATOR as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let ctrl;
        let act;
        unsafe {
            ctrl = PointerViewMut::new(self.ctrl.add(id as usize).as_mut().unwrap(), 1);
            act = mj_slice_view!(self.act, id as usize, self.model.actuator_actadr, self.model.nu as usize, self.model.na as usize);
        }

        Some(MjDataViewActuator { name: name.to_string(), id: id as usize, ctrl, act })
    } 

    /// Steps the MuJoCo simulation.
    /// # SAFETY
    /// This calls the C binding of mj_step, which is not safe by itself.
    pub fn step(&mut self) {
        unsafe {
            mj_step(self.model.deref(), self.data);
        }
    }

    /// Calculates new dynamics.
    /// # SAFETY
    /// This calls the C binding of mj_step1, which is not safe by itself.
    pub fn step1(&mut self) {
        unsafe {
            mj_step1(self.model.deref(), self.data);
        }
    }

    /// Calculates the rest after dynamics and integrates in time.
    /// # SAFETY
    /// This calls the C binding of mj_step2, which is not safe by itself.
    pub fn step2(&mut self) {
        unsafe {
            mj_step2(self.model.deref(), self.data);
        }
    }

    /// Calculates the contact force for the given `contact_id`.
    pub fn contact_force(&self, contact_id: usize) -> [f64; 6] {
        let mut force = [0.0; 6];
        unsafe {
            mj_contactForce(
                self.model.deref(), self.data,
                contact_id as i32, force.as_mut_ptr()
            );
        }
        force
    }

}

impl Drop for MjData<'_> {
    fn drop(&mut self) {
        unsafe {
            mj_deleteData(self.data);
        }
    }
}

impl Deref for MjData<'_> {
    type Target = mjData;
    fn deref(&self) -> &Self::Target {
        unsafe { self.data.as_ref().unwrap() }
    }
}

impl DerefMut for MjData<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.data.as_mut().unwrap() }
    }
}


/// A MjDataViewX which shows a slice of the joint.
/// # SAFETY
/// This is not thread-safe nor lifetime-safe.
/// The view must be dropped before MjData, which is the
/// RESPONSIBILITY OF THE USER.
#[derive(Debug, PartialEq)]
pub struct MjDataViewJoint {
    pub name: String,
    pub id: usize,
    pub qpos: PointerViewMut<f64>,
    pub qvel: PointerViewMut<f64>,
    pub qacc_warmstart: PointerViewMut<f64>,
    pub qacc: PointerViewMut<f64>,
    pub qfrc_applied: PointerViewMut<f64>,
    pub qfrc_bias: PointerViewMut<f64>
}

impl MjDataViewJoint {
    pub fn reset(&mut self) {
        self.qpos.fill(0.0);
        self.qvel.fill(0.0);
        self.qacc_warmstart.fill(0.0);
        self.qacc.fill(0.0);
        self.qfrc_applied.fill(0.0);
    }
}


/// A MjDataViewX which shows a slice of the geom.
/// # SAFETY
/// This is not thread-safe nor lifetime-safe.
/// The view must be dropped before MjData, which is the
/// RESPONSIBILITY OF THE USER.
#[derive(Debug, PartialEq)]
pub struct MjDataViewGeom {
    pub name: String,
    pub id: usize,
    pub xmat: PointerViewMut<f64>,
    pub xpos: PointerViewMut<f64>
}


/// A MjDataViewX which shows a slice of the actuator.
/// # SAFETY
/// This is not thread-safe nor lifetime-safe.
/// The view must be dropped before MjData, which is the
/// RESPONSIBILITY OF THE USER.
#[derive(Debug, PartialEq)]
pub struct MjDataViewActuator {
    pub name: String,
    pub id: usize,
    pub ctrl: PointerViewMut<f64>,
    pub act: Option<PointerViewMut<f64>>,
}
