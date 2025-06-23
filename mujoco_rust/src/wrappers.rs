//! A set of wrappers around the MuJoCo types.
use std::io::{Error, ErrorKind};
use std::ffi::{c_int, CString};
use std::ops::{Deref, DerefMut};
use std::path::Path;
use std::ptr;

use crate::mujoco_c::*;


/* MACROS */
/// Creates slice on the given ptr (attribute) based on the MuJoCo's id (e. g., on joint based on joint id).
/// The macro creates a slice from the start address to the difference of the next id and 
macro_rules! slice_from_id {
    ($ptr:expr, $id:expr, $addr_map:expr, $njnt:expr, $max_n:expr) => {
        {
            let start_addr = *$addr_map.add($id) as isize;
            if start_addr == -1 {
                None
            }
            else
            {
                let end_addr = if $id + 1 < $njnt {*$addr_map.add($id + 1) as usize} else {$max_n};
                let n = end_addr - start_addr as usize;
                Some(PointerView::new($ptr.add(start_addr as usize), n))
            }
        }
    };
}


/* Type redefinitions */
pub type MjOption = mjOption;
pub type MjvOption = mjvOption;
pub type MjContact = mjContact;
pub type MjRectangle = mjrRect;


#[derive(Debug)]
struct MjMutPtrWrapper<T>(*mut T);


/// Unsafe implementation of [`Send`] for raw mutable pointers to MuJoCo's types.
/// # SAFETY
/// The way this library is constructed, (used) raw pointers are wrapped by a Rust
/// struct that has complete access over it. The only place where they could be
/// used and presents a hazard, is the C++ viewer code.
unsafe impl<T> Send for MjMutPtrWrapper<T> {}

/// # SAFETY
/// See [`Send`] implementation of [`MjMutPtrWrapper`].
unsafe impl<T> Sync for MjMutPtrWrapper<T> {}

unsafe impl Send for mjrContext {}
unsafe impl Sync for mjrContext {}


/* STRUCTS */

/// Provides a more direct view to a C array.
#[derive(Debug)]
pub struct PointerView<T> {
    ptr: *mut T,
    len: usize,
}


/// Compares if the two views point to the same data.
impl<T> PartialEq for PointerView<T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr  // if the pointer differs, this isn't a view to the same data
    }
}

impl<T> PointerView<T> {
    fn new(ptr: *mut T, len: usize) -> Self {
        Self {ptr, len}
    }
}

impl<T> Deref for PointerView<T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}

impl<T> DerefMut for PointerView<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}


/// Unsafe implementation of [`Send`] for data view to MjData.
/// # SAFETY
/// This shall not be shared by multiple threads, however it is needed
/// due to PyO3.
unsafe impl<T> Send for PointerView<T> {}

/// # SAFETY
/// See [`Send`] implementation of [`PointerView`].
unsafe impl<T> Sync for PointerView<T> {}


/// Wrapper around MuJoCo's `mjvScene`.
pub struct MjvScene {
    raw: Box<mjvScene>
}

impl MjvScene {
    pub fn new(model: &MjModel, max_geom: usize) -> Self {
        let mut raw = Box::new(mjvScene::default());
        unsafe {
            mjv_makeScene(model.raw(), raw.as_mut(), max_geom as i32);
            Self {raw}
        }
    }
    pub unsafe fn raw(&mut self) -> &mut mjvScene {
        self.raw.as_mut()
    }
}

impl Drop for MjvScene {
    fn drop(&mut self) {
        unsafe {
            mjv_freeScene(&mut *self.raw);
        }
    }
}

/// Unsafe implementations required for bypassing thread safety checks.
/// This is NOT thread safe, responsibility is on the user.
unsafe impl Send for MjvScene {}
unsafe impl Sync for MjvScene {}


#[derive(Debug)]
pub struct MjModel {
    lowlevel: MjMutPtrWrapper<mjModel>
}

impl MjModel {
    /// Returns a raw mutable pointer to th [`mjModel`].
    /// # SAFETY
    /// Since this releases a raw pointer for use in the C and C++ code, special care
    /// needs to be taken to prevent data races or (inside C/C++) null pointers.
    /// Any code requiring the raw pointer should also store a reference to [`MjModel`]
    /// to prevent use after free. 
    pub unsafe fn raw_mut(&self) -> *mut mjModel {
        self.lowlevel.0
    }

    /// Returns a raw const pointer to th [`mjModel`].
    /// # SAFETY
    /// Since this releases a raw pointer for use in the C and C++ code, special care
    /// needs to be taken to prevent data races or (inside C/C++) null pointers.
    /// Any code requiring the raw pointer should also store a reference to [`MjModel`]
    /// to prevent use after free.
    pub unsafe fn raw(&self) -> *const mjModel {
        self.lowlevel.0
    }

    pub fn opt(&self) -> &MjOption {
        unsafe { &(*self.lowlevel.0).opt }
    }

    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, Error> {
        let mut error_buffer = [0i8; 100];
        unsafe {
            let path = CString::new(path.as_ref().to_str().expect("invalid utf")).unwrap();
            let raw_ptr = mj_loadXML(
                path.as_ptr(), ptr::null(),
                &mut error_buffer as *mut i8, error_buffer.len() as c_int
            );

            if raw_ptr.is_null() {
                let err_u8 = error_buffer.into_iter().map(|x| x as u8).take_while(|&x| x != 0).collect();
                Err(Error::new(ErrorKind::UnexpectedEof,  String::from_utf8(err_u8).expect("could not parse error")))
            }
            else {
                Ok(Self {lowlevel: MjMutPtrWrapper(raw_ptr)})
            }
        }
    }
}

impl Drop for MjModel {
    fn drop(&mut self) {
        unsafe {
            mj_deleteModel(self.lowlevel.0);
        }
    }
}

pub struct MjData<'a> {
    lowlevel: MjMutPtrWrapper<mjData>,
    model: &'a MjModel,
}

impl<'a> MjData<'a> {
    pub fn new(model: &'a MjModel) -> Self {
        unsafe {
            Self {
                lowlevel: MjMutPtrWrapper(mj_makeData(model.raw())),
                model: model
            }
        }
    }

    /// Returns a slice of detected contacts.
    /// # SAFETY
    /// This is NOT THREAD-SAFE, nor lifetime safe.
    /// The returned slice must ne dropped before [`MjData`].
    pub fn contacts(&self) -> &[MjContact] {
        unsafe {
            let ll = self.lowlevel.0.as_ref().unwrap();
            let n_contacts = ll.ncon as usize;
            std::slice::from_raw_parts(ll.contact, n_contacts)
        }
    }

    /// Returns a raw mutable pointer to th [`mjData`].
    /// # SAFETY
    /// Since this releases a raw pointer for use in the C and C++ code, special care
    /// needs to be taken to prevent data races or (inside C/C++) null pointers.
    /// Any code requiring the raw pointer should also store a reference to [`MjData`]
    /// to prevent use after free.
    pub unsafe fn raw_mut(&mut self) -> *mut mjData {
        self.lowlevel.0
    }

    /// Returns a raw const pointer to th [`mjData`].
    /// # SAFETY
    /// Since this releases a raw pointer for use in the C and C++ code, special care
    /// needs to be taken to prevent data races or (inside C/C++) null pointers.
    /// Any code requiring the raw pointer should also store a reference to [`MjData`]
    /// to prevent use after free.
    pub unsafe fn raw(&self) -> *const mjData {
        self.lowlevel.0
    }

    pub fn model(&self) -> &MjModel {
        self.model
    }
 
    pub fn joint(&self, name: &str) -> Option<MjDataViewJoint> {
        let id = unsafe { mj_name2id(self.model.raw(), mjtObj__mjOBJ_JOINT as i32, CString::new(name).unwrap().as_ptr())};
        let model_raw = unsafe { self.model.raw() };
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
            qpos = slice_from_id!((*self.lowlevel.0).qpos, id as usize, (*model_raw).jnt_qposadr, (*model_raw).njnt as usize, (*model_raw).nq as usize).unwrap();
            qvel = slice_from_id!((*self.lowlevel.0).qvel, id as usize, (*model_raw).jnt_dofadr, (*model_raw).njnt as usize, (*model_raw).nv as usize).unwrap();
            qacc_warmstart = slice_from_id!((*self.lowlevel.0).qacc_warmstart, id as usize, (*model_raw).jnt_dofadr, (*model_raw).njnt as usize, (*model_raw).nv as usize).unwrap();
            qacc = slice_from_id!((*self.lowlevel.0).qacc, id as usize, (*model_raw).jnt_dofadr, (*model_raw).njnt as usize, (*model_raw).nv as usize).unwrap();
            qfrc_applied = slice_from_id!((*self.lowlevel.0).qfrc_applied, id as usize, (*model_raw).jnt_dofadr, (*model_raw).njnt as usize, (*model_raw).nv as usize).unwrap();
            qfrc_bias = slice_from_id!((*self.lowlevel.0).qfrc_bias, id as usize, (*model_raw).jnt_dofadr, (*model_raw).njnt as usize, (*model_raw).nv as usize).unwrap();
        }

        Some(MjDataViewJoint {name: name.to_string(), id: id as usize, qpos, qvel, qacc_warmstart, qacc, qfrc_applied, qfrc_bias})
    }

    pub fn geom(&self, name: &str) -> Option<MjDataViewGeom> {
        const GEOM_XPOS_LEN: usize = 3;
        const GEOM_XMAT_LEN: usize = 9;

        let id = unsafe { mj_name2id(self.model.raw(), mjtObj__mjOBJ_GEOM as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let xpos;
        let xmat;
        unsafe {
            xpos = PointerView::new((*self.lowlevel.0).geom_xpos.add(GEOM_XPOS_LEN * id as usize), 3);
            xmat = PointerView::new((*self.lowlevel.0).geom_xpos.add(GEOM_XMAT_LEN * id as usize), 9);
        }

        Some(MjDataViewGeom { name: name.to_string(), id: id as usize, xmat, xpos })
    }

    pub fn actuator(&self, name: &str) -> Option<MjDataViewActuator> {
        let id = unsafe { mj_name2id(self.model.raw(), mjtObj__mjOBJ_ACTUATOR as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let ctrl;
        let act;
        let model_raw = unsafe { self.model.raw() };
        unsafe {
            ctrl = PointerView::new((*self.lowlevel.0).ctrl.add(id as usize).as_mut().unwrap(), 1);
            act = slice_from_id!((*self.lowlevel.0).act, id as usize, (*model_raw).actuator_actadr, (*model_raw).nu as usize, (*model_raw).na as usize);
        }

        Some(MjDataViewActuator { name: name.to_string(), id: id as usize, ctrl, act })
    } 

    /// Steps the MuJoCo simulation.
    /// # SAFETY
    /// This calls the C binding of mj_step, which is not safe by itself.
    pub fn step(&mut self) {
        unsafe {
            mj_step(self.model.raw(), self.lowlevel.0);
        }
    }

    /// Calculates new dynamics.
    /// # SAFETY
    /// This calls the C binding of mj_step1, which is not safe by itself.
    pub fn step1(&mut self) {
        unsafe {
            mj_step1(self.model.raw(), self.lowlevel.0);
        }
    }

    /// Calculates the rest after dynamics and integrates in time.
    /// # SAFETY
    /// This calls the C binding of mj_step2, which is not safe by itself.
    pub fn step2(&mut self) {
        unsafe {
            mj_step2(self.model.raw(), self.lowlevel.0);
        }
    }
}

impl Drop for MjData<'_> {
    fn drop(&mut self) {
        unsafe {
            mj_deleteData(self.lowlevel.0);
        }
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
    pub qpos: PointerView<f64>,
    pub qvel: PointerView<f64>,
    pub qacc_warmstart: PointerView<f64>,
    pub qacc: PointerView<f64>,
    pub qfrc_applied: PointerView<f64>,
    pub qfrc_bias: PointerView<f64>
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
    pub xmat: PointerView<f64>,
    pub xpos: PointerView<f64>
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
    pub ctrl: PointerView<f64>,
    pub act: Option<PointerView<f64>>,
}


pub struct MjContext {
    raw: mjrContext
}

impl MjContext {
    pub fn new(model: &MjModel) -> Self {
        let mut context: mjrContext_ = mjrContext::default();
        unsafe {
            mjr_defaultContext(&mut context);
            mjr_makeContext(model.raw(), &mut context, mjtFontScale__mjFONTSCALE_100 as i32);
        }
        Self {raw: context}
    }

    pub fn from_raw(raw: mjrContext) -> Self {
        Self {raw}
    }

    pub unsafe fn raw(&self) -> *const mjrContext {
        &self.raw
    }

    pub unsafe fn raw_mut(&mut self) -> *mut mjrContext {
        &mut self.raw
    }
}

impl Drop for MjContext {
    fn drop(&mut self) {
        unsafe {
            mjr_freeContext(&mut self.raw);
        }
    }
}

pub struct MjCamera {
    raw: mjvCamera,
}

impl MjCamera {
    pub fn new(camera_id: isize, model: &MjModel) -> Self {
        let mut raw = mjvCamera::default();
        unsafe { mjv_defaultCamera(&mut raw); };

        raw.fixedcamid = camera_id as i32;
        if camera_id == -1 {  // free camera
            raw.type_ = mjtCamera__mjCAMERA_FREE as i32;
            unsafe { mjv_defaultFreeCamera(model.raw(), &mut raw); }
        }
        else {
            raw.type_ = mjtCamera__mjCAMERA_FIXED as i32;
        }

        Self {raw}
    }

    pub unsafe fn raw_mut(&mut self) -> *mut mjvCamera {
        &mut self.raw
    }
}


#[cfg(test)]
mod tests {
    use std::sync::OnceLock;
    use super::*;

    const MODEL_PATH: &str = "/home/davidhozic/repo/FuzbAI/environment/models/miza.xml";
    static G_MJ_MODEL: OnceLock<MjModel> = OnceLock::new();

    fn init_model() -> &'static MjModel {
        G_MJ_MODEL.get_or_init(|| MjModel::from_xml(MODEL_PATH).expect("could not load model"))
    }

    #[test]
    fn test_view_joint() {
        const JOINT_NAME: &str = "ball";
        const JOINT_QPOS_SIZE: usize = 7;
        const JOINT_QVEL_SIZE: usize = 6;
        const NON_EXIST_JOINT_NAME: &str = "fake_name";

        // Load and create data
        let model = init_model();
        let data = MjData::new(&model);

        // Test non-existant name
        assert_eq!(data.joint(NON_EXIST_JOINT_NAME), None);  

        let joint_view = data.joint(JOINT_NAME).expect("cound not find joint");
        assert_eq!(joint_view.name, JOINT_NAME);
        assert_eq!(joint_view.qpos.len(), JOINT_QPOS_SIZE);
        assert_eq!(joint_view.qvel.len(), JOINT_QVEL_SIZE);        
    }

    #[test]
    fn test_view_actuator() {
        const ACT_NAME: &str = "p1_slide_ctrl";
        const CTRL_SIZE: usize = 1;
        const NON_EXIST_JOINT_NAME: &str = "fake_name";

        // Load and create data
        let model = init_model();
        let data = MjData::new(&model);

        // Test non-existant name
        assert_eq!(data.actuator(NON_EXIST_JOINT_NAME), None);

        let act_view = data.actuator(ACT_NAME).expect("cound not find joint");
        assert_eq!(act_view.name, ACT_NAME);
        assert_eq!(act_view.ctrl.len(), CTRL_SIZE);
    }
}


