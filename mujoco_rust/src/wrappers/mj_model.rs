//! Module for mjModel
use std::io::{Error, ErrorKind};
use std::ffi::{c_int, CString};
use std::ops::{Deref, DerefMut};
use std::path::Path;
use std::ptr;

use super::mj_auxilary::MjVfs;
use crate::mujoco_c::*;


#[derive(Debug)]
pub struct MjModel(*mut mjModel);

// Allow usage in threaded contexts as the data won't be shared anywhere outside Rust,
// except in the C++ code.
unsafe impl Send for MjModel {}
unsafe impl Sync for MjModel {}


impl MjModel {
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, Error> {
        let mut error_buffer = [0i8; 100];
        unsafe {
            let path = CString::new(path.as_ref().to_str().expect("invalid utf")).unwrap();
            let raw_ptr = mj_loadXML(
                path.as_ptr(), ptr::null(),
                &mut error_buffer as *mut i8, error_buffer.len() as c_int
            );

            Self::check_raw_model(raw_ptr, &error_buffer)
        }
    }

    pub fn from_buffer(data: &[u8]) -> Result<Self, Error> {
        unsafe {
            // Create a virtual FS since we don't have direct access to the load buffer function (or at least it isn't officially exposed).
            // let raw_ptr = mj_loadModelBuffer(data.as_ptr() as *const c_void, data.len() as i32);
            let mut vfs = MjVfs::new();
            let filename = "miza.mjb";

            // Add the file into a virtual file system
            vfs.add_from_buffer(filename, data)?;

            // Load the model from the virtual file system
            let raw_model = mj_loadModel(CString::new(filename).unwrap().as_ptr(), &vfs);
            Self::check_raw_model(raw_model, &[])
        }
    }

    pub fn name2id(&self, type_: i32, name: &str) -> i32 {
        unsafe {
            mj_name2id(self.0, type_, CString::new(name).unwrap().as_ptr())
        }
    }

    fn check_raw_model(ptr_model: *mut mjModel, error_buffer: &[i8]) -> Result<MjModel, Error> {
        if ptr_model.is_null() {
            let err_u8 = error_buffer.into_iter().map(|x| *x as u8).take_while(|&x| x != 0).collect();
            Err(Error::new(ErrorKind::UnexpectedEof,  String::from_utf8(err_u8).expect("could not parse error")))
        }
        else {
            Ok(MjModel(ptr_model))
        }
    }

    /// Returns the raw wrapped value.
    /// # SAFETY
    /// Once returned, the Rust's compiler will lose track of this reference.
    /// This is meant only for the compatibility with existing MuJoCo C++ code.
    pub(crate) unsafe fn __raw(&self) -> *mut mjModel {
        self.0
    }
}

impl Drop for MjModel {
    fn drop(&mut self) {
        unsafe {
            mj_deleteModel(self.0);
        }
    }
}

impl Deref for MjModel {
    type Target = mjModel;
    fn deref(&self) -> &Self::Target {
        unsafe { self.0.as_ref().unwrap() }
    }
}

impl DerefMut for MjModel {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.0.as_mut().unwrap() }
    }
}
