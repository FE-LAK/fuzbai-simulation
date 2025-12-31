//! Build script for building
extern crate mujoco_rs;
use mujoco_rs::mujoco_c::*;
use std::ffi::CString;
use std::ptr;


fn main() {
    println!("cargo::rerun-if-changed=./build.rs");  // This build script changed.
    println!("cargo::rerun-if-changed=../models/");  // XML files or it's dependencies changed.

    // Model compilation
    compile_model();
}


/// Compile the MuJoCo's XML model into the binary format.
pub fn compile_model() {
    const MODEL_PATH: &str = "../models/miza.xml";
    const OUTPUT_PATH: &str = "./src/miza.mjb";

    let model_path = CString::new(MODEL_PATH).unwrap();
    let output_path = CString::new(OUTPUT_PATH).unwrap();

    unsafe {
        let model = mj_loadXML(model_path.as_ptr(), ptr::null(), ptr::null_mut(), 0);

        if model.is_null() {
            panic!("could not load {MODEL_PATH}");
        }

        mj_saveModel(model, output_path.as_ptr(), ptr::null_mut(), 0);
        mj_deleteModel(model);
    }
}
