//! Compiler of MuJoCo's XML model into the binary format
use mujoco_rs::mujoco_c::*;
use std::ffi::CString;
use std::ptr;

const MODEL_PATH: &str = "./models/miza.xml";
const OUTPUT_PATH: &str = "./simulation/src/miza.mjb";

pub fn main() {
    let model_path = CString::new(MODEL_PATH).unwrap();
    let output_path = CString::new(OUTPUT_PATH).unwrap();

    unsafe {
        let model = mj_loadXML(model_path.as_ptr(), ptr::null(), ptr::null_mut(), 0);

        if model.is_null() {
            panic!("could not load {MODEL_PATH}");
        }

        mj_saveModel(model, output_path.as_ptr(), ptr::null_mut(), 0);
    }
}
