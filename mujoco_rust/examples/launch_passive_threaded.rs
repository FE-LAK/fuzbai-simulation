//! Example of using the passive MuJoCo viewer.
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


static G_MODEL: OnceLock<MjModel> = OnceLock::new();


/// Main program loop. This is where all the user's logic goes.
/// We can't use the main thread because GLFW demands to run in the main thread.
fn step_loop(viewer: Arc<Mutex<Box<MjViewer>>>, mut data: MjData) {
    let mut t0;
    loop {
        t0 = Instant::now();
        {
            let lock = viewer.lock().unwrap();
            if !lock.running() {
                break;
            }
            data.step();
        }

        // Accurately wait for 2 milliseconds
        while t0.elapsed().as_micros() < 2000 {}
    }
}


/// Main thread. This is where the initialization happens and then the rendering loop.
/// We can't spawn the loop in a different thread because the GLFW library doesn't allow it
/// (it would actually work on Rust, but would segment fault through Python bindings).
fn main() {
    const MODEL_PATH: &str = "/home/davidhozic/repo/FuzbAI/environment/models/miza.xml";//"model.xml";
    G_MODEL.set(MjModel::from_xml(MODEL_PATH).unwrap()).unwrap();
    for i in 0..10 {
        let mut items = vec![];
        for _ in 0..10000 {
            let model_ref = G_MODEL.get().unwrap();
            let mut data = MjData::new(model_ref);
            let ball_view: MjDataViewJoint = data.joint("ball").unwrap();
            for _ in 0..10 {
                unsafe {
                    mujoco_rs::mujoco_c::mj_step(model_ref.raw_mut(), data.raw_mut());
                }
            }
            items.push((data, ball_view));
        }
    }
}
