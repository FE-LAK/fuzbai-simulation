//! Build script for compiling an XML MuJoCo model into MJB.
//! This is used for easier embedding inside the binary.
#[cfg(windows)]
use std::path::PathBuf;

fn main() {
    println!("cargo::rerun-if-changed=./build.rs");  // This build script changed.
    println!("cargo::rerun-if-changed=../models/");  // XML files or its dependencies changed.

    // Model compilation
    const MODEL_PATH: &str = "../models/miza.xml";
    const OUTPUT_PATH: &str = "./src/miza.mjb";

    #[cfg(not(target_os = "macos"))]
    {
        // The compiler is part of the MuJoCo's sample programs in official MuJoCo builds.
        let _ = std::fs::remove_file(OUTPUT_PATH);
        std::process::Command::new("../mujoco-3.3.7/bin/compile")
            .arg(MODEL_PATH)
            .arg(OUTPUT_PATH)
            .output()
            .expect("failed to compile MuJoCo model from XML to MJB.");
    }
    #[cfg(target_os = "macos")]
    {
        println!("cargo:warning=Skipping MuJoCo model compilation on macOS (manual compilation required).");
    }

    // Copy the MuJoCo DLL for proper embedding in the Python wheel.
    #[cfg(windows)]
    #[cfg(feature = "python-bindings")]
    copy_mujoco()
}

#[cfg(windows)]
#[cfg(feature = "python-bindings")]
fn copy_mujoco() {
    // These are a dependency of MuJoCo-rs, thus one of them must exist

    if let Ok(mujoco_dir) = std::env::var("MUJOCO_DYNAMIC_LINK_DIR")
        .map(|md| PathBuf::from(md).parent().unwrap().to_path_buf())
    {
        let dll_path = mujoco_dir.join("bin/mujoco.dll");
        std::fs::copy(dll_path, "fuzbai_simulator/mujoco.dll").expect("failed to copy mujoco.dll");
    }
}
