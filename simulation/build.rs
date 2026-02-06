//! Build script for compiling an XML MuJoCo model into MJB.
//! This is used for easier embedding inside the binary.
use std::process::Command;
use std::path::Path;
use std::fs;

#[cfg(windows)]
use std::path::PathBuf;

fn main() {
    println!("cargo::rerun-if-changed=./build.rs");  // This build script changed.
    println!("cargo::rerun-if-changed=../models/");  // XML files or its dependencies changed.

    // Model compilation
    const MODEL_PATH: &str = "../models/miza.xml";
    const OUTPUT_PATH: &str = "./src/miza.mjb";

    // Try to compile the model 
    compile_model(MODEL_PATH, OUTPUT_PATH);

    // Copy the MuJoCo DLL for proper embedding in the Python wheel.
    #[cfg(windows)]
    #[cfg(feature = "python-bindings")]
    copy_mujoco()
}

fn compile_model(model_path: &str, output_path: &str) {   
    if try_compile_with_binary(model_path, output_path, "../mujoco-3.3.7/bin/compile") {
        return;
    }

    panic!(
        "Failed to compile MuJoCo model. Please ensure:\n\
        1. MuJoCo is properly installed at ../mujoco-3.3.7/\n\
        2. Or the 'compile' binary is available in PATH"
    );
}

fn try_compile_with_binary(model_path: &str, output_path: &str, binary_path: &str) -> bool {
    if !Path::new(binary_path).exists() {
        return false;
    }
    
    match Command::new(binary_path)
        .arg(model_path)
        .arg(output_path)
        .output() {
        Ok(output) if output.status.success() => {
            println!("Compiled MuJoCo model with {}", binary_path);
            true
        }
        Ok(output) => {
            eprintln!("Warning: Failed to compile with {}: {}", binary_path, String::from_utf8_lossy(&output.stderr));
            false
        }
        Err(_) => false,
    }
}

#[windows]
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
