//! Build script for compiling an XML MuJoCo model into MJB.
//! This is used for easier embedding inside the binary.

fn main() {
    println!("cargo::rerun-if-changed=./build.rs");  // This build script changed.
    println!("cargo::rerun-if-changed=../models/");  // XML files or it's dependencies changed.

    // Model compilation
    const MODEL_PATH: &str = "../models/miza.xml";
    const OUTPUT_PATH: &str = "./src/miza.mjb";

    // The compiler is part of the MuJoCo's sample programs in official MuJoCo builds.
    let _ = std::fs::remove_file(OUTPUT_PATH);
    std::process::Command::new("../mujoco-3.3.7/bin/compile")
        .arg(MODEL_PATH)
        .arg(OUTPUT_PATH)
        .output()
        .expect("failed to compile MuJoCo model from XML to MJB.");
}
