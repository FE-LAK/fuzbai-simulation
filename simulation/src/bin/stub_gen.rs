#[cfg(feature = "stub-gen")]
use pyo3_stub_gen::Result;

#[cfg(feature = "stub-gen")]
fn main() -> Result<()> {
    // env_logger::Builder::from_env(env_logger::Env::default().filter_or("RUST_LOG", "info")).init();
    let stub = fuzbai_simulator::stub_info()?;
    stub.generate()?;
    Ok(())
}


#[cfg(not(feature = "stub-gen"))]
fn main() {
    compile_error!("Stub generation requires the stub-gen feature!");
}
