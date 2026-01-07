//! Sets RPATH of the final binary to point to $ORIGIN,
//! which is the path of the binary.

fn main() {
    #[cfg(unix)]
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
}
