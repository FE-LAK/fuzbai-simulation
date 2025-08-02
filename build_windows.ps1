# Script for building the FuzbAI simulator for the Windows platform.

# Configuration
$CWD = Get-Location
$CMAKE_ARGS = @(
  '-DBUILD_SHARED_LIBS:BOOL=OFF'
  '-DMUJOCO_HARDEN:BOOL=OFF'
  '-DCMAKE_BUILD_TYPE:STRING=Release'
  '-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON'
  '-DMUJOCO_BUILD_EXAMPLES:BOOL=OFF'
  '-DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed'
)


# Build MuJoCo's libs
Set-Location "$CWD\mujoco_rust\mujoco"
cmake -B build -S . $CMAKE_ARGS
cmake --build build --parallel --target libsimulate --config=Release
Set-Location $CWD

# Build the model
cargo run --release --bin mujoco-model-compiler

# Build the simulator
Set-Location "$CWD\simulation"
cargo run --release --features stub-gen --bin stub_gen
maturin build -r
Set-Location $CWD

# Build built-in agent
Set-Location "$CWD\fuzbai-agent"
maturin build -r
