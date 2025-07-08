# Script for building the FuzbAI simulator for the Windows platform.

# Configuration
$CWD = Get-Location
$BUILD_CWD = "$env:TEMP\build"
$CMAKE_ARGS = @(
  '-DBUILD_SHARED_LIBS:BOOL=OFF'
  '-DMUJOCO_HARDEN:BOOL=OFF'
  '-DCMAKE_BUILD_TYPE:STRING=Release'
  '-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON'
  '-DMUJOCO_BUILD_EXAMPLES:BOOL=OFF'
  '-DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed'
)

# Build inside a temporary directory to prevent host<->container overhead
if (Test-Path $BUILD_CWD) {
    Remove-Item -Recurse -Force $BUILD_CWD
}
New-Item -ItemType Directory -Path $BUILD_CWD | Out-Null
Copy-Item "$CWD\*" -Destination $BUILD_CWD -Recurse -Force
Set-Location $BUILD_CWD

# Build MuJoCo's libs
Set-Location "$BUILD_CWD\mujoco_rust\mujoco"
cmake -B build -S . $CMAKE_ARGS
cmake --build build --parallel --target libsimulate --config=Release
Set-Location $BUILD_CWD

# Build the model
cargo run --release --bin mujoco-model-compiler

# Build the simulator
Set-Location "$BUILD_CWD\simulation"
cargo run --release --features stub-gen --bin stub_gen
maturin build -r
Set-Location $BUILD_CWD

# Build built-in agent
Set-Location "$BUILD_CWD\fuzbai-agent"
maturin build -r

# Copy back built files
Set-Location $CWD
Copy-Item "$BUILD_CWD\mujoco_rust\mujoco\build\*" -Destination "mujoco_rust\mujoco\build\" -Recurse -Force
New-Item -ItemType Directory -Path .\target | Out-Null
Copy-Item "$BUILD_CWD\target\*" -Destination "target\" -Recurse -Force
