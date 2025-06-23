#!/bin/bash
# Script for building the FuzbAI simulator for the Windows platform.

# Save current directory

# Define CMake arguments
$CMAKE_ARGS = @(
  '-DBUILD_SHARED_LIBS:BOOL=OFF'
  '-DMUJOCO_HARDEN:BOOL=OFF'
  '-DCMAKE_BUILD_TYPE:STRING=Release'
  '-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON'
  '-DMUJOCO_BUILD_EXAMPLES:BOOL=OFF'
  '-DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed'
)

# Change into the MuJoCo source directory
Push-Location 'mujoco_rust\mujoco'

# Configure
cmake -B build -S . @CMAKE_ARGS

# Build Release config
cmake --build build --config Release --parallel --target libsimulate

# Return to original directory
Pop-Location

# Build the simulator
Push-Location 'simulation'
cargo run --release --features stub-gen --bin stub_gen
maturin build -r
Pop-Location

# Build built-in agent
Push-Location 'agent_rust'
cargo run --release --features stub-gen --bin stub_gen
maturin build -r
Pop-Location
