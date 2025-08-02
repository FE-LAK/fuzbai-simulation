#!/bin/bash
# Script for building the FuzbAI simulator for linux platforms.


# Configuration
CWD=$(pwd)
CMAKE_ARGS=(
    # -DCMAKE_C_COMPILER:STRING=clang-14
    # -DCMAKE_CXX_COMPILER:STRING=clang++-14
    -DBUILD_SHARED_LIBS:BOOL=OFF
    -DMUJOCO_HARDEN:BOOL=OFF
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON
    -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF
    -DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed
)


# Build MuJoCo's libs
cd mujoco_rust/mujoco/
cmake -B build -S . "${CMAKE_ARGS[@]}"
cmake --build build --parallel --target libsimulate --config=Release
cd $CWD

# Build the model
cargo run --release --bin mujoco-model-compiler

# Build the simulator
cd simulation/
cargo run --release --features stub-gen --bin stub_gen
maturin build -r
cd $CWD

# Build built-in agent
cd fuzbai-agent/
maturin build -r

# Copy back built files
cd $CWD
