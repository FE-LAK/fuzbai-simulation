#!/bin/bash
# Script for building the FuzbAI simulator for linux platforms.


# Configuration
CWD=$(pwd)
BUILD_CWD=/tmpfs/build/
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


# Build inside the container's filesystem to prevent
# overhead of communicating between Docker and host.
rm $BUILD_CWD -rf
mkdir $BUILD_CWD -p
cp $CWD/* $BUILD_CWD -r
cd $BUILD_CWD

# Build MuJoCo's libs
cd mujoco_rust/mujoco/
cmake -B build -S . "${CMAKE_ARGS[@]}"
cmake --build build --parallel --target libsimulate --config=Release
cd $BUILD_CWD

# Build the model
cargo run --release --bin mujoco-model-compiler

# Build the simulator
cd simulation/
cargo run --release --features stub-gen --bin stub_gen
maturin build -r
cd $BUILD_CWD

# Build built-in agent
cd fuzbai-agent/
maturin build -r

# Copy back built files
cd $CWD
cp $BUILD_CWD/mujoco_rust/mujoco/build/ mujoco_rust/mujoco/build/ -r
mkdir ./target/
cp $BUILD_CWD/target/* ./target/ -r
