#!/bin/bash
# Script for building the FuzbAI simulator.

# Configuration
CWD=$(pwd)
CMAKE_ARGS=(
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON \
    -DCMAKE_INSTALL_PREFIX:STRING=${TMPDIR}/mujoco_install \
    -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
    -DBUILD_SHARED_LIBS:BOOL=OFF \
    -G Ninja \
    -DCMAKE_C_COMPILER:STRING=clang-13 \
    -DCMAKE_CXX_COMPILER:STRING=clang++-13 \
    -DMUJOCO_HARDEN:BOOL=ON
)

# Build MuJoCo
cd mujoco/
cmake -B build -S . "${CMAKE_ARGS[@]}"
sync
cmake --build build --parallel --target glfw libmujoco_simulate --config=Release
sync
cd $CWD

# Build simulation
cd simulation/
export MUJOCO_STATIC_LINK_DIR=$(realpath ../mujoco/build/lib/)
cargo build --release --lib  # compile the simulation itself
sync
cd $CWD
