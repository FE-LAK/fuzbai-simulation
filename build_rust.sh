#!/bin/bash
# Script for building the FuzbAI simulator.

COMPILER_C=clang-13
COMPILER_CPP=clang++-13

# Configuration
CWD=$(pwd)
CMAKE_ARGS=(
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
    -DCMAKE_INSTALL_PREFIX:STRING=${TMPDIR}/mujoco_install \
    -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
    -DBUILD_SHARED_LIBS:BOOL=OFF \
    -G Ninja \
    -DCMAKE_C_COMPILER:STRING=$COMPILER_C \
    -DCMAKE_CXX_COMPILER:STRING=$COMPILER_CPP \
    -DMUJOCO_HARDEN:BOOL=ON \
    -DCMAKE_INSTALL_LIBDIR=lib
)

# Build MuJoCo
cd mujoco/
cmake -B build -S . "${CMAKE_ARGS[@]}"
sync
cmake --build build --parallel --target mujoco --config=Release
sync
cd $CWD

# Build simulation
cd simulation/
export MUJOCO_STATIC_LINK_DIR=$(realpath ../mujoco/build/lib/)
cargo build --release --lib  # compile the simulation itself
sync
cd $CWD
