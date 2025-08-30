#!/bin/bash
# Script for building the FuzbAI simulator for linux platforms.


# Configuration
CWD=$(pwd)
TARGET_WHEELS=target/wheels/
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
cd mujoco-rs-w/mujoco/
cmake -B build -S . "${CMAKE_ARGS[@]}"
cmake --build build --parallel --target libsimulate --config=Release
sync
cd $CWD


# Build the simulator
cd simulation/
MUJOCO_STATIC_LINK_LIB=../mujoco-rs-w/mujoco/build/lib/ maturin build -r
sync
cd $CWD

# Build built-in agent
cd fuzbai-agent/
maturin build -r
sync

# Copy back built files
cd $CWD
mkdir -p $TARGET_WHEELS
cp simulation/target/wheels/* $TARGET_WHEELS
cp fuzbai-agent/target/wheels/* $TARGET_WHEELS
