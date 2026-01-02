#!/bin/bash
# Script for building the FuzbAI simulator.

# Build static MuJoCo library
./build_mujoco.sh

# Build simulation app
cd simulation-app/
export MUJOCO_STATIC_LINK_DIR=$(realpath ../mujoco/build/lib/)
cargo build --release
sync
cd $CWD
