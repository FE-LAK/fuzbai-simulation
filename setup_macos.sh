#!/bin/bash
# macOS does not ship GNU coreutils' realpath by default.
# Use a portable alternative: (cd <dir> && pwd)
export MUJOCO_DYNAMIC_LINK_DIR="$(cd mujoco-3.3.7/lib && pwd)"
export DYLD_LIBRARY_PATH="$(cd mujoco-3.3.7/lib && pwd)"
