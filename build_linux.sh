#!/bin/bash
# Builds the simulation-app/ competition application.
# Prepare variables
CWD=$(pwd)
OUTPUT=target/BUILD_OUTPUT_HERE
OUTPUT_APP=$OUTPUT/simulation-app
OUTPUT_PYTHON=$OUTPUT/python
CLEAN_DIRS=(target/)

# Default values
APP="y"
PYTHON="n"
CLEAN="n"

set -euo pipefail

# Parse CLI arguments
for arg in "$@"; do
    case $arg in
        --app=*) APP="${arg#*=}" ;;
        --python=*) PYTHON="${arg#*=}" ;;
        --clean) CLEAN=y ;;
        --help*) echo "\
Helper script for building this project.

Options:
    --app=y/n       build the simulation (competition) application.
    --python=y/n    build python bindings of the simulation (without competition application).
    --help          opens this help menu.
    --clean         deletes directories: ${CLEAN_DIRS[@]} \
"; exit 0 ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done


if [ "$CLEAN" = "y" ]; then
    for dir in "${CLEAN_DIRS[@]}"; do
        echo "Deleting $dir"
        rm -rf "$dir"
    done
    exit 0
fi


# Set MuJoCo variables
source setup_linux.sh

# Create output
mkdir $OUTPUT -p

# Build application
if [ "$APP" = "y" ]; then
    cargo build --release -p simulation-app --locked
    sync
    mkdir $OUTPUT_APP -p
    cp ./target/release/simulation-app $OUTPUT_APP
    cp -rf simulation-app/www/ $OUTPUT_APP
    cp mujoco-3.3.7/lib/* $OUTPUT_APP
fi

# Build Python bindings
if [ "$PYTHON" = "y" ]; then
    cd simulation/
    maturin build --release --locked
    sync
    cd $CWD
    mkdir $OUTPUT_PYTHON -p
    cp ./target/wheels/* -rf $OUTPUT_PYTHON
fi

echo "============================="
echo "Finished! Saved in $OUTPUT"
echo "============================="
