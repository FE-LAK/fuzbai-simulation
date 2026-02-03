#!/bin/bash
# Builds the simulation-app/ competition application.
# Prepare variables
CWD=$(pwd)
OUTPUT=target/BUILD_OUTPUT_HERE
OUTPUT_APP=$OUTPUT/simulation-app
OUTPUT_PYTHON=$OUTPUT/python
OUTPUT_DOC=$OUTPUT/doc
CLEAN_DIRS=(target/)

# Default values
APP="y"
PYTHON="n"
DOC="n"
LICENSES="n"
CLEAN="n"

set -euo pipefail

# Parse CLI arguments
for arg in "$@"; do
    case $arg in
        --app=*) APP="${arg#*=}" ;;
        --python=*) PYTHON="${arg#*=}" ;;
        --doc=*) DOC="${arg#*=}" ;;
        --licenses=*) LICENSES="${arg#*=}" ;;
        --clean) CLEAN=y ;;
        --help*) echo "\
Helper script for building this project.

Options:
    --app=y/n       [default=y] build the simulation (competition) application.
    --python=y/n    [default=n] build python bindings of the simulation (without competition application).
    --doc=y/n       [default=n] build documentation of the simulation (including the Python bindings).
    --licenses=y/n  [default=n] generate a licenses report (in HTML form) of embedded libraries.
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

    if [ -e mujoco-3.3.7/lib/libmujoco.so ]; then  # Linux
        cp mujoco-3.3.7/lib/libmujoco* $OUTPUT_APP
    else  # Windows
        cp mujoco-3.3.7/bin/mujoco.dll $OUTPUT_APP
    fi
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

# Build documentation
if [ "$DOC" = "y" ]; then
    DOCS_RS=y cargo doc -p fuzbai_simulator --no-deps --document-private-items --features python-bindings --locked
    sync
    mkdir $OUTPUT_DOC -p
    cp ./target/doc/* -rf $OUTPUT_DOC
    ln -sf doc/fuzbai_simulator/index.html $OUTPUT/DOCUMENTATION.html
fi

# Generate licenses
if [ "$LICENSES" = "y" ]; then
    # Setup cargo-about
    cargo install cargo-about --locked
    # Generate licenses file
    cargo about generate about.hbs --features python-bindings -o $OUTPUT/THIRD_PARTY_NOTICES.html
fi

echo "============================="
echo "Finished! Saved in $OUTPUT"
echo "============================="
