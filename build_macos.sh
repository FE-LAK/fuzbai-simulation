#!/bin/bash
# Builds the simulation-app/ competition application (macOS version).
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
Helper script for building this project (macOS).

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
source setup_macos.sh

# Create output
mkdir -p $OUTPUT

# Build application
if [ "$APP" = "y" ]; then
    cargo build --release -p simulation-app --locked
    sync
    mkdir -p $OUTPUT_APP
    cp ./target/release/simulation-app $OUTPUT_APP
    cp -rf simulation-app/www/ $OUTPUT_APP

    if [ -e mujoco-3.3.7/lib/libmujoco.dylib ]; then
        # Copy the dylib (resolves the symlink so we get the real file)
        cp -L mujoco-3.3.7/lib/libmujoco.dylib $OUTPUT_APP/libmujoco.dylib

        # Patch the binary so it can find libmujoco.dylib next to itself
        install_name_tool -add_rpath @executable_path $OUTPUT_APP/simulation-app 2>/dev/null || true

        # Also patch any residual framework-style references in the binary
        OLD_REF=$(otool -L $OUTPUT_APP/simulation-app | grep -i "mujoco" | awk '{print $1}' | head -1)
        if [ -n "$OLD_REF" ] && [ "$OLD_REF" != "libmujoco.dylib" ] && [ "$OLD_REF" != "@rpath/libmujoco.dylib" ]; then
            install_name_tool -change "$OLD_REF" @rpath/libmujoco.dylib $OUTPUT_APP/simulation-app 2>/dev/null || true
        fi
    else
        echo "Warning: libmujoco.dylib not found in mujoco-3.3.7/lib/"
    fi
fi

# Build Python bindings
if [ "$PYTHON" = "y" ]; then
    cd simulation/
    maturin build --release --locked
    sync
    cd "$CWD"
    mkdir -p $OUTPUT_PYTHON
    cp -rf ./target/wheels/* $OUTPUT_PYTHON
fi

# Build documentation
if [ "$DOC" = "y" ]; then
    DOCS_RS=y cargo doc -p fuzbai_simulator --no-deps --document-private-items --features python-bindings --locked
    sync
    mkdir -p $OUTPUT_DOC
    cp -rf ./target/doc/* $OUTPUT_DOC
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
