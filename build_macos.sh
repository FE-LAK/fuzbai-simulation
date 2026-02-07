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
ARCH="universal"  # Options: native, arm64, x86_64, universal

set -euo pipefail

# Parse CLI arguments
for arg in "$@"; do
    case $arg in
        --app=*) APP="${arg#*=}" ;;
        --python=*) PYTHON="${arg#*=}" ;;
        --doc=*) DOC="${arg#*=}" ;;
        --licenses=*) LICENSES="${arg#*=}" ;;
        --arch=*) ARCH="${arg#*=}" ;;
        --clean) CLEAN=y ;;
        --help*) echo "\
Helper script for building this project (macOS).

Options:
    --app=y/n           [default=y] build the simulation (competition) application.
    --python=y/n        [default=n] build python bindings of the simulation (without competition application).
    --doc=y/n           [default=n] build documentation of the simulation (including the Python bindings).
    --licenses=y/n      [default=n] generate a licenses report (in HTML form) of embedded libraries.
    --arch=native/arm64/x86_64/universal [default=universal] target architecture (native=current machine, universal=arm64+x86_64).
    --help              opens this help menu.
    --clean             deletes directories: ${CLEAN_DIRS[@]} \
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

# Function to uncomment the glutin patch in Cargo.toml
uncomment_patch() {
    sed -i '' 's/^# \[patch\.crates-io\]/[patch.crates-io]/' Cargo.toml
    sed -i '' 's/^# glutin = /glutin = /' Cargo.toml
}

# Function to re-comment the glutin patch in Cargo.toml
recomment_patch() {
    sed -i '' 's/^\[patch\.crates-io\]/# [patch.crates-io]/' Cargo.toml
    sed -i '' 's/^glutin = /# glutin = /' Cargo.toml
    # Revert glutin to the published crates.io version
    cargo update -p glutin
}

# Ensure patch is re-commented on exit (even if build fails)
trap recomment_patch EXIT

# Uncomment the patch for macOS build
uncomment_patch

# Update glutin to the patched version
echo "Updating glutin to patched version..."
cargo update -p glutin

# Determine target triples based on architecture argument
case $ARCH in
    native)
        TARGETS=()
        echo "Building for native architecture..."
        ;;
    arm64)
        TARGETS=("aarch64-apple-darwin")
        echo "Building for arm64 (Apple Silicon)..."
        ;;
    x86_64)
        TARGETS=("x86_64-apple-darwin")
        echo "Building for x86_64 (Intel)..."
        ;;
    universal)
        TARGETS=("aarch64-apple-darwin" "x86_64-apple-darwin")
        echo "Building universal binary (arm64 + x86_64)..."
        ;;
    *)
        echo "Unknown architecture: $ARCH"
        exit 1
        ;;
esac

# Install targets if needed
if [ ${#TARGETS[@]} -gt 0 ]; then
    for target in "${TARGETS[@]}"; do
        rustup target add "$target" 2>/dev/null || true
    done
fi

# Create output
mkdir -p $OUTPUT

# Build application
if [ "$APP" = "y" ]; then
    # Build for specified architecture(s)
    if [ ${#TARGETS[@]} -eq 0 ]; then
        # Native build
        cargo build --release -p simulation-app --locked
    elif [ ${#TARGETS[@]} -eq 1 ]; then
        # Single target
        cargo build --release -p simulation-app --target "${TARGETS[0]}" --locked
    else
        # Universal binary - build both, then lipo
        for target in "${TARGETS[@]}"; do
            cargo build --release -p simulation-app --target "$target" --locked
        done
    fi

    sync
    mkdir -p $OUTPUT_APP

    # Determine the binary location(s)
    if [ ${#TARGETS[@]} -eq 0 ]; then
        # Native build
        cp ./target/release/simulation-app $OUTPUT_APP
    elif [ ${#TARGETS[@]} -eq 1 ]; then
        # Single target build
        cp "./target/${TARGETS[0]}/release/simulation-app" $OUTPUT_APP
    else
        # Universal binary - combine with lipo
        echo "Creating universal binary..."
        lipo -create \
            "./target/aarch64-apple-darwin/release/simulation-app" \
            "./target/x86_64-apple-darwin/release/simulation-app" \
            -output "$OUTPUT_APP/simulation-app"
    fi

    cp -rf simulation-app/www/ $OUTPUT_APP

    if [ -e mujoco-3.3.7/lib/libmujoco.dylib ]; then
        # Copy the dylib (resolves the symlink so we get the real file)
        cp -L mujoco-3.3.7/lib/libmujoco.dylib $OUTPUT_APP/libmujoco.dylib

        # Find and change any mujoco library references to use @rpath
        while IFS= read -r line; do
            if [[ "$line" == *"mujoco"* ]] && [[ "$line" != *"@rpath"* ]]; then
                # Extract the library path (first whitespace-separated field)
                OLD_REF=$(echo "$line" | awk '{print $1}')
                if [ -n "$OLD_REF" ]; then
                    install_name_tool -change "$OLD_REF" @rpath/libmujoco.dylib "$OUTPUT_APP/simulation-app"
                fi
            fi
        done < <(otool -L "$OUTPUT_APP/simulation-app" | tail -n +2)

        # Add the rpath so dylib is found next to the executable
        install_name_tool -add_rpath @executable_path "$OUTPUT_APP/simulation-app"
    else
        echo "Warning: libmujoco.dylib not found in mujoco-3.3.7/lib/"
    fi
fi

# Build Python bindings
if [ "$PYTHON" = "y" ]; then
    cd simulation/

    # Ensure macOS mujoco dylib is bundled into the Python package when available
    if [ -e ../mujoco-3.3.7/lib/libmujoco.dylib ]; then
        echo "Bundling macOS mujoco dylib into Python package..."
        cp -L ../mujoco-3.3.7/lib/libmujoco.dylib fuzbai_simulator/libmujoco.dylib
    fi

    # Build for specified architecture(s)
    if [ ${#TARGETS[@]} -eq 0 ]; then
        # Native build
        maturin build --release --locked
    elif [ ${#TARGETS[@]} -eq 1 ]; then
        # Single target build
        case "${TARGETS[0]}" in
            aarch64-apple-darwin)
                maturin build --release --locked --target aarch64-apple-darwin
                ;;
            x86_64-apple-darwin)
                maturin build --release --locked --target x86_64-apple-darwin
                ;;
        esac
    else
        # Universal binary - build both architectures (PyPI standard: separate wheels per arch)
        rustup target add aarch64-apple-darwin 2>/dev/null || true
        maturin build --release --locked --target aarch64-apple-darwin
        maturin build --release --locked --target x86_64-apple-darwin
    fi

    sync
    cd "$CWD"
    mkdir -p $OUTPUT_PYTHON
    cp -rf ./target/wheels/* $OUTPUT_PYTHON

    # Use delocate to bundle native macOS libs and fix install names (if available)
    if command -v delocate-wheel >/dev/null 2>&1; then
        echo "Running delocate-wheel to repair macOS wheels..."
        for wheel in "$OUTPUT_PYTHON"/*.whl; do
            [ -f "$wheel" ] || continue
            tmpout=$(mktemp -d)
            # delocate-wheel writes repaired wheel(s) to the target directory
            if delocate-wheel -w "$tmpout" "$wheel"; then
                mv "$tmpout"/*.whl "$wheel"
            else
                echo "Warning: delocate-wheel failed for $wheel" >&2
            fi
            rm -rf "$tmpout"
        done
    else
        echo "Warning: 'delocate-wheel' not found in build environment. Install 'delocate' to make macOS wheels self-contained." >&2
    fi
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
