#!/bin/bash
set -euo pipefail

# --- Configuration ---
APP="y"
PYTHON="n"
DOC="n"
LICENSES="n"
CLEAN="n"
ARCH="universal"
CWD="$(pwd)"
OUTPUT="$CWD/target/BUILD_OUTPUT_HERE"

# --- MuJoCo Setup ---
source setup_macos.sh

# --- Argument Parsing ---
for arg in "$@"; do
    case $arg in
        --app=*)     APP="${arg#*=}" ;;
        --python=*)  PYTHON="${arg#*=}" ;;
        --doc=*)     DOC="${arg#*=}" ;;
        --licenses=*) LICENSES="${arg#*=}" ;;
        --arch=*)    ARCH="${arg#*=}" ;;
        --clean)     CLEAN="y" ;;
        --help)
            echo "Usage: ./build_macos.sh [--app=y/n] [--python=y/n] [--doc=y/n] [--licenses=y/n] [--arch=native/arm64/x86_64/universal] [--clean]"
            exit 0 ;;
        *) echo "Unknown: $arg"; exit 1 ;;
    esac
done

if [ "$CLEAN" = "y" ]; then
    echo "Cleaning..."
    rm -rf target/
    exit 0
fi

if [ ! -d "$MUJOCO_DYNAMIC_LINK_DIR" ]; then
    echo "Error: MuJoCo not found at $MUJOCO_DYNAMIC_LINK_DIR. Run fetch_mujoco_macos.sh first."
    exit 1
fi

# --- Helper Functions ---
toggle_patch() {
    # $1: "enable" or "disable"
    if [ "$1" = "enable" ]; then
        sed -i '' 's/^# \[patch\.crates-io\]/[patch.crates-io]/' Cargo.toml
        sed -i '' 's/^# glutin = /glutin = /' Cargo.toml
        cargo update -p glutin -q
    else
        sed -i '' 's/^\[patch\.crates-io\]/# [patch.crates-io]/' Cargo.toml
        sed -i '' 's/^glutin = /# glutin = /' Cargo.toml
        cargo update -p glutin -q
    fi
}

# --- Cleanup Trap ---
cleanup() {
    toggle_patch disable
}
trap cleanup EXIT

# Apply patch for macOS build
toggle_patch enable

# --- Build Logic ---

# 1. Simulation App (Rust)
if [ "$APP" = "y" ]; then
    echo "Building Simulation App ($ARCH)..."
    mkdir -p "$OUTPUT/simulation-app"
    
    case $ARCH in
        native)
            cargo build --release --locked --bin simulation-app
            cp target/release/simulation-app "$OUTPUT/simulation-app/"
            ;;
        universal)
            rustup target add aarch64-apple-darwin x86_64-apple-darwin
            cargo build --release --locked --target aarch64-apple-darwin --bin simulation-app
            cargo build --release --locked --target x86_64-apple-darwin --bin simulation-app
            lipo -create -output "$OUTPUT/simulation-app/simulation-app" \
                target/aarch64-apple-darwin/release/simulation-app \
                target/x86_64-apple-darwin/release/simulation-app
            ;;
        *)
            TARGET="${ARCH}-apple-darwin"
            rustup target add "$TARGET"
            cargo build --release --locked --target "$TARGET" --bin simulation-app
            cp "target/$TARGET/release/simulation-app" "$OUTPUT/simulation-app/"
            ;;
    esac
    
    cp -r simulation-app/www "$OUTPUT/simulation-app/"

    # Bundle MuJoCo
    if [ -e "$MUJOCO_DYNAMIC_LINK_DIR/libmujoco.dylib" ]; then
        # User requested keeping the specific version name and preserving signature
        TARGET_LIB="libmujoco.3.3.7.dylib"
        
        cp -L "$MUJOCO_DYNAMIC_LINK_DIR/libmujoco.dylib" "$OUTPUT/simulation-app/$TARGET_LIB"
        BIN="$OUTPUT/simulation-app/simulation-app"
        
        # Change any mujoco reference to @rpath/libmujoco.3.3.7.dylib
        # We modify the APP binary usage, but we do NOT touch the dylib's ID or signature.
        otool -L "$BIN" | grep "mujoco" | awk '{print $1}' | while read -r lib; do
             echo "Repointing $lib to @rpath/$TARGET_LIB"
             install_name_tool -change "$lib" "@rpath/$TARGET_LIB" "$BIN"
        done
    fi
fi

# 2. Python Bindings
if [ "$PYTHON" = "y" ]; then
    echo "Building Python Bindings..."
    mkdir -p "$OUTPUT/python"
    
    # Python builds are per-arch (wheels). Universal logic just loops.
    case $ARCH in
        native)    TARGETS=("") ;;
        universal) TARGETS=("aarch64-apple-darwin" "x86_64-apple-darwin") ;;
        arm64)     TARGETS=("aarch64-apple-darwin") ;;
        x86_64)    TARGETS=("x86_64-apple-darwin") ;;
        *)         TARGETS=("${ARCH}-apple-darwin") ;;
    esac

    # Ensure targets are installed if not native
    if [ "${TARGETS[0]}" != "" ]; then
        rustup target add "${TARGETS[@]}"
    fi

    pushd simulation > /dev/null
    for target in "${TARGETS[@]}"; do
        ARGS=("--release" "--locked")
        [ -n "$target" ] && ARGS+=("--target" "$target")
        maturin build "${ARGS[@]}"
    done
    popd > /dev/null

    # Collect and Repair Wheels
    cp target/wheels/*.whl "$OUTPUT/python/"
    echo "Repairing wheels..."
    for wheel in "$OUTPUT/python"/*.whl; do
        [ -f "$wheel" ] || continue
        TMP=$(mktemp -d)
        if delocate-wheel -w "$TMP" "$wheel"; then
            mv "$TMP"/*.whl "$wheel"
            echo "Fixed: $(basename "$wheel")"
        else
            echo "Failed to fix: $wheel"
        fi
        rm -rf "$TMP"
    done
fi

# 3. Documentation
if [ "$DOC" = "y" ]; then
    echo "Generating Docs..."
    mkdir -p "$OUTPUT/doc"
    DOCS_RS=y cargo doc -p fuzbai_simulator --no-deps --document-private-items --features python-bindings --locked
    cp -r target/doc/* "$OUTPUT/doc/"
    ln -sf doc/fuzbai_simulator/index.html "$OUTPUT/DOCUMENTATION.html"
fi

# 4. Licenses
if [ "$LICENSES" = "y" ]; then
    echo "Generating Licenses..."
    cargo install cargo-about --locked -q || true
    cargo about generate about.hbs --features python-bindings -o "$OUTPUT/THIRD_PARTY_NOTICES.html"
fi
echo "============================="
echo "Finished! Saved in $OUTPUT"
echo "============================="
