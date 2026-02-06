#!/bin/bash
set -e

VERSION=3.3.7
DMG=mujoco-$VERSION-macos-universal2.dmg
URL=https://github.com/google-deepmind/mujoco/releases/download/$VERSION/$DMG
MOUNT_POINT=/tmp/mujoco-dmg-$$
DEST=mujoco-$VERSION

# Download
if [ ! -f "$DMG" ]; then
    echo "Downloading $DMG from $URL..."
    curl -L -O "$URL"
fi

# Ensure cleanup on exit
cleanup() {
    if [ -n "$MOUNT_POINT" ] && [ -d "$MOUNT_POINT" ]; then
        echo "Unmounting DMG..."
        hdiutil detach "$MOUNT_POINT" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# Mount DMG
echo "Mounting $DMG..."
mkdir -p "$MOUNT_POINT"
hdiutil attach "$DMG" -mountpoint "$MOUNT_POINT" -nobrowse

FRAMEWORK="$MOUNT_POINT/mujoco.framework"
DYLIB_NAME="libmujoco.$VERSION.dylib"

if [ ! -d "$FRAMEWORK" ]; then
    echo "Error: Could not find $FRAMEWORK in DMG."
    exit 1
fi

# Copy the framework into lib/ (required by mujoco-rs)
mkdir -p "$DEST/lib"
cp -R "$FRAMEWORK" "$DEST/lib/"

# Fix the dylib's install_name so DYLD_LIBRARY_PATH / @rpath resolution works
DYLIB="$DEST/lib/mujoco.framework/Versions/A/$DYLIB_NAME"
chmod u+w "$DYLIB"
install_name_tool -id libmujoco.dylib "$DYLIB"

# Create the symlink expected by mujoco-rs in lib/
(cd "$DEST/lib" && ln -sf "mujoco.framework/Versions/Current/$DYLIB_NAME" libmujoco.dylib)

echo "Done. MuJoCo $VERSION for macOS extracted to $DEST/lib/"
