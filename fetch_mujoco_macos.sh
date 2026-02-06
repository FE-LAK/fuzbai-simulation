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

# Mount DMG to a deterministic temporary location
echo "Mounting $DMG..."
mkdir -p "$MOUNT_POINT"
hdiutil attach "$DMG" -mountpoint "$MOUNT_POINT" -nobrowse

# Prepare directory structure (matching Linux layout expected by mujoco-rs)
mkdir -p "$DEST/lib"
mkdir -p "$DEST/include/mujoco"

# --- Find the framework inside the DMG ---
echo "Looking for MuJoCo framework..."
FRAMEWORK=""
for candidate in \
    "$MOUNT_POINT/mujoco.framework" \
    "$MOUNT_POINT/MuJoCo.framework"; do
    if [ -d "$candidate" ]; then
        FRAMEWORK="$candidate"
        break
    fi
done

# Fallback: search for any .framework
if [ -z "$FRAMEWORK" ]; then
    FRAMEWORK=$(find "$MOUNT_POINT" -maxdepth 2 -name "*.framework" -type d 2>/dev/null | head -1)
fi

if [ -z "$FRAMEWORK" ]; then
    echo "Error: Could not find a MuJoCo framework in DMG. Contents:"
    ls -laR "$MOUNT_POINT"
    exit 1
fi
FRAMEWORK_NAME=$(basename "$FRAMEWORK")
echo "Found framework: $FRAMEWORK"

# --- Copy the entire framework into lib/ (as documented by mujoco-rs) ---
echo "Copying framework to $DEST/lib/$FRAMEWORK_NAME ..."
cp -R "$FRAMEWORK" "$DEST/lib/"

# --- Find the actual (non-symlink) versioned dylib inside the copied framework ---
DYLIB=$(find "$DEST/lib/$FRAMEWORK_NAME" -name "libmujoco*.dylib" ! -type l 2>/dev/null | head -1)
if [ -z "$DYLIB" ]; then
    echo "Error: Could not find libmujoco dylib inside framework. Contents:"
    find "$DEST/lib/$FRAMEWORK_NAME" -type f
    exit 1
fi
DYLIB_BASENAME=$(basename "$DYLIB")
echo "Found dylib: $DYLIB ($DYLIB_BASENAME)"

# --- Fix the dylib's install_name ---
# The framework dylib has an install_name like
#   @rpath/mujoco.framework/Versions/A/libmujoco.X.Y.Z.dylib
# which causes "library not found" errors at runtime because that path
# does not exist outside the framework bundle.  Change it to a simple
# leaf name so that DYLD_LIBRARY_PATH / @rpath resolution works.
echo "Fixing dylib install_name..."
chmod u+w "$DYLIB"
install_name_tool -id libmujoco.dylib "$DYLIB"
echo "  New install_name:"
otool -D "$DYLIB"

# --- Create the symlink expected by mujoco-rs in lib/ ---
# mujoco-rs looks for "libmujoco.dylib" in MUJOCO_DYNAMIC_LINK_DIR.
RELATIVE_DYLIB="$FRAMEWORK_NAME/Versions/Current/$DYLIB_BASENAME"
(cd "$DEST/lib" && ln -sf "$RELATIVE_DYLIB" libmujoco.dylib)
echo "Created symlink: $DEST/lib/libmujoco.dylib -> $RELATIVE_DYLIB"

echo ""
echo "Done. MuJoCo $VERSION for macOS extracted to $DEST/"
echo ""
echo "Verify:"
ls -la "$DEST/lib/libmujoco.dylib"
otool -D "$DEST/lib/libmujoco.dylib"
