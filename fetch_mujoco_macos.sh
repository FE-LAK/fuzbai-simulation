#!/bin/bash
set -euo pipefail

VERSION=3.3.7
DMG="mujoco-$VERSION-macos-universal2.dmg"
URL="https://github.com/google-deepmind/mujoco/releases/download/$VERSION/$DMG"
MOUNT="/tmp/mujoco-$$-mnt"
DEST="mujoco-$VERSION/lib"

# Download
curl -L -O "$URL"

# Extract
mkdir -p "$MOUNT" "$DEST"
trap "hdiutil detach '$MOUNT' -quiet || true; rm -rf '$MOUNT'" EXIT
hdiutil attach "$DMG" -mountpoint "$MOUNT" -nobrowse -quiet
cp -R "$MOUNT/mujoco.framework" "$DEST/"

# Symlink
(cd "$DEST" && ln -sf "mujoco.framework/Versions/Current/libmujoco.$VERSION.dylib" libmujoco.dylib)
echo "MuJoCo installed to $DEST"