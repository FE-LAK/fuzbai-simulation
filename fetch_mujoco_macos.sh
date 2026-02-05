#!/bin/bash
VERSION=3.3.7
DMG=mujoco-$VERSION-macos-universal2.dmg
URL=https://github.com/google-deepmind/mujoco/releases/download/$VERSION/$DMG

# Download
echo "Downloading $DMG from $URL..."
curl -L -O $URL

# Prepare directory
mkdir -p mujoco-$VERSION/include/mujoco
mkdir -p mujoco-$VERSION/lib

# Mount DMG
echo "Mounting $DMG..."
hdiutil attach $DMG -mountpoint /Volumes/MuJoCo -nobrowse

# Copy contents
echo "Copying files..."
# Copy headers to include/mujoco to match Linux structure expected by bindings
cp -R /Volumes/MuJoCo/mujoco.framework/Versions/Current/Headers/* mujoco-$VERSION/include/mujoco/

# Copy library
# Usually it is libmujoco.3.3.7.dylib, we rename/copy it to libmujoco.dylib for simpler linking
cp /Volumes/MuJoCo/mujoco.framework/Versions/Current/libmujoco.3.3.7.dylib mujoco-$VERSION/lib/libmujoco.dylib
# Also copy the versioned one just in case
cp /Volumes/MuJoCo/mujoco.framework/Versions/Current/libmujoco.3.3.7.dylib mujoco-$VERSION/lib/libmujoco.3.3.7.dylib

# Unmount
echo "Unmounting..."
hdiutil detach /Volumes/MuJoCo

echo "MuJoCo $VERSION for macOS fetched and extracted to mujoco-$VERSION/"
