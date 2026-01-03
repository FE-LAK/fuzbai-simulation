VERSION=3.3.7

wget -q -N https://github.com/google-deepmind/mujoco/releases/download/$VERSION/mujoco-$VERSION-linux-x86_64.tar.gz
sync
tar --no-same-owner -xzf mujoco-$VERSION-linux-x86_64.tar.gz
