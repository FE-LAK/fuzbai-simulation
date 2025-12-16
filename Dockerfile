# Docker file for building MuJoCo. This should match MuJoCo's official build environment.
ARG UBUNTU_VERSION=22.04
FROM ubuntu:${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive

# Pre-answer the keyboard questions
RUN echo "keyboard-configuration  keyboard-configuration/layoutcode string us" | debconf-set-selections && \
    echo "keyboard-configuration  keyboard-configuration/xkb-keymap select us" | debconf-set-selections

RUN apt update

# Build tools
RUN apt install -y \
    build-essential \
    cmake \
    curl \
    libgl1-mesa-dev \
    libwayland-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxkbcommon-dev \
    libxrandr-dev \
    libxi-dev \
    ninja-build \
    git \
    pkg-config \
    clang-13

# Cleanup
# Remove cached package files (*.deb)
RUN apt clean

# Remove package files that can no longer be downloaded
RUN apt autoclean

# Remove packages that were installed as dependencies but are no longer needed
RUN apt autoremove -y

# Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Python dependencies
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"
RUN uv venv -p 3.11

RUN echo "source /$ENVIRONMENT_PATH/.venv/bin/activate" >> /root/.bashrc
COPY requirements_py.txt /tmp/pydeps/
RUN uv pip install -r /tmp/pydeps/requirements_py.txt
RUN rm -rf /tmp/pydeps/

WORKDIR /build/

# Commands.
# Note that we enable IPO with -DBUILD_SHARED_LIBS:BOOL=OFF and
# tell CMake to build static libs with -DBUILD_SHARED_LIBS:BOOL=OFF.
# -----------------------------------------
# docker buildx build . -t fuzbai
# -----------------------------------------
# docker run --rm --volume .:/build/ -it fuzbai:latest
# -----------------------------------------
# cmake -S . -B build \
#     -DCMAKE_BUILD_TYPE:STRING=Release \
#     -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON \
#     -DCMAKE_INSTALL_PREFIX:STRING=${TMPDIR}/mujoco_install \
#     -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
#     -DMUJOCO_BUILD_TESTS=OFF \
#     -DBUILD_SHARED_LIBS:BOOL=OFF \
#     -G Ninja \
#     -DCMAKE_C_COMPILER:STRING=clang-13 \
#     -DCMAKE_CXX_COMPILER:STRING=clang++-13 \
#     -DMUJOCO_HARDEN:BOOL=ON
# -----------------------------------------
# cmake --build build --parallel --target mujoco --config=Release
# -----------------------------------------
# export MUJOCO_STATIC_LINK_DIR=$(realpath mujoco/build/lib/)
# cargo run ...
# -----------------------------------------
# maturin build --release
# -----------------------------------------
