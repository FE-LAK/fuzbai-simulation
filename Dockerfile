# Docker file for building the simulation in Rust only (no maturin).
# Docker file from MuJoCo-rs (https://github.com/davidhozic/mujoco-rs/blob/main/Dockerfile.ubuntu)
FROM mujoco-build:latest


ENV ENVIRONMENT_PATH=/root/env/
WORKDIR $ENVIRONMENT_PATH

# Install Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

# Prepare Python
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

RUN uv venv -p 3.11
RUN echo "source /$ENVIRONMENT_PATH/.venv/bin/activate" >> /root/.bashrc

WORKDIR /build/
# docker run --rm -v $(pwd):/build/ -it fuzbai-rust
