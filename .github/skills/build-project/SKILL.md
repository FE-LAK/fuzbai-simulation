---
name: build-project
description: >
  Builds the project using the platform-specific build scripts.
  Use this when asked to build, compile, or produce release artifacts.
  Covers simulation-app, Python bindings, documentation, and license reports.
---

# Build Project

## Build Scripts

Each platform has a dedicated build script in the repository root:

- **Linux**: `./build_linux.sh`
- **macOS**: `./build_macos.sh`
- **Windows**: `./build_windows.ps1`

## Prerequisites

Each build script sources its platform-specific setup script (`setup_linux.sh`, `setup_macos.sh`, `setup_windows.ps1`) automatically. These set the MuJoCo environment variables (`MUJOCO_DYNAMIC_LINK_DIR`, `LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH`).

MuJoCo 3.3.7 must be present at `mujoco-3.3.7/` in the repo root. Use `fetch_mujoco_<os>.sh` to download it if missing.

## Build Targets

All scripts accept the same CLI flags:

| Flag | Default | Description |
|------|---------|-------------|
| `--app=y/n` | `y` | Build the simulation-app binary (competition application) |
| `--python=y/n` | `n` | Build Python bindings via maturin (requires `python-bindings` feature) |
| `--doc=y/n` | `n` | Build Rust documentation (includes Python binding docs) |
| `--licenses=y/n` | `n` | Generate third-party license report (HTML) via cargo-about |
| `--clean` | — | Delete `target/` directory and exit |

macOS additionally supports `--arch=native/arm64/x86_64/universal` (default: `universal`).

## Output

All build artifacts go to `target/BUILD_OUTPUT_HERE/`:

```
target/BUILD_OUTPUT_HERE/
  simulation-app/       # Binary + www/ + MuJoCo shared library
  python/               # Python wheel(s)
  doc/                  # Rust documentation
  THIRD_PARTY_NOTICES.html
  DOCUMENTATION.html    # Symlink to doc index
```

## Quick Reference

```bash
# Build simulation-app only (default)
./build_linux.sh

# Build everything
./build_linux.sh --app=y --python=y --doc=y --licenses=y

# Clean build
./build_linux.sh --clean && ./build_linux.sh

# Development build (faster, without build script)
source setup_linux.sh && cargo build -p simulation-app

# Release build without build script
source setup_linux.sh && cargo build -p simulation-app --release --locked
```

## CLI options (via clap):

| Option | Default | Description |
|--------|---------|-------------|
| `--team1-port` | 8080 | Team 1 HTTP server port |
| `--team2-port` | 8081 | Team 2 HTTP server port |
| `--management-port` | 8000 | Management HTTP server port |
| `--team-host` | 0.0.0.0 | Bind address for team servers |
| `--management-host` | 127.0.0.1 | Bind address for management server |

## Notes

- The `--locked` flag in build scripts ensures reproducible builds from `Cargo.lock`
- The simulation-app binary requires the MuJoCo shared library at runtime (copied to output by build script)
- The `www/` directory contains static web assets (Swagger UI fallback) and is also copied to output
