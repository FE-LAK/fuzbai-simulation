# fuzbai-simulation
MuJoCo simulation environment of table football - specifically, the FuzbAI table.

## Git LFS setup
This repository uses [Git LFS](https://git-lfs.com/) for large assets. 

If you see small placeholder files **after cloning**, make sure you have `git-lfs` installed on your system. 
The first time you use Git LFS on a new machine, you must run:

```shell
git lfs install
```

Then, to download the actual file contents, run:

```shell
git lfs pull
```

# Packages
This repository contains multiple packages:
- **[Simulation Library](simulation/README.md)**: The core Rust library providing the MuJoCo-based physics engine and Python bindings.
- **[Simulation Application](simulation-app/README.md)**: The competition runner that provides an HTTP API and a web-based dashboard for participants.
- **[Demo FuzbAI Agent](demo-fuzbai-agent/)**: A reference agent implementation for controlling the rods.
- **[Examples](examples/)**: Sample implementations in Rust and Python for both direct library stepping and HTTP-based control.

-----

**Skip everything below unless you're building yourself.**


# Environment setup

A Dockerfile is prepared for reproducible (and manylinux-compatible) builds:
- Dockerfile.minimal (Recommended): for building anything but MuJoCo itself;
- *Dockerfile.full* environment for building the modified `mujoco/` repository
  which enables static linking. Note that the setup files consider dynamic linking.

# Build setup

## Linux

To build simulation dependencies, a path the MuJoCo simulation engine must be configured.

First download and extract MuJoCo:

```shell
./fetch_mujoco_linux.sh
```

Then set the environmental variable to the absolute path to its library files:

```shell
source ./setup_linux.sh
```


Note that if you move the compiled binary to a different computer,
``LD_LIBRARY`` will need to be re-updated.
It also needs to be re-updated each time you reopen the
terminal.

It is best to add "$ORIGIN" to the **RPATH** of the binary
so you don't have to worry about ``LD_LIBRARY`` anymore.
Note that the simulation application already does that.

Alternatively, copy ``mujoco-3.3.7/lib/*.so`` to e.g., ``/usr/local/lib/``
on each PC where this will be used.

The [Rust toolchain](https://rust-lang.org/tools/install/) must be installed to compile on Linux, as well as standard C build tools.

## Windows
In powershell, execute
```shell
.\fetch_mujoco_windows.ps1
```
to fetch and extract MuJoCo.

Then, setup the environmental variables
```shell
.\setup_windows.ps1
```

The [Rust toolchain](https://rust-lang.org/tools/install/) must be installed to compile on Windows,
as well as [Microsoft's C++ build tools](https://learn.microsoft.com/en-us/windows/dev-environment/rust/setup).
The "Desktop development with C++" option should be sufficient.

# Building
## Automatic (script)

- Linux:
  ```shell
  ./build_linux.sh --help
  ```

- Windows:
  ```shell
  .\build_windows.ps1 --help
  ```
## Manual

### Simulation application (HTTP)
Standard interface: `cargo build --release -p simulation-app`.

Make sure to copy MuJoCo's shared libraries (DLL/.so) next to the
output binaries or to somewhere where the system can find them
(i.e., the "path") when redistributing.

### Python bindings (simulation-only)
Packaging is done with maturin.

```shell
cd simulation/
maturin build --release
```
Maturin can be installed with ``pip install -r requirements_py.txt``.

Building must also be done on Windows or on a Linux system that is old enough to be
compatible with the `manylinux_2_35` requirement (glibc 2.35).
This is done for redistribution purposes.

Ubuntu 22.04 matches the `manylinux_2_35` requirement, however older systems
should work as well. A [Dockerfile](./Dockerfile.minimal) is available,
which provides Ubuntu 22.04 and required tools for compilation.

The bindings will work on Windows and any Linux distribution
newer (or equal to in age) than Ubuntu 22.04.
Python 3.8 or newer is required to use the compiled bindings.

#### Installation
Python bindings can be install with pip: `pip install fuzbai_simulator-...-.whl`.
