# fuzbai-simulation
MuJoCo simulation environment of table football - specifically, the FuzbAI table.

# Packages
This repository contains multiple possible target packages:
- the simulation library (under ``simulation/``);
- the simulation application (under ``simulation-app/``);
- other unrelated packages.


----- 

**Skip everything below unless you're building yourself.**


# Environment setup

A Dockerfile is prepared for reproducible (and manylinux-compatible) builds:
- Dockerfile.minimal (Recommended): for building anything but MuJoCo itself;
- *Dockerfile.full* environment for building the modified `mujoco/` repository
  which enables static linking. Note that the setup files consider dynamic linking.

# Build setup
*Below setup is also described in [here](https://mujoco-rs.readthedocs.io/en/v2.2.x/installation.html#dynamic-linking-official-mujoco-build) under "Manual download"*.

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


# Building

Both of the below cases can be handled via the build script:

- Linux:
  ```shell
  ./build_linux.sh --help
  ```

- Windows:
  ```shell
  .\build_windows.ps1 --help
  ```


## Rust
Standard interface (``cargo build --release``).

Make sure to copy mujoco-3.3.7/lib/*.so files next to the
output binaries or to somewhere where the system can find them
(i.e., the "path") when redistributing.

## Python
Packaging is done with maturin.
Only the simulation library can be compiled to Python bindings:

```shell
cd simulation/
maturin build --release
```
Maturin can be installed with ``pip install -r requirements_py.txt``.

or
```shell
# Linux
./build_linux.sh --app=n --python=y

# Windows
.\build_windows.ps1 --app=n --python=y
```