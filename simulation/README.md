## FuzbAI simulation library
This directory contains the source code for the FuzbAI simulation library.
The simulation library is written in Rust.
Python bindings, created with PyO3, are also available.

The underlying physics engine is MuJoCo.

The simulation library can be used to control the simulation through
code, which allows use in machine learning setups

## 3D model
The 3D model of the FuzbAI table, is available under the  `../models/` directory.
The main file is [miza.xml](../models/miza.xml).
DO NOT CHANGE the order of items as the simulation (for performance reasons)
sets some geometry IDs as a Rust constant.

The model is optimized to match the real table, thus no changes
should be necessary.

The only thing that users are optionally expected to change,
is the `<size memory="...">` attribute, which can be lowered
for purposes of large amount of parallel simulation instances.
We determined that a safe value should be above `<size memory="60k">`.
The current value includes a large margin to prevent problems during
the actual competition.

## Examples
Examples are available under the ``../examples/stepping`` directory.

## Building
Build instructions are available in the root directory's [README](../README.md).
