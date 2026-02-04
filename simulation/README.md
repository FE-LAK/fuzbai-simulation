## FuzbAI simulation library
This directory contains the source code for the FuzbAI simulation library.
The simulation library is written in Rust.
Python bindings, created with PyO3, are also available.

The underlying physics engine is MuJoCo.

The simulation library can be used to control the simulation through
code, which allows use in machine learning setups.

## State and Action Space
The library uses a standardized coordinate system and unit set, which is mirrored in the
[simulation application](../simulation-app).

### Observation Units
- **Ball Position**: Millimeters [mm] relative to the team's side.
- **Ball Velocity**: Meters per second [m/s].
- **Rod Translation**: Normalized `[0.0, 1.0]` across the rod's physical travel.
- **Rod Rotation**: Internal units where 64 units represent a full $2\pi$ revolution.
  - *Note*: On the real FuzbAI table, these values wrap at `[-32.0, 32.0]`.

### Action Units
- **Target Translation**: Normalized `[0.0, 1.0]`.
- **Target Rotation**: Normalized `[-1.0, 1.0]` where 1.0 is $2\pi$ radians.
- **Velocities**: Normalized `[0.0, 1.0]` multipliers of maximum hardware speed.

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
