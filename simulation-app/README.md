## FuzbAI simulation application
This directory contains the source code for the FuzbAI competition runner.
The application serves as a bridge between the [simulation library](../simulation) and the competition participants.

The application manages the match state, including match duration, scoring,
and resets. It provides an HTTP API for retrieving simulation state (camera data)
and sending rod control commands.

## Web application
The static assets for the web interface are located in the [www/](www/) directory.
The application serves these files to provide a real-time visualization
of the field in 2D, similar to how the real table does it.

### API Documentation
Interactive API documentation is available via Swagger UI. Once the application is running,
it can be accessed at `http://127.0.0.1:<team_port>/docs`.

## State variables
The simulation state can be retrieved from the `/Camera/State` or `/State` (experimental) endpoints.

### Ball
- `ball_x`, `ball_y`: Position on the field in millimeters [mm].
- `ball_vx`, `ball_vy`: Velocity in meters per second [m/s].
- `ball_size`: Estimated diameter of the ball in pixels. This value can be used to detect occlusions (e.g., when
  `ball_size` is significantly smaller than the expected value of ~35) and to decide when to switch between cameras.

### Rods
- `rod_position_calib`: Array of 8 values representing normalized translation of all rods on the table, starting from
  the team's own goal. Range: `[0.0, 1.0]`.
- `rod_angle`: Array of 8 values representing rotation of all rods. Range is approximately `[-64.0, 64.0]`, where 64
  units correspond to one full revolution ($2\pi$ radians). Note that on the real table, these values wrap around
  `[-32.0, 32.0]`.

### Encoders (Highly Experimental)
On the physical FuzbAI table, encoder data is currently only usable for the **goalkeeper**, **defense**, and
**attacker** rods at a frequency of 100Hz. The **midfield** rod (offense) provides information at a significantly lower
frequency.

**Note:** This data is strongly experimental and should only be used if the information provided by `/Camera/State` is
insufficient for your use case.

While the simulation matches encoder units to camera units for simplicity, the real table uses unknown/arbitrary units
for encoder data. Users are expected to perform calibration (e.g., calculation of a linear transformation) between
camera-based values and raw encoder values.

## Action format
Rod control commands are sent via a POST request to `/Motors/SendCommand`.

### Motor command structure
- `driveID`: Index of the rod to control (1-4).
  - 1: Goalkeeper (1 player)
  - 2: Defense (2 players)
  - 3: Offense (5 players)
  - 4: Attack (3 players)
- `translationTargetPosition`: Target translation in range `[0.0, 1.0]`.
- `rotationTargetPosition`: Target rotation in range `[-1.0, 1.0]`, where 1.0 represents a full revolution ($2\pi$ radians).
- `translationVelocity`: Maximum translational speed multiplier in range `[0.0, 1.0]`.
- `rotationVelocity`: Maximum rotational speed multiplier in range `[0.0, 1.0]`.

## Running
The application requires specifying the communication ports for both competing teams.

### Standalone Binary
On Linux:
```shell
./simulation-app <team1_port> <team2_port>
```

On Windows:
```shell
.\simulation-app.exe <team1_port> <team2_port>
```

### Development (Cargo)
```shell
cargo run --release -- <team1_port> <team2_port>
```

If no ports are provided, the application defaults to 8080 and 8081.

## Building
Build instructions are available in the root directory's [README](../README.md).
Additional runtime dependencies are identical to those of the simulation library.
