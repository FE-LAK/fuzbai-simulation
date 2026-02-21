from threading import Thread

import fuzbai_simulator as fs
import math
import time


# The actual simulation in a separate thread to improve performance.
def physics(simulator: fs.FuzbAISimulator):
    while simulator.viewer_running():
        # Terminated: goal scored or ball outside the field.
        # Truncated: ball stopped moving.
        if simulator.terminated() or simulator.truncated():
            simulator.reset_simulation()

        # Query delayed observation (actual measurement).
        # (ball_x [mm], ball_y [mm], ball_vx [m/s], ball_vy [m/s], rod_positions [0, 1], rod_rotations [-64, 64])
        obs = simulator.delayed_observation(
            fs.PlayerTeam.Red,  # Team
            None  # Delay override. When None is passed,
                  # delay will be equal to the value of simulated_delay_s (passed in constructor).
        )

        # Set motor commands
        simulator.set_motor_command([
            (
                i + 1,  # Player index (1 => goalkeeper)
                0.5 * (math.sin(time.time()) + 1),  # Target translation (0-1)
                0.5 * math.sin(2 * time.time()),  # Target rotation (-1, 1)
                1.0,  # Full translational velocity
                1.0   # Full rotational velocity
            ) for i in range(4)
        ], fs.PlayerTeam.Red)

        # Move time forward in simulation.
        simulator.step_simulation()

        # Draw some measurements or estimates (e.g., visualize your corrected values) inside the viewer.
        simulator.show_estimates(
            # Show the measured ball position
            [obs[0], obs[1], 0.0],
            # Show the measured rod position on the viewer
            # (rod_id, extension [0-1], rotation [-64, 64], mask)
            #       The mask is an 8 bit mask, where each bit corresponds to a figure on the rod.
            #       Enable individual bits to enable display for individual figure.
            [(0, obs[4][0], obs[5][0], 0b1)]
        )


# Create a simulation instance.
# Multiple of these can exist, just not with the viewer enabled.
sim = fs.FuzbAISimulator(
    # internal_step_factor: .step_simulation() = N * (2 ms)
    # sample_steps: save state to delay buffer every N * (2 ms). .delayed_observation() returns discrete samples every N * 2ms.
    internal_step_factor=10, sample_steps=5,
    realtime=True,
    simulated_delay_s=0.055,
    model_path=None,
    visual_config=fs.VisualConfig(
        trace_length=0, trace_ball=False,
        trace_rod_mask=0x0, enable_viewer=True
    )
)

# Disable the built-in agent on the red team.
sim.set_external_mode(fs.PlayerTeam.Red, True)

# Enable the built-in agent on the blue team.
sim.set_external_mode(fs.PlayerTeam.Blue, False)

# Start the physics simulation.
physics_thread = Thread(target=physics, args=(sim,))
physics_thread.start()

# Control the viewer in the main thread.
viewer = fs.ViewerProxy()
viewer.render_loop()
physics_thread.join()
