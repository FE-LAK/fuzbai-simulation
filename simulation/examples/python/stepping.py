from threading import Thread

import fuzbai_simulator as fs


# The actual simulation in a separate thread to improve performance.
def physics(simulator: fs.FuzbAISimulator):
    while simulator.viewer_running():
        # Terminated: goal scored or ball outside the field.
        # Truncated: ball stopped moving.
        if simulator.terminated() or simulator.truncated():
            simulator.reset_simulation()  # Reset physics state.

        # Query delayed observation (actual measurement).
        # (ball_x, ball_y, ball_vx, ball_vy, rod_positions, rod_rotations)
        obs = simulator.delayed_observation(
            fs.PlayerTeam.Red,  # Team
            None  # Delay override. When None is passed,
                  # delay will be equal to the value of simulated_delay_s (passed in constructor).
        )

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
            [(0, obs[4][0], obs[5][0], 0xFF)]
        )


# Create a simulation instance.
# Multiple of these can exist, just not with the viewer enabled.
sim = fs.FuzbAISimulator(
    internal_step_factor=10, sample_steps=5,
    realtime=True,
    simulated_delay_s=0.055,
    model_path=None,
    visual_config=fs.VisualConfig(
        trace_length=0, trace_ball=False,
        trace_rod_mask=0x0, enable_viewer=True
    )
)

# Enable the built-in agent on both sides.
sim.set_external_mode(fs.PlayerTeam.Red, False)
sim.set_external_mode(fs.PlayerTeam.Blue, False)

# Start the physics simulation.
physics_thread = Thread(target=physics, args=(sim,))
physics_thread.start()

# Control the viewer in the main thread.
viewer = fs.ViewerProxy()
viewer.render_loop()
physics_thread.join()
