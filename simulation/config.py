import numpy as np


class SimulationConfig:

    def __init__(self):

        # Simulator Settings
        self.screen_pixels = 800
        self.frame_rate_hz = 100

        self.robot_type = "cuboc"
        self.controller_type = "manual"
        self.scenario = "straight_line"

        self.waypoint_tolerance = 0.1

        # Numerical Settings
        self.dt = 0.01

        self.ode_method = "RK45"
        self.ode_relative_tolerance = 1e-5
        self.ode_absolute_tolerance = 1e-5

        self.angular_velocity_deadband = 1e-4

        # Fluids & Environment
        self.water_density = 1000.0
        self.gravity = np.array([0.0, 0.0])

        self.arena_size_meters = 8.0