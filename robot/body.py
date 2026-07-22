import numpy as np

class RobotBody:
    def __init__(self, config):
        """
        Tracks structural footprint and native dry inertia of a rectangular prism 
        undergoing 2D planar motion, using independent dimensions from config.
        """
        # Read independent dimensions and density from config
        self.width = config.body_width    # Local X dimension (meters)
        self.length = config.body_length  # Local Y dimension (meters)
        self.depth = config.body_depth    # Local Z dimension / Height (meters)
        self.mass = config.mass
        
        # Calculate true physical rigid properties (Dry states)
        self.volume = self.width * self.length * self.depth
        self.material_density = self.mass / self.volume
        
        # Moment of Inertia for a rectangular prism rotating around the Z-axis.
        # Notice that height (depth) affects mass, but not the geometric distribution formula.
        self.inertia = (1.0 / 12.0) * self.mass * (self.width**2 + self.length**2)

    def calculate_hydrodynamics(self, physics, position, velocity, angular_velocity, heading):
        """
        Calculates the hydrodynamic force and torque acting on the robot body.
        """
        body_force, body_torque = physics.calculate_rectangle_hydrodynamics(center_pos=position, linear_v=velocity, angular_v=angular_velocity, orientation_rad=heading, width=self.width, length=self.length, depth=self.depth)

        return body_force, body_torque