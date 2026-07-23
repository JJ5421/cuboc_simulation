import numpy as np

class JetThruster:
    def __init__(self, max_thrust_newtons=15.0, max_jerk_units=5.0, local_position=np.array([0.0, -0.1])):
        """
        Fixed-direction jet propulsion tracking smooth abstract throttle demands.
        """
        self.max_thrust = max_thrust_newtons
        self.max_jerk = max_jerk_units
        self.local_position = local_position
        
        # actual thrust output
        self.thrust = 0.0 
        
        # control interface request
        self.requested_throttle_jerk = 0.0
        self.requested_thrust = 0.0

    def compute_thrust_forces(self, body_heading):
        """
        Calculates global force vectors and local torque offsets based 
        on the robot's current validated heading configuration.
        """
        thrust_magnitude = self.thrust
        local_force = np.array([0.0, thrust_magnitude])

        # Transform local force vector to Global Coordinates
        cos_h, sin_h = np.cos(body_heading), np.sin(body_heading)
        rotation_matrix = np.array([
            [cos_h, -sin_h],
            [sin_h,  cos_h]
        ])
        global_force = rotation_matrix @ local_force

        # Torque generation via cross product (r_arm x F_local)
        torque = self.local_position[0] * local_force[1] - self.local_position[1] * local_force[0]


        return global_force, torque
