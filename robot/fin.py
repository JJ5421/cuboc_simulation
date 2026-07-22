import numpy as np

class Fin:
    def __init__(self, width=0.04, base_length=0.15, depth=0.1, mass=0.05,  ang_accel_max=10.0, ang_velocity_max=10.0, ang_damping_gain=20.0,  side_sign=1, attachment_offset_y=0.0,):
        """
        Independent fin/tentacle component with abstract control interface trackers
        for both rotation (pivot) and elongation (extension).
        """
        self.width = width
        self.base_length = base_length
        self.depth = depth
        self.side_sign = side_sign
        self.attachment_offset_y = attachment_offset_y

        self.fin_mass = mass

        # DYNAMIC GEOMETRY STATES (Read directly by Physics / Visualizer)
        self.length = base_length
        self.relative_angle = 0.0
        
        # JOINT SPEEDS AND MAXES (Read directly by Physics)
        self.relative_angular_velocity = 0.0
        self.relative_linear_velocity = 0.0

        self.fin_angular_acceleration_max = ang_accel_max
        self.fin_angular_velocity_max = ang_velocity_max
        self.fin_angular_damping_gain = ang_damping_gain

        
        # CONTROL INTERFACE (Written by manual keyboard)
        self.requested_angular_acceleration = 0.0   
        self.requested_linear_velocity = 0.0

    @property
    def volume(self):
        return self.length * self.width * self.depth

    @property
    def density(self):
        return self.fin_mass / self.volume

    @property
    def center_of_mass_from_hinge(self):
        return self.length / 2.0

    @property
    def inertia_about_center(self):
        return (self.fin_mass * (self.width**2 + self.length**2) / 12.0)

    @property
    def inertia_about_hinge(self):
        return (self.inertia_about_center + self.fin_mass * self.center_of_mass_from_hinge**2)

    @property
    def inertia_length_derivative(self):
        return (2.0 * self.fin_mass * self.length / 3.0)

    @property
    def inertia_rate(self):
        return (self.inertia_length_derivative * self.relative_linear_velocity)

    def calculate_global_kinematics(self, body_position, body_velocity, body_angular_velocity, body_heading, body_width):
        """
        Calculates the global position, velocity, orientation, and angular velocity
        of the fin assuming rigid-body kinematics.
        """
        cos_b, sin_b = np.cos(body_heading), np.sin(body_heading)
        body_forward = np.array([-sin_b, cos_b])
        body_right = np.array([cos_b, sin_b])

        # Rigid attachment offset from the center of gravity along the body's lateral axis
        r_attach = (self.side_sign * body_right * (body_width / 2.0)) + (body_forward * self.attachment_offset_y)

        # Total absolute orientation of this fin segment
        fin_heading = body_heading + self.relative_angle
        fin_direction = np.array([np.cos(fin_heading), np.sin(fin_heading)])

        # The fin's true center depends on its current length and angle
        r_hinge_to_center = self.side_sign * fin_direction * (self.length / 2.0)
        r_fin_center_local = r_attach + r_hinge_to_center
        fin_position = body_position + r_fin_center_local

        # Kinematic velocity composition containing BOTH rigid spin and active motorized swing
        # 1. Base rigid velocity from the body's movement and spin
        v_rigid = body_velocity + np.array([-body_angular_velocity * r_fin_center_local[1], body_angular_velocity * r_fin_center_local[0]])
        
        # 2. Active motorized velocity from the arm SWINGING (pivoting) around its own hinge joint
        v_motor_pivot = np.array([-self.relative_angular_velocity * r_hinge_to_center[1], self.relative_angular_velocity * r_hinge_to_center[0]])
        
        # Combined velocity vector ignores linear extension speeds per your request
        fin_velocity = v_rigid + v_motor_pivot

        # Total absolute rotation speed of this fin segment
        fin_angular_velocity = body_angular_velocity + self.relative_angular_velocity

        return fin_position, fin_velocity, fin_heading, fin_angular_velocity

    def calculate_hydrodynamics(self, physics, body_position, body_velocity, body_angular_velocity, body_heading, body_width):
        """
        Calculates the hydrodynamic force and torque acting on the fin.
        """
        fin_position, fin_velocity, fin_heading, fin_angular_velocity = self.calculate_global_kinematics(body_position, body_velocity, body_angular_velocity, body_heading, body_width)

        # Compute hydrodynamic coefficients for the fin segment
        fin_force, fin_torque = physics.calculate_rectangle_hydrodynamics(center_pos=fin_position, linear_v=fin_velocity, angular_v=fin_angular_velocity, orientation_rad=fin_heading, width=self.width, length=self.length, depth=self.depth)

        return fin_position, fin_force, fin_torque