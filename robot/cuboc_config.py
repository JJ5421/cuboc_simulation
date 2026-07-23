import numpy as np

class CubocConfig:

    def __init__(self):

        self.body_length = 0.2
        self.body_width = 0.2
        self.body_depth = 0.2

        self.mass = 3
        self.max_thrust = 5.0
        self.max_jerk = 5.0

        self.fin_width = 0.1
        self.fin_base_length = 0.2
        self.fin_depth = 0.1
        self.fin_mass = 0.1
        self.fin_angular_acceleration_max = 15.0
        self.fin_angular_damping_gain = 15.0
        self.fin_extension_speed_max = 0.25
        #self.fin_angular_velocity_max = 10.0

        self.tentacle_angle_max = (np.pi / 2 + 0.1)
        self.tentacle_angle_min = (-self.tentacle_angle_max)

        self.tentacle_max_length_factor = 2.5
        self.tentacle_min_length_factor = 0.5