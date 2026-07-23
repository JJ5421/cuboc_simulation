import numpy as np
from robot.body import RobotBody
from robot.fin import Fin
from robot.thruster import JetThruster
from robot.cuboc_config import CubocConfig

class CubocRobot:
    def __init__(self):
        """
        Assembles structural modules and acts as the central state tracker/storage location.
        """
        self.cuboc_config = CubocConfig()
        self.body = RobotBody(self.cuboc_config)
        
        # (this whole idea of "dry" mass is a holdover from including another physical principle which has since been removed, which models how accelerations are slowed in a displaced medium, but it proved troublesome and has been removed for now)
        self.dry_mass = self.body.mass
        self.dry_moment_of_inertia = self.body.inertia
        
        self.body_width = self.body.width
        self.body_length = self.body.length
        self.body_depth = self.body.depth
        
        # Attach independent left and right fin appendages
        self.left_fin = Fin(
            width=self.cuboc_config.fin_width,
            base_length=self.cuboc_config.fin_base_length,
            depth=self.cuboc_config.fin_depth,
            mass=self.cuboc_config.fin_mass,
            ang_accel_max=self.cuboc_config.fin_angular_acceleration_max,
            ang_velocity_max=self.cuboc_config.fin_angular_velocity_max,
            ang_damping_gain=self.cuboc_config.fin_angular_damping_gain,
            side_sign=-1,
            attachment_offset_y=0.0
        )

        self.right_fin = Fin(
            width=self.cuboc_config.fin_width,
            base_length=self.cuboc_config.fin_base_length,
            depth=self.cuboc_config.fin_depth,
            mass=self.cuboc_config.fin_mass,
            ang_accel_max=self.cuboc_config.fin_angular_acceleration_max,
            ang_velocity_max=self.cuboc_config.fin_angular_velocity_max,
            ang_damping_gain=self.cuboc_config.fin_angular_damping_gain,
            side_sign=1,
            attachment_offset_y=0.0
        )
        
        # Mount fixed jet thruster at the back center of the main body chassis
        self.thruster = JetThruster(max_thrust_newtons=self.cuboc_config.max_thrust, local_position=np.array([0.0, -self.cuboc_config.body_length / 2.0]))
        
        # Absolute 2D Cartesian Dynamics State Space Trackers
        self.position = np.array([4.0, 4.0]) # to start:
        self.velocity = np.array([0.0, 0.0])        
        self.heading = 0.0                          
        self.angular_velocity = 0.0                 

    def calculate_hydrodynamics(self, physics):
        """
        Calculates and combines the hydrodynamic forces and torques acting on the body and fins.
        """
        body_force, body_torque = self.body.calculate_hydrodynamics(physics, self.position, self.velocity, self.angular_velocity, self.heading)

        left_fin_position, left_fin_force, left_fin_torque = self.left_fin.calculate_hydrodynamics(physics, self.position, self.velocity, self.angular_velocity, self.heading, self.body_width)
        right_fin_position, right_fin_force, right_fin_torque = self.right_fin.calculate_hydrodynamics(physics, self.position, self.velocity, self.angular_velocity, self.heading, self.body_width)

        # Convert the fin forces into torque around the robot's center
        left_fin_offset = left_fin_position - self.position
        right_fin_offset = right_fin_position - self.position

        left_fin_force_torque = left_fin_offset[0] * left_fin_force[1] - left_fin_offset[1] * left_fin_force[0]
        right_fin_force_torque = right_fin_offset[0] * right_fin_force[1] - right_fin_offset[1] * right_fin_force[0]

        total_force = body_force + left_fin_force + right_fin_force
        total_torque = body_torque + left_fin_torque + right_fin_torque + left_fin_force_torque + right_fin_force_torque

        return total_force, total_torque

    def update_hardware_components(self, dt):

        # Handle fin movement integration
        self.left_fin.relative_angular_acceleration = self.left_fin.requested_angular_acceleration
        self.right_fin.relative_angular_acceleration = self.right_fin.requested_angular_acceleration

        # Handle fin extension
        self.left_fin.linear_velocity = self.left_fin.requested_linear_velocity
        self.right_fin.linear_velocity = self.right_fin.requested_linear_velocity

        # Handle thruster integration
        if self.thruster.throttle <= 0.0 and self.thruster.requested_throttle_velocity < 0.0:
            self.thruster.requested_throttle_velocity = 0.0

        elif self.thruster.throttle >= 1.0 and self.thruster.requested_throttle_velocity > 0.0:
            self.thruster.requested_throttle_velocity = 0.0
        
        self.thruster.throttle += (self.thruster.requested_throttle_velocity * dt)



    def calculate_total_mass(self):

        return (
            self.body.mass
            + self.left_fin.fin_mass
            + self.right_fin.fin_mass
        )

    def calculate_total_inertia(self):

        left_position, _, _, _ = (
            self.left_fin.calculate_global_kinematics(
                self.position,
                self.velocity,
                self.angular_velocity,
                self.heading,
                self.body_width
            )
        )

        right_position, _, _, _ = (
            self.right_fin.calculate_global_kinematics(
                self.position,
                self.velocity,
                self.angular_velocity,
                self.heading,
                self.body_width
            )
        )

        left_offset = left_position - self.position
        right_offset = right_position - self.position

        left_inertia = (
            self.left_fin.inertia_about_center
            + self.left_fin.fin_mass
            * np.dot(left_offset, left_offset)
        )

        right_inertia = (
            self.right_fin.inertia_about_center
            + self.right_fin.fin_mass
            * np.dot(right_offset, right_offset)
        )

        return (
            self.body.inertia
            + left_inertia
            + right_inertia
        )

    def calculate_internal_fin_angular_momentum(self):

        return (
            self.left_fin.inertia_about_hinge * self.left_fin.relative_angular_velocity
            + self.right_fin.inertia_about_hinge * self.right_fin.relative_angular_velocity
        )

    def build_state_vector(self):

        return [

            self.position[0],
            self.position[1],

            self.velocity[0],
            self.velocity[1],

            self.heading,
            self.angular_velocity,

            self.left_fin.relative_angle,
            self.left_fin.relative_angular_velocity,
            self.left_fin.length,

            self.right_fin.relative_angle,
            self.right_fin.relative_angular_velocity,
            self.right_fin.length

        ]

    def apply_state_vector(self, state):

        self.position = np.array([
            state[0],
            state[1]
        ])

        self.velocity = np.array([
            state[2],
            state[3]
        ])

        self.heading = state[4]
        self.angular_velocity = state[5]

        self.left_fin.relative_angle = state[6]
        self.left_fin.relative_angular_velocity = state[7]
        self.left_fin.length = state[8]

        self.right_fin.relative_angle = state[9]
        self.right_fin.relative_angular_velocity = state[10]
        self.right_fin.length = state[11]