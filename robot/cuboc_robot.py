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
        """
        Hardware constraint layer. Processes abstract velocity requests, checks limits, 
        and integrates component positions on the main loop.
        """
        # 1. Handle Fins Limits and Kinematic Integration
        fins = [
            {
                "fin": self.left_fin,  
                "angle_min": self.cuboc_config.tentacle_angle_min, "angle_max": self.cuboc_config.tentacle_angle_max,
                "len_min": self.cuboc_config.fin_base_length * 0.1, "len_max": self.cuboc_config.fin_base_length * 2.5
            },
            {
                "fin": self.right_fin, 
                "angle_min": self.cuboc_config.tentacle_angle_min, "angle_max": self.cuboc_config.tentacle_angle_max,
                "len_min": self.cuboc_config.fin_base_length * 0.1, "len_max": self.cuboc_config.fin_base_length * 2.5
            }
        ]

        for item in fins:
            fin = item["fin"]
            
            # Net actuator torque with angular-velocity damping
            commanded_torque = fin.requested_angular_acceleration
            net_torque = commanded_torque - fin.fin_angular_damping_gain * fin.relative_angular_velocity

            # Convert torque to angular acceleration
            angular_acceleration = net_torque #/ fin.moment_of_inertia

            # Clamp acceleration
            angular_acceleration = float(np.clip(angular_acceleration, -fin.fin_angular_acceleration_max, fin.fin_angular_acceleration_max))

            # At the lower limit, block motion and acceleration farther downward
            if fin.relative_angle <= item["angle_min"]:
                fin.relative_angle = item["angle_min"]

                if fin.relative_angular_velocity < 0.0:
                    fin.relative_angular_velocity = 0.0

                if angular_acceleration < 0.0:
                    angular_acceleration = 0.0

            # At the upper limit, block motion and acceleration farther upward
            elif fin.relative_angle >= item["angle_max"]:
                fin.relative_angle = item["angle_max"]

                if fin.relative_angular_velocity > 0.0:
                    fin.relative_angular_velocity = 0.0

                if angular_acceleration > 0.0:
                    angular_acceleration = 0.0

            # Integrate acceleration into velocity
            fin.relative_angular_velocity += angular_acceleration * dt

            # Clamp angular velocity
            fin.relative_angular_velocity = float(np.clip(fin.relative_angular_velocity, -fin.fin_angular_velocity_max, fin.fin_angular_velocity_max))

            # Integrate velocity into angle
            fin.relative_angle += fin.relative_angular_velocity * dt

            # Catch crossing a limit during this timestep
            if fin.relative_angle < item["angle_min"]:
                fin.relative_angle = item["angle_min"]
                fin.relative_angular_velocity = 0.0
            elif fin.relative_angle > item["angle_max"]:
                fin.relative_angle = item["angle_max"]
                fin.relative_angular_velocity = 0.0

            # Length limits check -----
            fin.relative_linear_velocity = fin.requested_linear_velocity
            if fin.length <= item["len_min"] and fin.requested_linear_velocity < 0:
                fin.length = item["len_min"]
                fin.relative_linear_velocity = 0.0 

            elif fin.length >= item["len_max"] and fin.requested_linear_velocity > 0:
                fin.length = item["len_max"]
                fin.relative_linear_velocity = 0.0   
            
            # Update length state
            fin.length += fin.relative_linear_velocity * dt

        # 2. Handle Jet Thruster Integration
        self.thruster.throttle += self.thruster.requested_throttle_velocity * dt
        if self.thruster.throttle <= 0.0:
            self.thruster.throttle = 0.0
            self.thruster.requested_throttle_velocity = 0.0
            
        elif self.thruster.throttle >= 1.0:
            self.thruster.throttle = 1.0
            self.thruster.requested_throttle_velocity = 0.0