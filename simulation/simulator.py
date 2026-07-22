import numpy as np

from scipy.integrate import solve_ivp


class Simulator:

    def __init__(self, world, physics_engine, config):

        self.world = world
        self.physics_engine = physics_engine
        self.config = config

    def get_physics_derivatives(self, t, state_vector):

        robot = self.world.robot
        robot.position = np.array([state_vector[0], state_vector[1]])
        robot.velocity = np.array([state_vector[2],state_vector[3]])
        robot.heading = state_vector[4]
        robot.angular_velocity = (state_vector[5])

        F_fluid, T_fluid = robot.calculate_hydrodynamics(self.physics_engine)
        F_propulsion, T_propulsion = (robot.thruster.compute_thrust_forces(robot.heading))

        F_net = (F_fluid + F_propulsion)
        T_net = (T_fluid + T_propulsion)

        linear_accel = (F_net / robot.dry_mass)
        angular_accel = (T_net / robot.dry_moment_of_inertia)

        return [

            robot.velocity[0],
            robot.velocity[1],

            linear_accel[0],
            linear_accel[1],

            robot.angular_velocity,

            angular_accel

        ]

    def step(self,dt):

        robot = self.world.robot

        current_state = [

            robot.position[0],
            robot.position[1],

            robot.velocity[0],
            robot.velocity[1],

            robot.heading,

            robot.angular_velocity

        ]

        solution = solve_ivp(
            fun=self.get_physics_derivatives,
            t_span=(0, dt),
            y0=current_state,
            method="RK45",
            rtol=1e-5,
            atol=1e-5
        )

        final_state = (solution.y[:, -1])

        robot.position = np.array([
            final_state[0],
            final_state[1]
        ])

        robot.velocity = np.array([
            final_state[2],
            final_state[3]
        ])

        robot.heading = (final_state[4])

        robot.angular_velocity = (final_state[5])

        if abs(robot.angular_velocity) < 1e-4:
            robot.angular_velocity = 0.0

        for idx in range(2):

            if robot.position[idx] < 0:
                robot.position[idx] += (self.config.arena_size_meters)

            elif (robot.position[idx] > self.config.arena_size_meters):
                robot.position[idx] -= (self.config.arena_size_meters)

        self.world.time += dt