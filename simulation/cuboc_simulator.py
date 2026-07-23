import numpy as np

from functools import partial
from scipy.integrate import solve_ivp


class Simulator:

    def __init__(self, scenario, physics_engine, config):
        self.scenario = scenario
        self.physics_engine = physics_engine
        self.config = config
        self.maximum_events_per_step = 10

        # this is crazy and silly
        self.events = [
            partial(Simulator.left_angle_min_event, self),
            partial(Simulator.left_angle_max_event, self),
            partial(Simulator.right_angle_min_event, self),
            partial(Simulator.right_angle_max_event, self),
        ]

        for event, direction in zip(self.events, [-1,1,-1,1]):
            event.terminal = True
            event.direction = direction

    def get_physics_derivatives(self, t, state_vector):
        robot = self.scenario.robot
        robot.apply_state_vector(state_vector)

        left_angle = state_vector[6]
        left_omega = state_vector[7]
        left_length = state_vector[8]

        right_angle = state_vector[9]
        right_omega = state_vector[10]
        right_length = state_vector[11]

        angle_min = robot.cuboc_config.tentacle_angle_min
        angle_max = robot.cuboc_config.tentacle_angle_max

        length_min = robot.cuboc_config.fin_base_length * 0.1
        length_max = robot.cuboc_config.fin_base_length * 2.5

        # Fin angular acceleration
        left_alpha = robot.left_fin.relative_angular_acceleration - robot.left_fin.fin_angular_damping_gain * left_omega *  np.abs(left_omega)
        right_alpha = robot.right_fin.relative_angular_acceleration - robot.right_fin.fin_angular_damping_gain * right_omega * np.abs(right_omega)

        left_alpha = float(np.clip(left_alpha, -robot.left_fin.fin_angular_acceleration_max, robot.left_fin.fin_angular_acceleration_max))
        right_alpha = float(np.clip(right_alpha, -robot.right_fin.fin_angular_acceleration_max, robot.right_fin.fin_angular_acceleration_max))

        # Prevent acceleration farther into an active hard stop.
        # This is arguably not needed given the even handling, but it seems to still have some effect in nullifying acceleration into hard stops
        # Note also that angular velocity max has not been included and it not used anymore... parameter still exists for implementation
        if left_angle <= angle_min and left_alpha < 0.0:
            left_alpha = 0.0
        elif left_angle >= angle_max and left_alpha > 0.0:
            left_alpha = 0.0
        if right_angle <= angle_min and right_alpha < 0.0:
            right_alpha = 0.0
        elif right_angle >= angle_max and right_alpha > 0.0:
            right_alpha = 0.0

        # Fin extension rates
        left_length_dot = robot.left_fin.linear_velocity
        right_length_dot = robot.right_fin.linear_velocity

        if left_length <= length_min and left_length_dot < 0:
            left_length_dot = 0

        elif left_length >= length_max and left_length_dot > 0:
            left_length_dot = 0

        if right_length <= length_min and right_length_dot < 0.0:
            right_length_dot = 0.0
        elif right_length >= length_max and right_length_dot > 0.0:
            right_length_dot = 0.0

        robot.left_fin.relative_linear_velocity = left_length_dot
        robot.right_fin.relative_linear_velocity = right_length_dot

        # External hydrodynamics
        fluid_force, fluid_torque = robot.calculate_hydrodynamics(self.physics_engine)
        propulsion_force, propulsion_torque = robot.thruster.compute_thrust_forces(robot.heading)

        net_force = fluid_force + propulsion_force
        external_torque = fluid_torque + propulsion_torque

        # Internal motor reactions
        left_motor_torque = robot.left_fin.inertia_about_hinge * left_alpha
        right_motor_torque = robot.right_fin.inertia_about_hinge * right_alpha

        body_reaction_torque = -left_motor_torque - right_motor_torque

        # Body accelerations
        linear_acceleration = net_force / robot.calculate_total_mass()
        angular_acceleration = (external_torque + body_reaction_torque) / robot.calculate_total_inertia()

        return np.array([
            robot.velocity[0],
            robot.velocity[1],
            linear_acceleration[0],
            linear_acceleration[1],
            robot.angular_velocity,
            angular_acceleration,
            left_omega,
            left_alpha,
            left_length_dot,
            right_omega,
            right_alpha,
            right_length_dot
        ], dtype=float)

    def apply_event_constraints(self, state, event_indices):
        robot = self.scenario.robot

        angle_min = robot.cuboc_config.tentacle_angle_min
        angle_max = robot.cuboc_config.tentacle_angle_max

        length_min = robot.cuboc_config.fin_base_length * 0.1
        length_max = robot.cuboc_config.fin_base_length * 2.5

        state = state.copy()

        left_impact_omega = 0.0
        right_impact_omega = 0.0

        if 0 in event_indices:
            state[6] = angle_min
            left_impact_omega = state[7]

        if 1 in event_indices:
            state[6] = angle_max
            left_impact_omega = state[7]

        if 2 in event_indices:
            state[9] = angle_min
            right_impact_omega = state[10]

        if 3 in event_indices:
            state[9] = angle_max
            right_impact_omega = state[10]

        if 4 in event_indices:
            state[8] = length_min

        if 5 in event_indices:
            state[8] = length_max

        if 6 in event_indices:
            state[11] = length_min

        if 7 in event_indices:
            state[11] = length_max

        # Synchronize geometry before calculating impact inertia.
        robot.apply_state_vector(state)

        total_inertia = robot.calculate_total_inertia()

        impact_angular_momentum = robot.left_fin.inertia_about_hinge * left_impact_omega + robot.right_fin.inertia_about_hinge * right_impact_omega

        # Transfer the fins' stopped relative angular momentum into the body.
        state[5] += impact_angular_momentum / total_inertia

        if 0 in event_indices or 1 in event_indices:
            state[7] = 0.0

        if 2 in event_indices or 3 in event_indices:
            state[10] = 0.0

        return state

    def step(self, dt):

        robot = self.scenario.robot

        current_state = np.asarray(robot.build_state_vector(), dtype=float)

        current_time = 0.0
        event_count = 0

        while current_time < dt:
            
            solution = solve_ivp(
                fun=self.get_physics_derivatives,
                t_span=(current_time, dt),
                y0=current_state,
                method="RK45",
                rtol=1e-6,
                atol=1e-8,
                events=self.events
            )

            if not solution.success:
                raise RuntimeError(f"Physics integration failed: {solution.message}")

            current_state = solution.y[:, -1].copy()
            current_time = float(solution.t[-1])

            if solution.status != 1:
                break

            event_indices = []

            for event_index, event_times in enumerate(solution.t_events):
                if len(event_times) > 0 and abs(event_times[-1] - current_time) <= 1e-8:
                    event_indices.append(event_index)

            if not event_indices:
                break

            current_state = self.apply_event_constraints(current_state, event_indices)

            event_count += 1

            if event_count >= self.maximum_events_per_step:
                raise RuntimeError("Too many joint-limit events occurred during one simulation timestep.")

            current_time = dt

        robot.apply_state_vector(current_state)

        if abs(robot.angular_velocity) < 1e-8:
            robot.angular_velocity = 0.0

        if abs(robot.left_fin.relative_angular_velocity) < 1e-8:
            robot.left_fin.relative_angular_velocity = 0.0

        if abs(robot.right_fin.relative_angular_velocity) < 1e-8:
            robot.right_fin.relative_angular_velocity = 0.0

        for index in range(2):
            if robot.position[index] < 0.0:
                robot.position[index] += self.config.arena_size_meters
            elif robot.position[index] > self.config.arena_size_meters:
                robot.position[index] -= self.config.arena_size_meters


        self.scenario.time += dt

        # We handle waypoint logic here:
        if not hasattr(self.scenario, "waypoints"):
            return False

        if len(self.scenario.waypoints) == 0:
            return False

        distance = np.linalg.norm(robot.position - self.scenario.waypoints[self.scenario.waypoint_index])

        if distance < self.scenario.waypoint_tolerance:
            print(f"Reached waypoint " f"{self.scenario.waypoints[self.scenario.waypoint_index]}.")

            self.scenario.waypoint_index += 1

            if (self.scenario.waypoint_index >= len(self.scenario.waypoints)):
                print("Final waypoint reached.")
                return True

        return False




    def left_angle_min_event(self, t, state):
        if state[7] >= 0.0:
            return 1.0
        return (state[6] - self.scenario.robot.cuboc_config.tentacle_angle_min)

    def left_angle_max_event(self, t, state):
        if state[7] <= 0.0:
            return -1.0
        return (state[6] - self.scenario.robot.cuboc_config.tentacle_angle_max)

    def right_angle_min_event(self, t, state):
        if state[10] >= 0.0:
            return 1.0
        return (state[9] - self.scenario.robot.cuboc_config.tentacle_angle_min)

    def right_angle_max_event(self, t, state):
        if state[10] <= 0.0:
            return -1.0
        return (state[9] - self.scenario.robot.cuboc_config.tentacle_angle_max)
