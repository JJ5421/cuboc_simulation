import numpy as np

from control.base_controller import BaseController


class ThrustController(BaseController):

    def __init__(self, config):

        super().__init__(config)

        self.kp = 5.0
        self.max_thrust = 10.0

    def update(self, robot, scenario, dt):

        if len(scenario.waypoints) == 0:
            robot.thruster.thrust = 0.0
            return

        waypoint = scenario.waypoints[scenario.waypoint_index]

        distance = np.linalg.norm(waypoint - robot.position)

        thrust = np.clip(self.kp * distance, 0.0, self.max_thrust)

        robot.thruster.requested_thrust = thrust