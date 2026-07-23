import numpy as np

from robot.cuboc_robot import CubocRobot
from control.keycontrol import KeyControl
from control.thrust_controller import ThrustController
from simulation.debris import Debris
#from control.waypoint_controller import WaypointController
#from control.nod_controller import NODController


class BaseScenario:

    def __init__(self, config):

        self.config = config
        self.time = 0.0

        if self.config.robot_type == "cuboc":
            self.robot = CubocRobot()
            self.robot.controller = self.config.controller_type
        else:
            print("Robot Type Not Known")

        self.debris = []
        self.waypoints = []

        # Default robot state
        self.robot.position = np.array([0.0, 0.0])
        self.robot.velocity = np.array([0.0, 0.0])
        self.robot.heading = 0.0
        self.robot.angular_velocity = 0.0


class ManualScenario(BaseScenario):

    def __init__(self, config):
        super().__init__(config)

        # Override initial conditions
        self.robot.position = np.array([4.0, 7.5])
        self.robot.velocity = np.array([0.0, 0.0])
        self.robot.heading = -np.pi
        self.robot.angular_velocity = 0.0


class StraightLineScenario(BaseScenario):

    def __init__(self, config):
        super().__init__(config)

        # Override initial conditions
        self.robot.position = np.array([4.0, 7.5])
        self.robot.velocity = np.array([0.0, 0.0])
        self.robot.heading = np.pi
        self.robot.angular_velocity = 0.0

        self.waypoint_index = 0
        self.waypoint_tolerance = self.config.waypoint_tolerance
        self.waypoints = [np.array([4.0, 0.5])]

        self.debris = [
            # Crosses left-to-right after 2 seconds.
            Debris(start_time=2.0, points=[[-1.0, 5.0],[9.0, 5.0]], speeds=[0.75], radius=0.15),

            # Diagonal crossing after 8 seconds.
            Debris(start_time=8.0, points=[[9.0, 1.0],[-1.0, 7.0]], speeds=[0.5],radius=0.15)
        ]


def build_scenario(config):

    if config.scenario == "manual":

        return ManualScenario(config)

    if config.scenario == "straight_line":

        return StraightLineScenario(config)

    raise ValueError(f"Unknown scenario: {config.scenario}")

def build_controller(config):

    if config.controller_type == "manual":
        return KeyControl(config)

    if config.controller_type == "thrust":
        return ThrustController(config)

    # if config.controller_type == "nod":
    #     return NODController(config)

    raise ValueError(f"Unknown controller type: "f"{config.controller_type}")