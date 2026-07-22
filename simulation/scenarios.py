import numpy as np


class ManualScenario:

    def __init__(self):

        self.robot_position = np.array([4.0, 4.0])

        self.robot_velocity = np.array([0.0, 0.0])

        self.robot_heading = 0.0

        self.robot_angular_velocity = 0.0


def build_scenario(config):

    if config.scenario == "manual":

        return ManualScenario()

    raise ValueError(f"Unknown scenario: " f"{config.scenario}")