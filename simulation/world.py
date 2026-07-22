from robot.cuboc_robot import CubocRobot


class SimulationWorld:

    def __init__(self, config):

        self.time = 0.0

        self.robot = CubocRobot()

        # Future stuff not yet implemented
        self.debris = []
        self.waypoints = []