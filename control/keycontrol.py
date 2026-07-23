import pygame

from control.base_controller import BaseController


class KeyControl(BaseController):

    def __init__(self, config):

        super().__init__(config)

    def update(self, robot, world, dt):

        keys = pygame.key.get_pressed()

        config = self.config

        if keys[pygame.K_KP7] or keys[pygame.K_7]:
            robot.left_fin.requested_angular_acceleration = (-robot.left_fin.fin_angular_acceleration_max)
        elif keys[pygame.K_KP8] or keys[pygame.K_8]:
            robot.left_fin.requested_angular_acceleration = (robot.left_fin.fin_angular_acceleration_max)
        else:
            robot.left_fin.requested_angular_acceleration = 0.0


        if keys[pygame.K_KP4] or keys[pygame.K_4]:
            robot.left_fin.requested_linear_velocity = (-robot.left_fin.fin_extension_speed_max)
        elif keys[pygame.K_KP5] or keys[pygame.K_5]:
            robot.left_fin.requested_linear_velocity = (robot.left_fin.fin_extension_speed_max)
        else:
            robot.left_fin.requested_linear_velocity = 0.0

        if keys[pygame.K_KP2] or keys[pygame.K_2]:
            robot.right_fin.requested_angular_acceleration = (-robot.right_fin.fin_angular_acceleration_max)
        elif keys[pygame.K_KP3] or keys[pygame.K_3]:
            robot.right_fin.requested_angular_acceleration = (robot.right_fin.fin_angular_acceleration_max)
        else:
            robot.right_fin.requested_angular_acceleration = 0.0

        if keys[pygame.K_KP0] or keys[pygame.K_0]:
            robot.right_fin.requested_linear_velocity = (-robot.right_fin.fin_extension_speed_max)
        elif (keys[pygame.K_KP_PERIOD] or keys[pygame.K_PERIOD]):
            robot.right_fin.requested_linear_velocity = (robot.right_fin.fin_extension_speed_max)
        else:
            robot.right_fin.requested_linear_velocity = 0.0

        if (keys[pygame.K_KP_PLUS] or keys[pygame.K_EQUALS]):
            robot.thruster.requested_throttle_jerk = (robot.thruster.max_jerk)
        elif (keys[pygame.K_KP_MINUS]or keys[pygame.K_MINUS]):
            robot.thruster.requested_throttle_jerk = (-robot.thruster.max_jerk)

        else:
            robot.thruster.requested_throttle_jerk = 0.0