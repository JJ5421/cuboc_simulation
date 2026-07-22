import pygame
import sys

from simulation.config import SimulationConfig
from simulation.world import SimulationWorld
from simulation.physics import WaterPhysics
from simulation.simulator import Simulator

from control.keycontrol import KeyControl

from vizualization.vizualizer import render_scene


def build_world(config):

    return SimulationWorld(config)


def build_controller(config):

    return KeyControl(config)


def main():

    pygame.init()

    config = SimulationConfig()
    world = build_world(config)
    controller = build_controller(config)
    physics_engine = WaterPhysics(config)

    simulator = Simulator(world, physics_engine, config)

    screen = pygame.display.set_mode((config.screen_pixels, config.screen_pixels))

    pygame.display.set_caption("CubOc Simplified Simulator")

    clock = pygame.time.Clock()

    ppm = (config.screen_pixels / config.arena_size_meters)

    running = True

    while running:
        clock.tick(config.frame_rate_hz)
        dt = config.dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:

                running = False

        controller.update(world.robot, world, dt)
        world.robot.update_hardware_components(dt)
        simulator.step(dt)
        render_scene(screen, world, config, ppm)

        pygame.display.flip()

    pygame.quit()

    sys.exit()


if __name__ == "__main__":

    main()