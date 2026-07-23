import pygame
import sys

from simulation.config import SimulationConfig
from simulation.physics import WaterPhysics
from simulation.cuboc_simulator import Simulator
from simulation.scenarios import build_scenario
from simulation.scenarios import build_controller

from control.keycontrol import KeyControl

from vizualization.vizualizer import render_scene


def main():

    pygame.init()

    config = SimulationConfig()

    controller = build_controller(config)
    scenario = build_scenario(config)
    physics_engine = WaterPhysics(config)
    simulator = Simulator(scenario, physics_engine, config)


    screen = pygame.display.set_mode((config.screen_pixels, config.screen_pixels))
    pygame.display.set_caption("CubOc Simplified Simulator")
    clock = pygame.time.Clock()
    ppm = (config.screen_pixels / config.arena_size_meters)

    running = True
    sim_stopped = False

    dt = config.dt

    while running and not sim_stopped:
        clock.tick(config.frame_rate_hz)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        controller.update(scenario.robot, scenario, dt)
        scenario.robot.update_hardware_components(dt)
        sim_stopped = simulator.step(dt)
        render_scene(screen, scenario, config, ppm)

        pygame.display.flip()

    pygame.quit()

    sys.exit()


if __name__ == "__main__":

    main()