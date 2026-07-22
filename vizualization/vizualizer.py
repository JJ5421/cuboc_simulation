import pygame
import numpy as np


def draw_rotated_rect(surface, color, center_pixels, width_pixels, length_pixels, heading_rad):

    rect_surf = pygame.Surface(
        (int(width_pixels), int(length_pixels)),
        pygame.SRCALPHA
    )

    rect_surf.fill(color)

    rotated_surf = pygame.transform.rotate(
        rect_surf,
        -np.degrees(heading_rad)
    )

    new_rect = rotated_surf.get_rect(
        center=(
            int(center_pixels[0]),
            int(center_pixels[1])
        )
    )

    surface.blit(rotated_surf, new_rect.topleft)


def draw_thrust_line(screen, robot, ppm, config, body_forward):

    if robot.thruster.throttle <= 0.0:
        return

    plume_origin = (
        robot.position
        + body_forward * -robot.body_length / 2.0
    )

    plume_tip = (
        plume_origin
        - body_forward * robot.thruster.throttle
    )

    pygame.draw.line(
        screen,
        (245, 110, 25),
        (
            int(plume_origin[0] * ppm),
            int(plume_origin[1] * ppm)
        ),
        (
            int(plume_tip[0] * ppm),
            int(plume_tip[1] * ppm)
        ),
        4
    )


def render_scene(screen, world, config, ppm):

    robot = world.robot

    screen.fill((12, 50, 95))

    cos_b = np.cos(robot.heading)
    sin_b = np.sin(robot.heading)

    body_forward = np.array([
        -sin_b,
        cos_b
    ])

    body_right = np.array([
        cos_b,
        sin_b
    ])

    draw_thrust_line(
        screen,
        robot,
        ppm,
        config,
        body_forward
    )

    left_attach = (
        robot.position
        - body_right * (robot.body_width / 2.0)
    )

    left_fin_heading = (
        robot.heading
        + robot.left_fin.relative_angle
    )

    left_fin_center = (
        left_attach
        + np.array([
            -np.cos(left_fin_heading),
            -np.sin(left_fin_heading)
        ]) * (robot.left_fin.length / 2.0)
    )

    draw_rotated_rect(
        screen,
        (190, 195, 200),
        left_fin_center * ppm,
        robot.left_fin.length * ppm,
        robot.left_fin.width * ppm,
        left_fin_heading
    )

    right_attach = (
        robot.position
        + body_right * (robot.body_width / 2.0)
    )

    right_fin_heading = (
        robot.heading
        + robot.right_fin.relative_angle
    )

    right_fin_center = (
        right_attach
        + np.array([
            np.cos(right_fin_heading),
            np.sin(right_fin_heading)
        ]) * (robot.right_fin.length / 2.0)
    )

    draw_rotated_rect(
        screen,
        (190, 195, 200),
        right_fin_center * ppm,
        robot.right_fin.length * ppm,
        robot.right_fin.width * ppm,
        right_fin_heading
    )

    draw_rotated_rect(
        screen,
        (230, 175, 35),
        robot.position * ppm,
        robot.body_width * ppm,
        robot.body_length * ppm,
        robot.heading
    )

    nose_indicator = (
        robot.position
        + body_forward * (robot.body_length / 2.0)
    )

    pygame.draw.circle(
        screen,
        (255, 30, 30),
        (
            int(nose_indicator[0] * ppm),
            int(nose_indicator[1] * ppm)
        ),
        5
    )