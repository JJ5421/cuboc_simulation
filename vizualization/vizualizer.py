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

    if robot.thruster.thrust <= 0.0:
        return

    plume_origin = (
        robot.position
        + body_forward * -robot.body_length / 2.0
    )

    plume_tip = (plume_origin - body_forward * 0.5 * robot.thruster.thrust / robot.thruster.max_thrust)

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


def render_scene(screen, scenario, config, ppm):

    robot = scenario.robot

    screen.fill((12, 50, 95))

    # Draw waypoints.
    if hasattr(scenario, "waypoints"):
        font = pygame.font.Font(None, 24)
        for index, waypoint in enumerate(scenario.waypoints):

            if index < scenario.waypoint_index:
                color = (0, 255, 0)
            else:
                color = (255, 0, 0)

            center = (int(waypoint[0] * ppm),int(waypoint[1] * ppm))

            pygame.draw.circle(screen,color,center,10)

            text = font.render(str(index),True,(255, 255, 255))

            text_rect = text.get_rect(center=center)

            screen.blit(text,text_rect)

    # Draw debris.
    if hasattr(scenario, "debris"):

        for debris in scenario.debris:

            if not debris.is_active(scenario.time):
                continue

            position = debris.get_position(
                scenario.time
            )

            pygame.draw.circle(
                screen,
                (120, 120, 120),
                (
                    int(position[0] * ppm),
                    int(position[1] * ppm)
                ),
                int(debris.radius * ppm)
            )

    # Drawing the robot.
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