import numpy as np

class WaterPhysics:

    # Some notably missing physics:
    # 1. Conservation of angular momentum (arms/tentacles/fins are masslessly modeled)
    #       this means that arm rotation (which should counter-rotate the body by virtue of conservation) and
    #       the rotational moment (which changes as mass moves in/out relative to the body) effects are not
    #       being modeled. Arm rotation does currently cause counter-rotation in the body via drag forces, though.
    # 2. The extension/retraction of the arms itself doesn't cause any dynamics. You could imagine how, by conservation
    #       and drag forces, extenting the arm in water might push the body back.
    # 3. The fluid model approximation of wet-movement-mass correction has been removed and isn't included.
    #       it was acting weird and I myself am not certain how theoretically applicable it is here.
    # 4. Obviously, the third dimension and buoyancy!!!

    def __init__(self, config):
        '''
        config must contain:
        - water_density: fluid density rho
        '''

        self.rho = config.water_density

        # Empirical coefficients for rectangular geometries
        self.Cd_perpendicular = 1.28 # Pressure drag coefficient
        self.Cd_parallel = 0.05 # Skin friction drag

        self.Ca_rect = 1.2 # NOT USED YET

    def calculate_rectangle_hydrodynamics(self, center_pos, linear_v, angular_v, orientation_rad, width, length, depth):
        """
        Calculates total force and torque acting on a single independent rectangle in a 2D plane.
        
        Parameters:
        - center_pos: np.array([x, y]) absolute center of the rectangle
        - linear_v: np.array([vx, vy]) absolute linear velocity vector at the center
        - angular_v: float (omega) absolute angular velocity around center
        - orientation_rad: float (theta) absolute heading angle of the rectangle
        - width: float (thickness dimension along local X)
        - length: float (span/extension dimension along local Y)
        - depth: float (3D thickness depth out of the 2D plane)
        """

        # 1. Establish local coordinate axes
        # local +Y is up along the length/span, local +X is right across the width
        cos_t, sin_t = np.cos(orientation_rad), np.sin(orientation_rad)
        local_y_axis = np.array([cos_t, sin_t])
        local_x_axis = np.array([-sin_t, cos_t])

        # 2. Translational drag
        # decompose the velocity of the center into local frame
        v_local_x = np.dot(linear_v, local_x_axis)
        v_local_y = np.dot(linear_v, local_y_axis)

        # areas for reference
        area_x = length*depth
        area_y = width*depth

        # (this is a holdover from when I thoughts asymmetric drag was necessary to swim, I have left it in, in case soft robots can/do exhibit this behavior and we want to apply it)
        if v_local_x < 0.0:
            effective_Cd_perp = self.Cd_perpendicular * 1.0
        else:
            effective_Cd_perp = self.Cd_perpendicular * 1.0

        f_drag_local_x = -0.5 * self.rho * effective_Cd_perp * area_x * np.abs(v_local_x) * v_local_x
        f_drag_local_y = -0.5 * self.rho * self.Cd_parallel * area_y * np.abs(v_local_y) * v_local_y

        # 4. Return to global fram
        force_translation = (f_drag_local_x * local_x_axis) + (f_drag_local_y * local_y_axis)

        # 5. Rotational dampting w/ geometric integration
        # On the subject of this physics/math: I have temporarily trusted the internet                                                      ============> (COME BACK AND CHECK IF/WHEN IMPORTANT) <============
        # Rotational drag torque: Integrate  dTau = -0.5 * rho * Cd * depth * w^2 * r^3 * dr
        # over the length spanning from -length/2 to +length/2:
        # Integral is: 2 * (0.5 * tho * Cd * depth * |w|w * (length/2)^4 / 4)
        # Simulating both halves moving in opposite directions:
        torque_rotational = -0.25 * self.rho * self.Cd_perpendicular * depth * ((length / 2.0)**4) * np.abs(angular_v) * angular_v

        return force_translation, torque_rotational

# Note to self: if the thruster calculates this natively, and the geometry doesn't the structure of the package is confusing.
# The idea here was that the thruster works physics-independent, but drag isn't (as in, water causes the drag) but this should
# probably be edited to increase the simplicity and generality of this package.