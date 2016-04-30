import pygame
import math


class CollisionDetector:
    """
    Detects collision using following techniques
        1.  Axis Aligned Bounding Boxes

    Input:
        Collision Detector takes list of obstacle co-ordinates as input while initializing.
    """

    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.obstacles_AABB = []

    def is_point_colliding(self, point):
        """
        Uses Precomputed Axis Aligned Bounding Boxes to detect the collision on the given point
        :return: True if collision; otherwise False
        """
        x, y, theta = point
        for obst in self.obstacles_AABB:
            if obst[0][0] <= x <= obst[2][0] and obst[0][1] <= y <= obst[2][1]:
                return True
        return False

    def is_path_colliding(self, start_pos, end_pos, epsilon, theta, robot_vel):
        # TODO Delete this method if not needed in future
        """
        **DEPRECATED**
        Check every step along the path is colliding with obstacle or not.
        Note: This uses Incremental Collision Checking, which is not very efficient.
        TODO: Write Subdivision Collision Checking Algorithm if time permits
        :param start_pos: Initial position of robot
        :param end_pos: Final position of robot
        :param epsilon: Time period that the robot is applied with these control inputs
        :param theta: Control Input: Robot's direction of motion
        :param robot_vel: Control Input: Robot Velocity
        :return:
        """
        for time in range(epsilon):
            x_new = start_pos[0] + time * robot_vel * math.cos(theta)
            y_new = start_pos[1] + time * robot_vel * math.sin(theta)
            q_new = x_new, y_new
            if self.is_point_colliding(q_new):
                print "holonomic_extend:: path colliding! Picking another q_rand!"
                return True
        return False

    def is_poly_colliding(self, vertices_list):
        pass

    def compute_AABB(self):
        """
        Computes Axis Aligned Bounding boxes(AABB) for each obstacle in the environment
        :return: list of AABB obstacles
        """
        for obst in self.obstacles:
            xs = []
            ys = []
            for x, y in obst:
                xs.append(x)
                ys.append(y)
            x_min = min(xs) - 1
            x_max = max(xs) + 1
            y_min = min(ys) - 1
            y_max = max(ys) + 1
            bounding_box = [(x_min, y_min), (x_min, y_max), (x_max, y_max), (x_max, y_min)]
            self.obstacles_AABB.append(bounding_box)
            pygame.draw.polygon(pygame.display.get_surface(), (255, 255, 255), bounding_box)
            pygame.display.update()
        return self.obstacles_AABB
