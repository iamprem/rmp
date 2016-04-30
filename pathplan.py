from operator import add
import sys
import itertools
from dijkstra import *
import random
import math
import numpy as np


class PathPlanner:

    robot_vel = 1.0   # 1 m/s

    def __init__(self, q_init, collision_detector):
        self.q_init = q_init
        self.cDetector = collision_detector
        if self.cDetector.is_point_colliding(q_init):
            print "PathPlanner:: q_init is colliding with an obstacle"
            sys.exit()

    def build_rrt(self, K=100, epsilon=1):
        """
        :param K: Number of nodes to be added to the RRT Tree
        :param epsilon: Amount of time the control input is applied (seconds). Default = 1s
        :return:
        """
        t = Graph()
        t.addVertex(self.q_init)
        q_nearest = None
        q_new = None
        for k in range(K):
            OBSTACLE_FREE = False
            while not OBSTACLE_FREE:
                q_rand = self.random_config(t)
                q_nearest, dist, _ = self.nearest_neighbour(q_rand, np.array(t.vertexMap.keys()))
                q_new, ctrls_path = self.holonomic_extend(q_nearest, q_rand, epsilon)
                if self.nh_obstacle_free(ctrls_path[1]) and t.getVertex(q_new) is None:
                    OBSTACLE_FREE = True
            # TODO control input to edge, Cost is distance between
            t.addUniEdge(q_nearest, q_new, self.robot_vel * epsilon, True)
        return t

    def build_rrtstar(self, K=100, epsilon=1):
        # Sampling-based algorithms for optimal motion planning by Sertac Karaman and Emilio Frazzoli
        d = 2  # d-dimension
        mu_Xfree = 800*800
        zeta_d = math.pi + math.pow(1, d)
        gamma = 1 + math.pow(2 * (1 + 1./d), 1./d) * math.pow(mu_Xfree/zeta_d, 1./d)

        t = Graph()
        t.addVertex(self.q_init).setDist(0)
        q_nearest = None
        q_new = None
        for k in range(K):
            OBSTACLE_FREE = False
            while not OBSTACLE_FREE:
                q_rand = self.random_config(t)
                q_nearest, dist, _ = self.nearest_neighbour(q_rand, np.array(t.vertexMap.keys()))
                q_new, ctrls_path = self.holonomic_extend(q_nearest, q_rand, epsilon)
                if self.nh_obstacle_free(ctrls_path[1]) and t.getVertex(q_new) is None:
                    # No Collision occurred, So back to outer loop
                    OBSTACLE_FREE = True
            Qs_near = self.k_nearest_neighbour(t, q_new, max_dist=20)
            # TODO control input to edge, Cost is distance between
            q_new_vtx = t.addVertex(q_new)
            q_nearest_vtx = t.getVertex(q_nearest)
            edge_cost = self.euc_distance(q_new, q_nearest_vtx.getName())
            q_min_vtx, cost_min = q_nearest_vtx, q_nearest_vtx.getDist() + edge_cost

            # Connect along minimum cost path
            for q_near_vtx in Qs_near:
                q_near = q_near_vtx.getName()
                edge_cost = self.euc_distance(q_new, q_near_vtx.getName())
                alt_cost = q_near_vtx.getDist() + edge_cost
                if alt_cost < cost_min \
                        and self.nh_obstacle_free(self.holonomic_extend(q_near, q_new)[1][1]):
                        # and self.obstacle_free(q_near_vtx.getName(), q_new, edge_cost/self.robot_vel):
                    q_min_vtx, cost_min = q_near_vtx, alt_cost
            edge_cost = self.euc_distance(q_min_vtx.getName(), q_new)
            t.addUniEdge(q_min_vtx.getName(), q_new, edge_cost, True)
            q_new_vtx.setDist(cost_min)

            # Rewiring the Tree
            for q_near_vtx in Qs_near:
                q_near = q_near_vtx.getName()
                edge_cost = self.euc_distance(q_new, q_near)
                alt_cost = q_new_vtx.getDist() + edge_cost
                if alt_cost < q_near_vtx.getDist() \
                        and self.nh_obstacle_free(self.holonomic_extend(q_new, q_near)[1][1]):
                        # and self.obstacle_free(q_new, q_near, edge_cost/self.robot_vel):
                    q_parent = q_near_vtx.getPrev()
                    t.removeUniEdge(q_parent.getName(), q_near)
                    t.addUniEdge(q_new, q_near, edge_cost, True)
                    q_near_vtx.setDist(alt_cost)
        return t

    def nh_build_rrt(self, K=100, epsilon=1):

        t = Graph()
        t.addVertex(self.q_init)
        q_nearest = None
        q_new = None
        for k in range(K):
            OBSTACLE_FREE = False
            while not OBSTACLE_FREE:
                q_rand = self.random_config(t)
                q_nearest, dist, _ = self.nearest_neighbour(q_rand, np.array(t.vertexMap.keys()))
                (u_s, u_p, q_new), ctrls_path = self.nh_steer(q_nearest, q_rand, epsilon)
                # print u_s, u_p, q_new, ctrls_path
                if self.nh_obstacle_free(ctrls_path[(u_s, u_p)]) and t.getVertex(q_new) is None:
                    # No Collision occurred, So back to outer loop
                    OBSTACLE_FREE = True
            # TODO control input to edge, Cost is distance between
            edge = t.addUniEdge(q_nearest, q_new, self.robot_vel * epsilon, True)
            edge.ctrl = (u_s, u_p)
            edge.path = ctrls_path[(u_s, u_p)]
        return t

    def nh_steer(self, q_nearest, q_rand, epsilon):
        """
        For a car like robot, where it takes two control input (u_speed, u_phi)
        All possible combinations of control inputs are generated and used to find the closest q_new to q_rand
        :param q_nearest:
        :param q_rand:
        :param epsilon:
        :return:
        """
        u_speed, u_phi = [-1.0, 1.0], [-math.pi/4, 0, math.pi/4]
        controls = list(itertools.product(u_speed, u_phi))
        # euler = lambda t_i, q, u_s, u_p, L: (u_s*math.cos(q[2]), u_s*math.sin(q[2]), u_s/L*math.tan(u_p))
        result = []
        ctrls_path = {c: [] for c in controls}
        for ctrl in controls:
            q_new = q_nearest
            for t_i in range(epsilon):  # h is assumed to be 1 here for euler integration
                q_new = tuple(map(add, q_new, self.euler(t_i, q_new, ctrl[0], ctrl[1], 20.0)))  # TODO Change L=1 here
                ctrls_path[ctrl].append(q_new)
            result.append((ctrl[0], ctrl[1], q_new))
        q_news = [x[2] for x in result]
        _, _, idx = self.nearest_neighbour(q_rand, np.array(q_news))
        return result[idx], ctrls_path

    def euler(self, t_i, q, u_s, u_p, L):
        return u_s*math.cos(q[2]), u_s*math.sin(q[2]), u_s/L*math.tan(u_p)


    def holonomic_extend(self, q_nearest, q_rand, epsilon=None):
        # TODO check if the new config overshoots the q_rand

        controls = [1.0]
        if epsilon is None:
            # Integrate till it reach q_rand
            dist = self.euc_distance(q_nearest, q_rand)
            epsilon = int(round(float(dist)/controls[0]))

        ctrls_path = {c: [] for c in controls}
        dx, dy, _ = np.array(q_rand) - np.array(q_nearest)
        theta = math.atan2(dy, dx)
        for ctrl in controls:
            q_new = q_nearest
            for t_i in range(epsilon):
                x_new = q_new[0] + ctrl * math.cos(theta)
                y_new = q_new[1] + ctrl * math.sin(theta)
                q_new = x_new, y_new, _
                ctrls_path[ctrl].append(q_new)
        return q_new, ctrls_path

    def random_config(self, t):
        # Returns a random configuration tuple (x, y, theta)
        # 800x800 is screen size in pygame display
        # TODO Generalize it!
        while True:
            q_rand = random.uniform(0, 800), random.uniform(0, 800), random.uniform(-3.14, 3.14)
            if t.getVertex(q_rand) is None: # and not self.cDetector.is_point_colliding(q_rand):
                return q_rand
            else:
                print 'random_config:: q_rand exist in Tree/Graph. Picking another!'

    def nearest_neighbour(self, q, q_tree, k=1):
        # TODO Generalize it to k nearest neighbour
        # Takes all vertices from Tree/Graph into a list
        # q_tree = np.array(t.vertexMap.keys())
        q_tree_xy, q_tree_theta = q_tree[:, 0:2], q_tree[:, 2]
        # TODO Include Orientation into distance metric
        theta = [math.cos(q[2]), math.sin(q[2])]
        q_tree_theta = [np.cos(q_tree_theta[:]), np.sin(q_tree_theta[:])]
        dist = np.linalg.norm(q_tree_xy - q[0:2], axis=1)
        index = np.argmin(dist)
        q_nearest = tuple(q_tree[index])
        return q_nearest, dist[index], index

    def nh_obstacle_free(self, points):
        for p in points:
            if self.cDetector.is_point_colliding(p):
                print "obstacle_free:: path/point collides!"
                return False
        return True

    def euc_distance(self, start_pos, end_pos):
        return np.linalg.norm(np.array(start_pos) - np.array(end_pos))

    def rad_wrap(self, radian):
        """
        Takes an angle in radians of any range and returns an angle from
        the half open interval of [-3.14, 3.14) in radians.
        Note: If (-3.14, 3.14] is desired, then multiply the result of the below expression with -1
        :param radian:
        :return:
        """
        return (radian + np.pi) % (2 * np.pi) - np.pi

    def k_nearest_neighbour(self, t, q, max_dist=50):
        q_tree = np.array(t.vertexMap.keys())
        dist = np.linalg.norm(q_tree - q, axis=1)
        knn_keys = q_tree[np.where(dist <= max_dist)]
        return [t.vertexMap[tuple(k)] for k in knn_keys]



