import sys
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
                q_nearest, dist = self.nearest_neighbour(q_rand, t)
                q_new = self.holonomic_extend(t, q_nearest, q_rand, epsilon)
                if self.obstacle_free(q_nearest, q_new, epsilon) and t.getVertex(q_new) is None:
                    # No Collision occurred, So back to outer loop
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
                q_nearest, dist = self.nearest_neighbour(q_rand, t)
                q_new = self.holonomic_extend(t, q_nearest, q_rand, epsilon)
                if self.obstacle_free(q_nearest, q_new, epsilon) and t.getVertex(q_new) is None:
                    # No Collision occurred, So back to outer loop
                    OBSTACLE_FREE = True
            Qs_near = self.k_nearest_neighbour(t, q_new, max_dist=20)
            # TODO control input to edge, Cost is distance between
            q_new_vtx = t.addVertex(q_new)
            q_nearest_vtx = t.getVertex(q_nearest)
            edge_cost = self.euc_distance(q_new, q_nearest_vtx.getName())
            # q_new_vtx.setDist(q_nearest_vtx.getDist() + edge_cost)
            # t.addUniEdge(q_nearest, q_new, edge_cost, True)
            q_min_vtx, cost_min = q_nearest_vtx, q_nearest_vtx.getDist() + edge_cost
            # Connect along minimum cost path
            for q_near_vtx in Qs_near:
                edge_cost = self.euc_distance(q_new, q_near_vtx.getName())
                alt_cost = q_near_vtx.getDist() + edge_cost
                if self.obstacle_free(q_near_vtx.getName(), q_new, edge_cost/self.robot_vel) \
                        and alt_cost < cost_min:
                    q_min_vtx, cost_min = q_near_vtx, alt_cost
            edge_cost = self.euc_distance(q_min_vtx.getName(), q_new)
            t.addUniEdge(q_min_vtx.getName(), q_new, edge_cost, True)
            q_new_vtx.setDist(cost_min)

            # Rewiring the Tree
            for q_near_vtx in Qs_near:
                edge_cost = self.euc_distance(q_new, q_near_vtx.getName())
                alt_cost = q_new_vtx.getDist() + edge_cost
                if self.obstacle_free(q_new, q_near_vtx.getName(), edge_cost/self.robot_vel) \
                        and alt_cost < q_near_vtx.getDist():
                    q_parent = q_near_vtx.getPrev()
                    t.removeUniEdge(q_parent.getName(), q_near_vtx.getName())
                    t.addUniEdge(q_new, q_near_vtx.getName(), edge_cost, True)
                    q_near_vtx.setDist(alt_cost)
        return t


    def holonomic_extend(self, t, q_nearest, q_rand, epsilon):
        # TODO check if the new config overshoots the q_rand

        dx, dy = np.array(q_rand) - np.array(q_nearest)
        theta = math.atan2(dy, dx)
        # robot_vel is one of the control inputs(u) which is applied along with the orientation
        x_new = q_nearest[0] + epsilon * self.robot_vel * math.cos(theta)
        y_new = q_nearest[1] + epsilon * self.robot_vel * math.sin(theta)
        q_new = x_new, y_new
        return q_new

    def random_config(self, t):
        # Returns a random configuration tuple
        # 800x800 is screen size in pygame display
        # TODO Generalize it!
        while True:
            q_rand = random.uniform(0, 800), random.uniform(0, 800)
            # q_rand = random.randint(0, 800), random.randint(0, 800)
            if t.getVertex(q_rand) is None: # and not self.cDetector.is_point_colliding(q_rand):
                return q_rand
            else:
                print 'random_config:: q_rand exist in Tree/Graph. Picking another!'

    def nearest_neighbour(self, q, t):
        # TODO Generalize it to k nearest neighbour
        # Takes all vertices from Tree/Graph into a list
        q_tree = np.array(t.vertexMap.keys())
        dist = np.linalg.norm(q_tree - q, axis=1)
        index = np.argmin(dist)
        q_nearest = tuple(q_tree[index])
        return q_nearest, dist[index]

    def obstacle_free(self, q_nearest, q_new, epsilon):
        dx, dy = np.array(q_new) - np.array(q_nearest)
        theta = math.atan2(dy, dx)
        if not self.cDetector.is_point_colliding(q_new) and \
                not self.cDetector.is_path_colliding(q_nearest, q_new, int(epsilon), theta, self.robot_vel):
            return True
        else:
            print "holonomic_extend:: q_new already exist in Tree/Graph"
            print "holonomic_extend:: q_new collides! Picking another!"
            return False

    def euc_distance(self, start_pos, end_pos):
        return np.linalg.norm(np.array(start_pos) - np.array(end_pos))

    def k_nearest_neighbour(self, t, q, max_dist=50):
        q_tree = np.array(t.vertexMap.keys())
        dist = np.linalg.norm(q_tree - q, axis=1)
        knn_keys = q_tree[np.where(dist <= max_dist)]
        return [t.vertexMap[tuple(k)] for k in knn_keys]



