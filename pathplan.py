import sys
from dijkstra import *
import random
import math
import numpy as np

class PathPlanner:
    robot_vel = 1   # 1 m/s
    def __init__(self, q_init):
        self.q_init = q_init
        pass

    def build_rrt(self, K=100, epsilon=1):
        '''

        :param K: Number of nodes to be added to the RRT Tree
        :param epsilon: Amount of time the control input is applied (seconds). Default = 1s
        :return:
        '''
        t = Graph()
        t.addVertex(self.q_init)
        for k in range(K):
            q_rand = self.random_config(t)
            self.holonomic_extend(t, q_rand, epsilon)
        return t

    def holonomic_extend(self, t, q_rand, epsilon):
        # TODO check whether the new config already exist in the Tree
        # TODO check if the new config overshoots the q_rand
        q_near, dist = self.nearest_neighbour(q_rand, t)
        dx, dy = np.array(q_rand) - np.array(q_near)
        theta = math.atan2(dy, dx)
        # robot_vel is one of the control inputs(u) which is applied along with the orientation
        x_new = q_near[0] + epsilon * self.robot_vel * math.cos(theta)
        y_new = q_near[1] + epsilon * self.robot_vel * math.sin(theta)
        q_new = x_new, y_new

        # Add q_new to Tree/Graph
        if t.getVertex(q_new) is None:
            q_new_vtx = t.addVertex(q_new)
            q_near_vtx = t.getVertex(q_near)
            q_near_vtx.addUniEdge(q_near, q_new, 1) # TODO control input to edge
            return q_new
        else:
            print "holonomic_extend:: q_new already exist in Tree/Graph"
            sys.exit(0)

    def random_config(self, t):
        # Returns a random configuration tuple
        # 800x800 is screen size in pygame display
        # TODO Generalize it!
        while True:
            q_rand = random.randint(0, 800), random.randint(0, 800)
            if t.getVertex(q_rand) is None:
                return q_rand
            else:
                print 'random_config:: q_rand exist in Tree/Graph'

    def nearest_neighbour(self, q, t):
        # TODO Generalize it to k nearest neighbour
        # Takes all veritces from Tree/Graph into a list
        q_tree = np.array(t.vertexMap.keys())
        dist = np.linalg.norm(q_tree - q, axis=1)
        index = np.argmin(dist)
        q_nearest = q_tree[index]
        return q_nearest, dist[index]


