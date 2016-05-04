import pygame
import sys
import numpy as np
import pickle
import time
from pathplan import PathPlanner
from visualizer import Visualizer
from collisiondetect import CollisionDetector
sys.setrecursionlimit(5000)

# Initial Config
q_init = (100.0, 500.0, 0.0)
q_goal = (700.0, 500.0, -2.15)

# Define and convert obstacles
vizer = Visualizer()
vizer.draw_square(q_init)
vizer.draw_square(q_goal, color=vizer.RED)
obstcls = vizer.define_obstacles()
cd = CollisionDetector(obstcls)
obstcls_aabb = cd.compute_AABB()

# Plan path using q_init and obstacles
planner = PathPlanner(q_init, cd)

start = time.time()
# Call algorithm
rrt_tree = planner.build_rrtstar(K=2000, epsilon=5)
end = time.time()
print('Time taken: %f' % (end - start))


# Holonomic
q_nearest, dist, _ = planner.nearest_neighbour(q_goal, np.array(rrt_tree.vertexMap.keys()))
q_goal_vtx = planner.reach_goal(rrt_tree, q_goal)
vizer.plot_graph(rrt_tree, q_init)
vizer.trace_path(q_goal_vtx)

vizer.draw_square(q_init)
vizer.draw_square(q_goal, color=vizer.RED)
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
