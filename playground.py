import pygame
import sys
import numpy as np
import pickle
from pathplan import PathPlanner
from visualizer import Visualizer
from collisiondetect import CollisionDetector

# Initial Config
q_init = (400.0, 400.0, 0.0)
q_goal = (750.0, 50.0, 0.0)

# Define and convert obstacles
vizer = Visualizer()
# obstcls = vizer.define_obstacles()
obstcls = pickle.load(open('scene_01.pkl', 'rb'))
cd = CollisionDetector(obstcls)
obstcls_aabb = cd.compute_AABB()

# Plan path using q_init and obstacles
planner = PathPlanner(q_init, cd)

# Call algorithm
rrt_tree = planner.build_rrt(10000, epsilon=5)
# rrt_tree = planner.build_rrtstar(K=10000, epsilon=5)
# rrt_tree = planner.nh_build_rrt(K=100, epsilon=40)

q_nearest, dist, _ = planner.nearest_neighbour(q_goal, np.array(rrt_tree.vertexMap.keys()))
q_new, ctrls_path = planner.holonomic_extend(q_nearest, q_goal)
if planner.nh_obstacle_free(ctrls_path[1]) and rrt_tree.getVertex(q_goal) is None:
    edge = rrt_tree.addUniEdge(q_nearest, q_goal, 1.0 * len(ctrls_path), True)
else: print 'Path to goal collides. Try more samples'
q_goal_vtx = rrt_tree.getVertex(q_goal)

vizer.plot_graph(rrt_tree)
# vizer.nh_plot_graph(rrt_tree)
vizer.trace_path(q_goal_vtx)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
