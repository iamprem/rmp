import pygame
import sys
import numpy as np
import pickle
from pathplan import PathPlanner
from visualizer import Visualizer
from collisiondetect import CollisionDetector

# Initial Config
q_init = (50.0, 700.0, 0.0)
q_goal = (750.0, 200.0, -2.15)

# Define and convert obstacles
vizer = Visualizer()
vizer.draw_square(q_init)
vizer.draw_square(q_goal, color=vizer.RED)
obstcls = vizer.define_obstacles()
# obstcls = pickle.load(open('scene_01.pkl', 'rb'))
cd = CollisionDetector(obstcls)
obstcls_aabb = cd.compute_AABB()

# Plan path using q_init and obstacles
planner = PathPlanner(q_init, cd)

# Call algorithm
# rrt_tree = planner.build_rrt(10000, epsilon=10)
rrt_tree = planner.build_rrtstar(K=20000, epsilon=5)
# rrt_tree = planner.nh_build_rrt(K=500, epsilon=40)

q_nearest, dist, _ = planner.nearest_neighbour(q_goal, np.array(rrt_tree.vertexMap.keys()))
q_new, ctrls_path = planner.holonomic_extend(q_nearest, q_goal)
if planner.nh_obstacle_free(ctrls_path[1]) and rrt_tree.getVertex(q_goal) is None:
    edge = rrt_tree.addUniEdge(q_nearest, q_goal, 1.0 * len(ctrls_path), True)
else:
    print 'Path to goal collides. Try more samples'
    sys.exit()
q_goal_vtx = rrt_tree.getVertex(q_goal)

vizer.plot_graph(rrt_tree, q_init)
vizer.trace_path(q_goal_vtx)
"""
vizer.nh_plot_graph(rrt_tree, q_init)
vizer.nh_trace_path(rrt_tree.getVertex(q_nearest))

a,b,c = planner.nh_reach_goal(q_nearest, q_goal)
vizer.plot_points(a[0], vizer.YELLOW, 2)
vizer.plot_points(a[1], vizer.YELLOW, 2)
vizer.plot_points(a[2], vizer.YELLOW, 2)
vizer.plot_points(b, vizer.BROWN, 2)
for t in c:
    vizer.plot_points(t[0], vizer.CYAN, 2)
    vizer.plot_points(t[1], vizer.CYAN, 2)
    vizer.plot_points(t[2], vizer.CYAN, 2)
"""
vizer.draw_square(q_init)
vizer.draw_square(q_goal, color=vizer.RED)
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
