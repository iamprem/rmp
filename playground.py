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
# obstcls = vizer.define_obstacles()
obstcls = pickle.load(open('scene_01.pkl', 'rb'))
cd = CollisionDetector(obstcls)
obstcls_aabb = cd.compute_AABB()

# Plan path using q_init and obstacles
planner = PathPlanner(q_init, cd)

start = time.time()
# Call algorithm
# rrt_tree = planner.build_rrt(10000, epsilon=5)
# rrt_tree = planner.build_rrtstar(K=10000, epsilon=5)
# rrt_tree = planner.nh_build_rrt(K=1000, epsilon=40)
# rrt_tree = planner.nh_build_rrtstar(K=1000, epsilon=40)
end = time.time()
print('Time taken: %f' % (end - start))

# Write Tree to File
# outfile = open('rrtstar_nonholo.pkl', 'wb')
# pickle.dump(rrt_tree, outfile, pickle.HIGHEST_PROTOCOL)
# outfile.close()

rrt_tree = pickle.load(open('rrt_nonholo.pkl', 'rb'))


# Holonomic
q_nearest, dist, _ = planner.nearest_neighbour(q_goal, np.array(rrt_tree.vertexMap.keys()))
# q_goal_vtx = planner.reach_goal(rrt_tree, q_goal)
# vizer.plot_graph(rrt_tree, q_init)
# vizer.trace_path(q_goal_vtx)

# Non Holonomic
# """
vizer.nh_plot_graph(rrt_tree, q_init)
vizer.nh_trace_path(rrt_tree.getVertex(q_nearest))

a,b,c,d = planner.nh_reach_goal(q_nearest, q_goal)
final_list = planner.append_t1_m_t2(a, b, c)
# TODO final_list collision check
vizer.plot_points(final_list, vizer.YELLOW, 3)
# vizer.plot_points(a[0], vizer.YELLOW, 2)
# vizer.plot_points(a[1], vizer.YELLOW, 2)
# vizer.plot_points(a[2], vizer.YELLOW, 2)
# vizer.plot_points(b, vizer.BROWN, 2)
# for t in c:
#     vizer.plot_points(t[0], vizer.CYAN, 2)
#     vizer.plot_points(t[1], vizer.CYAN, 2)
#     vizer.plot_points(t[2], vizer.CYAN, 2)
# """
vizer.draw_square(q_init)
vizer.draw_square(q_goal, color=vizer.RED)
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
