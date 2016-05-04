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
rrt_tree = planner.nh_build_rrtstar(K=100, epsilon=40)
end = time.time()
print('Time taken: %f' % (end - start))



q_nearest, dist, _ = planner.nearest_neighbour(q_goal, np.array(rrt_tree.vertexMap.keys()))


# Non Holonomic
vizer.nh_plot_graph(rrt_tree, q_init)
vizer.nh_trace_path(rrt_tree.getVertex(q_nearest))
a,b,c,d = planner.nh_reach_goal(q_nearest, q_goal)
final_list = planner.append_t1_m_t2(a, b, c)
vizer.plot_points(final_list, vizer.YELLOW, 3)
if not planner.nh_obstacle_free(final_list):
    print 'Final Maneuve collide with obstacle'


vizer.draw_square(q_init)
vizer.draw_square(q_goal, color=vizer.RED)
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
