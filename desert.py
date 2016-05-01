"""
This is where explosion happens
"""
import pickle
import pygame
import sys
from collisiondetect import CollisionDetector
from pathplan import PathPlanner
from visualizer import Visualizer

q_init = (400.0, 400.0, 0.0)
obstcls = pickle.load(open('scene_01.pkl', 'rb'))
cd = CollisionDetector(obstcls)
vizer = Visualizer()
obstcls_aabb = cd.compute_AABB()

# Plan path using q_init and obstacles
planner = PathPlanner(q_init, cd)

a,b,c = planner.nh_reach_goal(q_init, (200.0, 200.0, 3.14))
vizer.plot_points(a[0])
vizer.plot_points(a[1])
vizer.plot_points(a[2])
vizer.plot_points(b)
for t in c:
    vizer.plot_points(t[0])
    vizer.plot_points(t[1])
    vizer.plot_points(t[2])

print "done"

a,b,c = planner.t2_maneuver((200.0, 200.0, 0.0), 3.14)
vizer.plot_points(a)
vizer.plot_points(b)
vizer.plot_points(c)
a,b,c = planner.t1_maneuver(q_init, 20)
vizer.plot_points(a)
vizer.plot_points(b)
vizer.plot_points(c)



while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
