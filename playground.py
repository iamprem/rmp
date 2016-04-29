import pygame
import sys
from pathplan import PathPlanner
from visualizer import Visualizer
from collisiondetect import CollisionDetector

# Initial Config
q_init = (400, 400)

# Define and convert obstacles
vizer = Visualizer()
obstcls = vizer.define_obstacles()
cd = CollisionDetector(obstcls)
obstcls_aabb = cd.compute_AABB()

# Plan path using q_init and obstacles
planner = PathPlanner(q_init, cd)
rrt_tree = planner.build_rrtstar(K=10, epsilon=10)


vizer.plot_graph(rrt_tree)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
