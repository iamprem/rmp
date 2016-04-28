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
rrt_tree = planner.build_rrt(K=2000, epsilon=5)


vizer.plot_graph(rrt_tree)

while True:
    i = 1
