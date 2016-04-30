import pickle
from visualizer import Visualizer

# Use this to define a scene for the robot

vizer = Visualizer()
obstcls = vizer.define_obstacles()
outfile = open('scene_01.pkl', 'wb')
pickle.dump(obstcls, outfile)
outfile.close()