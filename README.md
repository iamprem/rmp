# Path Planning for Reeds-Shepp Car

## Table of Contents
 - [Introduction](#introduction)
 - [Model Description](#model-description)
 - [Limitations](#limitations)
 - [Collision Detection](#collision-detection)
 - [Path Planning Techniques](#path-planning-techniques)
    - [RRT](#rrt)
    - [RRT\*](#rrt-star)
    - [Type 1 and Type 2 maneuvers](#maneuvers)
 - [Results](#results)
    - [Holonomic Robot](#holonomic-robot)
    - [Non-Holonomic Robot](#non-holonomic-robot)
 - [Execution Instruction](#execution-instruction)
    - [Dependencies](#dependencies)
    - [Creating and Loading Scene](#creating-and-loading-scene)
    - [Running Path Planning Methods](#running-path-planning-methods)
 - [Source Code](#source-code)
 - [Future Work](#future-work)
 - [References](#references)
 
## Introduction
The goal of this project is to implement sampling based motion planning 
algorithms such as Rapidly-Exploring Random Tree(RRT) and its variant RRT* for a 
holonomic point robot and a Reeds-Shepp car like robot with 
non-holonomic constraints.

When I was implementing RRT* with non-holonomic constraints, I
noticed that we need to be able to reach a nearby point(q_near) with its 
orientation from the new configuration(q_new). This is mostly impossible to reach
by just trying the six combination of inputs that a Reeds-Shepp Car can
take. So I also implemented ***Type 1*** and ***Type 2*** maneuvers for a car like 
robot to reach a desired configuration by doing combinations of these two
maneuvers.
 
 
## Model Description
 - Robot Model &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;:   Point Robot and Reeds-Shepp Car(Rigid Body)
 - Type        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;:   Holonomic and Non-Holonomic
 - Workspace   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;:   2-D plane with Obstacles
 - Collision Detection: Axis Aligned Bounding Boxes(Implemented)

## Limitations
For the non-holonomic case, I'm still defining the robot as point
because of my collision detection methods can only tell whether a point
lies inside the obstacle or not. I didn't implement polygon-polygon 
collision detection methods to facilitate this and also using
existing libraries for collision detection even complicates my data structures.
 
## Collision Detection
### Axis Aligned Bounding Box
The collision detection method that I implemented and used to check
 collision for the below path planning techniques is Axis aligned 
 bounding boxes. Here every obstacle in the environment is bounded by a 
 rectangle of smallest possible size and checking the robot is inside the
 rectangle while planning the path.
 
## Path Planning Techniques
 - RRT
 - RRT*
 - Type 1 and Type 2 Maneuvers
 
### Type 1 and Type 2 Maneuvers
Type 1: This maneuver is to take the robot from a configuration to another
configuration without changing the orientation(Like parallel parking). 

![Type 1](https://raw.githubusercontent.com/iamprem/temp/master/assets/type_1.png)


Type 2: This maneuver is to bring a robot from some orientation to a
desired orientation in-place(Like a three-point turn)

![Type 2](https://raw.githubusercontent.com/iamprem/temp/master/assets/type_2.png)

***Images Source: Class Lectures***

There was a third type of movement is implemented to connect these two
maneuvers to reach the goal exactly. First, type 2 maneuver is done to 
change to desired orientation and then marching forward or backward till it 
finds a point that is exactly parallel to the goal configuration, then 
type 1 maneuver is done to reach the goal configuration. Below GIF shows
type 1, type 2 and marching in cyan, brown and yellow colors respectively
from the simulation.

![Maneuvers](https://raw.githubusercontent.com/iamprem/temp/master/assets/maneuver_gif.gif)

## Results

### Holonomic Robot

#### RRT with 10000 random samples 

![RRT Holonomic](https://raw.githubusercontent.com/iamprem/temp/master/assets/rrt_holo.gif)

#### RRT* with 10000 random samples

![RRT Star Holonomic](https://raw.githubusercontent.com/iamprem/temp/master/assets/rrtstar_holo.gif)

### Non-Holonomic Robot

***Yellow Lines near the goal(Red Square) represents the maneuvers performed to reach the goal as desired***

#### RRT with 1000 random samples 

![RRT Non Holonomic](https://raw.githubusercontent.com/iamprem/temp/master/assets/rrt_nonholo.png)

#### RRT* with 1000 random samples

The below images shows rewiring step(by doing type1 and type2 maneuvers)
in multiple places.

![RRT Star Non Holonomic](https://raw.githubusercontent.com/iamprem/temp/master/assets/rrtstar_nonholo.png)

## Execution Instruction

### Dependencies
 
 - Python 2.7
 - [Pygame](http://www.pygame.org/wiki/CompileUbuntu)
 - Numpy
 - Ubuntu 14.04 or 15.10
 
#### To Install Numpy and Pygame on Ubuntu

 ```bash
 sudo apt-get install pip            # Install pip(python package manager)
 sudo pip install -U numpy           # Install Numpy using pip
 sudo apt-get install python-pygame  # Install Pygame using Ubuntu package manager
 ```
 
***Note: This program can also be run on Windows and Mac with appropriate pygame [installation](http://www.pygame.org/download.shtml)***

### Creating and Loading Scene

To create scene(2D-environment with obstacles), run the below command from terminal/command prompt.
 
 ```bash
 cd rmp                              # Navigate inside the project folder
 python scene.py                     # Run scene definer
 ```

In the Screen resulted by running the above command, you can draw shapes by doing any of the following.
 1. Click and Drag to draw rectangles (like in MS Paint)
 2. Or Just **click on 3 or more places** on the screen to capture co-ordinates of the polygon and once done press **ENTER** to 
 see the polygonal shape(Make sure that you don't drag the mouse while clicking).
 
Once obstacles are defined, Close the window to save the scene to 'scene_01.pkl' file(will be located on the project root directory) 



### Running Path Planning Methods

Use ```playground.py``` located in project root to test the program.

#### To Define Initial and Goal configurations
 Change the ```q_init``` and ```q_goal``` at line 11 and 12 on playground.py. ```x,y and theta``` should be in the range ```(0-800), (0-800)
 and (-3.14, 3.14)``` respectively.
 ```python
 playground.py : Line 11-12
    
    q_init = (100.0, 500.0, 0.0)
    q_goal = (700.0, 500.0, -2.15)
 ```

#### Choose Algorithm

Uncomment any one line and change parameters to run the required algorithm.
```python
playground.py : Line 27-31
   
   # Call algorithm
   # rrt_tree = planner.build_rrt(10000, epsilon=5)
   # rrt_tree = planner.build_rrtstar(K=10000, epsilon=5)
   # rrt_tree = planner.nh_build_rrt(K=1000, epsilon=40)
   # rrt_tree = planner.nh_build_rrtstar(K=1000, epsilon=40)
```
***Warning: Non-Holonomic cases with K > 1000 might take longer time to produce result because of inefficient graph
plotting***

Once you uncommented the algorithm in this step, uncomment/comment the required plotting line to see the result.


#### Holonomic Plot
For Holonomic cases, Uncomment the below lines and comment the Lines (51-57) shown in the next step(Non-Holonomic Plot)

```python
playground.py : Line 45-47

   # q_goal_vtx = planner.reach_goal(rrt_tree, q_goal)
   # vizer.plot_graph(rrt_tree, q_init)
   # vizer.trace_path(q_goal_vtx)
```



#### Non-Holonomic Plot
For Non-Holonomic Case, comment the three lines (45-47) and uncomment lines (51-57)

```python
playground.py : Lines 51-57

   vizer.nh_plot_graph(rrt_tree, q_init)
   vizer.nh_trace_path(rrt_tree.getVertex(q_nearest))
   
   a,b,c,d = planner.nh_reach_goal(q_nearest, q_goal)
   final_list = planner.append_t1_m_t2(a, b, c)
   # TODO final_list collision check
   vizer.plot_points(final_list, vizer.YELLOW, 3)
```

#### Start and Stop
After selecting the appropriate lines from the above steps, 
 - run ***```python playground.py```*** to start the program.
 - Close the window to exit the program once done(Or Press CTRL + C on console to break)

## Source Code

[On Github](http://github.com/iamprem/rmp)

## Future Work
 
 - Improve Collision detection to more accurate methods and define the car as a polygon instead of point.
 - Type 1 and Type 2 maneuvers should be optimized to reduce number of intermediate steps
 - Implement bidirectional RRTs
 
## References

 1. Karaman, Sertac, and Emilio Frazzoli. "Incremental sampling-based algorithms for optimal motion planning." arXiv preprint arXiv:1005.0416 (2010).
 2. Karaman, Sertac, and Emilio Frazzoli. "Sampling-based algorithms for optimal motion planning." The International Journal of Robotics Research 30.7 (2011): 846-894.
 3. Karaman, Sertac, and Emilio Frazzoli. "Optimal kinodynamic motion planning using incremental sampling-based methods." Decision and Control (CDC), 2010 49th IEEE Conference on. IEEE, 2010.
 4. Karaman, Sertac, and Emilio Frazzoli. "Sampling-based optimal motion planning for non-holonomic dynamical systems." Robotics and Automation (ICRA), 2013 IEEE International Conference on. IEEE, 2013.