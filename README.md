# Path Planning for Reeds-Shepp Car

## Table of Contents
 - [Introduction](#introduction)
 - [Model Description](#model-description)
 - [Path Planning Techniques](#path-planning-techniques)
    - [RRT](#rrt)
    - [RRT\*](#rrt-star)
    - [Type 1 and Type 2 maneuvers](#maneuvers)
    - [Collision Detection](#collision-detection)
 - [Results](#results)
    - [Holonomic Robot](#non-holonomic-demo)
    - [Non-Holonomic Robot](#holonomic-demo)
 - [Evaluation](#evaluation)
 - [Installation](#installation)
 - [Execution Instruction](#execution-instruction)
 - [Source Code](#source-code)
 - [References](#references)
 
## Introduction
In this project Rapidly-Exploring Random Tree(RRT) and its variant RRT* 
are implemented for holonomic point robot and only RRT for non-holonomic
robot. The non-holonomic robot model considered for this project is also
a point robot but with Reeds-Sheep car's behaviour.

When I was implementing RRT* with non-holonomic constraints, I
noticed that we need to be able to reach a nearby point with its 
orientation from the new configuration. This is mostly impossible to reach
by just trying the six combination of inputs that a Reeds-Shepp Car can
take. So I deviated a little from my original goal of implementing RRT*
for non-holonomic robot to more specific goal of making a robot to move 
from the new configuration to the nearby configuration by doing combinations
of Type 1 and Type 2 maneuvers once it is close enough to the point.
 
 
## Model Description
Robot Model :   Point Robot with Reeds-Shepp
Type        :   Holonomic and Non-Holonomic
Workspace   :   2-D plane with Obstacles
Collision Detection: Axis Aligned Bounding Boxes(Implemented)


## Path Planning Techniques

### RRT
 
### RRT*

### Type 1 and Type 2 Maneuvers

### Axis Aligned Bounding Box

## Results

### Holonomic Robot

### Non-Holonomic Robot
