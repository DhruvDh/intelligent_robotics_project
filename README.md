# Robotic Arm Manipulation using Quaternion Neural Networks

- Dhruv Dhamani
- ddhamani@uncc.edu

## Table of Contents
- [Robotic Arm Manipulation using Quaternion Neural Networks](#robotic-arm-manipulation-using-quaternion-neural-networks)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [A more formal description](#a-more-formal-description)
  - [Tools used for Implementation](#tools-used-for-implementation)
  - [Milestones](#milestones)
    - [What I will definitely do](#what-i-will-definitely-do)
    - [What I am likely to do](#what-i-am-likely-to-do)
    - [What I will ideally do](#what-i-will-ideally-do)

## Introduction

Neural networks are a beautiful biologically-inspired programming paradigm which enables a computer to learn from observational data, the efficacy of which is well-documented in a variety of domains.

Most neural networks in use today operate on real-valued numbers, however the approach itself is number system agnostic. As long as we can define a suitable mathematical function for a neuron to apply on incoming values, a suitable activation function, and a suitable learning algorithm (which is almost always backpropagation), then the choice of number system used is irrelevant.

It has been found that using a number system appropriate to the task at hand can vastly simplify the complexity of network required, without adversely affecting the performance at the task.

For instance, consider a classification problem, where the scatter plot of the data looks like the figure below -

|![Data](https://iq.opengenus.org/content/images/2018/08/generated-1.JPG) |
|:--:|
| Class A - Purple, Class B - Yellow|

Solving this problem effectively would require a network multiple hidden layers (since it's a non-linear classification problem). If polar co-ordinates are used, the problem becomes a linear classification problem, which requires only a single hidden layer.

Less complex neural networks are preferable since they offer a lower computational complexity.

## A more formal description

A 2R, 3R, and 4R robotic arm would be simulated using ROS and Gazebo. The user would input either via console or through a web interface the desired co-ordinates for the end-effector.

The desired end-effector co-ordinates would be fed into a neural network as in the form of a quaternion that describes the transformation required to go from the origin to the desired end-effector co-ordinates, along with the quaternions describing the current orientation of each link in the robotic arm.

The neural network would be expected to give as many output quaternions as there are links in the robotic arm, each representing what change must be made in the links location and orientation in space for the end-effector to reach the desired co-ordinates.

## Tools used for Implementation

Most work would be done in **Python**, if there is time to implement a web-interface, javascript would also be used.

Libraries and software packages to be used -
- PyTorch, a neural network framework.
- The ROS ecosystem and Gazebo
- [Code released](https://github.com/Orkis-Research/Quaternion-Recurrent-Neural-Networks) with the paper "Quaternion Recurrent Neural Networks" ([Parcollet et al., 2018](https://arxiv.org/abs/1806.04418)), which has useful abstractions for quaternion operations, and quaternion neural network layers, etc.

## Milestones

### What I will definitely do

- Compute and create suitable training data for an 3R robotic arm.
- Implement and train a Quaternion neural network to perform inverse kinematics for the 3R robot.
- Experiment with different neural network topologies and architecture.
- Evaluate how well it performs.


### What I am likely to do

- Also do all of the above for 2R, and 4R robotic arms.

### What I will ideally do

- Implement a simulation that shows the arm manipulation using quaternion neural networks in action.
- Implement an interface to input desired end-effector co-ordinates.
