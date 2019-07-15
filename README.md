# Distributed-Simplex
An implementation of a distributed version of the well-known simplex algorithm, is given in robot operating system (ROS).
The aim is to solve degenerate linear programs on asynchronous peer-to-peer networks with distributed information structures.
So every agent knows only a subset of information, and they run simplex algorithm by updating and sharing their base, until they will agree on a common solution. We show how the multi agent assignment problem can be efficiently solved by using a distributed structure, and we provide simulations in the robot operating system environment.     

## Introduction
The present report provides a methodological approach to define and solve a multi-agent assignment problem. The goal is to generate a distributed algorithm, with local and sharing information structure.
The overall problem to be solved is divided in two chapters. In Chapter 1 we were asked to understand and implement a distributed algorithm which we tried to make as more general as possible. 
In the Chapter 2 we show a simulation platform by using the developed software in Chapter 1 to simulate a fleet of wheeled-robots , by specializing our algorithm to the dynamic of a network of robots.

## Set-up
We are interested in a network of autonomous agents which are able to receive information from their neighbours, and a set of tasks. Neighbour relations between distinct pairs of agents are characterized by directed graph. 
The assignment problem can be illustrated with a bipartite assignment graph, where an edge (i,k) exists if and only if agent 'i' can be assigned to the task 'k'. The cost c(i,k) for agent 'i' to perform the task 'k' is a weight on the edge.
For each agent 'i' and task 'k' a binary decision variable x(i,k)={0,1} is introduced, which is 1 if agent 'i' is assigned to task 'k' and 0 otherwise. 
The assignment problem corresponds to the integer optimization problem with the constraint that a full assignment is achived.
