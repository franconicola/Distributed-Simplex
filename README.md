# Distributed-Simplex    

## Introduction
An implementation of a distributed version of the well-known simplex algorithm, is given in robot operating system (ROS).
The aim is to solve degenerate linear programs on asynchronous peer-to-peer networks with distributed information structures.
So every agent knows only a subset of information, and they run simplex algorithm by updating and sharing their base, until they will agree on a common solution. We show how the multi agent assignment problem can be efficiently solved by using a distributed structure, and we provide simulations in the robot operating system environment. 

## Set-up
We are interested in a network of autonomous agents which are able to receive information from their neighbours, and a set of tasks. Neighbour relations between distinct pairs of agents are characterized by directed graph. 
The assignment problem can be illustrated with a bipartite assignment graph, where an edge (i,k) exists if and only if agent 'i' can be assigned to the task 'k'. The cost c(i,k) for agent 'i' to perform the task 'k' is a weight on the edge.
For each agent 'i' and task 'k' a binary decision variable x(i,k)={0,1} is introduced, which is 1 if agent 'i' is assigned to task 'k' and 0 otherwise. 
The assignment problem corresponds to the integer optimization problem with the constraint that a full assignment is achived.

## Code explanation

Let's start looking to the main program, at beginnig we receive identification number, used to distinguish between nodes, and then we create an object of Agent's class.
Initialization of Agent's class consist of a base matrix B, a constrained matrix A, a cost vector inside the matrix H, a publisher and a subscriber with the id number given at node, in order to create the ring digraph.
At last element, a publisher is used to send velocity's comand to robot base, and move the robot to the desired target position. 

![Agents initialization](/images/AgentsStart.png)
![Agents initialization](/images/PermanentCols.png)

The simplex algorithm is implemented inside *matrixcb* function, and starts by creating the H_tmp matrix with columns received in a base message from its neighbour and its permanent columns. So the function lexicographic sorting all the columns of the matrix H_tmp, to provides a unique ordering of the columns. The lexicographic ordering ensures that all agents optimize with respect to the same lexicographic objective.
Then the Simplex algorithm is defined with three parameters H_tmp, B and P, inside the file *Simplex.cpp*. At beginning it initializes all matrices to zero, and enters inside the  while cycle until all columns of H_tmp are checked.
If vector *e* is already inside the base B, is skipped, else it used to check the reduced cost as described previously. If the reduced cost is lexico-negative, a leaving column is choosen by lexicographic radio test to be changed with vector *e*.
The base obtained by simplex function, is stored to a temporary base B_tmp and it is checked with the previous one B.
If the base sended by the previous agent is changed, then all the agents are connected and I can start to converge to the common solution.
After a while, if the base is not more changing then matrixcb function will stop to execute, and the *end* function will be used to define the solution. Inside the common base, it will be present one of the permanent columns of matrix P, by looking only to the column related to our agent, so from the top, the columns with the ones to the same row as our agent *id*.

![Agents solution](/images/SameBase.png)
![Agents solution](/images/Solution1.png)

Gazebo is the simulation environment used to test the algorithm with a fleet of wheeled-robots. The simulations are given with a maximum of 4 agents, and we used TurtleBot as wheeled mobile robot. The generation of the constrained vector is given by calcute distance between the initial robot position and target positions. Where targets are positionated at each corner of arena, so at positions (10,10),(-10,10),(10,-10) and (-10,-10).
After launching robots inside gazebo, we need to run the simplex in order to establish the communication and to select the target for each of them.
The function *goalPosition* uses solution provides from *end* function, and sets the distance to cover and the angle to turn, then it sends these parameters to functions *rotate* and *move*.

![Gazebo](/images/Agent.png)
![Gazebo](/images/Graph_with_Gazebo.png)

[![Alt Text](http://img.youtube.com/vi/sib7pv8JoH4/0.jpg)](https://www.youtube.com/watch?v=sib7pv8JoH4 "Alt Text")
