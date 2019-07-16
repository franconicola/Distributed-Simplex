#include "Eigen/Dense"
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <map>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"


using namespace std;
using namespace Eigen;

const unsigned int M = 10000000;    		//Maximum number by bigM method
const unsigned int AGENTS = 3;			//Number of Agents
const unsigned int PARAMETERS =  pow(AGENTS,2);	//Number of parameters
const unsigned int ARTIFICIAL_PARAM = AGENTS*2;	//Number of artificial parameters


MatrixXf lexicRowsOrder(MatrixXf Mat);

MatrixXf lexicColsOrder(MatrixXf Mat, bool direction);

MatrixXf baseInitialization(void);

MatrixXf H_generation(VectorXf constraints);

MatrixXf simplex(MatrixXf H, MatrixXf B, MatrixXf* P);
