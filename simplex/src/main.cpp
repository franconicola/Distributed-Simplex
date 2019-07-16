#include "Simplex.hpp"

#define ANGULAR_VELOCITY_MINIMUM_THRESHOLD 0.2

class Agent
{
	public:	

	const double PI = 3.14159265359;
	

	Agent(unsigned int n, unsigned int agents, VectorXf start_pos)
	{
		M = 10000000;
		AGENTS = agents;
		PARAMETERS =  pow(AGENTS,2);
		ARTIFICIAL_PARAM = AGENTS*2;
		k = 0; w = 0; id = n;													// Iterator used to convergence
		find = false; 
		exit_simplex = false;
		base_changed = false;

		// Matrices used for the simplex algorithm:
		// H is combination of permanent columns P and base B 
		H = MatrixXf::Zero(ARTIFICIAL_PARAM+1,PARAMETERS);
		B = MatrixXf::Zero(ARTIFICIAL_PARAM+1,ARTIFICIAL_PARAM);
		P = MatrixXf::Zero(ARTIFICIAL_PARAM+1,AGENTS);
		B_tmp = MatrixXf::Zero(ARTIFICIAL_PARAM+1,ARTIFICIAL_PARAM);
		B_received = MatrixXf::Zero(ARTIFICIAL_PARAM+1,ARTIFICIAL_PARAM);		//Matrix Base received
		solution = MatrixXf::Zero(ARTIFICIAL_PARAM+1,1);
		constraints = VectorXf::Zero(PARAMETERS);
		init_position = start_pos;

		
		//Generation of constrained vector
		for (i = 0; i < AGENTS; i++){
			int sign_x = pow(-1,i), sign_y = pow(-1,i+1);
			if(i > 1)
				sign_y = pow(-1,i);

			constraints[i+(id-1)*AGENTS] = getDistance(10*sign_x,-10*sign_y,start_pos[0],start_pos[1]);
		}
		// Permanent columns simplex matrix generation 
		H = H_generation(constraints);
		H = H.block(0,(id-1)*AGENTS,ARTIFICIAL_PARAM+1,AGENTS);
		P = H;

		// Trouble with .block()
		if(n ==  1){
			P(0,0) = 1;	P(1,0) = 0; H(0,0) = 1; H(1,0) = 0;
		}

		cout << "P = " << endl << P << endl;

		// Base initialization
		B = baseInitialization();
		B_tmp = baseInitialization();

		// Graph definition
		if(n == 1)
			publish_edge +=  boost::lexical_cast<string>(id+1);

		else if(n == AGENTS)
			publish_edge +=  boost::lexical_cast<string>(1);

		else
			publish_edge +=  boost::lexical_cast<string>(id+1);

		subscribe_edge += boost::lexical_cast<string>(id);

		sub = node.subscribe(subscribe_edge, 10, &Agent::matrixcb, this);
			
		// Publish base to next agent and receiving by previous agent
		pub = node.advertise<std_msgs::Float32MultiArray>(publish_edge, 1);

		// Publish velocity 
		vel_name = vel_name + boost::lexical_cast<string>(id) + "/cmd_vel";	
		velocity_pub = node.advertise<geometry_msgs::Twist>(vel_name, 1);
		
	}

	// Simplex function
	void matrixcb(const std_msgs::Float32MultiArray::ConstPtr& hisMsg)
	{
		// Base received from another agent
		data = hisMsg->data;
		
		Map<MatrixXf> mat(data.data(), ARTIFICIAL_PARAM+1, ARTIFICIAL_PARAM);

		H_tmp = MatrixXf::Zero(ARTIFICIAL_PARAM+1,ARTIFICIAL_PARAM + AGENTS); 		//Matrix H temporary

		/* If the base sended by the previous agent is changed then all the agents are connected and 
		I can start to converge to the common solution*/ 
		flag1 = true;
		for(i= 0; i < mat.rows(); i++)
			if(!mat.row(i).isApprox(B_received.row(i)))
				flag1 = false;
		if(!flag1)
			k++;

		B_received = mat;

		for (j = 0; j< H_tmp.cols(); j++)
		{
			if(j < mat.cols())
				H_tmp.col(j) = mat.col(j);
			else 
				H_tmp.col(j) = P.col(j-ARTIFICIAL_PARAM);
		}

		H_tmp = lexicColsOrder(H_tmp, true);
		
		B_tmp = simplex(H_tmp,B,&P);

		/* If my base is changed then I can start to converge to the common solution*/ 
		if (k > 1){
			flag1 = true;
			for(i= 0; i < B_tmp.rows(); i++)
				if(!B_tmp.row(i).isApprox(B.row(i)))
					flag1 = false;
			if(flag1)
				w++;
			else
				w = 0;	
		}

		// Update the base
		B = lexicColsOrder(B_tmp, false);

		// cout << "Base = " << endl << B << endl;
		// cout << "Permanent columns = " << endl << P << endl;
		// cout << "k = " << endl << k << endl;
		// cout << "w = " << endl << w << endl;

		return;

	}

	// Simplex function
	void sendBase()
	{
		// Clear msg
		myMsg.data.clear();
		
		// Conversion of base matrix to vector
		vector<float> matMsg;
		for (j = 0; j < B.cols(); j++)
			for (i = 0; i < B.rows(); i++)
				matMsg.push_back(B(i,j));

		myMsg.data = matMsg;

		pub.publish(myMsg);

		return;
	}

	// Simplex function
	bool end()
	{
		if(w > ARTIFICIAL_PARAM && !find)
		{	
			cout << "Base = " << endl << B << endl;
			
			k = -1;
			x = 0;

			for(i = (id-1)*2; i < id*2; i++)
				for(j= 0; j < H.cols(); j++)
					if(B.col(i).isApprox(H.col(j)))
					{
						k = i;	
						x++;
					}
			
			if(x == 1){
				find = true;
				solution = B.col(k); 
			}
			else if(x > 1)
			{
				i = (id-1)*2;

				while(i < id*2 && !find)
				{
					flag2 = true;
					flag1 = true;
					z = AGENTS;
					while(B(z,i) == 0 && z <= ARTIFICIAL_PARAM)
						z++;	

					for(j= 0; j < B.cols(); j++)
						if(B(z,j) != 0 && j != i)
							flag2 = false;
												
					if(flag2)
					{
						find = true;
						solution = B.col(i); 
					}

				i++;
				}

				if(!find){
					i = (id-1)*2;

					while(i < id*2 && !find)
					{
						flag2 = true;
						flag1 = true;
						z = AGENTS;
						while(B(z,i) == 0 && z <= ARTIFICIAL_PARAM)
							z++;	

						for(j= 0; j < B.cols(); j++){
							if(B(z,j) != 0 && j != i){
								if(j == 0)
									if(B(ARTIFICIAL_PARAM,j+1) > M/2)
										flag1 = false;
								else if(j == B.cols()-1)
									if(B(ARTIFICIAL_PARAM,j-1) > M/2)
										flag1 = false;
								else if(j%2 == 0){
									if(B(ARTIFICIAL_PARAM,j+1) > M/2)
										flag1 = false;
										ROS_INFO("B(ARTIFICIAL_PARAM,j+1): %f", B(ARTIFICIAL_PARAM,j+1));
								}
								else{
									if(B(ARTIFICIAL_PARAM,j-1) > M/2)
										flag1 = false;
									ROS_INFO("B(ARTIFICIAL_PARAM,j+1): %f", B(ARTIFICIAL_PARAM,j-1));
								}
								float bnknn = j%2;
								
								ROS_INFO("j: %i (id:%i)", j, id);
								ROS_INFO("j/2: %f flag %d",bnknn , flag1);
							}
						}

						if(flag1)
						{
						find = true;
						solution = B.col(i); 
						}
						
					}
				}
			}
		}
		else if(find)
			w++;
		if(w > PARAMETERS*2 && find){
			exit_simplex = true;
			cout << "Solution: " << endl << solution << endl;
		}

		return exit_simplex;	
	}


	float getDistance(float x1, float y1, float x2, float y2)
	{
		return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
	}


	void goalPosition(){
		//ros::Rate loop(0.5);
		geometry_msgs::PoseStamped desired_pose;

		double dist = (double) solution(ARTIFICIAL_PARAM); 

		int target = 0;
		for(j= 0; j < H.cols(); j++)
			if(H.col(j).isApprox(solution))
				target = j;
		
		int sign_x = pow(-1,target), sign_y = pow(-1,target+1);
		if(target > 1)
			sign_y = pow(-1,target);

		double angle = atan2((-10*sign_y - init_position[1]),(10*sign_x - init_position[0]));
		
		float K_plus = 0.1;
		float K_minus = 0.06;
		if(angle < 0){
			angle += 2*PI;
			angle += K_minus*angle;
		}else
			angle -= K_plus*angle;

		cout << "id: " << id << "  angle: " << radian2degree(angle) << endl;

		rotate(0.5, angle, false);

		move(0.5,dist,true);
	}	
	
	void move(double speed, double distance, bool isForward){
		//declare a Twist message to send velocity commands
		geometry_msgs::Twist VelocityMessage;
		//declare tf transform listener: this transform listener will be used to listen and capture the transformation between
		// the /odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
		tf::TransformListener listener;
		//declare tf transform
		//init_transform: is the transformation before starting the motion
		tf::StampedTransform init_transform;
		//current_transformation: is the transformation while the robot is moving
		tf::StampedTransform current_transform;

		string first_frame = "/robot" + boost::lexical_cast<string>(id) + "/base_footprint";
		string second_frame = "/robot" + boost::lexical_cast<string>(id) + "/odom";

		//set the linear velocity to a positive value if isFoward is true
		if (isForward)
			VelocityMessage.linear.x =abs(speed);
		else //else set the velocity to negative value to move backward
			VelocityMessage.linear.x =-abs(speed);
		//all velocities of other axes must be zero.
		VelocityMessage.linear.y =0;
		VelocityMessage.linear.z =0;
		//The angular velocity of all axes must be zero because we want  a straight motion
		VelocityMessage.angular.x = 0;
		VelocityMessage.angular.y = 0;
		VelocityMessage.angular.z =0;

		double distance_moved = 0.0;
		ros::Rate loop_rate(10); // we publish the velocity at 10 Hz (10 times a second)

		/*
		* First, we capture the initial transformation before starting the motion.
		* we call this transformation "init_transform"
		* It is important to "waitForTransform" otherwise, it might not be captured.
		*/
		try{
			//wait for the transform to be found
			listener.waitForTransform(first_frame, second_frame, ros::Time(0), ros::Duration(10.0) );   ///base_footprint
			//Once the transform is found,get the initial_transform transformation.
			listener.lookupTransform(first_frame, second_frame,ros::Time(0), init_transform); ///base_footprint
		}
		catch (tf::TransformException & ex){
			ROS_ERROR(" Problem %s",ex.what());
			ros::Duration(1.0).sleep();
		}



		do{
			/***************************************
			* STEP1. PUBLISH THE VELOCITY MESSAGE
			***************************************/
			velocity_pub.publish(VelocityMessage);
			ros::spinOnce();
			loop_rate.sleep();
			/**************************************************
			* STEP2. ESTIMATE THE DISTANCE MOVED BY THE ROBOT
			*************************************************/
			try{
				//wait for the transform to be found
				listener.waitForTransform(first_frame, second_frame, ros::Time(0), ros::Duration(10.0) );
				//Once the transform is found,get the initial_transform transformation.
				listener.lookupTransform(first_frame, second_frame,ros::Time(0), current_transform);
			}
			catch (tf::TransformException & ex){
				ROS_ERROR(" Problem %s",ex.what());
				ros::Duration(1.0).sleep();
			}
			/*
			* Calculate the distance moved by the robot
			* There are two methods that give the same result
			*/

			/*
			* Method 1: Calculate the distance between the two transformations
			* Hint:
			* 	  --> transform.getOrigin().x(): represents the x coordinate of the transformation
			* 	  --> transform.getOrigin().y(): represents the y coordinate of the transformation
			*/
			//calculate the distance moved
			//cout<<"Initial Transform: "<<init_transform <<" , "<<"Current Transform: "<<current_transform<<endl;

			distance_moved = sqrt(pow((current_transform.getOrigin().x()-init_transform.getOrigin().x()), 2) +
					pow((current_transform.getOrigin().y()-init_transform.getOrigin().y()), 2));


		}while((distance_moved<distance)&&(ros::ok()));
		//finally, stop the robot when the distance is moved
		VelocityMessage.linear.x =0;
		velocity_pub.publish(VelocityMessage);
	}

	double rotate(double angular_velocity, double radians,  bool clockwise)
	{

		//delcare a Twist message to send velocity commands
		geometry_msgs::Twist VelocityMessage;
		//declare tf transform listener: this transform listener will be used to listen and capture the transformation between
		// the odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
		tf::TransformListener TFListener;
		//declare tf transform
		//init_transform: is the transformation before starting the motion
		tf::StampedTransform init_transform;
		//current_transformation: is the transformation while the robot is moving
		tf::StampedTransform current_transform;
		//initial coordinates (for method 3)
		nav_msgs::Odometry initial_turtlebot_odom_pose;

		double angle_turned =0.0;

		string first_frame = "/robot" + boost::lexical_cast<string>(id) + "/base_footprint";
		string second_frame = "/robot" + boost::lexical_cast<string>(id) + "/odom";

		//validate angular velocity; ANGULAR_VELOCITY_MINIMUM_THRESHOLD is the minimum allowed
		angular_velocity=((angular_velocity>ANGULAR_VELOCITY_MINIMUM_THRESHOLD)?angular_velocity:ANGULAR_VELOCITY_MINIMUM_THRESHOLD);

		while(radians < 0) radians += 2*M_PI;
		while(radians > 2*M_PI) radians -= 2*M_PI;

		//wait for the listener to get the first message
		TFListener.waitForTransform(first_frame, second_frame, ros::Time(0), ros::Duration(1.0));


		//record the starting transform from the odometry to the base frame
		TFListener.lookupTransform(first_frame, second_frame, ros::Time(0), init_transform);


		//the command will be to turn at 0.75 rad/s
		VelocityMessage.linear.x = VelocityMessage.linear.y = 0.0;
		VelocityMessage.angular.z = angular_velocity;
		if (clockwise) VelocityMessage.angular.z = -VelocityMessage.angular.z;

		//the axis we want to be rotating by
		tf::Vector3 desired_turn_axis(0,0,1);
		if (!clockwise) desired_turn_axis = -desired_turn_axis;

		ros::Rate rate(100.0);
		bool done = false;
		while (!done )
		{
			//send the drive command
			velocity_pub.publish(VelocityMessage);
			rate.sleep();
			//get the current transform
			try
			{
				TFListener.waitForTransform(first_frame, second_frame, ros::Time(0), ros::Duration(1.0));
				TFListener.lookupTransform(first_frame, second_frame, ros::Time(0), current_transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				break;
			}
			tf::Transform relative_transform = init_transform.inverse() * current_transform;
			tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
			angle_turned = relative_transform.getRotation().getAngle();

			if (fabs(angle_turned) < 1.0e-2) continue;
			if (actual_turn_axis.dot(desired_turn_axis ) < 0 )
				angle_turned = 2*M_PI - angle_turned;

			if (!clockwise)
				VelocityMessage.angular.z = (angular_velocity-ANGULAR_VELOCITY_MINIMUM_THRESHOLD) * (fabs(radian2degree(radians-angle_turned)/radian2degree(radians)))+ANGULAR_VELOCITY_MINIMUM_THRESHOLD;
			else
				if (clockwise)
					VelocityMessage.angular.z = (-angular_velocity+ANGULAR_VELOCITY_MINIMUM_THRESHOLD) * (fabs(radian2degree(radians-angle_turned)/radian2degree(radians)))-ANGULAR_VELOCITY_MINIMUM_THRESHOLD;

			if (angle_turned > radians) {
				done = true;
				VelocityMessage.linear.x = VelocityMessage.linear.y = VelocityMessage.angular.z = 0;
				velocity_pub.publish(VelocityMessage);
			}


		}
		if (done) return angle_turned;
		return angle_turned;
	}

	/* makes conversion from radian to degree */
	double radian2degree(double radianAngle){
		return (radianAngle*57.2957795);
	}

	/* makes conversion from degree to radian */
	double degree2radian(double degreeAngle){
		return (degreeAngle/57.2957795);
	}

	private:

	// ROS variables
	ros::NodeHandle node; 
  	ros::Publisher pub, velocity_pub;
  	ros::Subscriber sub; 
	//geometry_msgs::PoseStamped robot_pose;
	nav_msgs::Odometry robot_pose;

	// Topic names  
	string publish_edge = "edge_";
	string subscribe_edge = "edge_";
	string vel_name = "robot";
	//string pos_name = "robot";

	std_msgs::Float32MultiArray myMsg;										// Message send to agent (i.e. the base)
	vector<float> data;														// Message received

	// Variables
	unsigned int AGENTS, PARAMETERS, ARTIFICIAL_PARAM, M;
	MatrixXf H,H_tmp, test;														//Matrix H, H_tmp
	MatrixXf B,P,B_tmp,B_received;											//Base, Base temporary and Base received
	MatrixXf solution;
	VectorXf constraints, init_position;									//Constraints random

	// Iterator
	unsigned int id,i,j,k,w,x,z;
	bool flag1,flag2, find, base_changed, exit_simplex;

};



int main(int argc, char* argv[])
{
	// Agent's identification number
	int n = atoi(argv[1]);			// Agent's number
	cout << "Agent ID: " << n << endl;
	string name = "agent" + boost::lexical_cast<string>(n);

	// Initial Position of agent
	VectorXf init_pos = VectorXf::Zero(2);
	init_pos << atoi(argv[2]), atoi(argv[3]);

	ros::init(argc, argv, name);

	Agent I(n, AGENTS, init_pos);	//Create I object of class Agent

	ros::Rate loop_rate(10);

	bool end_simplex = false;		// When solution is founded the communication can stop
	while (!end_simplex)
	{

		I.sendBase();				// Send the base to the next agent
		end_simplex = I.end();		// Check if the base is not changing anymore

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	I.goalPosition();
	
    return 0;

}
