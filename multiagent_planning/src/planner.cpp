#include <cstdlib>
#include "ros/ros.h"
#include <iostream>
#include <string>
#include "multiagent_planning/Pose.h"
#include "std_msgs/String.h"

bool get_plan_func(){}

int main(int argc, char const *argv[]){

	ros::init(argc, argv, "planner");
  	ros::NodeHandle n("~");

	ros::ServiceServer get_plan_server = n.advertiseService("/get_plan", get_plan_func);
	return 0;
}
