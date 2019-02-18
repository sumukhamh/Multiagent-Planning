#include <ros/ros.h>
#include "../include/planner.h"

int main(int argc, char **argv){ 
	ros::init(argc, argv, "planner");
  	ros::NodeHandle node_server;
	ros::NodeHandle node_subscriber;

  	Planner planner(node_server, node_subscriber);

  	planner.run();
}	