/*
Date created: 02/05/2019
Author: Sumukha M. Harish
email: sumukhamh@gmail.com

This is the starter code for the planner node. It instantiates 
a Planner object and executes the run() method.

*/

#include <ros/ros.h>
#include "../include/planner.h"

int main(int argc, char **argv){ 
	ros::init(argc, argv, "planner");
  	ros::NodeHandle node_server;
	ros::NodeHandle node_subscriber;

  	Planner planner(node_server, node_subscriber);

  	planner.run();
}	