/*
Date created: 02/05/2019
Author: Sumukha M. Harish
email: sumukhamh@gmail.com

This is the planner node class. The node has the following

1. get_plan_server()
Which takes in /get_plan request from the agent nodes and 
computes the path from by using A* algorithm. The path is 
returned as a service response.

2. agent_feedback_subscriber()
The subscriber listens to the /agent_feedback topic from the
agent. This gives the information about the current position 
required to be used in A* algorithm.

*/

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <iostream>
#include <string>

#include "../include/planner.h"
#include "../include/multiagent_planning.h"
#include "multiagent_planning/Pose.h"
#include "multiagent_planning/Point.h"
#include "multiagent_planning/GetPlan.h"

Planner::Planner(){
}

Planner::~Planner(){
}

Planner::Planner(ros::NodeHandle &node_for_server, ros::NodeHandle &node_for_subscriber){
	n_server = node_for_server;
	n_subscriber = node_for_subscriber;
	topic_name = "/agent_feedback";
	get_plan_server = n_server.advertiseService("/get_plan", &Planner::get_plan_server_func, this);
}

void Planner::run(){
	// Create grid
	make_grid();

	while(ros::ok()){
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}
}

bool Planner::get_plan_server_func(multiagent_planning::GetPlan::Request &plan_req, multiagent_planning::GetPlan::Response &plan_res){
	// Get data from service request
	serial_id = plan_req.serial_id;
	goal_pose.x = plan_req.pose.x;
	goal_pose.y = plan_req.pose.y;
	goal_pose.yaw = plan_req.pose.yaw;

	topic_name = "/" + serial_id + topic_name;

	// Initialise subscriber based on serial_id from service request
  	agent_feedback_subscriber = n_subscriber.subscribe<multiagent_planning::Pose>(topic_name, 10, &Planner::callback_agent_feedback, this);	
	ros::spinOnce();

	////////////////////////////////////////////////////////////////////////
	// Please	 uncomment the lines below for hard set current_pose values. 
	// The subscriber initially does not subscribe to the topic. Hence the 
	// current_pose values would be set to 0.
	// To work around this I think an asynchronous spinner must be used. The 
	// implementation of it was tried, but in vain and hence the code has been
	// compromised here.

	// current_pose.x = 0;
	// current_pose.y = 0;
	// current_pose.yaw = 0;

	////////////////////////////////////////////////////////////////////////

	std::vector<std::pair<int, int>> path;

	ROS_INFO("Node id: %s", serial_id.c_str());
	ROS_INFO("Current (x,y): %d %d", current_pose.x, current_pose.y);
	ROS_INFO("Goal (x,y): %d %d", goal_pose.x, goal_pose.y);

	// Get path
	path = a_star(current_pose.x, current_pose.y, goal_pose.x, goal_pose.y);

	// Set path to the response
	plan_res.path = convert_path_to_message_type(path);

	// On terminal
	display_path(path);

	// Save to buffer
	save_path(current_pose.x, current_pose.y, goal_pose.x, goal_pose.x, path);
	
	// Restore defaults
	clear_path();
}

void Planner::callback_agent_feedback(multiagent_planning::Pose agent_pose_msg){
	current_pose.x = agent_pose_msg.x;
	current_pose.y = agent_pose_msg.y;
}
