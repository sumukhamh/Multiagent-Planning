#include "ros/ros.h"
#include <ros/callback_queue.h>
// #include <ros/callback_queue_interface.h>
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
	make_grid();

	while(ros::ok()){
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}
}

bool Planner::get_plan_server_func(multiagent_planning::GetPlan::Request &plan_req, multiagent_planning::GetPlan::Response &plan_res){
	serial_id = plan_req.serial_id;
	goal_pose.x = plan_req.pose.x;
	goal_pose.y = plan_req.pose.y;
	goal_pose.yaw = plan_req.pose.yaw;

	topic_name = "/" + serial_id + topic_name;

	ros::CallbackQueue agent_feedback_queue;
	// agent_feedback_queue.addCallback(&Planner::callback_agent_feedback);
  	agent_feedback_subscriber = n_subscriber.subscribe<multiagent_planning::Pose>(topic_name, 10, &Planner::callback_agent_feedback, this);	

	// ros::AsyncSpinner spinner(1, &agent_feedback_queue);
	// spinner.start();
	// ros::spinOnce();
	
	current_pose.x = 9;
	current_pose.y = 9;
	std::vector<std::pair<int, int>> path;
	ROS_INFO("%d %d %d %d", current_pose.x, current_pose.y, goal_pose.x, goal_pose.y);
	path = a_star(current_pose.x, current_pose.y, goal_pose.x, goal_pose.y);

	plan_res.path = convert_path_to_message_type(path);

	display_path(path);

	save_path(current_pose.x, current_pose.y, goal_pose.x, goal_pose.x, path);
	
	clear_path();
}

void Planner::callback_agent_feedback(multiagent_planning::Pose agent_pose_msg){
	current_pose.x = agent_pose_msg.x;
	current_pose.y = agent_pose_msg.y;
}
