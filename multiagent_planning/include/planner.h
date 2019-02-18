#pragma once
#include <ros/ros.h>
#include <iostream>
#include <string>

#include "multiagent_planning.h"
#include "multiagent_planning/Pose.h"
#include "multiagent_planning/GetPlan.h"

class Planner{
	private:
		Pose2D current_pose;
		Pose2D goal_pose;
		std::string serial_id;
		std::string topic_name;

		ros::ServiceServer get_plan_server;
		ros::Subscriber agent_feedback_subscriber;

		bool get_plan_server_func(multiagent_planning::GetPlan::Request &plan_req, multiagent_planning::GetPlan::Response &plan_res);
		void callback_agent_feedback(multiagent_planning::Pose agent_pose_msg);

	public:
		Planner();
		~Planner();
		Planner(ros::NodeHandle &node_for_server, ros::NodeHandle &node_for_subscriber);
		ros::NodeHandle n_server;
		ros::NodeHandle n_subscriber;
		void run();
};