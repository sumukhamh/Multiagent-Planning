#include "ros/ros.h"
#include <iostream>
#include <string>
#include "../include/multiagent_planning.h"
#include "multiagent_planning/Pose.h"
#include "multiagent_planning/Point.h"
#include "multiagent_planning/GoalPose.h"
#include "multiagent_planning/GetPlan.h"
#include "std_msgs/String.h"

Pose2D current_pose;
Pose2D goal_pose;
bool launch_planner = false;
std::string node_serial_id;

bool goal_update_server_func(multiagent_planning::GoalPose::Request &goal_req, multiagent_planning::GoalPose::Response &goal_res){
  goal_res.id = node_serial_id;

  goal_pose.x = goal_req.pose.x;
  goal_pose.y = goal_req.pose.y;
  goal_pose.yaw = goal_req.pose.yaw;

  launch_planner = true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "agent");
  ros::NodeHandle n("~");

  ros::Publisher pose_publisher = n.advertise<multiagent_planning::Pose>("/agent_feedback", 1000);
  ros::Publisher id_publisher = n.advertise<std_msgs::String>("/agent_id",10);
  ros::ServiceServer goal_update_server = n.advertiseService("/update_goal", goal_update_server_func);
  ros::ServiceClient get_plan_client = n.serviceClient<multiagent_planning::GetPlan>("/get_plan");

  int start_x, start_y;
  float start_yaw;

  n.getParam("serial_id", node_serial_id); 
  n.getParam("x", start_x);  
  n.getParam("y", start_y);  
  n.getParam("yaw", start_yaw);  

  std_msgs::String id_msg;
  id_msg.data = node_serial_id.c_str();

  current_pose.x = start_x;
  current_pose.y = start_y;
  current_pose.yaw = start_yaw; 
  
  while (ros::ok())
  {
    if(launch_planner){

      multiagent_planning::GetPlan srv;
      srv.request.serial_id = node_serial_id;
      srv.request.pose.x = goal_pose.x;
      srv.request.pose.y = goal_pose.y;
      srv.request.pose.yaw = goal_pose.yaw;

      if(get_plan_client.call(srv)){
        ROS_INFO("Sumukha");
      }

      launch_planner = false;
    }

    multiagent_planning::Pose pose_msg;
    pose_msg.x = current_pose.x;
    pose_msg.y = current_pose.y;
    pose_msg.yaw = current_pose.yaw;

    pose_publisher.publish(pose_msg);
    id_publisher.publish(id_msg);

    ros::spinOnce();
  }
  return 0;
}
