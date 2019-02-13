#include "ros/ros.h"
#include <iostream>
#include <string>
#include "std_msgs/String.h"

#include "multiagent_planning/Pose.h"
#include "multiagent_planning/GoalPose.h"

struct Pose2D{
  int x;
  int y;
  float yaw;
};

Pose2D current_pose;

bool goal_update_server_func(multiagent_planning::GoalPose::Request &req, multiagent_planning::GoalPose::Response &res){
  res.id = "agent";
  res.goal = "goal";
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "agent");
  ros::NodeHandle n("~");

  ros::Publisher pose_publisher = n.advertise<multiagent_planning::Pose>("/agent_feedback", 1000);
  ros::Publisher id_publisher = n.advertise<std_msgs::String>("/agent_id",10);
  ros::ServiceServer goal_update_server = n.advertiseService("/update_goal", goal_update_server_func);

  std::string serial_id;
  int start_x, start_y;
  float start_yaw;

  n.getParam("serial_id", serial_id); 
  n.getParam("x", start_x);  
  n.getParam("y", start_y);  
  n.getParam("yaw", start_yaw);  

  std_msgs::String id_msg;
  id_msg.data = serial_id.c_str();

  current_pose.x = start_x;
  current_pose.y = start_y;
  current_pose.yaw = start_yaw;

  while (ros::ok())
  {
    multiagent_planning::Pose pose_msg;
    pose_msg.x = current_pose.x;
    pose_msg.y = current_pose.y;
    pose_msg.yaw = current_pose.yaw;

    pose_publisher.publish(pose_msg);
    id_publisher.publish(id_msg);

    ros::Duration(0.1).sleep();

    ros::spinOnce();
  }
  return 0;
}
