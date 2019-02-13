#include <cstdlib>
#include "ros/ros.h"
#include <iostream>
#include <string>
#include "multiagent_planning/Pose.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "agent");
  ros::NodeHandle n;

  ros::Publisher location_publisher = n.advertise<multiagent_planning::Pose>("/agent_feedback", 1000);

  std::string serial_id;
  std::string start_position;

  n.getParam("serial_id", serial_id);
  n.getParam("start_position", start_position);  

  multiagent_planning::Pose location;

  while (ros::ok())
  {
    std::stringstream ss;

    location.x = 0;
    location.y = 1;
    location.yaw = 2;
    location.pose = start_position;

    location_publisher.publish(location);

    ros::spinOnce();
  }
  return 0;
}
