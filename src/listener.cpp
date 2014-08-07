#include "ros/ros.h"
#include "highlevel_planner/CoorData.h"

void chatterCallback(const highlevel_planner::CoorData msg)
{ 
  ROS_INFO("I heard: [%d]",msg.uid);
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"listener");
  ros::NodeHandle n; 
  ros::Subscriber sub = n.subscribe("chatter",1000,chatterCallback);
  ros::spin();
  return 0;
}

