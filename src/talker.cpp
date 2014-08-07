#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float64.h"
#include "highlevel_planner/CoorData.h"

int main(int argc, char**argv)
{
  ros::init(argc,argv,"talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<highlevel_planner::CoorData>("chatter",1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while(ros::ok())
  {
    highlevel_planner::CoorData msg;    
    msg.uid = count;


    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep(); 
    ++count;
  }

  return 0; 
}


