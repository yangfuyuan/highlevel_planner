//This is the dataManager class
// it serves as an inteface for rostopics, which allows for direct porting of algorithms from the C++ tester.

#ifndef _DATAMANAGER__H
#define _DATAMANAGER__H

//I don't think all of these are necessary
#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "ap_network_bridge/NetPoseStamped.h"
#include "highlevel_planner/CoorData.h"
#include "highlevel_planner/SpWaypoints.h"
#include "autopilot_bridge/LLA.h"
#include "coordination_data.h"
#include <iostream>

//Define the number of agents in the network at any time
const int MAX_NEIGHBORS = 50;

typedef struct waypoint{
  float lat;
  float lon;  
}Waypoint; 

typedef struct pose{
  float xN; 
  float yN; 
  float zN; 
  float qxN; 
  float qyN; 
  float qzN; 
  float qwN;
}Pose; 

class dataManager{
  public:
    dataManager(ros::NodeHandle nodeHandle);
    bool* get_neighborhood();
    Pose* get_pose_list();
    coorData* get_coor_data();
    void send_coor_data(coorData* cdata, int uid);
    void send_waypoints(Waypoint* spWps,int numWps);
    void recv_pose_callback(const ap_network_bridge::NetPoseStamped& networkData);
    void odom_msg_callback(const geometry_msgs::PoseWithCovarianceStamped& selfData);
    void coor_data_callback(const highlevel_planner::CoorData& cdata);
    void resetNeighborList();
  private:
     //Forgive the static memory allocation - C habits die hard.
     bool neighbors[MAX_NEIGHBORS];
     Pose poseList[MAX_NEIGHBORS+1];
     coorData cdataList[MAX_NEIGHBORS+1];
     ros::NodeHandle nh;
     ros::Subscriber sub_recv_pose;
     ros::Subscriber sub_odom_msg; 
     ros::Subscriber sub_coor_data;
     ros::Publisher  pub_coor_data;
     ros::Publisher  pub_sp_waypoints;
};

#endif
