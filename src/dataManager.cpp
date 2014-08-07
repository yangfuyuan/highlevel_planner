#include "dataManager.h"

dataManager::dataManager(ros::NodeHandle nodeHandle)
{
   //Initialize ROS services:
   nh = nodeHandle;

   sub_recv_pose = nh.subscribe("/network/recv_pose",1000,&dataManager::recv_pose_callback,this);
   sub_odom_msg = nh.subscribe("/local_estim/odom_combined",1000,&dataManager::odom_msg_callback,this);
   sub_coor_data = nh.subscribe("/highlevel_planner/CoorData",1000,&dataManager::coor_data_callback,this);

   //Set up publishers here, once you figure out how.
   pub_coor_data = nh.advertise<highlevel_planner::CoorData>("/highlevel_planner/CoorData",1000);
//   pub_sp_waypoints = nh.advertise<highlevel_planner::SpWaypoints>("spWaypoints",1000); 
  pub_sp_waypoints = nh.advertise<autopilot_bridge::LLA>("/autopilot/guided_goto",1000);
}

bool* dataManager::get_neighborhood()
{
  return neighbors;
}

Pose* dataManager::get_pose_list()
{
  return poseList;
}

coorData* dataManager::get_coor_data()
{
  return cdataList;
}

void dataManager::send_coor_data(coorData* cdata)
{
  highlevel_planner::CoorData msg; 
  msg.msgType = cdata->msgType;
  msg.field1 = cdata->field1;
  msg.field2 = cdata->field2;
  msg.field3 = cdata->field3;
  msg.uid = 0; //right now I have no access to my own UID; inject at network node?
  pub_coor_data.publish(msg); //does this work?
}

void dataManager::send_waypoints(Waypoint* spWps, int numWps)
{
//  highlevel_planner::SpWaypoints msg;
  autopilot_bridge::LLA msg;
  //call the publisher and push the waypoints
  for(int i = 0; i < numWps; i++){
    msg.lat = spWps[i].lat;
    msg.lon = spWps[i].lon;
    msg.alt = 300; //fixed altitude
    printf("Publishing waypoint!(%f, %f, %f)\n",msg.lat,msg.lon,msg.alt); fflush(stdout);
    pub_sp_waypoints.publish(msg); 
  }
}

void dataManager::recv_pose_callback(const ap_network_bridge::NetPoseStamped& networkdata)
{
  if(networkdata.sender_id > MAX_NEIGHBORS){
      //Complain.
  }else{
     neighbors[networkdata.sender_id] = true;
     Pose* updated = poseList + networkdata.sender_id;

     updated->xN = networkdata.pose.pose.position.x;
     updated->yN = networkdata.pose.pose.position.y;
     updated->zN = networkdata.pose.pose.position.z;

     updated->qxN = networkdata.pose.pose.orientation.x;
     updated->qyN = networkdata.pose.pose.orientation.y;
     updated->qzN = networkdata.pose.pose.orientation.z;
     updated->qwN = networkdata.pose.pose.orientation.w;
  }
}

void dataManager::odom_msg_callback(const geometry_msgs::PoseWithCovarianceStamped& selfData)
{
     Pose* selfUpdate = &(poseList[MAX_NEIGHBORS]); //extra entry at the end is for self.
     poseList[MAX_NEIGHBORS].xN = selfData.pose.pose.position.x;
     poseList[MAX_NEIGHBORS].yN = selfData.pose.pose.position.y;
     poseList[MAX_NEIGHBORS].zN = selfData.pose.pose.position.z;
     poseList[MAX_NEIGHBORS].qxN = selfData.pose.pose.orientation.x;
     poseList[MAX_NEIGHBORS].qyN = selfData.pose.pose.orientation.y;
     poseList[MAX_NEIGHBORS].qzN = selfData.pose.pose.orientation.z;
     poseList[MAX_NEIGHBORS].qwN = selfData.pose.pose.orientation.w;
     
}


void dataManager::coor_data_callback(const highlevel_planner::CoorData& cdata)
{
  neighbors[cdata.uid] = true;
  cdataList[cdata.uid].msgType = cdata.msgType;
  cdataList[cdata.uid].field1 = cdata.field1;
  cdataList[cdata.uid].field2 = cdata.field2;
  cdataList[cdata.uid].field3 = cdata.field3; 
}

/* PRIVATE METHODS */

void dataManager::resetNeighborList()
{
  for(int i = 0; i < MAX_NEIGHBORS; i++)
     neighbors[i] = false;
}
