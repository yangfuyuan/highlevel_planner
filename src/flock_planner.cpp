/*
   Main routine for running the high level planning routine in the autonomy payload
*/


//Custom includes:
#include "dataManager.h"
#include "flockAlgorithm.h"

//Standard includes:
#include <iostream>
#include <ros/ros.h>

int main(int argc, char**argv){

  //Initialize ROS:
  ros::init(argc,argv,"highlevel_planner");
  ros::NodeHandle n;

  //Set up subscribers in data_manager object:
  dataManager data_manager(n);
//  sleep(1);
  //Initialize instance of algorithm to run
  flockAlgorithm flock(&data_manager);

  ros::Rate loop_rate(10);
  //Run loop:
  while(ros::ok())
  {
    //gathering data happens automagically
    flock.update();  //Here the algorithm takes in data and updates data_manager with data to send
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


