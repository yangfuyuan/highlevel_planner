

#include "simpleAlgorithm.h"

void simpleAlgorithm::update()
{
  //get self position:
  Pose* poslist = data->get_pose_list();
//  printf("%f,%f,%f - ", poslist[MAX_NEIGHBORS].xN,poslist[MAX_NEIGHBORS].yN,poslist[MAX_NEIGHBORS].zN);
  Pose selfPose; 
  selfPose.xN = poslist[MAX_NEIGHBORS].xN;
  selfPose.yN = poslist[MAX_NEIGHBORS].yN;
  selfPose.zN = poslist[MAX_NEIGHBORS].zN;
 
  //valid waypoint data?
 const int TOL = 0.0001;
  if(abs(selfPose.xN) > TOL || abs(selfPose.yN) > TOL || abs(selfPose.zN) > TOL){ 
  //Close to next waypoint?
//TODO Fix the tolerance on this, so that it is the loiter radius.
    if(state == -1 || dist(&selfPose,&(box[state])) < 0.001)
    {
      state++;
      if(state > 3){ state = 0;}
      printf("\nReached vertex %d!(%f) \n",state,dist(&selfPose,&(box[state])));fflush(stdout);
      Waypoint wp; 
      wp.lat = box[state].xN;
      wp.lon = box[state].yN;
      data->send_waypoints(&wp,1);
    }
  }
}

float simpleAlgorithm::dist(Pose* n1, Pose* n2)
{
  //Simple euclidian distance function, in units degrees.
  float x,y,z; 
  x = n1->xN - n2->xN;
  y = n1->yN - n2->yN; 
  return sqrt(x*x + y*y);
}
