#include "flockAlgorithm.h"

void flockAlgorithm::update()
{
/*
  0) Check whether any other agents in the neighborhood
  1) Check if already at goal (if so, update goal)
  2) Check if any agents closer to goal than you, who are aiming for the same goal
    a) Choose waypoint as position near to agent in front (is there a way to do this without loiter?)
  2b) else choose waypoint as current goal
  3) if waypoint changed, pass to data object.
*/


  //get current data:
  Pose* poslist = data->get_pose_list();
  bool* neighbors = data->get_neighborhood();
  coorData* coordata = data->get_coor_data();
  
  //update self position
  selfPose.xN = (poslist[MAX_NEIGHBORS]).xN;
  selfPose.yN = (poslist[MAX_NEIGHBORS]).yN;
  selfPose.zN = (poslist[MAX_NEIGHBORS]).zN;

  //update goal & transmit
  bool goalFlag = updateGoal();
  Pose tmpGoal; 
  tmpGoal.xN = (box[(int)state]).xN;
  tmpGoal.yN = (box[(int)state]).yN; 
  tmpGoal.zN = (box[(int)state]).zN;

  //Compute coordination value and share (even if no one is there)  
  int N = 1; //number of sources
  Waypoint tmpHeading; //holder for estimate of goal direction
  
  getHeading(selfPose, tmpGoal, &tmpHeading); //self-estimate

  //pack into coorData msg: 
  coorData msg; //share
  msg.msgType = FLOCK_MSGTYPE;
  msg.FLOCK_TARGETID = state;
  msg.FLOCK_TARGETX = tmpHeading.lat;
  msg.FLOCK_TARGETY = tmpHeading.lon;
  data->send_coor_data(&msg);
 
  //check if nonzero neighborhood:
  bool nbrFlag = false;
  for(int i = 0; i < MAX_NEIGHBORS && nbrFlag == false; i++)
    if(neighbors[i] == true) nbrFlag = true;

  //Initialize next position to goal
  Waypoint nextPos;
  nextPos.lat = tmpGoal.xN;
  nextPos.lon = tmpGoal.yN;

 //if no other neighbors, send goal and return.
  if(!nbrFlag && goalFlag){
    data->send_waypoints(&nextPos,1);
  }else{ //other neighbors, so attempt to flock
  //Run consensus to find goal direction (most susceptible to asynchronous failure)

    float minDist = INFINITY; 
    Pose tmpPos;
    //For every neighbor: 
    for(int i = 0; i < MAX_NEIGHBORS; i++)
    {
      if(neighbors[i]){
        //If same goal: 
        if(coordata[i].msgType == FLOCK_MSGTYPE && coordata[i].FLOCK_TARGETID == state)
        {
          //run consensus algorithm
          tmpHeading.lat+=coordata[i].FLOCK_TARGETX;
          tmpHeading.lon+=coordata[i].FLOCK_TARGETY;
          N++;
          
	  //Closer to goal and closest to you?
          
	  //Vector to neighbor:
	  tmpPos.lat = (poselist[i]).lat - selfPose.lat;
          tmpPos.lon = (poselist[i]).lon - selfPose.lon;      
	  //Shift coordinates:
          proj2D(targetDirection, &tmpPos, &tmpPos);
          float w = 0.1;
          tmpPos.xN *= 0.1;
          float nearness = tmpPos.xN*tmpPos.xN + tmpPos.yN*tmpPos.yN;
          tmpPos.xN /= 0.1;
          if(tmpPos.xN > 0.0001 && nearness < leaderDist)
	  {
 	    leaderDist = nearness;
	    tmpPos.yN = SEPY*(1-2*(tmpPos[y] < 0)); //second term is sign(tmpPos[y]
	    tmpPos.xN = SEPX; tmpPos.yN = SEPY;
	    proj2D(-targetDirection, &tmpPos, tmpPos);
	    nextPos.xN = tmpPos
        }
      }
    }

  //send waypoint
    if(nextPos.lat != lastPos.lat || nextPos.lon != lastPos.lon) //dont send duplicate waypoints
      data->send_waypoints(&nextPos,1);
    //update last position
    lastPos.lat = nextPos.lat; 
    lastPos.lon = nextPos.lon;
  }
  //TODO change to meters.
  targetDirection = atan2f(lastPos.lat,lastPos.lon);
}

bool flockAlgorithm::updateGoal()
{
  //be sure selfPose is valid
  if(abs(selfPose.xN) > 0.001 || abs(selfPose.yN) > 0.001)
  {
    //goal needs to be updated
    if(state==-1 || dist(&selfPose,&(box[(int)state])) < 0.001)
    {
      state++; if(state>3){state = 0;} //update state
      return true;
    }
  }
  return false;
}

float flockAlgorithm::dist(Pose* n1, Pose* n2)
{
  //Simple euclidian distance function, in units degrees.
  float x,y,z; 
  x = n1->xN - n2->xN;
  y = n1->yN - n2->yN; 
  return sqrt(x*x + y*y);
}

void flockAlgorithm::getHeading(Pose& source, Pose& dest, Waypoint* unit)
{
  float dx = dest.xN - source.xN; 
  float dy = dest.yN - source.yN;
  float mag =sqrt(dx*dx + dy*dy);
  unit->lat = dx/mag;
  unit->lon = dy/mag;
}

void flockAlgorithm::proj2D(float angle, Pose& original, Pose& output)
{
  Pose tmp;
  tmp.xN = cos(angle)*original.xN + sin(angle)*original.yN;
  tmp.yN = cos(angle)*original.yN - sin(angle)*original.xN;
  output.xN = tmp.xN;
  output.yN = tmp.yN;
}
