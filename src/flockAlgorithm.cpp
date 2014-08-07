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

  printf("\n*\n");fflush(stdout);
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
  coorData cmsg; //share
  cmsg.msgType = FLOCK_MSGTYPE;
  cmsg.FLOCK_TARGETID = state;
  cmsg.FLOCK_TARGETX = tmpHeading.lat;
  cmsg.FLOCK_TARGETY = tmpHeading.lon;
  int uid = 0;

  //check if nonzero neighborhood:
  bool nbrFlag = false;
  for(int i = 0; i < MAX_NEIGHBORS && nbrFlag == false; i++)
  {
    if(neighbors[i] == true)
    {
      printf("n%d ",i);
      nbrFlag = true;
    }
  }
  //Initialize next position to goal
  Waypoint nextPos;
  nextPos.lat = tmpGoal.xN;
  nextPos.lon = tmpGoal.yN;

 //if no other neighbors, send goal and return.
  if(!nbrFlag && goalFlag){
     //I don't think this ever triggers...
  }else if(nbrFlag){ //other neighbors, so attempt to flock
  //Run consensus to find goal direction (most susceptible to asynchronous failure)
    printf("Neighbors\n");
    float leaderDist = INFINITY; 
    Pose tmpPos;
    //For every neighbor: 
    for(int i = 0; i < MAX_NEIGHBORS; i++)
    {
      if(neighbors[i]){
        //If same goal: 
        if(coordata[i].msgType == FLOCK_MSGTYPE && coordata[i].FLOCK_TARGETID == state)
        {

	  //Vector to neighbor:
	  tmpPos.xN = (poslist[i]).xN - selfPose.xN;
          tmpPos.yN = (poslist[i]).yN - selfPose.yN;
          	  
	  if(poslist[i].xN < 1 && poslist[i].yN < 1) break;//not a legitimate point.
	  if(dist(&(poslist[i]),&selfPose)>TOL) //not yourself
	  {
            //run consensus algorithm
            tmpHeading.lat+=coordata[i].FLOCK_TARGETX;
            tmpHeading.lon+=coordata[i].FLOCK_TARGETY;
            N++;
          
	  //Closer to goal and closest to you?
          
	  
	  //Shift coordinates:
            proj2D(targetDirection, &tmpPos, &tmpPos);
            float w = 0.1;
            tmpPos.xN *= w;
            float nearness = tmpPos.xN*tmpPos.xN + tmpPos.yN*tmpPos.yN;
            tmpPos.xN /= w;
            if(tmpPos.xN > TOL  && nearness < leaderDist)
	    {
 	      leaderDist = nearness;
	      tmpPos.yN = SEPY*(1-2*(tmpPos.yN< 0)); //second term is sign(tmpPos[y])
	      tmpPos.xN = SEPX; tmpPos.yN = SEPY;
	      proj2D(-targetDirection, &tmpPos, &tmpPos);
	      nextPos.lat = tmpPos.xN + (poslist[i]).xN;
              nextPos.lon = tmpPos.yN + (poslist[i]).yN;
            }
	  }else{ //Assume that this must be your uid
	    //This might be dangerous...
	    uid = i;
	  }
        }
      }
    }
  }
  //send waypoint
  if(abs(nextPos.lat-lastPos.lat)>TOL || abs(nextPos.lon- lastPos.lon)>TOL || goalFlag) //dont send duplicate waypoints
    data->send_waypoints(&nextPos,1);

  data->send_coor_data(&cmsg, uid);
  
  lastPos.lat = nextPos.lat; 
  lastPos.lon = nextPos.lon;
  
  //TODO change to meters.
  tmpHeading.lat /= N; tmpHeading.lon/=N;
  atan2f(tmpHeading.lat,tmpHeading.lon);
}

bool flockAlgorithm::updateGoal()
{
  //be sure selfPose is valid
  if(abs(selfPose.xN) > 0.001 || abs(selfPose.yN) > 0.001)
  {
    printf("\nd = %f\n",dist(&selfPose,&(box[(int)state])));
    //goal needs to be updated
//TODO fix the waypoint following so you actually get within 10m (typically hits 80).
    if(state==-1.0 || dist(&selfPose,&(box[(int)state])) < 0.001)
    {
      printf("%f",state);
      state++; if(state>3.0){state = 0.0;} //update state
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

void flockAlgorithm::proj2D(const float angle, Pose* original, Pose* output)
{
  Pose tmp;
  tmp.xN = cos(angle)*original->xN + sin(angle)*original->yN;
  tmp.yN = cos(angle)*original->yN - sin(angle)*original->xN;
  output->xN = tmp.xN;
  output->yN = tmp.yN;
}

float flockAlgorithm::metricDistance(Pose* lla_1, Pose* lla_2)
{
    float R = 6378.137; // Radius of earth in KM
    float dLat = (lla_1->xN - lla_1->xN) * M_PI / 180;
    float dLon = (lla_2->yN - lla_1->yN) * M_PI / 180;
    float a =sin(dLat/2)*sin(dLat/2);
    a+=cos(lla_1->xN*M_PI/180)*cos(lla_2->yN*M_PI/180)*sin(dLon/2)*sin(dLon/2);
    float c = 2*atan2(sqrt(a), sqrt(1-a));
    return (R*c*1000);//meters
}
