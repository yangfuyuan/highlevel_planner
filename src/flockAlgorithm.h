#ifndef _FLOCK_ALGORITHM__
#define _FLOCK_ALGORITHM__

#include <math.h>
#include "dataManager.h"
#include "Distributed_libraries/coord.h"
#include "coordination_data.h"
#include <iostream>


//TODO Convert all internal units to meters. Makes the math too messy (and sensitive) otherwise.
//Flocking separation
const float SEPX = 0.0005;
const float SEPY = 0.0005;

class flockAlgorithm{
  public: 
    flockAlgorithm(dataManager* dm){
	data = dm;
        lastPos.lat = 0; 
        lastPos.lon = 0;
        targetDirection = 0;
        state = -1;
//Load box waypoints
        box[0].xN = 35.716431;
        box[0].yN = -120.762474;
        box[0].zN = 300;
        box[1].xN = 35.717876;
        box[1].yN = -120.766838;
        box[1].zN = 300;
        box[2].xN = 35.719574;
        box[2].yN = -120.765930;
        box[2].zN = 300;
        box[3].xN = 35.718086;
        box[3].yN = -120.761398;
        box[3].zN = 300;
    }
    void update();

  private:
    //returns the distance between n1, n2 in the x,y plane
    float dist(Pose* n1, Pose* n2);
    //updates goal if necessary, returns whether goal changed.
    bool updateGoal();
    //sets unit to the unit vector pointing from source to dest
    void getHeading(Pose& source, Pose& dest, Waypoint* unit);
    //Projects original 2D vector onto coordinates rotated by angle
    void proj2D(float angle, Pose& original, Pose& output);
    Waypoint lastPos;
    dataManager* data;
    float state;
    float targetDirection;
    Pose box[4];
    Pose selfPose;
};

#endif //_FLOCK_ALGORITHM__
