#ifndef _SIMPLE_ALGORITHM__
#define _SIMPLE_ALGORITHM__

#include <math.h>
#include "dataManager.h"
#include <iostream>

class simpleAlgorithm{
  public: 
    simpleAlgorithm(dataManager* dm){
	data = dm;
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
    float dist(Pose* n1, Pose* n2);

    dataManager* data;
    int state;
    Pose box[4];
};

#endif //_SIMPLE_ALGORITHM__
