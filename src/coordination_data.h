#ifndef _COORDINATION_DATA_H
#define _COORDINATION_DATA_H
//This includes a define list of all field types.
typedef struct coordData{
    int msgType;
    float field1;
    float field2;
    float field3;
}coorData;


//Defines for fields in flockAlgorithm
#define FLOCK_MSGTYPE 1
#define FLOCK_TARGETID field1
#define FLOCK_TARGETX field2
#define FLOCK_TARGETY field3

#endif //_COORDINATION_DATA_H
