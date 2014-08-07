/*
 *  coord.h
 *  Simulator
 *
 *  Created by Stefan Jorgensen on 10/8/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef _COORD_H__
#define _COORD_H__

typedef float coord[2]; 
#define X 0
#define Y 1

//vector to scalar:
float coord_mult(coord c1, coord c2);
float coord_angle(coord c1);
float coord_mag(coord c1);

//vector to vector:
void coord_diff(coord c1, coord c2, coord res);
void coord_add(coord c1, coord c2, coord res);

void coord_scale(float a, coord c1);
void coord_rotate(float angle, coord c1);
void coord_normalize(coord c1, coord c2);

void coord_proj(float angle, coord c1, coord c2);

void coord_set(coord dest, coord source);

#endif //_COORD_H__
