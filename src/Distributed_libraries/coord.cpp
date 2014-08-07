/*
 *  coord.cpp
 *  Simulator
 *
 *  Created by Stefan Jorgensen on 10/8/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#include "coord.h"
#include <math.h>

float coord_mult(coord c1, coord c2)
{
	return (c1[X]*c2[X]+c1[Y]*c2[Y]);
}

float coord_angle(coord c1)
{
    return atan2(c1[Y], c1[X]);
}

float coord_mag(coord c1)
{
    return coord_mult(c1, c1);
}

void coord_diff(coord c1, coord c2, coord res)
{
	res[X] = c1[X] - c2[X];
	res[Y] = c1[Y] - c2[Y];
}

void coord_add(coord c1, coord c2, coord res)
{
	res[X] = c1[X] + c2[X];
	res[Y] = c1[Y] + c2[Y];
}

void coord_scale(float a, coord c1)
{
	c1[X] = a*c1[X];
	c1[Y] = a*c1[Y];
}

void coord_rotate(float angle, coord c1)
{
	//Get magnitude:
	//assumed to be unit vector
	
	//Get current angle:
	float theta = atan2f(c1[Y],c1[X]);
	theta += angle;
	
	c1[X] = cos(theta);
	c1[Y] = sin(theta);
}

void coord_normalize(coord c1, coord c2)
{
    c2[X] = c1[X];
    c2[Y] = c1[Y];
    coord_rotate(0, c2);
}

void coord_proj(float angle, coord c1, coord c2)
{
    c2[X] = c1[X]*cos(angle) + c1[Y]*sin(angle);
    c2[Y] = c1[Y]*cos(angle) - c1[X]*sin(angle);
    
}

void coord_set(coord dest, coord source)
{
    dest[X] = source[X];
    dest[Y] = source[Y];
};
