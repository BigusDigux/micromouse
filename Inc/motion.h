#ifndef MOTION_H
#define MOTION_H

#include "config.h"
#include "init.h"

/* Motion helpers */
void resetMotion(void);
int  getBaseSpeed(void);

/* Movements */
void driveForward(int cells);
void turn_pivot(float target_deg);
void turn_curve(float target_deg);
void turn_diagonal(void);
void turn90(bool left);
void turn180(void);

#endif // MOTION_H
