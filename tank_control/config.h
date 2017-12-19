#ifndef CONFIG_H
#define CONFIG_H
/* ------------------------------------------------------------------------- *\
|	Configuration File - Tank XY Control                            					  |
|	larry12193@gmail.com                                                        |
|  -------------------------------------------------------------------------  |
|	Revision:                                                                   |
|  =========================================================================  |
|	2017-03-11: Initial commit 												                          |
\* ------------------------------------------------------------------------- */

#include "stp_drv_6575.h"

#define X_AXIS_SPR	STEP_PREV_5000
#define Y_AXIS_SPR  STEP_PREV_5000	

// Time belt drive pully circumference (mm)
#define PULLY_CIRC 	  	89.5

// Define mm per step for x/y axes
#define MM_PER_STEP  	PULLY_CIRC/X_AXIS_SPR

// Min delay time for stepper speeds, microseconds (~500mm/s @ 2000 stp/rev)
#define MIN_STEP_DELAY	90

// Max delay time for stepper speeds, microseconds (~0.5mm/s @ 2000 stp/rev)
#define MAX_STEP_DELAY	90000

// Define min/max steprates
#define MAX_STEPRATE	1000000.0/MIN_STEP_DELAY
#define MIN_STEPRATE	1000000.0/MAX_STEP_DELAY

// Deifine min/max feedrates (mm/s)
#define MAX_FEEDRATE MAX_STEPRATE*MM_PER_STEP
#define MIN_FEEDRATE MIN_STEPRATE*MM_PER_STEP

#define BAUD 			115200
#define MAX_BUF			64

#define MM_PER_SEGMENT 1

#endif
