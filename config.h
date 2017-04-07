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

#define X_AXIS_SPR	2000
#define Y_AXIS_SPR  2000	

// Time belt drive pully circumference (mm)
#define PULLY_CIRC 	  	89.5

// Define mm per step for x/y axes
#define MM_PER_STEP  	PULLY_CIRC/X_AXIS_SPR

// Min delay time for stepper speeds, microseconds (~500mm/s @ 2000 stp/rev)
#define MIN_STEP_DELAY	90

// Max delay time for stepper speeds, microseconds (~0.5mm/s @ 2000 stp/rev)
#define MAX_STEP_DELAY	90000

// Define min/max feedrates
#define MAX_FEEDRATE	1000000.0/MIN_STEP_DELAY
#define MIN_FEEDRATE	1000000.0/MAX_STEP_DELAY

#define BAUD 			57600
#define MAX_BUF			64

#endif
