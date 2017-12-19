#ifndef STP_DRV_6575_H
#define STP_DRV_6575_H
/* ------------------------------------------------------------------------- *\
|	STP-DRV-6575 Automation Direct Stepper driver library				            	  |
|	larry12193@gmail.com                                                        |
|  -------------------------------------------------------------------------  |
|	Revision:															                                   	  |
|  =========================================================================  |
|	2017-03-11: Initial commit                                                  |
\* ------------------------------------------------------------------------- */

#include <stdint.h>

#define STEP_PREV_200	  200
#define STEP_PREV_400	  400
#define STEP_PREV_2000	2000
#define STEP_PREV_5000  5000
#define STEP_PREV_12800 12800
#define STEP_PREV_20000 20000

#define ENABLE			0
#define DISABLE			1

#define CW				0
#define CCW				1

// Library description
class STP_DRV_6575 
{
public:
	// Constructors
	STP_DRV_6575(uint16_t nStepsPerRev, uint8_t mDirPin, uint8_t mEnbPin, uint8_t mStepPin );

	// Steps motor given number of steps
	void step(uint16_t nSteps);
	// Changes motor direction
	void setDir(uint8_t mDir);
	// Disable motor
	void disable(void);
	// Enable Motor
	void enable(void);

private:
	uint8_t  mDir;			// Motor direction
	uint16_t nStepsPerRev;	// Number of steps per revolution (DIP switch set)
	uint8_t  mDirPin;		// Motor direction pin number
	uint8_t  mEnbPin;		// Motor enable pin number
	uint8_t  mStepPin;		// Motor step pin number

	int32_t  stepCount;		// Keeps track of step count
	uint8_t  state;			// Enable/disable state
};

#endif // STP_DRV_6575_H
