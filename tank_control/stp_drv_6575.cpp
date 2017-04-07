/* ------------------------------------------------------------------------- *\
|	STP-DRV-6575 Automation Direct Stepper driver library					              |
|	larry12193@gmail.com                                                        |
|  -------------------------------------------------------------------------  |
|	Revision:																                                    |
|  =========================================================================  |
|	2017-03-11: Initial commit                                                  |
\* ------------------------------------------------------------------------- */

#include <stdint.h>
#include "Arduino.h"
#include "config.h"
#include "stp_drv_6575.h"

// Constructor
STP_DRV_6575::STP_DRV_6575(uint16_t nStepsPerRev, uint8_t mDirPin, uint8_t mEnbPin, uint8_t mStepPin )
{
	// Save motor configuration
	this->nStepsPerRev = nStepsPerRev;
	this->mDirPin      = mDirPin;
	this->mEnbPin      = mEnbPin;
	this->mStepPin     = mStepPin;

	// Initialize constants
	this->stepCount    = 0;
	this->state		   = DISABLE;

	// Initilize pin modes
	pinMode(this->mDirPin,  OUTPUT);
	pinMode(this->mEnbPin,  OUTPUT);
	pinMode(this->mStepPin, OUTPUT);

	// Initialize pin states
	digitalWrite(this->mDirPin, CW);
	digitalWrite(this->mEnbPin, this->state);
	digitalWrite(this->mStepPin, LOW);
}

// Steps motor given number of steps
void STP_DRV_6575::step(uint16_t nSteps)
{
	for( uint16_t i = 0; i < nSteps; i++ ) 
	{
		digitalWrite(this->mStepPin, HIGH);
		digitalWrite(this->mStepPin, LOW);	
	}
}

// Changes motor direction
void STP_DRV_6575::setDir(uint8_t mDir)
{
	digitalWrite(this->mDirPin, mDir);
}

// Disable motor
void STP_DRV_6575::disable(void)
{
	digitalWrite(this->mEnbPin, DISABLE);
	this->state = DISABLE;
}

// Disable motor
void STP_DRV_6575::enable(void)
{
	digitalWrite(this->mEnbPin, ENABLE);
	this->state = ENABLE;
}
