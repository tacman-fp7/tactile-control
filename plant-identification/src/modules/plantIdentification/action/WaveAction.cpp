#include "iCub/plantIdentification/action/WaveAction.h"

using iCub::plantIdentification::WaveAction;

using yarp::os::Bottle;

#include <cmath>

WaveAction::WaveAction(){

	enabled = false;
}
	
void WaveAction::init(iCub::plantIdentification::ControllersUtil *controllersUtil,double actionDuration,double joint,double period,double amplitude,iCub::plantIdentification::Wave waveType,int threadPeriod) {

	this->controllersUtil = controllersUtil;
	this->actionDuration = actionDuration;
	this->joint = joint;
    this->wavePeriod = period;
    this->waveAmplitude = amplitude;
	this->waveType = waveType;
	this->threadPeriod = threadPeriod;


	counter = 0;
	enabled = true;
	waveMeanInitialized = false;

	controllersUtil->setControlMode(joint,VOCAB_CM_POSITION_DIRECT,false);

	dbgTag = "WaveAction: ";
}

bool WaveAction::isEnabled(){

	return enabled;
}

void WaveAction::executeNextStep(){

	// if this is the first step, initialize the wave mean with the current joint angle
	if (!waveMeanInitialized){
		controllersUtil->getEncoderAngle(joint,&waveMean);
		waveMeanInitialized = true;
	}

	// calculate the joint target position
	double jointTargetPosition;

    if (waveType == SINE){
		int numCallsPerPeriod = (int)(wavePeriod/(threadPeriod/1000.0));
        int callsNumberMod = counter%numCallsPerPeriod;
        double ratio = (1.0*callsNumberMod)/numCallsPerPeriod;
        jointTargetPosition = waveMean + waveAmplitude * sin(ratio*2*3.14159265);
    } else 	if (waveType == SQUARE){

		int div = (int)((counter*threadPeriod/1000.0)/(wavePeriod/2));
		if (div%2 == 1){
			jointTargetPosition = waveMean + waveAmplitude;
		} else {
			jointTargetPosition = waveMean - waveAmplitude;
		}
	}

	// move the joint to the target position
	controllersUtil->setJointAnglePositionDirect(joint,jointTargetPosition);

	// increment the counter to simulate time
	counter++;

	// check if action is out of time
    if (counter >= (int)(actionDuration/(threadPeriod/1000.0))){
		enabled = false;
        release();
	}

}

void WaveAction::release(){

    controllersUtil->setControlMode(joint,VOCAB_CM_POSITION,false);
	controllersUtil->setJointAngle(joint,waveMean);

}



