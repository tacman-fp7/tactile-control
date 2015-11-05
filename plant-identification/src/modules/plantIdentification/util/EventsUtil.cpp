#include "iCub/plantIdentification/util/EventsUtil.h"

#include <yarp/os/Network.h>
#include <yarp/os/Value.h>

using std::string;

using yarp::os::Value;

using iCub::plantIdentification::EventsUtil;

EventsUtil::EventsUtil(iCub::plantIdentification::TaskCommonData *commonData){

	this->commonData = commonData;

	dbgTag = "EventsUtil: ";
}

bool EventsUtil::init(){

	// settings
	fpWindowSize = 10;
	fpFinalCheckThreshold = 40;
	fpMinTimeBetweenActivations = 100;

	int nFingers = commonData->overallFingerPressureMedian.size();
	fpWindowIndex = 0;
	fpTimeFromLastEventReset.resize(nFingers,-1);
	fpEventTriggered.resize(nFingers,false);
	fpPressureMemory.resize(nFingers);
	for(size_t i = 0; i < nFingers; i++){
		fpPressureMemory[i].resize(fpWindowSize,0);
	}

	return true;
}

void EventsUtil::checkEvents(){

	double tempPressureDifference;
	int fpNextWindowIndex = (fpWindowIndex + 1)%fpWindowSize;

	for(size_t i = 0; i < fpPressureMemory.size(); i++){
			
		fpPressureMemory[i][fpWindowIndex] = commonData->overallFingerPressureMedian[i];
		tempPressureDifference = fpPressureMemory[i][fpWindowIndex] - fpPressureMemory[i][fpNextWindowIndex];

		if (fpEventTriggered[i] == false && fpTimeFromLastEventReset[i] > fpMinTimeBetweenActivations && tempPressureDifference > fpFinalCheckThreshold){
			fpEventTriggered[i] = true;
		} 

		fpTimeFromLastEventReset[i]++;
	}

	fpWindowIndex = fpNextWindowIndex;

}


bool EventsUtil::eventFPTriggered(int whichFinger){

	if (fpEventTriggered[whichFinger] == true){
		fpEventTriggered[whichFinger] = false;
		fpTimeFromLastEventReset[whichFinger] = 0;
		return true;
	}

	return false;

}