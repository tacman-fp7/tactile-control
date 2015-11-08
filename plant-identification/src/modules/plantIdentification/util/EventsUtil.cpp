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
	fpWindowSize = commonData->getNumOfThreadCallsFromTime(commonData->tpInt(41));
	fpFinalCheckThreshold = commonData->tpDbl(42);
	fpMinTimeBetweenActivations = commonData->getNumOfThreadCallsFromTime(commonData->tpInt(43));
	fpEnabled = commonData->tpInt(44) != 0;

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


	// EVENT FINGER PUSHED
	double tempPressureDifference;
	int fpNextWindowIndex = (fpWindowIndex + 1)%fpWindowSize;

	for(size_t i = 0; i < fpPressureMemory.size(); i++){
			
		fpPressureMemory[i][fpWindowIndex] = commonData->overallFingerPressureMedian[i];
		tempPressureDifference = fpPressureMemory[i][fpWindowIndex] - fpPressureMemory[i][fpNextWindowIndex];

		if (fpEventTriggered[i] == false && (fpTimeFromLastEventReset[i] > fpMinTimeBetweenActivations || fpTimeFromLastEventReset[i] == -1) && tempPressureDifference > fpFinalCheckThreshold){
			fpEventTriggered[i] = true;
			std::cout << "\n\n\n<<< EVENT TRIGGERED: FINGER " << i << " PUSHED >>>\n\n\n";
		} 

		fpTimeFromLastEventReset[i]++;
	}

	fpWindowIndex = fpNextWindowIndex;

}


bool EventsUtil::eventFPTriggered(int whichFinger){


	if (fpEventTriggered[whichFinger] == true){
		fpEventTriggered[whichFinger] = false;
		fpTimeFromLastEventReset[whichFinger] = 0;
		if (fpEnabled){ // if finger pushed event is not enabled, it is triggered anyway, but it cannot be used by any function
			return true;
		}
	}
	return false;

}