#include "iCub/plantIdentification/thread/EventsThread.h"

#include "iCub/plantIdentification/util/ICubUtil.h"

#include <yarp/os/Network.h>
#include <yarp/os/Value.h>

using std::string;

using yarp::os::Value;

using iCub::plantIdentification::EventsThread;
using iCub::plantIdentification::EventToTrigger;
using iCub::plantIdentification::ICubUtil;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;


EventsThread::EventsThread(int period,ControllersUtil *controllersUtil,PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData)
	: RateThread(period){

	this->period = period;
	this->controllersUtil = controllersUtil;
	this->portsUtil = portsUtil;
	this->commonData = commonData;

	dbgTag = "EventsThread: ";
}

bool EventsThread::threadInit(){

	// settings
	fpWindowSize = commonData->getNumOfThreadCallsFromTime(EVENTS_THREAD,commonData->tpDbl(41));
	fpFinalCheckThreshold = commonData->tpDbl(42);
	fpMinTimeBetweenActivations = commonData->getNumOfThreadCallsFromTime(EVENTS_THREAD,commonData->tpDbl(43));

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

void EventsThread::run(){

	// update module data
	ICubUtil::updateExternalData(controllersUtil,portsUtil,commonData);

	// check events
	checkEvents();
}


void EventsThread::checkEvents(){
	// EVENT: FINGER PUSHED
	double tempPressureDifference;
	int fpNextWindowIndex = (fpWindowIndex + 1)%fpWindowSize;

	for(size_t i = 0; i < fpPressureMemory.size(); i++){
			
		fpPressureMemory[i][fpWindowIndex] = commonData->overallFingerPressureMedian[i];
		tempPressureDifference = fpPressureMemory[i][fpWindowIndex] - fpPressureMemory[i][fpNextWindowIndex];
        //if (i ==3) std::cout << tempPressureDifference << " / " << fpFinalCheckThreshold << "\n";
		if (fpEventTriggered[i] == false && (fpTimeFromLastEventReset[i] > fpMinTimeBetweenActivations || fpTimeFromLastEventReset[i] == -1) && tempPressureDifference > fpFinalCheckThreshold){
			fpEventTriggered[i] = true;
			//std::cout << "\n\n\n<<< EVENT TRIGGERED: FINGER " << i << " PUSHED >>>\n\n\n";
		} 

		fpTimeFromLastEventReset[i]++;
	}

	fpWindowIndex = fpNextWindowIndex;

}


bool EventsThread::eventFPTriggered(int whichFinger){

    bool fpEnabled = commonData->tpInt(44) != 0;

	if (fpEventTriggered[whichFinger] == true){
		fpEventTriggered[whichFinger] = false;
		fpTimeFromLastEventReset[whichFinger] = 0;
		if (fpEnabled){ // if finger pushed event is not enabled, it is triggered anyway, but it cannot be used by any function
			return true;
		}
	}
	return false;

}

bool EventsThread::eventTriggered(EventToTrigger eventToTrigger,int index = 0){

	switch(eventToTrigger){

	case FINGERTIP_PUSHED:

		return eventFPTriggered(index);
		break;

	}

	return false;
}
 
EventsThread::~EventsThread() {}

void EventsThread::threadRelease() {}

