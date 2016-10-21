#include "iCub/plantIdentification/thread/EventsThread.h"

#include "iCub/plantIdentification/util/ICubUtil.h"

#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/Time.h>

#include <ctime>

using std::string;

using yarp::os::Value;

using iCub::plantIdentification::EventsThread;
using iCub::plantIdentification::EventToTrigger;
using iCub::plantIdentification::ICubUtil;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;


EventsThread::EventsThread(yarp::os::ResourceFinder &rf,int period,ControllersUtil *controllersUtil,PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData)
	: RateThread(period){

	this->period = period;
	this->controllersUtil = controllersUtil;
	this->portsUtil = portsUtil;
	this->commonData = commonData;

	xyzCoordEnabled = rf.check("xyzCoordEnabled",Value(0)).asInt() != 0;

	counter = 0;

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

	forceSensorBiasCounter = -1;
	forceSensorBiasPartial.resize(6,0.0);

	return true;
}

void EventsThread::run(){


	// update module data
	ICubUtil::updateExternalData(controllersUtil,portsUtil,commonData,xyzCoordEnabled,forceSensorBiasCounter,forceSensorBiasPartial);

	// check events
	checkEvents();

	executeWaveAction();

    logData();

	if (counter%10 == 0){
	// log various data
        printData();
	}


	counter++;
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

void EventsThread::logData(){


    bool gripStrengthDataPortLoggingEnabled = commonData->tpInt(59) != 0;

    if (gripStrengthDataPortLoggingEnabled){
        portsUtil->sendGripStrengthData(commonData->tpStr(16),commonData->tpStr(17),commonData->tpDbl(7),commonData);
    }

}

void EventsThread::printData(){



    bool xyzCoordLoggingEnabled = commonData->tpInt(57) != 0;
    bool tactileDataLoggingEnabled = commonData->tpInt(58) != 0;

    if (xyzCoordLoggingEnabled){

        std::cout << "XYZ | Th - Ind - Mid : " << commonData->thumbXYZ[0] << "  " << commonData->thumbXYZ[1] << "  " << commonData->thumbXYZ[2] << "   -  " << commonData->indexXYZ[0] << "  " << commonData->indexXYZ[1] << "  " << commonData->indexXYZ[2] << "   -  " << commonData->middleXYZ[0] << "  " << commonData->middleXYZ[1] << "  " << commonData->middleXYZ[2] << "\n";

    }
    if (tactileDataLoggingEnabled){

        std::cout << "Force (In Mi Ri Pi Th): ";

        for(size_t i = 0; i < commonData->overallFingerPressure.size(); i++){
            std::cout << "\t" << commonData->overallFingerPressure[i];
        }

        std::cout << "\n";
    }


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

void EventsThread::setWaveAction(double actionDuration,double joint,double period,double amplitude,iCub::plantIdentification::Wave waveType){

    waveAction.init(controllersUtil,actionDuration,joint,period,amplitude,waveType,this->period);
}

void EventsThread::executeWaveAction(){

	if (waveAction.isEnabled()){
		waveAction.executeNextStep();
	}
}

EventsThread::~EventsThread() {}

void EventsThread::threadRelease() {}

