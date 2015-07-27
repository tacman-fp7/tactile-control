#include "iCub/plantIdentification/task/ApproachTask.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <sstream>
#include <string>

using iCub::plantIdentification::ApproachTask;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::ApproachTaskData;

using iCub::ctrl::parallelPID;
using yarp::os::Bottle;
using yarp::os::Value;

using std::string;

ApproachTask::ApproachTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,ApproachTaskData *approachData):Task(controllersUtil,portsUtil,commonData,approachData->lifespan,approachData->jointsList,approachData->fingersList) {
	using yarp::sig::Vector;
	using yarp::sig::Matrix;
    this->approachData = approachData;

	fingerIsInContact.resize(commonData->objDetectPressureThresholds.size(),false);

	taskName = APPROACH;
	dbgTag = "Approach: ";
	
}

void ApproachTask::init(){
	using std::cout;

	controllersUtil->saveHandJointsMaxPwmLimits();
	controllersUtil->setTaskControlModes(jointsList,VOCAB_CM_VELOCITY);
	controllersUtil->setJointsMaxPwmLimit(jointsList,approachData->jointsPwmLimitsList);
	
	cout << "\n\n" << dbgTag << "TASK STARTED" << "\n\n";
}

void ApproachTask::calculateControlInput(){
	using yarp::sig::Vector;


	for(size_t i = 0; i < jointsList.size(); i++){

		if (!fingerIsInContact[fingersList[i]] && commonData->overallFingerPressureMedian[fingersList[i]] > commonData->objDetectPressureThresholds[fingersList[i]]){
            fingerIsInContact[fingersList[i]] = true;
		}
		
		if (fingerIsInContact[fingersList[i]]){
			inputCommandValue[i] = 0.0;
		} else {
			inputCommandValue[i] = approachData->velocitiesList[i];
		}

	}
}

void ApproachTask::sendCommands(){

	for(size_t i = 0; i < inputCommandValue.size(); i++){
		controllersUtil->sendVelocity(jointsList[i],inputCommandValue[i]);
	}
}

void ApproachTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	logData.taskType = APPROACH;

	logData.taskOperationMode = 0;

    for(size_t i = 0; i < fingersList.size(); i++){
        logData.targetValue[i] = commonData->objDetectPressureThresholds[i];
    }
}

void ApproachTask::release(){

    controllersUtil->restoreHandJointsMaxPwmLimits();
}

bool ApproachTask::taskIsOver(){

	return callsNumber >= maxCallsNumber || eachFingerIsInContact();
}


bool ApproachTask::eachFingerIsInContact(){

	bool eachFingerIsInContact = true;

	for(size_t i = 0; eachFingerIsInContact && i < jointsList.size(); i++){
		eachFingerIsInContact = eachFingerIsInContact && fingerIsInContact[fingersList[i]];
	}

	return eachFingerIsInContact;
}
