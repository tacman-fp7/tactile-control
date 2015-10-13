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

	controlMode = commonData->tpInt(32);
	stopCondition = commonData->tpInt(33);
	windowSize = commonData->tpInt(34);
	positionIndex = 0;
	initialCheckThreshold = commonData->tpDbl(35);
	finalCheckThreshold = commonData->tpDbl(36);

	fingerIsInContact.resize(jointsList.size(),false);

	fingerState.resize(jointsList.size(),0);
	fingerPositions.resize(jointsList.size());


	taskName = APPROACH;
	dbgTag = "Approach: ";
	
}

void ApproachTask::init(){
	using std::cout;

	if (controlMode == 0){ // velocity control mode
		controllersUtil->saveHandJointsMaxPwmLimits();
		controllersUtil->setTaskControlModes(jointsList,VOCAB_CM_VELOCITY);
		controllersUtil->setJointsMaxPwmLimit(jointsList,approachData->jointsPwmLimitsList);
	} else { // openloop control mode
		controllersUtil->setTaskControlModes(jointsList,VOCAB_CM_OPENLOOP);
	}

	// valid if stop condition is position
	double encoderAngle;
	for(size_t i = 0; i < jointsList.size(); i++){
		encoderAngle = commonData->armEncodersAngles[jointsList[i]];
		fingerPositions[i].resize(windowSize,encoderAngle);
	}

	cout << "\n\n" << dbgTag << "TASK STARTED" << "\n\n";
}

void ApproachTask::calculateControlInput(){
	using yarp::sig::Vector;

	if (stopCondition == 0){ // tactile
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
	} else { // position
		
    	if (callsNumber%commonData->screenLogStride == 0){
    		std::stringstream printLog("");
			printLog << " [state ";
			for(size_t i = 0; i < jointsList.size(); i++){
	    		printLog << fingerState[i] << " ";
			}
			printLog << "]";
			printLog << " [diff ";
			for(size_t i = 0; i < jointsList.size(); i++){
	    		printLog << fingerPositions[i][positionIndex] - fingerPositions[i][(positionIndex + 1)%windowSize] << " ";
			}
			printLog << "]";
			optionalLogString.append(printLog.str());
	    }


		double encoderAngle;
		double angleDifference;
		int nextPositionIndex = (positionIndex + 1)%windowSize;

		for(size_t i = 0; i < jointsList.size(); i++){
			
			encoderAngle = commonData->armEncodersAngles[jointsList[i]];
			fingerPositions[i][positionIndex] = encoderAngle;
			angleDifference = fingerPositions[i][positionIndex] - fingerPositions[i][nextPositionIndex];

			if (fingerState[i] == 0){
				if (angleDifference > initialCheckThreshold){
					fingerState[i] = 1;
				}
				inputCommandValue[i] = approachData->pwmList[i];
			} else if (fingerState[i] == 1){
				if (angleDifference < finalCheckThreshold){
					fingerState[i] = 2;
					inputCommandValue[i] = 0;
				} else {
					inputCommandValue[i] = approachData->pwmList[i];
				}
			} else {
				inputCommandValue[i] = 0;
			}

		}

		positionIndex = nextPositionIndex;

	}
}

void ApproachTask::sendCommands(){

	if (controlMode == 0){ // velocity control mode
		for(size_t i = 0; i < inputCommandValue.size(); i++){
			controllersUtil->sendVelocity(jointsList[i],inputCommandValue[i]);
		}
	} else { // openloop control mode
		for(size_t i = 0; i < inputCommandValue.size(); i++){
			controllersUtil->sendPwm(jointsList[i],commonData->pwmSign*inputCommandValue[i]);
		}
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

	if (controlMode == 0){ // velocity control mode
		controllersUtil->restoreHandJointsMaxPwmLimits();
	}
}

bool ApproachTask::taskIsOver(){

	return callsNumber >= maxCallsNumber || eachFingerIsInContact();
}


bool ApproachTask::eachFingerIsInContact(){

	bool eachFingerIsInContact = true;

	if (stopCondition == 0){ // tactile
		for(size_t i = 0; eachFingerIsInContact && i < jointsList.size(); i++){
			eachFingerIsInContact = eachFingerIsInContact && fingerIsInContact[fingersList[i]];
		}
	} else { // position
		for(size_t i = 0; eachFingerIsInContact && i < jointsList.size(); i++){
			eachFingerIsInContact = eachFingerIsInContact && fingerState[i] == 2;
		}
	}

	return eachFingerIsInContact;
}
