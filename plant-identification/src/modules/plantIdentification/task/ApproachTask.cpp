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

	thresholdScaleFactor = commonData->tpDbl(34);
	double secondsForAvarage = commonData->tpDbl(35);

	windowSize = commonData->tpInt(36);
	finalCheckThreshold = commonData->tpDbl(37);
	double secondsForMovementTimeout = commonData->tpDbl(38);

	stopFingers = commonData->tpInt(39) != 0;

	callsNumberForAvarage = secondsToCallsNumber(secondsForAvarage);
	callsNumberForMovementTimeout = secondsToCallsNumber(secondsForMovementTimeout);;

	positionIndex = 0;

	thresholdExceeded.resize(jointsList.size(),false);
	tactileAvarage.resize(jointsList.size(),0);
	tactileMaximum.resize(jointsList.size(),0);
	
	fingerState.resize(jointsList.size(),0);
	fingerPositions.resize(jointsList.size());

	fingerIsInContact.resize(jointsList.size(),false);

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

	cout << "\n\n" << dbgTag << "TASK STARTED" << "\n\n";
}

void ApproachTask::calculateControlInput(){
	using yarp::sig::Vector;

	bool manageFingers = false; // true if fingers can be moved
	int medianWindowSize = commonData->previousOverallFingerPressures[0].size();
	
	// if (callsNumber < medianWindowSize) nothing happens, because the tactile median cannot be evaluated
	if (callsNumber >= medianWindowSize){

		// evaluate the avarage of the tactile median while not in contact
		if (callsNumber < medianWindowSize + callsNumberForAvarage){

			for(size_t i = 0; i < jointsList.size(); i++){
				tactileAvarage[i] += commonData->overallFingerPressureMedian[fingersList[i]];
				tactileMaximum[i] = std::max(tactileMaximum[i],commonData->overallFingerPressureMedian[fingersList[i]]);
			}
		} else {
			// evaluate contact threshold on each fingertip
			if (callsNumber == medianWindowSize + callsNumberForAvarage){

				for(size_t i = 0; i < jointsList.size(); i++){
					if (callsNumberForAvarage != 0) tactileAvarage[i] /= callsNumberForAvarage;
					tactileThreshold[i] = tactileAvarage[i] + (tactileMaximum[i]-tactileAvarage[i])*thresholdScaleFactor;
				}

				// valid if stop condition is position
				double encoderAngle;
				for(size_t i = 0; i < jointsList.size(); i++){
					encoderAngle = commonData->armEncodersAngles[jointsList[i]];
					fingerPositions[i].resize(windowSize,encoderAngle);
				}
			}

			manageFingers = true;

			// evaluate stop condition by tactile feedback
			for(size_t i = 0; i < jointsList.size(); i++){
				if (!thresholdExceeded[i] && commonData->overallFingerPressureMedian[fingersList[i]] > tactileThreshold[i]){
					thresholdExceeded[i] = true;
				}
			}

			// evaluate stop condition by position
			// debug log
			if (callsNumber%commonData->screenLogStride == 0){
    			std::stringstream printLog("");
				printLog << " [state ";
				for(size_t i = 0; i < jointsList.size(); i++){
	    			printLog << (thresholdExceeded[i] == true ? 1 : 0) << "/" << fingerState[i] << " ";
				}
				printLog << "]";
				printLog << " [tax ";
				for(size_t i = 0; i < jointsList.size(); i++){
	    			printLog << tactileAvarage[i] << "/" << tactileMaximum[i] << "/" << tactileThreshold[i] << " ";
				}
				printLog << "]";
				printLog << " [pDiff ";
				for(size_t i = 0; i < jointsList.size(); i++){
	    			printLog << fingerPositions[i][positionIndex] - fingerPositions[i][(positionIndex + 1)%windowSize] << " ";
				}
				printLog << "]";
				printLog << "[t/o " << (callsNumber > callsNumberForMovementTimeout  ? 1 : 0) << "]";
				optionalLogString.append(printLog.str());
			}


			double tempAngleDifference;
			int nextPositionIndex = (positionIndex + 1)%windowSize;

			for(size_t i = 0; i < jointsList.size(); i++){
			
				fingerPositions[i][positionIndex] = commonData->armEncodersAngles[jointsList[i]];
				tempAngleDifference = fingerPositions[i][positionIndex] - fingerPositions[i][nextPositionIndex];

				if (fingerState[i] == 0 && callsNumber > callsNumberForMovementTimeout && tempAngleDifference > finalCheckThreshold){
					fingerState[i] = 1;
				} 

			}

			positionIndex = nextPositionIndex;

		}
	}

	// set fingers in contact
	for(size_t i = 0; i < jointsList.size(); i++){	
		if (!fingerIsInContact[i]){
			if (stopCondition == 1){ // tactile
				fingerIsInContact[i] = thresholdExceeded[i];
			} else if (stopCondition == 2){ // position
				fingerIsInContact[i] = (fingerState[i] == 1);
			} else if (stopCondition == 3){ // both tactile and position
				fingerIsInContact[i] = thresholdExceeded[i] || (fingerState[i] == 1);
			}
		}
	}

	// move fingers
	for(size_t i = 0; i < jointsList.size(); i++){	
		if (!manageFingers || (fingerIsInContact[i] && stopFingers)){
			stopFinger(i);	
		} else {
			moveFinger(i);
		}
	}



	//if (stopCondition == 0){ // tactile
	//	for(size_t i = 0; i < jointsList.size(); i++){

	//		if (!thresholdExceeded[fingersList[i]] && commonData->overallFingerPressureMedian[fingersList[i]] > commonData->objDetectPressureThresholds[fingersList[i]]){
	//			thresholdExceeded[fingersList[i]] = true;
	//		}
	//	
	//		if (thresholdExceeded[fingersList[i]]){
	//			inputCommandValue[i] = 0.0;
	//		} else {
	//			inputCommandValue[i] = approachData->velocitiesList[i];
	//		}
	//	}
	//} else { // position
	//	
 //   	if (callsNumber%commonData->screenLogStride == 0){
 //   		std::stringstream printLog("");
	//		printLog << " [state ";
	//		for(size_t i = 0; i < jointsList.size(); i++){
	//    		printLog << fingerState[i] << " ";
	//		}
	//		printLog << "]";
	//		printLog << " [diff ";
	//		for(size_t i = 0; i < jointsList.size(); i++){
	//    		printLog << fingerPositions[i][positionIndex] - fingerPositions[i][(positionIndex + 1)%windowSize] << " ";
	//		}
	//		printLog << "]";
	//		optionalLogString.append(printLog.str());
	//    }


	//	double encoderAngle;
	//	double angleDifference;
	//	int nextPositionIndex = (positionIndex + 1)%windowSize;

	//	for(size_t i = 0; i < jointsList.size(); i++){
	//		
	//		encoderAngle = commonData->armEncodersAngles[jointsList[i]];
	//		fingerPositions[i][positionIndex] = encoderAngle;
	//		angleDifference = fingerPositions[i][positionIndex] - fingerPositions[i][nextPositionIndex];

	//		if (fingerState[i] == 0){
	//			if (angleDifference > initialCheckThreshold){
	//				fingerState[i] = 1;
	//			}
	//			inputCommandValue[i] = approachData->pwmList[i];
	//		} else if (fingerState[i] == 1){
	//			if (angleDifference < finalCheckThreshold){
	//				fingerState[i] = 2;
	//				inputCommandValue[i] = 0;
	//			} else {
	//				inputCommandValue[i] = approachData->pwmList[i];
	//			}
	//		} else {
	//			inputCommandValue[i] = 0;
	//		}

	//	}

	//	positionIndex = nextPositionIndex;

	//}
}

void ApproachTask::moveFinger(int finger){
	
	if (controlMode == 0){ // velocity control mode
		inputCommandValue[finger] = approachData->velocitiesList[finger];
	} else { // openloop control mode
		inputCommandValue[finger] = approachData->pwmList[finger];
	}
}
void ApproachTask::stopFinger(int finger){
	
	inputCommandValue[finger] = 0;
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

	for(size_t i = 0; eachFingerIsInContact && i < jointsList.size(); i++){
		eachFingerIsInContact = eachFingerIsInContact && fingerIsInContact[i];
	}

	return eachFingerIsInContact;
}
