#include "iCub/objectGrasping/task/RampTask.h"

#include "iCub/objectGrasping/ObjectGraspingEnums.h"

#include <sstream>

using iCub::objectGrasping::RampTask;
using iCub::objectGrasping::LogData;
using iCub::objectGrasping::ControllersUtil;
using iCub::objectGrasping::PortsUtil;
using iCub::objectGrasping::TaskCommonData;
using iCub::objectGrasping::RampTaskData;

RampTask::RampTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,RampTaskData *rampData,std::vector<double> &pressureTargetValue):Task(controllersUtil,portsUtil,commonData,rampData->lifespan,rampData->jointsList,rampData->fingersList) {

	this->rampData = rampData;
	this->pressureTargetValue.resize(fingersList.size());
	for(size_t i = 0; i < this->pressureTargetValue.size(); i++){
		this->pressureTargetValue[i] = pressureTargetValue[i < pressureTargetValue.size() ? i : pressureTargetValue.size() - 1];
	}

	internalState.resize(fingersList.size(),DECREASING);

	callsNumberAfterStabilization = 0;
	maxCallsNumberAfterStabilization = rampData->lifespanAfterStabilization*1000/commonData->threadRate;

	taskName = RAMP;
	dbgTag = "RampTask: ";
}

void RampTask::init(){
	using std::cout;

	controllersUtil->setTaskControlModes(jointsList,VOCAB_CM_OPENLOOP);

	cout << "\n\n" << dbgTag << "TASK STARTED - Target: ";
	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		cout << pressureTargetValue[i] << " ";
	}
	cout << "\n\n";
}

void RampTask::calculatePwm(){

	double pwmToUse;
	
	for(size_t i = 0; i < jointsList.size(); i++){

		switch (internalState[i]){
	
		case DECREASING:

			pwmToUse = rampData->intercept + rampData->slope*callsNumber*commonData->threadRate;

			if (commonData->overallFingerPressureMedian[i] < pressureTargetValue[i]){
				internalState[i] = STEADY;
			}
			break;

		case STEADY:

			pwmToUse = 0.0;
			break;
		}

		this->pwmToUse[i] = pwmToUse;
	}
}

void RampTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	logData.taskType = RAMP;
	logData.taskOperationMode = 0;
	//TODO it should log all the array values
	logData.targetValue = pressureTargetValue[0];

}

void RampTask::saveProgress(){

	callsNumber++;

	if (areAllJointsSteady()) callsNumberAfterStabilization++;
}

bool RampTask::taskIsOver(){

	return callsNumber >= maxCallsNumber || callsNumberAfterStabilization >= maxCallsNumberAfterStabilization;
}

bool RampTask::areAllJointsSteady(){

	bool allSteady = true;
	
	for(size_t i = 0; i < fingersList.size() && allSteady; i++){
		allSteady = allSteady && (internalState[i] == STEADY);
	}

	return allSteady;
}


std::string RampTask::getPressureTargetValueDescription(){

	std::stringstream description("");

	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		description << pressureTargetValue[i] << " ";
	}
	
	return description.str();
}