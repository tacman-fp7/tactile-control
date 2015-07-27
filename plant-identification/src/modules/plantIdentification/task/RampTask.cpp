#include "iCub/plantIdentification/task/RampTask.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <sstream>

using iCub::plantIdentification::RampTask;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::RampTaskData;

RampTask::RampTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,RampTaskData *rampData,std::vector<double> &targetList):Task(controllersUtil,portsUtil,commonData,rampData->lifespan,rampData->jointsList,rampData->fingersList) {

	this->rampData = rampData;
	pressureTargetValue.resize(fingersList.size());
	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		pressureTargetValue[i] = (i >= targetList.size() ? targetList[targetList.size()-1] : targetList[i]);
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

void RampTask::calculateControlInput(){

	double inputCommandValue;
	
	for(size_t i = 0; i < jointsList.size(); i++){

		switch (internalState[i]){
	
		case DECREASING:

			inputCommandValue = rampData->intercept + rampData->slope*callsNumber*commonData->threadRate;

			if (commonData->overallFingerPressureMedian[i] < pressureTargetValue[i]){
				internalState[i] = STEADY;
			}
			break;

		case STEADY:

			inputCommandValue = 0.0;
			break;
		}

		this->inputCommandValue[i] = inputCommandValue;
	}
}

void RampTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	logData.taskType = RAMP;
	logData.taskOperationMode = 0;
    for(size_t i = 0; i < fingersList.size(); i++){
        logData.targetValue[i] = pressureTargetValue[0];
    }

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
