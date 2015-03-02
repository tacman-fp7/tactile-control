#include "iCub/plantIdentification/task/RampTask.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

using iCub::plantIdentification::RampTask;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::RampTaskData;

RampTask::RampTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,RampTaskData *rampData,double pressureTargetValue):Task(controllersUtil,portsUtil,commonData,rampData->lifespan) {

	this->rampData = rampData;
	this->pressureTargetValue = pressureTargetValue;

	internalState = DECREASING;

	callsNumberAfterStabilization = 0;
	maxCallsNumberAfterStabilization = rampData->lifespanAfterStabilization*1000/commonData->threadRate;

	taskName = RAMP;
	dbgTag = "RampTask: ";
}

void RampTask::init(){
	
	std::cout << dbgTag << "\n" <<
        "TASK STARTED - Target: " << pressureTargetValue << "\n" <<
        "\n";
}

void RampTask::calculatePwm(){

	double pwmToUse;

	switch (internalState){
	
	case DECREASING:

		pwmToUse = rampData->intercept + rampData->slope*callsNumber*commonData->threadRate;

		if (commonData->overallFingerPressureMedian < pressureTargetValue){
			internalState = STEADY;
		}
		break;

	case STEADY:

		pwmToUse = 0.0;
		break;
	}

	this->pwmToUse = pwmToUse;
}

void RampTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	logData.taskType = RAMP;
	logData.taskOperationMode = 0;
	logData.targetValue = pressureTargetValue;

}

void RampTask::saveProgress(){

	callsNumber++;
	if (internalState == 1) callsNumberAfterStabilization++;
}

bool RampTask::taskIsOver(){

	return callsNumber >= maxCallsNumber || callsNumberAfterStabilization >= maxCallsNumberAfterStabilization;
}

