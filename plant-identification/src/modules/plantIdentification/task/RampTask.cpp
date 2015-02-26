#include "iCub/plantIdentification/task/RampTask.h"

#include "iCub/plantIdentification/plantIdentificationEnums.h"

using iCub::plantIdentification::RampTask;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::RampTaskData;

RampTask::RampTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,RampTaskData *rampData,double pressureTargetValue):Task(controllersUtil,portsUtil,commonData,rampData->lifespan) {

	this->rampData = rampData;
	this->pressureTargetValue = pressureTargetValue;

	callsNumberAfterStabilization = 0;
	maxCallsNumberAfterStabilization = rampData->lifespanAfterStabilization*1000/commonData->threadRate;

	taskName = RAMP;
	dbgTag = "RampTask: ";
}

void RampTask::calculatePwm(){

	double pwmToUse;

	//TODO use switch
	if (internalState == 0){

		pwmToUse = rampData->intercept + rampData->slope*callsNumber*commonData->threadRate;

		if (commonData->overallFingerPressure < pressureTargetValue){
			internalState = 1;
		}

	} else {

		pwmToUse = 0.0;

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

bool RampTask::isOver(){

	return callsNumber >= maxCallsNumber || callsNumberAfterStabilization >= maxCallsNumberAfterStabilization;
}

/* *********************************************************************************************************************** */

