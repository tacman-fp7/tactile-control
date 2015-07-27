#include "iCub/plantIdentification/task/StepTask.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <sstream>

using iCub::plantIdentification::StepTask;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::StepTaskData;

StepTask::StepTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,StepTaskData *stepData,std::vector<double> &targetList):Task(controllersUtil,portsUtil,commonData,stepData->lifespan,stepData->jointsList,stepData->fingersList) {

	this->stepData = stepData;
	constantPwm.resize(jointsList.size());
	for(size_t i = 0; i < constantPwm.size(); i++){
		constantPwm[i] = (i >= targetList.size() ? targetList[targetList.size()-1] : targetList[i]);
	}

	taskName = STEP;
	dbgTag = "StepTask: ";
}

void StepTask::init(){
	using std::cout;

	controllersUtil->setTaskControlModes(jointsList,VOCAB_CM_OPENLOOP);

	cout << "\n\n" << dbgTag << "TASK STARTED - Target: ";
	for(size_t i = 0; i < constantPwm.size(); i++){
		cout << constantPwm[i] << " ";
	}
	cout << "\n\n";

}

void StepTask::calculateControlInput(){

	for(size_t i = 0; i < constantPwm.size(); i++){
		inputCommandValue[i] = constantPwm[i];
	}
}

void StepTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	logData.taskType = STEP;
	logData.taskOperationMode = 0;
    // TODO it should be jointsList
    for(size_t i = 0; i < fingersList.size(); i++){
        logData.targetValue[i] = constantPwm[i];
    }
}

std::string StepTask::getConstantPwmDescription(){

	std::stringstream description("");

	for(size_t i = 0; i < constantPwm.size(); i++){
		description << constantPwm[i] << " ";
	}
	
	return description.str();
}
