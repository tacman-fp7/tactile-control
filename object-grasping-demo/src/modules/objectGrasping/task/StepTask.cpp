#include "iCub/objectGrasping/task/StepTask.h"

#include "iCub/objectGrasping/ObjectGraspingEnums.h"

#include <sstream>

using iCub::objectGrasping::StepTask;
using iCub::objectGrasping::LogData;
using iCub::objectGrasping::ControllersUtil;
using iCub::objectGrasping::PortsUtil;
using iCub::objectGrasping::TaskCommonData;
using iCub::objectGrasping::StepTaskData;

StepTask::StepTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,StepTaskData *stepData,double constantPwm):Task(controllersUtil,portsUtil,commonData,stepData->lifespan,stepData->jointsList,stepData->fingersList) {

	this->stepData = stepData;
	this->constantPwm.resize(jointsList.size(),constantPwm);

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

void StepTask::calculatePwm(){

	for(size_t i = 0; i < constantPwm.size(); i++){
		pwmToUse[i] = constantPwm[i];
	}
}

void StepTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	logData.taskType = STEP;
	logData.taskOperationMode = 0;
	//TODO it should log all the array values
	logData.targetValue = constantPwm[0];
}

std::string StepTask::getConstantPwmDescription(){

	std::stringstream description("");

	for(size_t i = 0; i < constantPwm.size(); i++){
		description << constantPwm[i] << " ";
	}
	
	return description.str();
}