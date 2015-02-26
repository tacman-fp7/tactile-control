#include "iCub/plantIdentification/task/ControlTask.h"

#include "iCub/plantIdentification/plantIdentificationEnums.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

using iCub::plantIdentification::ControlTask;
using iCub::plantIdentification::LogData;
using iCub::ctrl::parallelPID;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::ControlTaskData;

ControlTask::ControlTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,ControlTaskData *controlData,double pressureTargetValue):Task(controllersUtil,portsUtil,commonData,controlData->lifespan) {
	using yarp::sig::Vector;
	using yarp::sig::Matrix;

	this->pressureTargetValue = pressureTargetValue;

	//TODO check parrallel pid usage
	Vector kpVector(1,controlData->pidKpf);
	Vector kiVector(1,controlData->pidKif);
	Vector kdVector(1,controlData->pidKdf);
	Vector wParam(1,1.0);
	Vector nParam(1,10);
	Vector ttParam(1,1.0);
	Matrix satLim(1,1);

	pid = new parallelPID(commonData->threadRate,kpVector,kiVector,kdVector,wParam,wParam,wParam,nParam,ttParam,satLim);

	taskName = CONTROL;
	dbgTag = "StepTask: ";
}

void ControlTask::calculatePwm(){
	using yarp::sig::Vector;

	Vector ref(1,pressureTargetValue);
	Vector fb(1,commonData->overallFingerPressure);

	Vector a = pid->compute(ref,fb);
	
	pwmToUse = a[0];
}

void ControlTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	logData.taskType = CONTROL;
	logData.taskOperationMode = controlData->controlMode;
	logData.targetValue = pressureTargetValue;
	
	logData.pidKpf = controlData->pidKpf;
	logData.pidKif = controlData->pidKif;
	logData.pidKdf = controlData->pidKdf;
	logData.pidKpb = controlData->pidKpb;
	logData.pidKib = controlData->pidKib;
	logData.pidKdb = controlData->pidKdb;
}

void ControlTask::release(){

	delete(pid);
}

