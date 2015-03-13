#include "iCub/plantIdentification/task/ControlTask.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <sstream>
#include <string>

using iCub::plantIdentification::ControlTask;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::ControlTaskData;

using iCub::ctrl::parallelPID;
using yarp::os::Bottle;
using yarp::os::Value;

using std::string;

ControlTask::ControlTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,ControlTaskData *controlData,double pressureTargetValue):Task(controllersUtil,portsUtil,commonData,controlData->lifespan) {
	using yarp::sig::Vector;
	using yarp::sig::Matrix;
	using std::stringstream;

    this->controlData = controlData;
	this->pressureTargetValue = pressureTargetValue;

    double threadRateSec = commonData->threadRate/1000.0;
	double ttPeOption,ttNeOption;
	
	ttPeOption = calculateTt(GAINS_SET_POS_ERR);
	ttNeOption = calculateTt(GAINS_SET_NEG_ERR);
	
	Vector kpPeOptionVect(1,controlData->pidKpf);
	Vector kiPeOptionVect(1,controlData->pidKif);
	Vector kdPeOptionVect(1,controlData->pidKdf);
	Vector ttPeOptionVect(1,ttPeOption);

	Vector kpNeOptionVect(1,controlData->pidKpb);
	Vector kiNeOptionVect(1,controlData->pidKib);
	Vector kdNeOptionVect(1,controlData->pidKdb);
	Vector ttNeOptionVect(1,ttNeOption);

	Vector wpOptionVect(1,controlData->pidWp);
	Vector wiOptionVect(1,controlData->pidWi);
	Vector wdOptionVect(1,controlData->pidWd);
	Vector nOptionVect(1,controlData->pidN);
	
	Matrix satLimMatrix(1,2);
	satLimMatrix[0][0] = controlData->pidMinSatLim;
	satLimMatrix[0][1] = controlData->pidMaxSatLim;

	Bottle commonOptions;
	
	addOption(commonOptions,"Wp",Value(controlData->pidWp));
	addOption(commonOptions,"Wi",Value(controlData->pidWi));
	addOption(commonOptions,"Wd",Value(controlData->pidWd));
	addOption(commonOptions,"N",Value(controlData->pidN));
	addOption(commonOptions,"satLim",Value(controlData->pidMinSatLim),Value(controlData->pidMaxSatLim));

	addOption(pidOptionsPE,"Kp",Value(controlData->pidKpf));
	addOption(pidOptionsPE,"Ki",Value(controlData->pidKif));
	addOption(pidOptionsPE,"Kd",Value(controlData->pidKdf));
	addOption(pidOptionsPE,"Tt",Value(ttPeOption));
	pidOptionsPE.append(commonOptions);

	addOption(pidOptionsNE,"Kp",Value(controlData->pidKpb));
	addOption(pidOptionsNE,"Ki",Value(controlData->pidKib));
	addOption(pidOptionsNE,"Kd",Value(controlData->pidKdb));
	addOption(pidOptionsNE,"Tt",Value(ttNeOption));
	pidOptionsNE.append(commonOptions);

	// TODO to be removed
	kpPe = controlData->pidKpf;
	kpNe = controlData->pidKpb;
	previousError = 0;

	switch (controlData->controlMode){

	case GAINS_SET_POS_ERR:
		pid = new parallelPID(threadRateSec,kpPeOptionVect,kiPeOptionVect,kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsPE);
		currentKp = kpPe;
		break;

	case GAINS_SET_NEG_ERR:
		pid = new parallelPID(threadRateSec,kpNeOptionVect,kiNeOptionVect,kdNeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttNeOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsNE);
		currentKp = kpNe;
		break;

	case BOTH_GAINS_SETS:
		pid = new parallelPID(threadRateSec,kpPeOptionVect,kiPeOptionVect,kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsPE);
		currentKp = kpPe;
		break;
	}

	taskName = CONTROL;
	dbgTag = "ControlTask: ";

}

void ControlTask::init(){

	std::cout << "\n\n" << dbgTag << "TASK STARTED - Target: " << pressureTargetValue << "\n\n";
}

void ControlTask::calculatePwm(){
	using yarp::sig::Vector;

	double error = pressureTargetValue - commonData->overallFingerPressure;



	if (controlData->controlMode == BOTH_GAINS_SETS){

		if (error >= 0 && previousError < 0){
			pid->setOptions(pidOptionsPE);
			currentKp = kpPe;
		} else if (error < 0 && previousError >= 0){
			pid->setOptions(pidOptionsNE);
			currentKp = kpNe;
		}
	}

	Vector ref(1,pressureTargetValue);
	Vector fb(1,commonData->overallFingerPressure);
	Vector result = pid->compute(ref,fb);
	
	// TODO to be removed
	if (controlData->pidResetEnabled && error > 0 && result[0] < currentKp*error - 1.0){
		pid->reset(result);
		optionalLogString.append("[ PID RESET ] ");
	}

	pwmToUse = result[0];

	previousError = error;
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

void ControlTask::addOption(Bottle &bottle,char *paramName,Value paramValue){

	Bottle valueBottle,paramBottle;

	valueBottle.add(paramValue);

	paramBottle.add(paramName);
	paramBottle.addList() = valueBottle;

	bottle.addList() = paramBottle;
}

void ControlTask::addOption(Bottle &bottle,char *paramName,Value paramValue1,Value paramValue2){

	Bottle valueBottle,paramBottle;

	valueBottle.add(paramValue1);
	valueBottle.add(paramValue2);

	paramBottle.add(paramName);
	paramBottle.addList() = valueBottle;

	bottle.addList() = paramBottle;
}

double ControlTask::calculateTt(iCub::plantIdentification::ControlTaskOpMode gainsSet){

	double tt,ti,td,minTt,maxTt;

	switch (gainsSet){

	case GAINS_SET_POS_ERR:
		ti = controlData->pidKpf/controlData->pidKif;
	    td = controlData->pidKdf/controlData->pidKpf;
		break;

	case GAINS_SET_NEG_ERR:
		ti = controlData->pidKpf/controlData->pidKif;
	    td = controlData->pidKdf/controlData->pidKpf;
		break;
	}

	// TODO check the Tt rule
	minTt = controlData->pidWindUpCoeff*ti;
	maxTt = ti;
	if (td < minTt){
		tt = minTt;
	} else if (td > maxTt){
		tt = maxTt;
	} else tt = td;

	return tt;
}
