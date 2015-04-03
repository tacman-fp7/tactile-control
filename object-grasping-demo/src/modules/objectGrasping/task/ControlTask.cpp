#include "iCub/objectGrasping/task/ControlTask.h"

#include "iCub/objectGrasping/ObjectGraspingEnums.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <sstream>
#include <string>

using iCub::objectGrasping::ControlTask;
using iCub::objectGrasping::LogData;
using iCub::objectGrasping::ControllersUtil;
using iCub::objectGrasping::PortsUtil;
using iCub::objectGrasping::TaskCommonData;
using iCub::objectGrasping::ControlTaskData;

using iCub::ctrl::parallelPID;
using yarp::os::Bottle;
using yarp::os::Value;

using std::string;

ControlTask::ControlTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,ControlTaskData *controlData,double pressureTargetValue):Task(controllersUtil,portsUtil,commonData,controlData->lifespan,controlData->jointsList,controlData->fingersList) {
	using yarp::sig::Vector;
	using yarp::sig::Matrix;

    this->controlData = controlData;

	this->pressureTargetValue.resize(fingersList.size(),pressureTargetValue);
	this->pid.resize(jointsList.size());

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


	for(size_t i = 0; i < jointsList.size(); i++){

		// TODO to be removed
		kpPe[i] = controlData->pidKpf;
		kpNe[i] = controlData->pidKpb;
		previousError[i] = 0;

		switch (controlData->controlMode){

			case GAINS_SET_POS_ERR:
				pid[i] = new parallelPID(threadRateSec,kpPeOptionVect,kiPeOptionVect,kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect,satLimMatrix);
				pid[i]->setOptions(pidOptionsPE);
				currentKp[i] = kpPe[i];
				break;

			case GAINS_SET_NEG_ERR:
				pid[i] = new parallelPID(threadRateSec,kpNeOptionVect,kiNeOptionVect,kdNeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttNeOptionVect,satLimMatrix);
				pid[i]->setOptions(pidOptionsNE);
				currentKp[i] = kpNe[i];
				break;

			case BOTH_GAINS_SETS:
				pid[i] = new parallelPID(threadRateSec,kpPeOptionVect,kiPeOptionVect,kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect,satLimMatrix);
				pid[i]->setOptions(pidOptionsPE);
				currentKp[i] = kpPe[i];
				break;
		}

	}
	taskName = CONTROL;
	dbgTag = "ControlTask: ";

}

void ControlTask::init(){
	using std::cout;

	controllersUtil->setTaskControlModes(jointsList,VOCAB_CM_OPENLOOP);

	cout << "\n\n" << dbgTag << "TASK STARTED - Target: ";
	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		cout << pressureTargetValue[i] << " ";
	}
	cout << "\n\n";
}

void ControlTask::calculatePwm(){
	using yarp::sig::Vector;


	for(size_t i = 0; i < jointsList.size(); i++){

		double error = pressureTargetValue[i] - commonData->overallFingerPressure[i];



		if (controlData->controlMode == BOTH_GAINS_SETS){

			if (error >= 0 && previousError[i] < 0){
				pid[i]->setOptions(pidOptionsPE);
				currentKp[i] = kpPe[i];
			} else if (error < 0 && previousError[i] >= 0){
				pid[i]->setOptions(pidOptionsNE);
				currentKp[i] = kpNe[i];
			}
		}

		Vector ref(1,pressureTargetValue[i]);
		Vector fb(1,commonData->overallFingerPressure[i]);
		Vector result = pid[i]->compute(ref,fb);
	
		// TODO to be removed
		if (controlData->pidResetEnabled && error > 0 && result[0] < currentKp[i]*error - 1.0){
			pid[i]->reset(result);
			optionalLogString.append("[ PID RESET ] ");
		}

		pwmToUse[i] = result[0];

		previousError[i] = error;

	}
}

void ControlTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	logData.taskType = CONTROL;
	logData.taskOperationMode = controlData->controlMode;
	//TODO only the first element is logged!
	logData.targetValue = pressureTargetValue[0];
	
	logData.pidKpf = controlData->pidKpf;
	logData.pidKif = controlData->pidKif;
	logData.pidKdf = controlData->pidKdf;
	logData.pidKpb = controlData->pidKpb;
	logData.pidKib = controlData->pidKib;
	logData.pidKdb = controlData->pidKdb;
}

void ControlTask::release(){

	for(size_t i = 0; i < jointsList.size(); i++){
		delete(pid[i]);
	}
	
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

double ControlTask::calculateTt(iCub::objectGrasping::ControlTaskOpMode gainsSet){

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

std::string ControlTask::getPressureTargetValueDescription(){

	std::stringstream description("");

	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		description << pressureTargetValue[i] << " ";
	}
	
	return description.str();
}
