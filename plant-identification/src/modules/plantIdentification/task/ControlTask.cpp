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

ControlTask::ControlTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,ControlTaskData *controlData,std::vector<double> &targetList,bool resetErrOnContact):Task(controllersUtil,portsUtil,commonData,controlData->lifespan,controlData->jointsList,controlData->fingersList) {
	using yarp::sig::Vector;
	using yarp::sig::Matrix;
    this->controlData = controlData;

	this->resetErrOnContact = resetErrOnContact;
	fingerIsInContact.resize(commonData->objDetectPressureThresholds.size(),false);

	pressureTargetValue.resize(fingersList.size());
	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		pressureTargetValue[i] = (i >= targetList.size() ? targetList[targetList.size()-1] : targetList[i]);
	}
	
	pid.resize(jointsList.size());
	pidOptionsPE.resize(jointsList.size());
	pidOptionsNE.resize(jointsList.size());
	
	currentKp.resize(jointsList.size());
    kpPe.resize(jointsList.size());
	kpNe.resize(jointsList.size());
	previousError.resize(jointsList.size());

    double threadRateSec = commonData->threadRate/1000.0;
	std::vector<double> ttPeOption,ttNeOption;
	ttPeOption.resize(jointsList.size());
	ttNeOption.resize(jointsList.size());
	for(size_t i = 0; i < jointsList.size(); i++){
		ttPeOption[i] = calculateTt(GAINS_SET_POS_ERR,i);
		ttNeOption[i] = calculateTt(GAINS_SET_NEG_ERR,i);
	}
	
	std::vector<Vector> kpPeOptionVect;
	kpPeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < kpPeOptionVect.size(); i++){
		kpPeOptionVect[i].resize(1,controlData->pidKpf[i]);
	}
	std::vector<Vector> kiPeOptionVect;
	kiPeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < kiPeOptionVect.size(); i++){
		kiPeOptionVect[i].resize(1,controlData->pidKif[i]);
	}
	Vector kdPeOptionVect(1,0.0);
	std::vector<Vector> ttPeOptionVect;
	ttPeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < ttPeOptionVect.size(); i++){
		ttPeOptionVect[i].resize(1,ttPeOption[i]);
	}

	
	std::vector<Vector> kpNeOptionVect;
	kpNeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < kpNeOptionVect.size(); i++){
		kpNeOptionVect[i].resize(1,controlData->pidKpb[i]);
	}
	std::vector<Vector> kiNeOptionVect;
	kiNeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < kiNeOptionVect.size(); i++){
		kiNeOptionVect[i].resize(1,controlData->pidKib[i]);
	}
	Vector kdNeOptionVect(1,0.0);
	std::vector<Vector> ttNeOptionVect;
	ttNeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < ttNeOptionVect.size(); i++){
		ttNeOptionVect[i].resize(1,ttNeOption[i]);
	}

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

	for(size_t i = 0; i < jointsList.size(); i++){
		addOption(pidOptionsPE[i],"Kp",Value(controlData->pidKpf[i]));
		addOption(pidOptionsPE[i],"Ki",Value(controlData->pidKif[i]));
		addOption(pidOptionsPE[i],"Kd",Value(0.0));
		addOption(pidOptionsPE[i],"Tt",Value(ttPeOption[i]));
		pidOptionsPE[i].append(commonOptions);

		addOption(pidOptionsNE[i],"Kp",Value(controlData->pidKpb[i]));
		addOption(pidOptionsNE[i],"Ki",Value(controlData->pidKib[i]));
		addOption(pidOptionsNE[i],"Kd",Value(0.0));
		addOption(pidOptionsNE[i],"Tt",Value(ttNeOption[i]));
		pidOptionsNE[i].append(commonOptions);
	}

	for(size_t i = 0; i < jointsList.size(); i++){

		// TODO to be removed
		kpPe[i] = controlData->pidKpf[i];
		kpNe[i] = controlData->pidKpb[i];
		previousError[i] = 0;

		switch (controlData->controlMode){

			case GAINS_SET_POS_ERR:
				pid[i] = new parallelPID(threadRateSec,kpPeOptionVect[i],kiPeOptionVect[i],kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect[i],satLimMatrix);
				pid[i]->setOptions(pidOptionsPE[i]);
				currentKp[i] = kpPe[i];
				break;

			case GAINS_SET_NEG_ERR:
				pid[i] = new parallelPID(threadRateSec,kpNeOptionVect[i],kiNeOptionVect[i],kdNeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttNeOptionVect[i],satLimMatrix);
				pid[i]->setOptions(pidOptionsNE[i]);
				currentKp[i] = kpNe[i];
				break;

			case BOTH_GAINS_SETS:
				pid[i] = new parallelPID(threadRateSec,kpPeOptionVect[i],kiPeOptionVect[i],kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect[i],satLimMatrix);
				pid[i]->setOptions(pidOptionsPE[i]);
				currentKp[i] = kpPe[i];
				break;
		}
	}
	if (resetErrOnContact){
		taskName = APPROACH_AND_CONTROL;
		dbgTag = "Approach&ControlTask: ";
	} else {
		taskName = CONTROL;
		dbgTag = "ControlTask: ";
	}
	

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

void ControlTask::calculateControlInput(){
	using yarp::sig::Vector;


	for(size_t i = 0; i < jointsList.size(); i++){

		double error = pressureTargetValue[i] - commonData->overallFingerPressure[fingersList[i]];



		if (controlData->controlMode == BOTH_GAINS_SETS){

			if (error >= 0 && previousError[i] < 0){
				pid[i]->setOptions(pidOptionsPE[i]);
				currentKp[i] = kpPe[i];
			} else if (error < 0 && previousError[i] >= 0){
				pid[i]->setOptions(pidOptionsNE[i]);
				currentKp[i] = kpNe[i];
			}
		}

		Vector ref(1,pressureTargetValue[i]);
		Vector fb(1,commonData->overallFingerPressure[fingersList[i]]);
		Vector result = pid[i]->compute(ref,fb);
	
		// TODO to be removed
		if (controlData->pidResetEnabled && error > 0 && result[0] < currentKp[i]*error - 1.0){
			pid[i]->reset(result);
			optionalLogString.append("[ PID RESET ] ");
		}

		// if a finger gets in touch with the object then reset its PID
		if (resetErrOnContact && !fingerIsInContact[fingersList[i]] && commonData->overallFingerPressureMedian[fingersList[i]] > commonData->objDetectPressureThresholds[fingersList[i]]){
			pid[i]->reset(result);
			fingerIsInContact[fingersList[i]] = true;
			std::stringstream output("");
			output << "[ Finger " << fingersList[i] << " PID RESET ] ";
			optionalLogString.append(output.str());
		}

		inputCommandValue[i] = result[0];

		previousError[i] = error;

	}
}

void ControlTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	if (resetErrOnContact) {
		logData.taskType = APPROACH_AND_CONTROL;
	} else {
		logData.taskType = CONTROL;
	}
	logData.taskOperationMode = controlData->controlMode;
	//TODO only first elements are logged!
	logData.targetValue = pressureTargetValue[0];
	
	logData.pidKpf = controlData->pidKpf[0];
	logData.pidKif = controlData->pidKif[0];
	logData.pidKdf = 0.0;
	logData.pidKpb = controlData->pidKpb[0];
	logData.pidKib = controlData->pidKib[0];
	logData.pidKdb = 0.0;
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

double ControlTask::calculateTt(iCub::plantIdentification::ControlTaskOpMode gainsSet,int index){

	double tt,ti,td,minTt,maxTt;

	switch (gainsSet){

	case GAINS_SET_POS_ERR:
		ti = controlData->pidKpf[index]/controlData->pidKif[index];
	    td = 0.0; //controlData->pidKdf/controlData->pidKpf[index];
		break;

	case GAINS_SET_NEG_ERR:
		ti = controlData->pidKpf[index]/controlData->pidKif[index];
	    td = 0.0; //controlData->pidKdf/controlData->pidKpf[index];
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

void ControlTask::setTargetListRealTime(std::vector<double> &targetList){

	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		pressureTargetValue[i] = (i >= targetList.size() ? targetList[targetList.size()-1] : targetList[i]);
	}

}