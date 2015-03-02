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

	this->pressureTargetValue = pressureTargetValue;

	double ttOption;
	double ti = controlData->pidKpf/controlData->pidKif;
	double td = controlData->pidKdf/controlData->pidKpf;
	// TODO check the rule
	double minTt = 0.1 + 0.9*controlData->pidWindUpCoeff*ti;
	double maxTt = ti;
	if (td < minTt){
		ttOption = minTt;
	} else if (td > maxTt){
		ttOption = maxTt;
	} else ttOption = td;

	Vector kpPeOptionVect(1,controlData->pidKpf);
	Vector kiPeOptionVect(1,controlData->pidKif);
	Vector kdPeOptionVect(1,controlData->pidKdf);

	Vector kpNeOptionVect(1,controlData->pidKpb);
	Vector kiNeOptionVect(1,controlData->pidKib);
	Vector kdNeOptionVect(1,controlData->pidKdb);

	Vector wpOptionVect(1,controlData->pidWp);
	Vector wiOptionVect(1,controlData->pidWi);
	Vector wdOptionVect(1,controlData->pidWd);
	Vector nOptionVect(1,controlData->pidN);
	Vector ttOptionVect(1,ttOption);
	
	Matrix satLimMatrix(1,2);
	satLimMatrix[0][0] = controlData->pidMinSatLim;
	satLimMatrix[0][1] = controlData->pidMaxSatLim;

	Bottle commonOptions;
	
	addOption(commonOptions,"Wp",Value(controlData->pidWp));
	addOption(commonOptions,"Wi",Value(controlData->pidWi));
	addOption(commonOptions,"Wd",Value(controlData->pidWd));
	addOption(commonOptions,"N",Value(controlData->pidN));
	addOption(commonOptions,"Tt",Value(ttOption));
	addOption(commonOptions,"satLim",Value(controlData->pidMinSatLim),Value(controlData->pidMaxSatLim));

	addOption(pidOptionsPE,"Kp",Value(controlData->pidKpf));
	addOption(pidOptionsPE,"Ki",Value(controlData->pidKif));
	addOption(pidOptionsPE,"Kd",Value(controlData->pidKdf));
	pidOptionsPE.append(commonOptions);

	addOption(pidOptionsNE,"Kp",Value(controlData->pidKpb));
	addOption(pidOptionsNE,"Ki",Value(controlData->pidKib));
	addOption(pidOptionsNE,"Kd",Value(controlData->pidKdb));
	pidOptionsNE.append(commonOptions);

	
	switch (controlData->controlMode){

	case GAINS_SET_POS_ERR:
		pid = new parallelPID(commonData->threadRate,kpPeOptionVect,kiPeOptionVect,kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsPE);
		break;

	case GAINS_SET_NEG_ERR:
		pid = new parallelPID(commonData->threadRate,kpNeOptionVect,kiNeOptionVect,kdNeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsNE);
		break;

	case BOTH_GAINS_SETS:
		pid = new parallelPID(commonData->threadRate,kpPeOptionVect,kiPeOptionVect,kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsPE);
		break;
	}
	
	taskName = CONTROL;
	dbgTag = "ControlTask: ";
}

void ControlTask::init(){

	std::cout << dbgTag << "TASK STARTED - Target: " << pressureTargetValue << "\n";
}

void ControlTask::calculatePwm(){
	using yarp::sig::Vector;

	if (controlData->controlMode == BOTH_GAINS_SETS){

		if (pressureTargetValue - commonData->overallFingerPressureMedian >= 0){
			pid->setOptions(pidOptionsPE);
		} else {
			pid->setOptions(pidOptionsNE);
		}
	}

	Vector ref(1,pressureTargetValue);
	Vector fb(1,commonData->overallFingerPressureMedian);
	Vector result = pid->compute(ref,fb);
	
	pwmToUse = result[0];
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

	Bottle rowBottle,valueBottle,paramBottle;

	rowBottle.add(paramValue1);
	rowBottle.add(paramValue2);

	valueBottle.addList() = rowBottle;

	paramBottle.add(paramName);
	paramBottle.addList() = valueBottle;

	bottle.addList() = paramBottle;
}

