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
	double ttPeOption,ttNeOption,ti,td,minTt,maxTt;
	
    // calculating Tt when error >= 0
    ti = controlData->pidKpf/controlData->pidKif;
	td = controlData->pidKdf/controlData->pidKpf;
	minTt = (0.1 + 0.9*controlData->pidWindUpCoeff)*ti;
	maxTt = ti;
	if (td < minTt){
		ttPeOption = minTt;
	} else if (td > maxTt){
		ttPeOption = maxTt;
	} else ttPeOption = td;
    
    // calculating Tt when error < 0
	ti = controlData->pidKpf/controlData->pidKib;
	td = controlData->pidKdf/controlData->pidKpb;
	minTt = (0.1 + 0.9*controlData->pidWindUpCoeff)*ti;
	maxTt = ti;
	if (td < minTt){
		ttNeOption = minTt;
	} else if (td > maxTt){
		ttNeOption = maxTt;
	} else ttNeOption = td;

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

	switch (controlData->controlMode){

	case GAINS_SET_POS_ERR:
		pid = new parallelPID(threadRateSec,kpPeOptionVect,kiPeOptionVect,kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsPE);
		break;

	case GAINS_SET_NEG_ERR:
		pid = new parallelPID(threadRateSec,kpNeOptionVect,kiNeOptionVect,kdNeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttNeOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsNE);
		break;

	case BOTH_GAINS_SETS:
		pid = new parallelPID(threadRateSec,kpPeOptionVect,kiPeOptionVect,kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect,satLimMatrix);
		pid->setOptions(pidOptionsPE);
		break;
	}
	
    Bottle bot;
    pid->getOptions(bot);
    std::cout << bot.toString() << "\n";


	taskName = CONTROL;
	dbgTag = "ControlTask: ";
}

void ControlTask::init(){

	std::cout << dbgTag << "\n" <<
        "TASK STARTED - Target: " << pressureTargetValue << "\n" <<
        "\n";
}

void ControlTask::calculatePwm(){
	using yarp::sig::Vector;

	if (controlData->controlMode == BOTH_GAINS_SETS){

		if (pressureTargetValue - commonData->overallFingerPressureMedian >= 0){
			pid->setOptions(pidOptionsPE);
		} else {
			pid->setOptions(pidOptionsNE);
    std::cout << "YEE";
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

	Bottle valueBottle,paramBottle;

	valueBottle.add(paramValue1);
	valueBottle.add(paramValue2);

	paramBottle.add(paramName);
	paramBottle.addList() = valueBottle;

	bottle.addList() = paramBottle;
}

