#include "iCub/objectGrasping/data/TaskData.h"

#include "iCub/objectGrasping/ObjectGraspingEnums.h"

#include <yarp/os/Value.h>

using yarp::os::Value;

using iCub::objectGrasping::TaskData;


TaskData::TaskData(yarp::os::ResourceFinder &rf,int threadRate) {
	using iCub::plantIdentification::ControlTaskOpMode;
	using yarp::os::Bottle;

	dbgTag = "TaskData: ";

	commonData.threadRate = threadRate;

	// load data from resource file
	commonData.pwmSign = rf.check("pwmSign",Value(1)).asInt();
	commonData.screenLogStride = rf.check("screenLogStride",Value(10)).asInt();
	//TODO generalize fingers and taxels number
	commonData.fingerTaxelsData.resize(5);
	for(size_t i = 0; i < commonData.fingerTaxelsData.size(); i++){
		commonData.fingerTaxelsData[i].resize(12,0.0);
	}
	commonData.previousOverallFingerPressures.resize(5);
	for(size_t i = 0; i < commonData.previousOverallFingerPressures.size(); i++){
		commonData.previousOverallFingerPressures[i].resize(rf.check("medianWidth",Value(20)).asInt(),0.0);
	}
	commonData.previousPressuresIndex.resize(5,0);
	commonData.overallFingerPressure.resize(5,0.0);
	commonData.overallFingerPressureMedian.resize(5,0.0);

	//TODO change the default value
	Bottle* stepTaskJoints = rf.find("stepTaskJoints").asList();
	stepData.jointsList.resize(stepTaskJoints->size(),0);
	stepData.fingersList.resize(stepTaskJoints->size(),0);
	for(int i = 0; i < stepTaskJoints->size(); i++){
		stepData.jointsList[i] = stepTaskJoints->get(i).asInt();
		stepData.fingersList[i] = getFingerFromJoint(stepData.jointsList[i]);
	}	stepData.lifespan = rf.check("stepTaskLifespan",Value(10)).asInt();

	controlData.pidKpf = rf.check("pidKpPe",Value(1.0)).asDouble();
	controlData.pidKif = rf.check("pidKiPe",Value(1.0)).asDouble();
	controlData.pidKdf = rf.check("pidKdPe",Value(0.0)).asDouble();
	controlData.pidKpb = rf.check("pidKpNe",Value(1.0)).asDouble();
	controlData.pidKib = rf.check("pidKiNe",Value(1.0)).asDouble();
	controlData.pidKdb = rf.check("pidKdNe",Value(0.0)).asDouble();

	//TODO change the default value
	Bottle* controltTaskJoints = rf.find("controlTaskJoints").asList();
	controlData.jointsList.resize(controltTaskJoints->size(),0);
	controlData.fingersList.resize(controltTaskJoints->size(),0);
	for(int i = 0; i < controltTaskJoints->size(); i++){
		controlData.jointsList[i] = controltTaskJoints->get(i).asInt();
		controlData.fingersList[i] = getFingerFromJoint(controlData.jointsList[i]);
	}
	controlData.pidWp = rf.check("pidWp",Value(1.0)).asDouble();
	controlData.pidWi = rf.check("pidWi",Value(1.0)).asDouble();
	controlData.pidWd = rf.check("pidWd",Value(1.0)).asDouble();
	controlData.pidN = rf.check("pidN",Value(10)).asInt();
	controlData.pidWindUpCoeff = rf.check("pidWindUpCoeff",Value(0.5)).asDouble();
	controlData.pidMinSatLim = rf.check("pidMinSatLim",Value(-1333.0)).asDouble();
	controlData.pidMaxSatLim = rf.check("pidMaxSatLim",Value(1333.0)).asDouble();
	controlData.pidKdb = rf.check("pidKdNe",Value(0.0)).asDouble();
	controlData.controlMode = static_cast<ControlTaskOpMode>(rf.check("controlMode",Value(2)).asInt());
	controlData.pidResetEnabled = rf.check("pidResetEnabled",Value(0)).asInt() != 0;
	controlData.lifespan = rf.check("controlTaskLifespan",Value(10)).asInt();

	//TODO change the default value
	Bottle* rampTaskJoints = rf.find("rampTaskJoints").asList();
	rampData.jointsList.resize(rampTaskJoints->size(),0);
	rampData.fingersList.resize(rampTaskJoints->size(),0);
	for(int i = 0; i < rampTaskJoints->size(); i++){
		rampData.jointsList[i] = rampTaskJoints->get(i).asInt();
		rampData.fingersList[i] = getFingerFromJoint(rampData.jointsList[i]);
	}
	rampData.slope = rf.check("slope",Value(-0.0025)).asDouble();
	rampData.intercept = rf.check("intercept",Value(-90.0)).asDouble();
	rampData.lifespan = rf.check("rampTaskLifespan",Value(10)).asInt();
	rampData.lifespanAfterStabilization = rf.check("rampTaskLifespanAfterStabilization",Value(5)).asInt();

}

int TaskData::getFingerFromJoint(int joint){
	
	//TODO to be modified

	if (joint >= 8 && joint <= 10) return 4;
	if (joint == 11 || joint == 12) return 0;
	if (joint == 13 || joint == 14) return 1;
	if (joint == 15) return 2;

}
