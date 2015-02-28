#include "iCub/plantIdentification/data/TaskData.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

using iCub::plantIdentification::TaskData;


TaskData::TaskData(yarp::os::ResourceFinder &rf,int threadRate) {
	using iCub::plantIdentification::ControlTaskOpMode;

	commonData.threadRate = threadRate;

	// load data from resource file
	commonData.fingerToMove = rf.check("fingerToMove",1).asInt();
	commonData.jointToMove = rf.check("jointToMove",13).asInt();
	commonData.pwmSign = rf.check("pwmSign",1).asInt();
	commonData.screenLogStride = rf.check("screenLogStride",10).asInt();
	commonData.previousOverallFingerPressures.resize(rf.check("medianWidth",20).asInt(),0.0);
	commonData.previousPressuresIndex = 0;

	stepData.lifespan = rf.check("stepTaskLifespan",10).asInt();

	controlData.pidKpf = rf.check("pidKpPe",1.0).asDouble();
	controlData.pidKif = rf.check("pidKiPe",1.0).asDouble();
	controlData.pidKdf = rf.check("pidKdPe",0.0).asDouble();
	controlData.pidKpb = rf.check("pidKpNe",1.0).asDouble();
	controlData.pidKib = rf.check("pidKiNe",1.0).asDouble();
	controlData.pidKdb = rf.check("pidKdNe",0.0).asDouble();

	controlData.pidWp = rf.check("pidWp",1.0).asDouble();
	controlData.pidWi = rf.check("pidWi",1.0).asDouble();
	controlData.pidWd = rf.check("pidWd",1.0).asDouble();
	controlData.pidN = rf.check("pidN",10).asInt();
	controlData.pidWindUpCoeff = rf.check("pidWindUpCoeff",0.5).asDouble();
	controlData.pidMinSatLim = rf.check("pidMinSatLim",-1333.0).asDouble();
	controlData.pidMaxSatLim = rf.check("pidMaxSatLim",1333.0).asDouble();
	controlData.pidKdb = rf.check("pidKdNe",0.0).asDouble();
	controlData.controlMode = static_cast<ControlTaskOpMode>(rf.check("controlMode",2).asInt());
	controlData.lifespan = rf.check("controlTaskLifespan",10).asInt();

	rampData.slope = rf.check("slope",-0.0025).asDouble();
	rampData.intercept = rf.check("intercept",-90.0).asDouble();
	rampData.lifespan = rf.check("rampTaskLifespan",10).asInt();
	rampData.lifespanAfterStabilization = rf.check("rampTaskLifespanAfterStabilization",5).asInt();

	dbgTag = "TaskData: ";
}
