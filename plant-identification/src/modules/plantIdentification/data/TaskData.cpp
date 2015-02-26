#include "iCub/plantIdentification/data/TaskData.h"


using iCub::plantIdentification::TaskData;


TaskData::TaskData(yarp::os::ResourceFinder &rf,int threadRate) {

	commonData.threadRate = threadRate;

	// load data from resource file
	commonData.fingerToMove = rf.check("fingerToMove",1).asInt();
	commonData.jointToMove = rf.check("jointToMove",13).asInt();
	commonData.pwmSign = rf.check("pwmSign",1).asInt();
	commonData.previousOverallFingerPressures.resize(10,0.0);
	commonData.previousPressuresIndex = 0;

	stepData.lifespan = rf.check("stepTaskLifespan",10).asInt();

	controlData.pidKpf = rf.check("pidKpf",1.0).asDouble();
	controlData.pidKif = rf.check("pidKif",1.0).asDouble();
	controlData.pidKdf = rf.check("pidKdf",0.0).asDouble();
	controlData.pidKpb = rf.check("pidKpb",1.0).asDouble();
	controlData.pidKib = rf.check("pidKib",1.0).asDouble();
	controlData.pidKdb = rf.check("pidKdb",0.0).asDouble();
	controlData.lifespan = rf.check("controlTaskLifespan",10).asInt();

	rampData.slope = rf.check("slope",-0.0025).asDouble();
	rampData.intercept = rf.check("intercept",-90.0).asDouble();
	rampData.lifespan = rf.check("rampTaskLifespan",10).asInt();
	rampData.lifespanAfterStabilization = rf.check("rampTaskLifespanAfterStabilization",5).asInt();

	dbgTag = "TaskData: ";
}
