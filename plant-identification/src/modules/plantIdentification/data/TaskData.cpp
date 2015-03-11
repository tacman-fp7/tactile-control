#include "iCub/plantIdentification/data/TaskData.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/Value.h>

using yarp::os::Value;

using iCub::plantIdentification::TaskData;


TaskData::TaskData(yarp::os::ResourceFinder &rf,int threadRate) {
	using iCub::plantIdentification::ControlTaskOpMode;

	commonData.threadRate = threadRate;

	// load data from resource file
	commonData.fingerToMove = rf.check("fingerToMove",Value(1)).asInt();
	commonData.jointToMove = rf.check("jointToMove",Value(13)).asInt();
	commonData.pwmSign = rf.check("pwmSign",Value(1)).asInt();
	commonData.screenLogStride = rf.check("screenLogStride",Value(10)).asInt();
    commonData.fingerTaxelsData.resize(12,0.0);
	commonData.previousOverallFingerPressures.resize(rf.check("medianWidth",Value(20)).asInt(),0.0);
	commonData.previousPressuresIndex = 0;

	stepData.lifespan = rf.check("stepTaskLifespan",Value(10)).asInt();

	controlData.pidKpf = rf.check("pidKpPe",Value(1.0)).asDouble();
	controlData.pidKif = rf.check("pidKiPe",Value(1.0)).asDouble();
	controlData.pidKdf = rf.check("pidKdPe",Value(0.0)).asDouble();
	controlData.pidKpb = rf.check("pidKpNe",Value(1.0)).asDouble();
	controlData.pidKib = rf.check("pidKiNe",Value(1.0)).asDouble();
	controlData.pidKdb = rf.check("pidKdNe",Value(0.0)).asDouble();

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

	rampData.slope = rf.check("slope",Value(-0.0025)).asDouble();
	rampData.intercept = rf.check("intercept",Value(-90.0)).asDouble();
	rampData.lifespan = rf.check("rampTaskLifespan",Value(10)).asInt();
	rampData.lifespanAfterStabilization = rf.check("rampTaskLifespanAfterStabilization",Value(5)).asInt();

	dbgTag = "TaskData: ";
}
