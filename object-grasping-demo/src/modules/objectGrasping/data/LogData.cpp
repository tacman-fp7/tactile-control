#include "iCub/objectGrasping/data/LogData.h"

using iCub::objectGrasping::LogData;

using yarp::os::Bottle;


LogData::LogData() {
    
	taskId = "";
	taskType = 0;
	taskOperationMode = 0;
	targetValue = 0;
	fingerTaxelValues.resize(12,0);
	overallFingerPressure = 0;
	overallFingerPressureMedian = 0;
	pwm = 0;
	realProximalPwm = 0;
	realDistalPwm = 0;
	proximalJointAngle = 0;
	distalJointAngle = 0;
	pidKpf = 0;
	pidKif = 0;
	pidKdf = 0;
	pidKpb = 0;
	pidKib = 0;
	pidKdb = 0;
	error = 0;
	errorIntegral = 0;

	dbgTag = "LogData: ";

}

void LogData::toBottle(Bottle &bottle){

	// #? : parameter number in the log file acquired using datadumper 
	bottle.clear();
	bottle.addString(taskId); // #3
	bottle.addInt(taskType); // #4
	bottle.addInt(taskOperationMode); // #5
	bottle.addDouble(targetValue); // #6
	for(int i = 0; i < fingerTaxelValues.size(); i++){
		bottle.addDouble(fingerTaxelValues[i]); // #7-18
	}
	bottle.addDouble(overallFingerPressure); // #19
	bottle.addDouble(overallFingerPressureMedian); // #20
	bottle.addDouble(pwm); // #21
	bottle.addDouble(realProximalPwm); // #22
	bottle.addDouble(realDistalPwm); // #23
	bottle.addDouble(proximalJointAngle); // #24
	bottle.addDouble(distalJointAngle); // #25
	bottle.addDouble(pidKpf); // #26
	bottle.addDouble(pidKif); // #27
	bottle.addDouble(pidKdf); // #28
	bottle.addDouble(pidKpb); // #29
	bottle.addDouble(pidKib); // #30
	bottle.addDouble(pidKdb); // #31
	bottle.addDouble(error); // #32
	bottle.addDouble(errorIntegral); // #33

}
