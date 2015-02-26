#include "iCub/plantIdentification/data/LogData.h"

using iCub::plantIdentification::LogData;

using yarp::os::Bottle;


LogData::LogData() {
     
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

	bottle.clear();
	bottle.addInt(taskType);
	bottle.addInt(taskOperationMode);
	bottle.addDouble(targetValue);
	for(int i = 0; i < fingerTaxelValues.size(); i++){
		bottle.addDouble(fingerTaxelValues[i]);
	}
	bottle.addDouble(overallFingerPressure);
	bottle.addDouble(overallFingerPressureMedian);
	bottle.addDouble(pwm);
	bottle.addDouble(realProximalPwm);
	bottle.addDouble(realDistalPwm);
	bottle.addDouble(proximalJointAngle);
	bottle.addDouble(distalJointAngle);
	bottle.addDouble(pidKpf);
	bottle.addDouble(pidKif);
	bottle.addDouble(pidKdf);
	bottle.addDouble(pidKpb);
	bottle.addDouble(pidKib);
	bottle.addDouble(pidKdb);
	bottle.addDouble(error);
	bottle.addDouble(errorIntegral);

}
