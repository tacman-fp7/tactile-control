#include "iCub/plantIdentification/data/LogData.h"

using iCub::plantIdentification::LogData;

using yarp::os::Bottle;


LogData::LogData(int fingersNum) {


    taskId = "";
    taskType = 0;
    taskOperationMode = 0;
    this->fingersNum = fingersNum;
    targetValue.resize(fingersNum,0);
    fingerTaxelValues.resize(fingersNum);
    for(size_t i = 0; i < fingerTaxelValues.size(); i++){
        fingerTaxelValues[i].resize(12,0);
    }
    overallFingerPressure.resize(fingersNum,0);
    overallFingerPressureMedian.resize(fingersNum,0);
    // TODO pwm should be 'jointsNum' long
    pwm.resize(fingersNum,0);
//    realProximalPwm.resize(fingersNum,0);
//    realDistalPwm.resize(fingersNum,0);
    proximalJointAngle.resize(fingersNum,0);
    distalJointAngle.resize(fingersNum,0);
    pidKpf.resize(fingersNum,0);
    pidKif.resize(fingersNum,0);
    pidKdf.resize(fingersNum,0);
    pidKpb.resize(fingersNum,0);
    pidKib.resize(fingersNum,0);
    pidKdb.resize(fingersNum,0);
    error.resize(fingersNum,0);
    errorIntegral.resize(fingersNum,0);

    dbgTag = "LogData: ";

}

void LogData::toBottle(Bottle &bottle){

    // #? : parameter number in the log file acquired using datadumper 
    bottle.clear();
    bottle.addString(taskId); // #3
    bottle.addInt(taskType); // #4
    bottle.addInt(taskOperationMode); // #5
    bottle.addInt(fingersNum);
    for(size_t i = 0; i < fingersNum; i++){
        bottle.addDouble(targetValue[i]); // #6
        for(int j = 0; j < fingerTaxelValues[i].size(); j++){
            bottle.addDouble(fingerTaxelValues[i][j]); // #7-18
        }
        bottle.addDouble(overallFingerPressure[i]); // #19
        bottle.addDouble(overallFingerPressureMedian[i]); // #20
        bottle.addDouble(pwm[i]); // #21
//        bottle.addDouble(realProximalPwm[i]); // #22
//        bottle.addDouble(realDistalPwm[i]); // #23
        bottle.addDouble(proximalJointAngle[i]); // #24
        bottle.addDouble(distalJointAngle[i]); // #25
        bottle.addDouble(pidKpf[i]); // #26
        bottle.addDouble(pidKif[i]); // #27
        bottle.addDouble(pidKdf[i]); // #28
        bottle.addDouble(pidKpb[i]); // #29
        bottle.addDouble(pidKib[i]); // #30
        bottle.addDouble(pidKdb[i]); // #31
        bottle.addDouble(error[i]); // #32
        bottle.addDouble(errorIntegral[i]); // #33
    }
}
