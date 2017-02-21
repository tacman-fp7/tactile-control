#include "iCub/plantIdentification/data/TaskData.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/Value.h>

#include <sstream>

using yarp::os::Value;

using iCub::plantIdentification::TaskData;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::MyThread;


int TaskCommonData::tpInt(int index){
    if (tempParameters.size() > index){
        return tempParameters[index].asInt();
    }
    return 0;
}

double TaskCommonData::tpDbl(int index){
    if (tempParameters.size() > index){
        return tempParameters[index].asDouble();
    }
    return 0.0;
}

std::string TaskCommonData::tpStr(int index){
    if (tempParameters.size() > index){
        return tempParameters[index].asString();
    }
    return "";
}

int TaskCommonData::getNumOfThreadCallsFromTime(MyThread whichThread,double time){

    int threadPeriod;

    switch(whichThread){

    case TASK_THREAD:
        threadPeriod = taskThreadPeriod;
        break;
    case EVENTS_THREAD:
        threadPeriod = eventsThreadPeriod;
        break;
    }

    return (int)(time*1000.0/threadPeriod);

}


double TaskCommonData::getTimeFromNumOfThreadCalls(MyThread whichThread,int numOfThreadCalls){
    
    int threadPeriod;

    switch(whichThread){

    case TASK_THREAD:
        threadPeriod = taskThreadPeriod;
        break;
    case EVENTS_THREAD:
        threadPeriod = eventsThreadPeriod;
        break;
    }

    return numOfThreadCalls*threadPeriod/1000.0;

}


TaskData::TaskData(yarp::os::ResourceFinder &rf,iCub::plantIdentification::ControllersUtil *controllersUtil) {
    using iCub::plantIdentification::ControlTaskOpMode;
    using yarp::os::Bottle;

    dbgTag = "TaskData: ";

    commonData.taskThreadPeriod = rf.check("taskThreadPeriod", 20).asInt();
    commonData.eventsThreadPeriod = rf.check("eventsThreadPeriod", 15).asInt();

    std::string whichHand = rf.check("whichHand", Value("left"), "The hand to be used for the grasping.").asString().c_str();
    std::string whichICub = rf.check("whichICub", Value("black"), "The iCub used for the task.").asString().c_str();
    std::string whichTask = rf.check("whichTask", Value("grasp"), "The code of the task [grasp/objrec]").asString().c_str();

    // load data from resource file
    commonData.pwmSign = rf.check("pwmSign",Value(1)).asInt();
    commonData.screenLogStride = rf.check("screenLogStride",Value(10)).asInt();
    //TODO generalize fingers and taxels number
    commonData.fingerTaxelsData.resize(5);
    for(size_t i = 0; i < commonData.fingerTaxelsData.size(); i++){
        commonData.fingerTaxelsData[i].resize(12,0.0);
    }
    commonData.fingerTaxelsRawData.resize(5);
    for(size_t i = 0; i < commonData.fingerTaxelsRawData.size(); i++){
        commonData.fingerTaxelsRawData[i].resize(12,0.0);
    }
    commonData.previousOverallFingerPressures.resize(5);
    for(size_t i = 0; i < commonData.previousOverallFingerPressures.size(); i++){
        commonData.previousOverallFingerPressures[i].resize(rf.check("medianWidth",Value(20)).asInt(),0.0);
    }
    commonData.previousPressuresIndex.resize(5,0);
    commonData.overallFingerPressure.resize(5,0.0);
    commonData.overallFingerPressureBySimpleSum.resize(5,0.0);
    commonData.overallFingerPressureByWeightedSum.resize(5,0.0);
    commonData.overallFingerPressureMedian.resize(5,0.0);
    commonData.armEncodersAngles.resize(controllersUtil->armJointsNum,0.0);      
    commonData.armEncodersAnglesReferences.resize(controllersUtil->armJointsNum,0.0);      
    //TODO generalize number of port components
    commonData.fingerEncodersRawData.resize(16,0.0);      

    controllersUtil->getArmEncodersAngles(commonData.armEncodersAngles,true);
    controllersUtil->getArmEncodersAnglesReferences(commonData.armEncodersAnglesReferences,true);

    Bottle* objDetectPressureThresholds = rf.find("objDetectPressureThresholds").asList();
    commonData.objDetectPressureThresholds.resize(objDetectPressureThresholds->size(),0);
    for(int i = 0; i < objDetectPressureThresholds->size(); i++){
        commonData.objDetectPressureThresholds[i] = objDetectPressureThresholds->get(i).asDouble();
    }

    commonData.realProximalPwm.resize(5,0.0);
    commonData.realDistalPwm.resize(5,0.0);
    commonData.proximalJointAngle.resize(5,0.0);
    commonData.distalJointAngle.resize(5,0.0);

    Bottle* tempParameters = rf.find("tempParameters").asList();
    commonData.tempParameters.resize(tempParameters->size());
    for(int i = 0; i < tempParameters->size(); i++){
        commonData.tempParameters[i] = tempParameters->get(i);
    }
/*
    // temp parameters updated according to the hand and/or the robot
    if (whichICub == "black"){
        if (whichHand == "right"){
            
            commonData.tempParameters[52] = 1.0;
            commonData.tempParameters[53] = 0.6;
            commonData.tempParameters[54] = 1.0;
            // found using the force sensors
            //commonData.tempParameters[52] = 1.0/13;
            //commonData.tempParameters[53] = 1.0/45;
            //commonData.tempParameters[54] = 1.0/20;

            commonData.tempParameters[75] = -50;

            commonData.tempParameters[78] = 0.0;
            commonData.tempParameters[79] = 0.0;
            commonData.tempParameters[80] = 0.0;

        } else {

            commonData.tempParameters[75] = -30;

            commonData.tempParameters[52] = 1.0;
            commonData.tempParameters[53] = 0.6;
            commonData.tempParameters[54] = 0.6;
        }
    } else if (whichICub == "purple"){
        if (whichHand == "right"){
            // default values
        } else {
            // approach
            commonData.tempParameters[34] = 1.5;
            commonData.tempParameters[37] = 1.3;

            commonData.tempParameters[52] = 0.5;
            commonData.tempParameters[53] = 1.0;
            commonData.tempParameters[54] = 1.0;
        }
    } else { // darmstadt
        if (whichHand == "right"){
            // default values
        } else {
            commonData.tempParameters[52] = 0.6;
                    commonData.tempParameters[53] = 0.6;
            commonData.tempParameters[54] = 0.6;
        }
    }
*/
    commonData.iCubThumb = new iCub::iKin::iCubFinger(whichHand + "_thumb");
    commonData.iCubIndexFinger = new iCub::iKin::iCubFinger(whichHand + "_index");
    commonData.iCubMiddleFinger = new iCub::iKin::iCubFinger(whichHand + "_middle");
    commonData.iCubThumb->alignJointsBounds(controllersUtil->getArmJointLimits());
    commonData.iCubIndexFinger->alignJointsBounds(controllersUtil->getArmJointLimits());
    commonData.iCubMiddleFinger->alignJointsBounds(controllersUtil->getArmJointLimits());

    commonData.forceSensorData.resize(6,0.0);      
    commonData.forceSensorBias.resize(6,0.0);      
    commonData.procForceSensorData.resize(5,0.0);      

    commonData.realForceData.resize(5,0.0);

    Bottle* armJoints = rf.find("armJoints_" + whichTask).asList();
    commonData.armJointsHome.resize(armJoints->size());
    for(int i = 0; i < armJoints->size(); i++){
        commonData.armJointsHome[i] = armJoints->get(i).asDouble();
    }
    commonData.requestOpen = false;


    Bottle* stepTaskJoints = rf.find("stepTaskJoints").asList();
    stepData.jointsList.resize(stepTaskJoints->size(),0);
    stepData.fingersList.resize(stepTaskJoints->size(),0);
    for(int i = 0; i < stepTaskJoints->size(); i++){
        stepData.jointsList[i] = stepTaskJoints->get(i).asInt();
        stepData.fingersList[i] = getFingerFromJoint(stepData.jointsList[i]);
    }
    stepData.lifespan = rf.check("stepTaskLifespan",Value(10)).asInt();

    Bottle* controlTaskPidKpPe = rf.find("pidKpPe").asList();
    controlData.pidKpf.resize(controlTaskPidKpPe->size(),0);
    for(int i = 0; i < controlTaskPidKpPe->size(); i++){
        controlData.pidKpf[i] = controlTaskPidKpPe->get(i).asDouble();
    }
    Bottle* controlTaskPidKiPe = rf.find("pidKiPe").asList();
    controlData.pidKif.resize(controlTaskPidKiPe->size(),0);
    for(int i = 0; i < controlTaskPidKiPe->size(); i++){
        controlData.pidKif[i] = controlTaskPidKiPe->get(i).asDouble();
    }
    Bottle* controlTaskPidKpNe = rf.find("pidKpNe").asList();
    controlData.pidKpb.resize(controlTaskPidKpNe->size(),0);
    for(int i = 0; i < controlTaskPidKpNe->size(); i++){
        controlData.pidKpb[i] = controlTaskPidKpNe->get(i).asDouble();
    }
    Bottle* controlTaskPidKiNe = rf.find("pidKiNe").asList();
    controlData.pidKib.resize(controlTaskPidKiNe->size(),0);
    for(int i = 0; i < controlTaskPidKiNe->size(); i++){
        controlData.pidKib[i] = controlTaskPidKiNe->get(i).asDouble();
    }

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
    controlData.pidMinSatLim = rf.check("pidMinSatLim",Value(-2666.0)).asDouble();
    controlData.pidMaxSatLim = rf.check("pidMaxSatLim",Value(2666.0)).asDouble();
    controlData.controlMode = static_cast<ControlTaskOpMode>(rf.check("controlMode",Value(2)).asInt());
    controlData.pidResetEnabled = rf.check("pidResetEnabled",Value(0)).asInt() != 0;
    controlData.lifespan = rf.check("controlTaskLifespan",Value(10)).asInt();
    controlData.gmmDataStandard = new GMMData(STANDARD);
    controlData.gmmDataObjectInclinedThumbDown = new GMMData(INCLINED_THUMB_DOWN);
    controlData.gmmDataObjectInclinedThumbUp = new GMMData(INCLINED_THUMB_UP);



        //TO UNCOMMENT IF GMM HAS TO BE TESTED - START
/*
        std::vector<int> qIndexes(1);
        qIndexes[0] = 0;

        std::vector<int> rIndexes(2);
        rIndexes[0] = 1; rIndexes[1] = 2;

        controlData.gmmDataObjectInclinedThumbDown->buildQRStructures(qIndexes,rIndexes);


        yarp::sig::Vector queryPoint,output;

        queryPoint.resize(1);

        //for (size_t i = 70; i < 130;i = i + 1){

                queryPoint[0] = i*1.0;
                controlData.gmmDataObjectInclinedThumbDown->runGaussianMixtureRegression(queryPoint,output);
                std::cout << i << ": " << output[0] << "  " << output[1] << "\n";
        }
 
*/



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

    Bottle* approachTaskJoints = rf.find("approach.taskJoints").asList();
    approachData.jointsList.resize(approachTaskJoints->size());
    approachData.fingersList.resize(approachTaskJoints->size(),0);
    for(int i = 0; i < approachTaskJoints->size(); i++){
        approachData.jointsList[i] = approachTaskJoints->get(i).asInt();
        approachData.fingersList[i] = getFingerFromJoint(approachData.jointsList[i]);
    }
    Bottle* approachJointsVelocities = rf.find("approach.jointsVelocities").asList();
    approachData.velocitiesList.resize(approachJointsVelocities->size());
    for(int i = 0; i < approachJointsVelocities->size(); i++){
        approachData.velocitiesList[i] = approachJointsVelocities->get(i).asDouble();
    }
    Bottle* approachJointsPwm = rf.find("approach.jointsPwm").asList();
    approachData.pwmList.resize(approachJointsPwm->size());
    for(int i = 0; i < approachJointsPwm->size(); i++){
        approachData.pwmList[i] = approachJointsPwm->get(i).asDouble();
    }
    Bottle* approachJointsPwmLimits = rf.find("approach.jointsPwmLimits").asList();
    approachData.jointsPwmLimitsList.resize(approachJointsVelocities->size());
    for(int i = 0; i < approachJointsPwmLimits->size(); i++){
        approachData.jointsPwmLimitsList[i] = approachJointsPwmLimits->get(i).asDouble();
    }
    approachData.jointsPwmLimitsEnabled = rf.check("approach.jointsPwmLimitsEnabled",Value(0)).asInt() != 0;
    approachData.lifespan = rf.check("approach.lifespan",Value(5)).asInt();

    /** ICUB RESOURCE FINEDER **/

    yarp::os::ResourceFinder iCubRF;
    iCubRF.setDefaultContext("plantIdentification");
    std::string iCubRFFileName = "iCub_" + whichICub + ".ini";
    iCubRF.setDefaultConfigFile(iCubRFFileName.c_str());
    char **fakeArgV;
    iCubRF.configure(0,fakeArgV,false);

    Bottle* sensitivity = iCubRF.find("sensitivity_" + whichHand).asList();
    commonData.tempParameters[52] = sensitivity->get(0).asDouble();
    commonData.tempParameters[53] = sensitivity->get(1).asDouble();
    commonData.tempParameters[54] = sensitivity->get(2).asDouble();

    commonData.tempParameters[75] = iCubRF.find("thumbAbductionOffset_" + whichHand).asDouble();

    Bottle* pidGains = iCubRF.find("pidGains_" + whichHand).asList();
    commonData.tempParameters[1] = pidGains->get(0).asDouble();
    commonData.tempParameters[2] = pidGains->get(1).asDouble();
    commonData.tempParameters[3] = pidGains->get(2).asDouble();

    Bottle &iCubTaskRF = iCubRF.findGroup(whichTask);

    Bottle* approach = iCubTaskRF.find("approach_" + whichHand).asList();
    commonData.tempParameters[32] = approach->get(0).asInt();
    commonData.tempParameters[33] = approach->get(1).asInt();
    commonData.tempParameters[34] = approach->get(2).asDouble();
    commonData.tempParameters[35] = approach->get(3).asDouble();
    commonData.tempParameters[36] = approach->get(4).asInt();
    commonData.tempParameters[37] = approach->get(5).asDouble();
    commonData.tempParameters[38] = approach->get(6).asDouble();
    commonData.tempParameters[39] = approach->get(7).asInt();

    Bottle* hysteresis = iCubTaskRF.find("hysteresis_" + whichHand).asList();
    commonData.tempParameters[77] = hysteresis->get(0).asInt();
    commonData.tempParameters[78] = hysteresis->get(1).asDouble();
    commonData.tempParameters[79] = hysteresis->get(2).asDouble();
    commonData.tempParameters[80] = hysteresis->get(3).asDouble();
    commonData.tempParameters[81] = hysteresis->get(4).asDouble();
    commonData.tempParameters[82] = hysteresis->get(5).asInt();

    commonData.tempParameters[7] = iCubTaskRF.find("gripStrength_" + whichHand).asDouble();

    Bottle* handJoints = iCubTaskRF.find("handJoints_" + whichHand).asList();
    commonData.handJointsHome.resize(handJoints->size());
    for(int i = 0; i < handJoints->size(); i++){
        commonData.handJointsHome[i] = handJoints->get(i).asDouble();
    }

    /** COMMON RESOURCE FINEDER **/

    yarp::os::ResourceFinder commonRF;
    commonRF.setDefaultContext("plantIdentification");
    std::string commonRFFileName = "confCommon.ini";
    commonRF.setDefaultConfigFile(commonRFFileName.c_str());
    char **fakeArgV;
    commonRF.configure(0,fakeArgV,false);

    Bottle* wholeArmJointsDown = commonRF.find("wholeArmJoints_down").asList();
    commonData.wholeArmJointsDown.resize(wholeArmJointsDown->size());
    for(int i = 0; i < wholeArmJointsDown->size(); i++){
        commonData.wholeArmJointsDown[i] = wholeArmJointsDown->get(i).asDouble();
    }

    /****/

    controllersUtil->buildWholeArmJointsHome(commonData.armJointsHome,commonData.handJointsHome);

    controllersUtil->buildWholeArmJointsDown(commonData.wholeArmJointsDown);


}

int TaskData::getFingerFromJoint(int joint){
    
    //TODO to be modified

    if (joint >= 8 && joint <= 10) return 4;
    if (joint == 11 || joint == 12) return 0;
    if (joint == 13 || joint == 14) return 1;
    if (joint == 15) return 2;
    
    return -1;
}

std::string TaskData::getValueDescription(iCub::plantIdentification::RPCSetCmdArgName cmdName){

    std::stringstream description("");

    switch(cmdName){

    case CTRL_PID_KPF:
        for(size_t i = 0; i < controlData.pidKpf.size(); i++){
            description << controlData.pidKpf[i] << " ";
        }
        break;

    case CTRL_PID_KIF:
        for(size_t i = 0; i < controlData.pidKif.size(); i++){
            description << controlData.pidKif[i] << " ";
        }
        break;

    case CTRL_PID_KPB:
        for(size_t i = 0; i < controlData.pidKpb.size(); i++){
            description << controlData.pidKpb[i] << " ";
        }
        break;

    case CTRL_PID_KIB:
        for(size_t i = 0; i < controlData.pidKib.size(); i++){
            description << controlData.pidKib[i] << " ";
        }
        break;

    case OBJ_DETECT_PRESS_THRESHOLDS:
        for(size_t i = 0; i < commonData.objDetectPressureThresholds.size(); i++){
            description << commonData.objDetectPressureThresholds[i] << " ";
        }
        break;
    
    case APPR_JOINTS_VELOCITIES:
        for(size_t i = 0; i < approachData.velocitiesList.size(); i++){
            description << approachData.velocitiesList[i] << " ";
        }
        break;
    
    case APPR_JOINTS_PWM_LIMITS:
        for(size_t i = 0; i < approachData.jointsPwmLimitsList.size(); i++){
            description << approachData.jointsPwmLimitsList[i] << " ";
        }
        break;
    
    case TEMPORARY_PARAM:
        for(size_t i = 0; i < commonData.tempParameters.size(); i++){
            description << "(" << i << ": " << commonData.tempParameters[i].toString() << ") ";
        }
        break;
    }
    
    return description.str();
}
