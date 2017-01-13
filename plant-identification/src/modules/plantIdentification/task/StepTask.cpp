#include "iCub/plantIdentification/task/StepTask.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <sstream>

using iCub::plantIdentification::StepTask;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::StepTaskData;

StepTask::StepTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,StepTaskData *stepData,std::vector<double> &targetList):Task(controllersUtil,portsUtil,commonData,stepData->lifespan,stepData->jointsList,stepData->fingersList) {

    this->stepData = stepData;
    constantPwm.resize(jointsList.size());
    for(size_t i = 0; i < constantPwm.size(); i++){
        constantPwm[i] = (i >= targetList.size() ? targetList[targetList.size()-1] : targetList[i]);
    }

    taskName = STEP;
    dbgTag = "StepTask: ";
}

void StepTask::init(){
    using std::cout;

    controllersUtil->setTaskControlModes(jointsList,VOCAB_CM_OPENLOOP);

    cout << "\n\n" << dbgTag << "TASK STARTED - Target: ";
    for(size_t i = 0; i < constantPwm.size(); i++){
        cout << constantPwm[i] << " ";
    }
    cout << "\n\n";

    //TODO TO BE REMOVED
    pinkyAngleReference = commonData->armEncodersAngles[15];

}

void StepTask::calculateControlInput(){

    bool forceSensorReadingEnabled = commonData->tpInt(70) != 0;


    for(size_t i = 0; i < constantPwm.size(); i++){
        inputCommandValue[i] = constantPwm[i];
    }

    // if forceSensorReading is enabled, log the processed values
    if (forceSensorReadingEnabled == true){
        if (callsNumber%commonData->screenLogStride == 0){
            std::stringstream printLog("");
            printLog << " [F " << commonData->procForceSensorData[0] << " - " << commonData->procForceSensorData[1] << " - " << commonData->procForceSensorData[2] << " - " << commonData->procForceSensorData[3] << " - " << commonData->procForceSensorData[4] << " - " << commonData->procForceSensorData[5] << "]";
            optionalLogString.append(printLog.str());
        }
    }

    //TODO TO BE REMOVED!
    pinkyAngleReference = 45;
    if (commonData->tpInt(15) != 0){
        if (inputCommandValue.size() == 3){
            double pinkyEnc = commonData->armEncodersAngles[15];
            double diff = pinkyEnc - pinkyAngleReference;
            inputCommandValue[0] = constantPwm[0] - diff*10;
            inputCommandValue[1] = constantPwm[1] + diff*5;
            inputCommandValue[2] = constantPwm[2] + diff*5;
            
            if (callsNumber%commonData->screenLogStride == 0){
                std::stringstream printLog("");
                printLog << "[ref " << pinkyAngleReference << " pinky " << pinkyEnc << " diff " << diff << "]";
                optionalLogString.append(printLog.str());
            }

            for(size_t i = 0; i < inputCommandValue.size(); i++){
                //if (inputCommandValue[i]<0) inputCommandValue[i]=0;
            }

        }
    }
}

void StepTask::buildLogData(LogData &logData){

    addCommonLogData(logData);

    logData.taskType = STEP;
    logData.taskOperationMode = 0;
    // TODO it should be jointsList
    for(size_t i = 0; i < fingersList.size(); i++){
        logData.targetValue[i] = constantPwm[i];
    }
}

std::string StepTask::getConstantPwmDescription(){

    std::stringstream description("");

    for(size_t i = 0; i < constantPwm.size(); i++){
        description << constantPwm[i] << " ";
    }
    
    return description.str();
}
