#include "iCub/plantIdentification/task/Task.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"
#include "iCub/plantIdentification/util/ICubUtil.h"

#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>

#include <yarp/os/Time.h>

#include <ctime>

using iCub::plantIdentification::Task;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::ICubUtil;

Task::Task(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,int taskLifespan,std::vector<int> &jointsList,std::vector<int> &fingersList){
	
	this->controllersUtil = controllersUtil;
	this->portsUtil = portsUtil;
	this->commonData = commonData;
	this->jointsList.resize(jointsList.size());
	for(size_t i = 0; i < jointsList.size(); i++){
		this->jointsList[i] = jointsList[i];
	}
	this->fingersList.resize(fingersList.size());
	for(size_t i = 0; i < fingersList.size(); i++){
		this->fingersList[i] = fingersList[i];
	}
	inputCommandValue.resize(jointsList.size(),0.0);

	isFirstCall = true;
	callsNumber = 0;
	maxCallsNumber = secondsToCallsNumber(static_cast<double>(taskLifespan));
    isClean = false;
	optionalLogString = "";
}

void Task::createTaskId(){

	time_t now = time(0);
	tm *ltm = localtime(&now);
	char myDate[15];
	strftime(myDate,15,"%m%d%H%M%S",ltm);
	
	taskId = std::string(myDate);
}


bool Task::manage(bool keepActive){
	
	if (isFirstCall){
		createTaskId();
		init();
		isFirstCall = false;
	}

	loadICubData();

	calculateControlInput();

	sendCommands();

    LogData logData(fingersList.size());
	buildLogData(logData);

	portsUtil->sendLogData(logData);
	portsUtil->sendInfoData(commonData);

	if (callsNumber%commonData->screenLogStride == 0){
		printScreenLog();
	}

	saveProgress();

	if (taskIsOver() && !keepActive){
		release();
        isClean = true;
		return false;
	}

	return true;
}

void Task::clean(){

    if (!isClean){
        release();
        isClean = true;
    }

}

void Task::sendCommands(){

	for(size_t i = 0; i < inputCommandValue.size(); i++){
		controllersUtil->sendPwm(jointsList[i],commonData->pwmSign*inputCommandValue[i]);
	}
}

bool Task::loadICubData(){
	using yarp::sig::Vector;

	if (!portsUtil->readFingerSkinCompData(commonData->fingerTaxelsData)){
		return false;
	}

	controllersUtil->getArmEncodersAngles(commonData->armEncodersAngles);

	processTactileData();
	
    // index finger
    controllersUtil->getEncoderAngle(11,&commonData->proximalJointAngle[0]);
    controllersUtil->getEncoderAngle(12,&commonData->distalJointAngle[0]);
//    controllersUtil->getRealPwmValue(11,&commonData->realProximalPwm[0]);
//    controllersUtil->getRealPwmValue(12,&commonData->realDistalPwm[0]);

    // middle finger
    controllersUtil->getEncoderAngle(13,&commonData->proximalJointAngle[1]);
    controllersUtil->getEncoderAngle(14,&commonData->distalJointAngle[1]);
//    controllersUtil->getRealPwmValue(13,&commonData->realProximalPwm[1]);
//    controllersUtil->getRealPwmValue(14,&commonData->realDistalPwm[1]);

    // TODO ring finger

    // TODO pinky

    // thumb
    controllersUtil->getEncoderAngle(9,&commonData->proximalJointAngle[4]);
    controllersUtil->getEncoderAngle(10,&commonData->distalJointAngle[4]);
//    controllersUtil->getRealPwmValue(9,&commonData->realProximalPwm[4]);
//    controllersUtil->getRealPwmValue(10,&commonData->realDistalPwm[4]);

	return true;
}

void Task::processTactileData(){

	double partialOverallFingerPressure;

	for(size_t i = 0; i < commonData->fingerTaxelsData.size(); i++){
		
		commonData->overallFingerPressureBySimpleSum[i] = ICubUtil::getForce(commonData->fingerTaxelsData[i],SIMPLE_SUM);
		commonData->overallFingerPressureByWeightedSum[i] = ICubUtil::getForce(commonData->fingerTaxelsData[i],WEIGHTED_SUM);
		
		/*partialOverallFingerPressure = 0.0;
		for(size_t j = 0; j < commonData->fingerTaxelsData[i].size(); j++){
			partialOverallFingerPressure += commonData->fingerTaxelsData[i][j];
		}*/
		
		commonData->overallFingerPressure[i] = commonData->overallFingerPressureByWeightedSum[i];

		commonData->previousOverallFingerPressures[i][commonData->previousPressuresIndex[i]] = commonData->overallFingerPressure[i];
		commonData->previousPressuresIndex[i] = (commonData->previousPressuresIndex[i] + 1)%commonData->previousOverallFingerPressures[i].size();

		std::vector<double> previousOverallFingerPressuresCopy(commonData->previousOverallFingerPressures[i]);

		gsl_sort(&previousOverallFingerPressuresCopy[0],1,previousOverallFingerPressuresCopy.size());
		commonData->overallFingerPressureMedian[i] = gsl_stats_median_from_sorted_data(&previousOverallFingerPressuresCopy[0],1,previousOverallFingerPressuresCopy.size());

	}

}

void Task::buildLogData(LogData &logData){

	addCommonLogData(logData);
}

void Task::addCommonLogData(LogData &logData){

	logData.taskId = taskId;
	//TODO only the first element is logged!
    for (size_t i = 0; i < fingersList.size(); i++){
        for (size_t j = 0; j < commonData->fingerTaxelsData[fingersList[i]].size(); j++){
            logData.fingerTaxelValues[i][j] = commonData->fingerTaxelsData[fingersList[i]][j];
        }
        logData.overallFingerPressure[i] = commonData->overallFingerPressure[fingersList[i]];
        logData.overallFingerPressureMedian[i] = commonData->overallFingerPressureMedian[fingersList[i]];

        logData.proximalJointAngle[i] = commonData->proximalJointAngle[fingersList[i]];
        logData.distalJointAngle[i] = commonData->distalJointAngle[fingersList[i]];
//        logData.realProximalPwm[i] = commonData->realProximalPwm[fingersList[i]];
//        logData.realDistalPwm[i] = commonData->realDistalPwm[fingersList[i]];


        // TODO inputCommandValue has actually 'jointsList.size' values, but it works if I just command proximals joints
        logData.pwm[i] = inputCommandValue[i];
    }
}

void Task::printScreenLog(){
	using std::cout;
	
	cout << dbgTag << "Sum: ";
	
	for(size_t i = 0; i < fingersList.size(); i++){
		cout << commonData->overallFingerPressure[fingersList[i]] << "(" << fingersList[i] << ") ";
	}
	cout << "\t   Pwm: ";

	for(size_t i = 0; i < jointsList.size(); i++){
		cout << inputCommandValue[i] << "(" << jointsList[i] << ") ";
	}
	
	cout << optionalLogString << "\n";

	optionalLogString.clear();
}

void Task::saveProgress(){

	callsNumber++;
}

bool Task::taskIsOver(){

	return callsNumber >= maxCallsNumber;
}

int Task::secondsToCallsNumber(double seconds){

	return static_cast<int>(seconds*1000/commonData->threadRate);
}