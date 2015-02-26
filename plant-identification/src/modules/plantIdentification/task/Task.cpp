#include "iCub/plantIdentification/task/Task.h"

#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>

using iCub::plantIdentification::Task;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;

Task::Task(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,int taskLifespan){
	
	this->controllersUtil = controllersUtil;
	this->portsUtil = portsUtil;
	this->commonData = commonData;

	isFirstCall = true;
	callsNumber = 0;
	maxCallsNumber = taskLifespan*1000/commonData->threadRate;

}

bool Task::manage(bool keepActive){

	if (isFirstCall){
		init();
		isFirstCall = false;
	}

	loadICubData();

	calculatePwm();

	controllersUtil->sendPwm(commonData->pwmSign*pwmToUse);
	
	LogData logData;
	buildLogData(logData);

	portsUtil->sendLogData(logData);

	saveProgress();

	if (isOver() && !keepActive){
		release();
		return false;
	}

	return true;
}

void Task::loadICubData(){
	using yarp::sig::Vector;

	portsUtil->readFingerSkinCompData(commonData->fingerToMove,commonData->fingerTaxelsData);
	
	//TODO move this section in another utility class
	double partialOverallFingerPressure = 0.0;
	for(size_t i = 0; i < commonData->fingerTaxelsData.size(); i++){
		partialOverallFingerPressure += commonData->fingerTaxelsData[i];
	}
	commonData->overallFingerPressure = partialOverallFingerPressure;

	commonData->previousOverallFingerPressures[commonData->previousPressuresIndex] = commonData->overallFingerPressure;
	commonData->previousPressuresIndex++;

	gsl_sort(&commonData->previousOverallFingerPressures[0],1,commonData->previousOverallFingerPressures.size());
	commonData->overallFingerPressureMedian = gsl_stats_median_from_sorted_data(&commonData->previousOverallFingerPressures[0],1,commonData->previousOverallFingerPressures.size());
	//TODO end section

	controllersUtil->getProximalEncoderAngle(&commonData->proximalJointAngle);
	controllersUtil->getDistalEncoderAngle(&commonData->distalJointAngle);
	controllersUtil->getProximalRealPwmValue(&commonData->realProximalPwm);
	controllersUtil->getDistalRealPwmValue(&commonData->realDistalPwm);
}

void Task::buildLogData(LogData &logData){

	addCommonLogData(logData);
}

void Task::addCommonLogData(LogData &logData){

	for (size_t i = 0; i < commonData->fingerTaxelsData.size(); i++){
		logData.fingerTaxelValues[i] = commonData->fingerTaxelsData.size();
	}
	logData.overallFingerPressure = commonData->overallFingerPressure;
	logData.overallFingerPressureMedian = commonData->overallFingerPressureMedian;

	logData.proximalJointAngle = commonData->proximalJointAngle;
	logData.distalJointAngle = commonData->distalJointAngle;
	logData.realProximalPwm = commonData->realProximalPwm;
	logData.realDistalPwm = commonData->realDistalPwm;

	logData.pwm = pwmToUse;

}

void Task::saveProgress(){

	callsNumber++;
}

bool Task::isOver(){

	return callsNumber >= maxCallsNumber;
}
