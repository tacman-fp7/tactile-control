#include "iCub/plantIdentification/task/Task.h"

#include <iCub/plantIdentification/PlantIdentificationEnums.h>

#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>

#include <yarp/os/Time.h>

#include <ctime>

using iCub::plantIdentification::Task;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;

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
	maxCallsNumber = taskLifespan*1000/commonData->threadRate;
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

	LogData logData;
	buildLogData(logData);

	portsUtil->sendLogData(logData);

	if (callsNumber%commonData->screenLogStride == 0){
		printScreenLog();
	}

	saveProgress();

	if (taskIsOver() && !keepActive){
		release();
		return false;
	}

	return true;
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

	processTactileData();
	
	//TODO define what encoders should be logged
//	controllersUtil->getEncoderAngle(PROXIMAL,&commonData->proximalJointAngle);
//	controllersUtil->getEncoderAngle(DISTAL,&commonData->distalJointAngle);

//	controllersUtil->getRealPwmValue(PROXIMAL,&commonData->realProximalPwm);
//	controllersUtil->getRealPwmValue(DISTAL,&commonData->realDistalPwm);

	return true;
}

void Task::processTactileData(){

	double partialOverallFingerPressure;

	for(size_t i = 0; i < commonData->fingerTaxelsData.size(); i++){
		partialOverallFingerPressure = 0.0;
		for(size_t j = 0; j < commonData->fingerTaxelsData[i].size(); j++){
			partialOverallFingerPressure += commonData->fingerTaxelsData[i][j];
		}
		commonData->overallFingerPressure[i] = partialOverallFingerPressure;

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
	for (size_t i = 0; i < commonData->fingerTaxelsData[fingersList[0]].size(); i++){
		logData.fingerTaxelValues[i] = commonData->fingerTaxelsData[fingersList[0]][i];
	}
	logData.overallFingerPressure = commonData->overallFingerPressure[fingersList[0]];
	logData.overallFingerPressureMedian = commonData->overallFingerPressureMedian[fingersList[0]];

//	logData.proximalJointAngle = commonData->proximalJointAngle;
//	logData.distalJointAngle = commonData->distalJointAngle;
//	logData.realProximalPwm = commonData->realProximalPwm;
//	logData.realDistalPwm = commonData->realDistalPwm;

	logData.pwm = inputCommandValue[0];
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
