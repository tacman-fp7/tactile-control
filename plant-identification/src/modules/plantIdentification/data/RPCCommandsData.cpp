#include "iCub/plantIdentification/data/RPCCommandsData.h"

#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>

using iCub::plantIdentification::RPCCommandsData;
using iCub::plantIdentification::RPCMainCmdName;
using iCub::plantIdentification::RPCSetCmdArgName;
using iCub::plantIdentification::RPCTaskCmdArgName;
using iCub::plantIdentification::RPCViewCmdArgName;
using iCub::plantIdentification::TaskName;

using std::string;
using std::pair;





// TO UNCOMMENT IF NEURAL NETWORKS HAVE TO BE TESTED
#include <yarp/os/Property.h>
#include "iCub/plantIdentification/util/ICubUtil.h"
#include <yarp/os/Bottle.h>
#include <iCub/ctrl/neuralNetworks.h>
#include <yarp/sig/Vector.h>
using iCub::plantIdentification::ICubUtil;
using yarp::sig::Vector;

RPCCommandsData::RPCCommandsData(){

	add("help",HELP,"THIS HELP");
	add("set",SET,"SET PARAMETERS (usage: 'set <paramName> <paramValue>')");
	add("task",TASK,"MANAGE TASKS (usage: 'task [pop | empty | add <taskType> <targetValue>]')");
	add("view",VIEW,"VIEW PARAMETERS/TASKS (usage: 'view [set | tasks]')");
	add("start",START,"START TASKS");
	add("stop",STOP,"STOP TASKS");
	add("open",OPEN,"STOP TASKS AND OPEN HAND");
	add("arm",ARM,"set ARM IN TASK POSITION");
	add("quit",QUIT,"QUIT MODULE");

	add("pwm_sign",PWM_SIGN,"PWM SIGN");
	add("obj_det_thresholds",OBJ_DETECT_PRESS_THRESHOLDS,"OBJECT DETECTION PRESSURE THRESHOLDS");
	add("tmp_par",TEMPORARY_PARAM,"TEMPORARY PARAMETER (usage: 'set tmp_par <n>_<value>'");
	add("step_joints",STEP_JOINTS_LIST,"STEP TASK JOINTS LIST");
	add("step_ls",STEP_LIFESPAN,"STEP TASK LIFESPAN");
	add("ctrl_joints",CTRL_JOINTS_LIST,"CONTROL TASK JOINTS LIST");
	add("kp_pe",CTRL_PID_KPF,"CONTROL PID Kp (error >= 0)");
	add("ki_pe",CTRL_PID_KIF,"CONTROL PID Ki (error >= 0)");
	add("kp_ne",CTRL_PID_KPB,"CONTROL PID Kp (error < 0)");
	add("ki_ne",CTRL_PID_KIB,"CONTROL PID Ki (error < 0)");
	add("ctrl_op_mode",CTRL_OP_MODE,"CONTROL OPERATION MODE [0: err >= 0; 1: err < 0; 2: both]");
	add("ctrl_pid_reset",CTRL_PID_RESET_ENABLED,"PID RESET ENABLED [0: false; 1: true]");
	add("ctrl_target_rt",CTRL_TARGET_REAL_TIME,"CONTROL TARGET (CHANGED IN REAL TIME IF A CONTROL TASK IS RUNNING)");
	add("ctrl_ls",CTRL_LIFESPAN,"CONTROL TASK LIFESPAN");
	add("slope",RAMP_SLOPE,"RAMP SLOPE");
	add("intercept",RAMP_INTERCEPT,"RAMP INTERCEPT");
	add("ramp_joints",RAMP_JOINTS_LIST,"RAMP TASK JOINTS LIST");
	add("ramp_ls",RAMP_LIFESPAN,"RAMP TASK LIFESPAN");
	add("ramp_ls_after_stab",RAMP_LIFESPAN_AFTER_STAB,"RAMP TASK LIFESPAN AFTER STABILIZATION");
	add("appr_joints",APPR_JOINTS_LIST,"APPROACH TASK JOINTS LIST");
	add("appr_vel",APPR_JOINTS_VELOCITIES,"APPROACH TASK JOINTS VELOCITIES");
	add("appr_pwm_lim",APPR_JOINTS_PWM_LIMITS,"APPROACH TASK JOINTS PWM LIMITS");
	add("appr_pwm_lim_enabl",APPR_JOINTS_PWM_LIMITS_ENABLED,"APPROACH TASK JOINTS PWM LIMITS ENABLED [0: false; 1: true]");
	add("appr_ls",APPR_LIFESPAN,"APPROACH TASK LIFESPAN");

	add("add",ADD,"add");
	add("empty",EMPTY,"empty");
	add("pop",POP,"pop");

	add("step",STEP,"STEP TASK");
	add("ctrl",CONTROL,"CONTROL TASK");
	add("apprctrl",APPROACH_AND_CONTROL,"APPROACH & CONTROL TASK");
	add("appr",APPROACH,"APPROACH TASK");
	add("ramp",RAMP,"RAMP TASK");

	add("set",SETTINGS,"SETTINGS");
	add("tasks",TASKS,"TASKS");

	dbgTag = "RPCCommandsData: ";
	
}

void RPCCommandsData::add(string rpcLabel,RPCMainCmdName enumLabel,string description){
	mainCmdMap.insert(std::pair<RPCMainCmdName,string>(enumLabel,rpcLabel));
	mainCmdDescMap.insert(std::pair<RPCMainCmdName,string>(enumLabel,description));
	mainCmdRevMap.insert(std::pair<string,RPCMainCmdName>(rpcLabel,enumLabel));
}

void RPCCommandsData::add(string rpcLabel,RPCSetCmdArgName enumLabel,string description){
	setCmdArgMap.insert(std::pair<RPCSetCmdArgName,string>(enumLabel,rpcLabel));
	setCmdArgDescMap.insert(std::pair<RPCSetCmdArgName,string>(enumLabel,description));
	setCmdArgRevMap.insert(std::pair<string,RPCSetCmdArgName>(rpcLabel,enumLabel));
}

void RPCCommandsData::add(string rpcLabel,RPCTaskCmdArgName enumLabel,string description){
	taskCmdArgMap.insert(pair<RPCTaskCmdArgName,string>(enumLabel,rpcLabel));
	taskCmdArgDescMap.insert(pair<RPCTaskCmdArgName,string>(enumLabel,description));
	taskCmdArgRevMap.insert(pair<string,RPCTaskCmdArgName>(rpcLabel,enumLabel));
}

void RPCCommandsData::add(string rpcLabel,RPCViewCmdArgName enumLabel,string description){
	viewCmdArgMap.insert(pair<RPCViewCmdArgName,string>(enumLabel,rpcLabel));
	viewCmdArgDescMap.insert(pair<RPCViewCmdArgName,string>(enumLabel,description));
	viewCmdArgRevMap.insert(pair<string,RPCViewCmdArgName>(rpcLabel,enumLabel));
}

void RPCCommandsData::add(string rpcLabel,TaskName enumLabel,string description){
	taskMap.insert(pair<TaskName,string>(enumLabel,rpcLabel));
	taskDescMap.insert(pair<TaskName,string>(enumLabel,description));
	taskRevMap.insert(pair<string,TaskName>(rpcLabel,enumLabel));
}

std::string RPCCommandsData::getFullDescription(RPCSetCmdArgName setCmdArgName){

	return setCmdArgDescMap[setCmdArgName] + " ('" + setCmdArgMap[setCmdArgName] + "')";
}

std::string RPCCommandsData::getFullDescription(RPCMainCmdName mainCmdName){

	return "'" + mainCmdMap[mainCmdName] + "' - " + mainCmdDescMap[mainCmdName];
}

void RPCCommandsData::setValues(yarp::os::Value &value,std::vector<double> &valueList){

    if (value.isString()){

        std::string values = value.asString();

	    valueList.resize(0);

	    char *valuesChar = new char[values.length() + 1];
	    strcpy(valuesChar,values.c_str());
	    char *target;

	    target = strtok(valuesChar,"_");

        while(target != NULL){
            valueList.push_back(atof(target));
            target = strtok(NULL,"_");
        }
    } else {

        valueList.push_back(value.asDouble());
    }





    //TO UNCOMMENT IF NEURAL NETWORKS HAVE TO BE TESTED
	// create the neural network and configure it
    iCub::ctrl::ff2LayNN_tansig_purelin neuralNetwork;
	yarp::os::Property nnConfProperty;
	yarp::os::Bottle nnConfBottle;
	if (valueList.size() == 2){
		ICubUtil::getNNOptionsForErrorPrediction2Fingers(nnConfBottle);
	} else {
		ICubUtil::getNNOptionsForErrorPrediction3Fingers(nnConfBottle);
	}
	nnConfProperty.fromString(nnConfBottle.toString());
	neuralNetwork.configure(nnConfProperty);
	if (valueList.size() == 2){
		// using the neural network
		std::vector<double> actualAngles(2);
		std::vector<double> rotatedAngles;
		actualAngles[0] = valueList[0];
		actualAngles[1] = valueList[1];
		ICubUtil::rotateFingersData(actualAngles,rotatedAngles);
		Vector rotatedAnglesVector;
		rotatedAnglesVector.resize(2);
		rotatedAnglesVector[0] = rotatedAngles[0];
		rotatedAnglesVector[1] = rotatedAngles[1];
		Vector estimatedBestPositionNNVector = neuralNetwork.predict(rotatedAnglesVector);
		double estimatedFinalPose = estimatedBestPositionNNVector[0];
		std::cout << "INPUT: " << valueList[0] << "  " << valueList[1] << "\n"; 
		std::cout << "ESTIMATED POSE <<<<<<<   " << estimatedFinalPose << "   >>>>>>>\n";

	} else {

		// using the neural network
		std::vector<double> actualAngles(3);
		std::vector<double> rotatedAngles;
		actualAngles[0] = valueList[0];
		actualAngles[1] = valueList[1];
		actualAngles[2] = valueList[2];
		ICubUtil::rotateFingersData(actualAngles,rotatedAngles);
		Vector rotatedAnglesVector;
		rotatedAnglesVector.resize(3);
		rotatedAnglesVector[0] = rotatedAngles[0];
		rotatedAnglesVector[1] = rotatedAngles[1];
		rotatedAnglesVector[2] = rotatedAngles[2];
		Vector estimatedBestPositionNNVector = neuralNetwork.predict(rotatedAnglesVector);
		double estimatedFinalPose = estimatedBestPositionNNVector[0];
		std::cout << "INPUT: " << valueList[0] << "  " << valueList[1] << "  " << valueList[2] << "\n"; 
		std::cout << "ESTIMATED POSE <<<<<<<   " << estimatedFinalPose << "   >>>>>>>\n";

    }


}

bool RPCCommandsData::setTemporaryParam(yarp::os::Value &value,std::vector<yarp::os::Value> &valueList){
	using yarp::os::Value;

    std::string values = value.asString();
	double tempParamIndex;
	Value tempParamValue;

	char *valuesChar = new char[values.length() + 1];
	strcpy(valuesChar,values.c_str());
	char *target;

	target = strtok(valuesChar,"_");

    if (target != NULL){
        tempParamIndex = atoi(target);
        std::cout << tempParamIndex << " " << valueList.size() << "\n";
		if (tempParamIndex < valueList.size()){
			target = strtok(NULL,"_");

			if (target != NULL){
				if (values.find('.') != string::npos){
					tempParamValue = Value(atof(target));
				} else if (values.find('#') != string::npos){
					tempParamValue = Value(target);
				} else {
					tempParamValue = Value(atoi(target));
				}

				valueList[tempParamIndex] = tempParamValue;

				return true;
			}

		} 

	}

	return false;

}


std::string RPCCommandsData::printValue(yarp::os::Value &value){

	std::stringstream valueString("");

	if (value.isDouble()) valueString << value.asDouble();
	else if (value.isInt()) valueString << value.asInt();
	else valueString << value.asString();

	return valueString.str();
}
