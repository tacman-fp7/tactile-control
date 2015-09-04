#include "iCub/plantIdentification/thread/TaskThread.h"
#include "iCub/plantIdentification/task/StepTask.h"
#include "iCub/plantIdentification/task/ControlTask.h"
#include "iCub/plantIdentification/task/ApproachTask.h"
#include "iCub/plantIdentification/task/RampTask.h"

#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <iostream>
#include <sstream>
#include <algorithm>
#include <ctime>
#include <cmath>

//TODO TO REMOVE!
#include "iCub/plantIdentification/util/ICubUtil.h"
#include <yarp/sig/Vector.h>
#include <iCub/ctrl/neuralNetworks.h>


using std::cerr;
using std::cout;
using std::string;

using iCub::plantIdentification::TaskThread;
using iCub::plantIdentification::RPCSetCmdArgName;
using iCub::plantIdentification::RPCTaskCmdArgName;
using iCub::plantIdentification::TaskName;
using iCub::plantIdentification::RPCViewCmdArgName;
using iCub::plantIdentification::RPCCommandsData;

using yarp::os::RateThread;
using yarp::os::Value;


/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
TaskThread::TaskThread(const int period, const yarp::os::ResourceFinder &aRf) 
    : RateThread(period) {
		this->period = period;
        rf = aRf;

        dbgTag = "TaskThread: ";
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Destructor                                                       ********************************************** */   
TaskThread::~TaskThread() {}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Initialise thread                                                ********************************************** */
bool TaskThread::threadInit() {
    using yarp::os::Property;
    using yarp::os::Network;
    using yarp::os::Bottle;
    using yarp::os::Time;
    using std::vector;

	cout << dbgTag << "Initialising. \n";

	currentTaskIndex = 0;

	// initialize controllers
	controllersUtil = new ControllersUtil();
	if (!controllersUtil->init(rf)) {
        cout << dbgTag << "failed to initialize controllers utility\n";
        return false;
    }

	// initialize ports
	portsUtil = new PortsUtil();
	if (!portsUtil->init(rf)) {
        cout << dbgTag << "failed to initialize ports utility\n";
        return false;
    }
	// initialize task data
	taskData = new TaskData(rf,period,controllersUtil);

	// save current arm position, to be restored when the thread ends
//	if (!controllersUtil->saveCurrentArmPosition()) {
//        cout << dbgTag << "failed to store current arm position\n";
//        return false;
//    }
//	if (!controllersUtil->setArmInTaskPosition()) {
//        cout << dbgTag << "failed to set arm in task position\n";
//        return false;
//    }
 
	// this prevent the run() method to be executed between the taskThread->start() and the taskThread->suspend() calls during the PlantIdentificationModule initialization
	runEnabled = false;

    cout << dbgTag << "Initialised correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


bool TaskThread::initializeGrasping(){

	if (!controllersUtil->saveCurrentControlMode()) return false;
	runEnabled = true;

	return true;
}


/* *********************************************************************************************************************** */
/* ******* Run thread                                                       ********************************************** */
void TaskThread::run() {

	if (runEnabled){

		if (currentTaskIndex < taskList.size()){

			// if it is the last task of the list, keep it active until further tasks are added
			bool keepActive = (currentTaskIndex == taskList.size() - 1);
			bool taskIsActive;

			taskIsActive = taskList[currentTaskIndex]->manage(keepActive);

			if (!taskIsActive){
				currentTaskIndex++;
			}
		}
	}
}

bool TaskThread::afterRun(bool openHand){

	if (!controllersUtil->restorePreviousControlMode()) return false;
	if (openHand) {
		if (!controllersUtil->openHand()) return false;
	}
	runEnabled = false;
    currentTaskIndex = 0;

	// clear task list
	for (size_t i = 0; i < taskList.size(); i++){
		delete(taskList[i]);
	}
	taskList.clear();

	return true;
}

/* *********************************************************************************************************************** */
/* ******* Release thread                                                   ********************************************** */
void TaskThread::threadRelease() {
    cout << dbgTag << "Releasing. \n";
    
    // closing ports
	portsUtil->release();

//    controllersUtil->restorePreviousArmPosition();
	controllersUtil->release();

	delete(portsUtil);
	delete(controllersUtil);
	delete(taskData);

    cout << dbgTag << "Released. \n";
}
/* *********************************************************************************************************************** */

bool TaskThread::setArmInTaskPosition(){

	if (!controllersUtil->setArmInTaskPosition()) {
        cout << dbgTag << "failed to set arm in task position\n";
        return false;
    }

    return true;

}

void TaskThread::set(RPCSetCmdArgName paramName,Value paramValue,RPCCommandsData &rpcCmdData){

	bool setSuccessful = true;

	switch (paramName){

	// common data
	case PWM_SIGN:
		taskData->commonData.pwmSign = paramValue.asInt();
		break;
	case OBJ_DETECT_PRESS_THRESHOLDS:
		rpcCmdData.setValues(paramValue,taskData->commonData.objDetectPressureThresholds);
		break;
	case TEMPORARY_PARAM:
		if (!rpcCmdData.setTemporaryParam(paramValue,taskData->commonData.tempParameters)){
			setSuccessful = false;
			cout << "\n\n" << "CANNOT SET TEMPORARY PARAM" << "\n";
		}
		break;

	// step task data
	case STEP_LIFESPAN:
		taskData->stepData.lifespan = paramValue.asDouble();
		break;

	// control task data
	case CTRL_PID_KPF:
		rpcCmdData.setValues(paramValue,taskData->controlData.pidKpf);
		break;
	case CTRL_PID_KIF:
		rpcCmdData.setValues(paramValue,taskData->controlData.pidKif);
		break;
	case CTRL_PID_KPB:
		rpcCmdData.setValues(paramValue,taskData->controlData.pidKpb);
		break;
	case CTRL_PID_KIB:
		rpcCmdData.setValues(paramValue,taskData->controlData.pidKib);
		break;
	case CTRL_PID_RESET_ENABLED:
		taskData->controlData.pidResetEnabled = paramValue.asInt() != 0;
		break;
	case CTRL_OP_MODE:
		taskData->controlData.controlMode = static_cast<ControlTaskOpMode>(paramValue.asInt());
		break;
	case CTRL_TARGET_REAL_TIME:
		if (currentTaskIndex < taskList.size()){
			if (ControlTask* currentTask = dynamic_cast<ControlTask*>(taskList[currentTaskIndex])){
				std::vector<double> targetList;
				rpcCmdData.setValues(paramValue,targetList);
				currentTask->setTargetListRealTime(targetList);
			} else {
				setSuccessful = false;
				cout << "\n\n" << "CANNOT EXECUTE SET COMMAND (CONTROL TASK IS NOT RUNNING)" << "\n";
			}
		} else {
			setSuccessful = false;
			cout << "\n\n" << "CANNOT EXECUTE SET COMMAND (NO TASK RUNNING)" << "\n";
		}
		break;
	case CTRL_LIFESPAN:
		taskData->controlData.lifespan = paramValue.asInt();
		break;

	// ramp task data
	case RAMP_SLOPE:
		taskData->rampData.slope = paramValue.asDouble();
		break;
	case RAMP_INTERCEPT:
		taskData->rampData.intercept = paramValue.asDouble();
		break;
	case RAMP_LIFESPAN:
		taskData->rampData.lifespan = paramValue.asInt();
		break;
	case RAMP_LIFESPAN_AFTER_STAB:
		taskData->rampData.lifespanAfterStabilization = paramValue.asInt();
		break;

	// approach task data
	case APPR_JOINTS_VELOCITIES:
		rpcCmdData.setValues(paramValue,taskData->approachData.velocitiesList);
		break;
	case APPR_JOINTS_PWM_LIMITS:
		rpcCmdData.setValues(paramValue,taskData->approachData.jointsPwmLimitsList);
		break;
	case APPR_JOINTS_PWM_LIMITS_ENABLED:
		taskData->approachData.jointsPwmLimitsEnabled = paramValue.asInt() != 0;
		break;
	case APPR_LIFESPAN:
		taskData->approachData.lifespan = paramValue.asInt();
		break;


	}

	if (setSuccessful){
		cout << "\n" <<
				"\n" << 
				"'" << rpcCmdData.setCmdArgMap[paramName] << "' SET TO " << rpcCmdData.printValue(paramValue) << "\n";
	}
}

void TaskThread::task(RPCTaskCmdArgName paramName,TaskName taskName,Value paramValue,RPCCommandsData &rpcCmdData){

	std::vector<double> targetList;


	switch (paramName){

	case ADD:

		rpcCmdData.setValues(paramValue,targetList);

		switch (taskName){
		case STEP:
			taskList.push_back(new StepTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->stepData,targetList));
			break;

		case CONTROL:
			taskList.push_back(new ControlTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->controlData,targetList));
			break;

		case APPROACH_AND_CONTROL:
			taskList.push_back(new ControlTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->controlData,targetList,true));
			break;

		case APPROACH:
			taskList.push_back(new ApproachTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->approachData));
			break;

		case RAMP:
			taskList.push_back(new RampTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->rampData,targetList));
			break;
		}
		cout << "\n" <<
				"\n" << 
				"ADDED '" << rpcCmdData.taskMap[taskName] << "' TASK" << "\n";
		break;

	case EMPTY:
		for (size_t i = 0; i < taskList.size(); i++){
			delete(taskList[i]);
		}
		taskList.clear();
		cout << "\n" <<
				"\n" << 
				"'" << "TASK LIST CLEARED" << "\n";
		break;

	case POP:
		delete(taskList[taskList.size() - 1]);
		taskList.pop_back();
		cout << "\n" <<
				"\n" << 
				"'" << "LAST TASK REMOVED" << "\n";
		break;
	}



	// TODO TO REMOVE!!!!!!!
	//if (targetList.size() == 2){
	//	iCub::ctrl::ff2LayNN_tansig_purelin neuralNetwork;
	//	yarp::os::Property nnConfProperty;
	//	yarp::os::Bottle nnConfBottle;
	//	ICubUtil::getNNOptionsForErrorPrediction2Fingers(nnConfBottle);
	//	nnConfProperty.fromString(nnConfBottle.toString());
	//	neuralNetwork.configure(nnConfProperty);

	//	std::cout << "Bottle: " << nnConfBottle.toString() << "\n";
	//	std::cout << "Property: " << nnConfProperty.toString() << "\n";
	//	std::cout << "b1: " << neuralNetwork.get_b1().toString() << "\n";
	//	std::cout << "b2: " << neuralNetwork.get_b2().toString() << "\n";
	//	
	//	std::deque<yarp::sig::Vector,std::allocator<yarp::sig::Vector> > iwDeque = neuralNetwork.get_IW();
	//	for (int i=0;i<iwDeque.size();i++){
	//		yarp::sig::Vector tmp = iwDeque.at(i);
	//		std::cout << "IW" << i << ": " << tmp.toString() << "\n";
	//	}
	//	std::deque<yarp::sig::Vector,std::allocator<yarp::sig::Vector> > lwDeque = neuralNetwork.get_LW();
	//	for (int i=0;i<lwDeque.size();i++){
	//		yarp::sig::Vector tmp = lwDeque.at(i);
	//		std::cout << "LW" << i << ": " << tmp.toString() << "\n";
	//	}

	//	std::vector<double> actualAngles(2);
	//	std::vector<double> rotatedAngles;
	//	actualAngles[0] = targetList[0];
	//	actualAngles[1] = targetList[1];
	//	iCub::plantIdentification::ICubUtil::rotateFingersData(actualAngles,rotatedAngles);

	//	std::cout << "Actual angles: " << actualAngles[0] << " " << actualAngles[1] << "\n";
	//	std::cout << "Rotated angles: " << rotatedAngles[0] << " " << rotatedAngles[1] << "\n";

	//	yarp::sig::Vector rotatedAnglesVector;
	//	rotatedAnglesVector.resize(2);
	//	rotatedAnglesVector[0] = rotatedAngles[0];
	//	rotatedAnglesVector[1] = rotatedAngles[1];
	//	yarp::sig::Vector svErrNNVector = neuralNetwork.predict(rotatedAnglesVector);
	//	double svErrNN = svErrNNVector[0];

	//	std::cout << "Error: " << svErrNN << "\n";

	//} else if (targetList.size() == 3){
	//	iCub::ctrl::ff2LayNN_tansig_purelin neuralNetwork;
	//	yarp::os::Property nnConfProperty;
	//	yarp::os::Bottle nnConfBottle;
	//	ICubUtil::getNNOptionsForErrorPrediction3Fingers(nnConfBottle);
	//	nnConfProperty.fromString(nnConfBottle.toString());
	//	neuralNetwork.configure(nnConfProperty);

	//	std::cout << "Bottle: " << nnConfBottle.toString() << "\n";
	//	std::cout << "Property: " << nnConfProperty.toString() << "\n";
	//	std::cout << "b1: " << neuralNetwork.get_b1().toString() << "\n";
	//	std::cout << "b2: " << neuralNetwork.get_b2().toString() << "\n";

	//	std::deque<yarp::sig::Vector,std::allocator<yarp::sig::Vector> > iwDeque = neuralNetwork.get_IW();
	//	for (int i=0;i<iwDeque.size();i++){
	//		yarp::sig::Vector tmp = iwDeque.at(i);
	//		std::cout << "IW" << i << ": " << tmp.toString() << "\n";
	//	}
	//	std::deque<yarp::sig::Vector,std::allocator<yarp::sig::Vector> > lwDeque = neuralNetwork.get_LW();
	//	for (int i=0;i<lwDeque.size();i++){
	//		yarp::sig::Vector tmp = lwDeque.at(i);
	//		std::cout << "LW" << i << ": " << tmp.toString() << "\n";
	//	}

	//	std::vector<double> actualAngles(3);
	//	std::vector<double> rotatedAngles;
	//	actualAngles[0] = targetList[0];
	//	actualAngles[1] = targetList[1];
	//	actualAngles[2] = targetList[2];
	//	ICubUtil::rotateFingersData(actualAngles,rotatedAngles);

	//	std::cout << "Actual angles: " << actualAngles[0] << " " << actualAngles[1] << " " << actualAngles[2] << "\n";
	//	std::cout << "Rotated angles: " << rotatedAngles[0] << " " << rotatedAngles[1] << " " << rotatedAngles[2] << "\n";

	//	yarp::sig::Vector rotatedAnglesVector;
	//	rotatedAnglesVector.resize(3);
	//	rotatedAnglesVector[0] = rotatedAngles[0];
	//	rotatedAnglesVector[1] = rotatedAngles[1];
	//	rotatedAnglesVector[2] = rotatedAngles[2];
	//	yarp::sig::Vector svErrNNVector = neuralNetwork.predict(rotatedAnglesVector);
	//	double svErrNN = svErrNNVector[0];

	//	std::cout << "Error: " << svErrNN << "\n";
	//}

}

void TaskThread::view(RPCViewCmdArgName paramName,RPCCommandsData &rpcCmdData){

	switch (paramName){

	case SETTINGS:
		cout << "\n" <<
		        "\n" <<
                "-------- SETTINGS --------" << "\n" <<
		        "\n" <<
		        "--- TASK COMMON DATA -----" << "\n" <<
		        rpcCmdData.getFullDescription(PWM_SIGN) << ": " << taskData->commonData.pwmSign << "\n" <<
				rpcCmdData.getFullDescription(OBJ_DETECT_PRESS_THRESHOLDS) << ": " << taskData->getValueDescription(OBJ_DETECT_PRESS_THRESHOLDS) << "\n" <<
				rpcCmdData.getFullDescription(TEMPORARY_PARAM) << ": " << taskData->getValueDescription(TEMPORARY_PARAM) << "\n" <<
		        "\n" <<
		        "--- STEP TASK DATA -------" << "\n" <<
		        rpcCmdData.getFullDescription(STEP_LIFESPAN) << ": " << taskData->stepData.lifespan << "\n" <<
		        "\n" <<
		        "--- CONTROL TASK DATA ----" << "\n" <<
				rpcCmdData.getFullDescription(CTRL_PID_KPF) << ": " << taskData->getValueDescription(CTRL_PID_KPF) << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KIF) << ": " << taskData->getValueDescription(CTRL_PID_KIF) << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KPB) << ": " << taskData->getValueDescription(CTRL_PID_KPB) << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KIB) << ": " << taskData->getValueDescription(CTRL_PID_KIB) << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_OP_MODE) << ": " << taskData->controlData.controlMode << "\n" <<
				rpcCmdData.getFullDescription(CTRL_PID_RESET_ENABLED) << ": " << taskData->controlData.pidResetEnabled << "\n" <<
				rpcCmdData.getFullDescription(CTRL_TARGET_REAL_TIME) << ": " << "(to be defined while control task is running)" << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_LIFESPAN) << ": " << taskData->controlData.lifespan << "\n" <<
		        "\n" <<
		        "--- RAMP TASK DATA ---" << "\n" <<
		        rpcCmdData.getFullDescription(RAMP_SLOPE) << ": " << taskData->rampData.slope << "\n" <<
		        rpcCmdData.getFullDescription(RAMP_INTERCEPT) << ": " << taskData->rampData.intercept << "\n" <<
		        rpcCmdData.getFullDescription(RAMP_LIFESPAN) << ": " << taskData->rampData.lifespan << "\n" <<
		        rpcCmdData.getFullDescription(RAMP_LIFESPAN_AFTER_STAB) << ": " << taskData->rampData.lifespanAfterStabilization << "\n" <<
		        "\n" <<
		        "--- APPROACH TASK DATA ---" << "\n" <<
		        rpcCmdData.getFullDescription(APPR_JOINTS_VELOCITIES) << ": " << taskData->getValueDescription(APPR_JOINTS_VELOCITIES) << "\n" <<
		        rpcCmdData.getFullDescription(APPR_JOINTS_PWM_LIMITS) << ": " << taskData->getValueDescription(APPR_JOINTS_PWM_LIMITS) << "\n" <<
				rpcCmdData.getFullDescription(APPR_JOINTS_PWM_LIMITS_ENABLED) << ": " << taskData->approachData.jointsPwmLimitsEnabled << "\n" <<
		        rpcCmdData.getFullDescription(APPR_LIFESPAN) << ": " << taskData->approachData.lifespan << "\n" <<

				"";
		break;

	case TASKS:
		cout << "\n" <<
				"\n" << 
				"------- TASK LIST -------" << "\n" <<
					"\n";

		for (size_t i = 0; i < taskList.size(); i++){
			
			cout << "- ";
			switch (taskList[i]->getTaskName()){

			case STEP:
				cout << "STEP\t" << (dynamic_cast<StepTask*>(taskList[i]))->getConstantPwmDescription() << "\n";
				break;
			case CONTROL:
				cout << "CONTROL\t" << (dynamic_cast<ControlTask*>(taskList[i]))->getPressureTargetValueDescription() << "\n";
				break;
			case APPROACH_AND_CONTROL:
				cout << "APPROACH & CONTROL\t" << (dynamic_cast<ControlTask*>(taskList[i]))->getPressureTargetValueDescription() << "\n";
				break;
			case APPROACH:
				cout << "APPROACH\t" << "\n";
				break;
			case RAMP:
				cout << "RAMP\t" << (dynamic_cast<RampTask*>(taskList[i]))->getPressureTargetValueDescription() << "\n";
				break;

			}
		}
	}
}

void TaskThread::help(RPCCommandsData &rpcCmdData){

	cout << "\n" <<
		    "\n" <<
            "----------- HELP -----------" << "\n" <<
		    "---- AVAILABLE COMMANDS ----" << "\n" <<
		    "\n" <<
		    rpcCmdData.getFullDescription(HELP) << "\n" <<
		    rpcCmdData.getFullDescription(SET) << "\n" <<
		    rpcCmdData.getFullDescription(TASK) << "\n" <<
		    rpcCmdData.getFullDescription(VIEW) << "\n" <<
		    rpcCmdData.getFullDescription(START) << "\n" <<
		    rpcCmdData.getFullDescription(STOP) << "\n" <<
		    rpcCmdData.getFullDescription(OPEN) << "\n" <<
		    rpcCmdData.getFullDescription(ARM) << "\n" <<
		    rpcCmdData.getFullDescription(QUIT) << "\n";
	
}			

void TaskThread::testShowEndEffectors(){

    controllersUtil->testShowEndEffectors();    

}
