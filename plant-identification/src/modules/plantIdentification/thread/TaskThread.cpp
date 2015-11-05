#include "iCub/plantIdentification/thread/TaskThread.h"
#include "iCub/plantIdentification/task/StepTask.h"
#include "iCub/plantIdentification/task/ControlTask.h"
#include "iCub/plantIdentification/task/ApproachTask.h"
#include "iCub/plantIdentification/task/RampTask.h"

#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>

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
using iCub::plantIdentification::ICubUtil;

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
	using yarp::os::Log;

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


	// initialize events
	eventsUtil = new EventsUtil(&taskData->commonData);
	eventsUtil->init();
 
	// this prevent the run() method to be executed between the taskThread->start() and the taskThread->suspend() calls during the PlantIdentificationModule initialization
	runEnabled = false;

    cout << dbgTag << "Initialised correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


bool TaskThread::initializeGrasping(){

	if (!controllersUtil->saveCurrentControlMode()) return false;
	runEnabled = true;

	controllersUtil->lookAtTheHand();

	return true;
}


/* *********************************************************************************************************************** */
/* ******* Run thread                                                       ********************************************** */
void TaskThread::run() {
	
	ICubUtil::updateExternalData(controllersUtil,portsUtil,&taskData->commonData);

	eventsUtil->checkEvents();

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

	controllersUtil->restoreFixationPoint();

	if (!controllersUtil->restorePreviousControlMode()) return false;
	if (openHand) {
		if (!controllersUtil->openHand()) return false;
	}
	runEnabled = false;
    taskList[currentTaskIndex]->clean();
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
			yInfo() << "\n\n" << "CANNOT SET TEMPORARY PARAM";
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
				yInfo()  << "\n\n" << "CANNOT EXECUTE SET COMMAND (CONTROL TASK IS NOT RUNNING)";
			}
		} else {
			setSuccessful = false;
			yInfo() << "\n\n" << "CANNOT EXECUTE SET COMMAND (NO TASK RUNNING)";
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
		yInfo()	 << "\n" <<
					"\n" << 
					"'" << rpcCmdData.setCmdArgMap[paramName] << "' SET TO " << rpcCmdData.printValue(paramValue);
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
		yInfo()  << "\n" <<
					"\n" << 
					"ADDED '" << rpcCmdData.taskMap[taskName] << "' TASK";
		break;

	case EMPTY:
		for (size_t i = 0; i < taskList.size(); i++){
			delete(taskList[i]);
		}
		taskList.clear();
		yInfo()  << "\n" <<
					"\n" << 
					"'" << "TASK LIST CLEARED";
		break;

	case POP:
		delete(taskList[taskList.size() - 1]);
		taskList.pop_back();
		yInfo()  << "\n" <<
					"\n" << 
					"'" << "LAST TASK REMOVED" << "\n";
		break;
	}


}

void TaskThread::view(RPCViewCmdArgName paramName,RPCCommandsData &rpcCmdData){

	switch (paramName){

	case SETTINGS:
		yInfo()  << "\n" <<
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
					rpcCmdData.getFullDescription(APPR_LIFESPAN) << ": " << taskData->approachData.lifespan;
		break;

	case TASKS:
		yInfo()  << "\n" <<
						"\n" << 
				"------- TASK LIST -------" << "\n";

		for (size_t i = 0; i < taskList.size(); i++){
			
			switch (taskList[i]->getTaskName()){

			case STEP:
				yInfo() << "- STEP\t" << (dynamic_cast<StepTask*>(taskList[i]))->getConstantPwmDescription();
				break;
			case CONTROL:
				yInfo() << "- CONTROL\t" << (dynamic_cast<ControlTask*>(taskList[i]))->getPressureTargetValueDescription();
				break;
			case APPROACH_AND_CONTROL:
				yInfo() << "- APPROACH & CONTROL\t" << (dynamic_cast<ControlTask*>(taskList[i]))->getPressureTargetValueDescription();
				break;
			case APPROACH:
				yInfo() << "- APPROACH";
				break;
			case RAMP:
				yInfo() << "- RAMP\t" << (dynamic_cast<RampTask*>(taskList[i]))->getPressureTargetValueDescription();
				break;

			}
		}
	}
}

void TaskThread::help(RPCCommandsData &rpcCmdData){

	yInfo()  << "\n" <<
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
				rpcCmdData.getFullDescription(QUIT);
	
}			

void TaskThread::testShowEndEffectors(){

    controllersUtil->testShowEndEffectors();    

}

bool TaskThread::eventTriggered(EventToTrigger eventToTrigger,int index = 0){

	switch(eventToTrigger){

	case FINGERTIP_PUSHED:

		return eventsUtil->eventFPTriggered(index);
		break;

	}

	return false;
}