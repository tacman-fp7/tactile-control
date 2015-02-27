#include "iCub/plantIdentification/thread/TaskThread.h"
#include "iCub/plantIdentification/task/StepTask.h"
#include "iCub/plantIdentification/task/ControlTask.h"
#include "iCub/plantIdentification/task/RampTask.h"

#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <iostream>
#include <sstream>
#include <algorithm>
#include <ctime>
#include <cmath>

using std::cerr;
using std::cout;
using std::string;

using iCub::plantIdentification::TaskThread;

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
bool TaskThread::threadInit(void) {
    using yarp::os::Property;
    using yarp::os::Network;
    using yarp::os::Bottle;
    using yarp::os::Time;
    using std::vector;

	cout << dbgTag << "Initialising. \n";

	currentTaskIndex = 0;

	// initialize controllers
	controllersUtil = new ControllersUtil(rf);
	
	// initialize ports
	portsUtil = new PortsUtil(rf);
	
	// initialize task data
	taskData = new TaskData(rf,period);

	// save current arm position, to be restored when the thread ends
	controllersUtil->saveCurrentArmPosition();

	controllersUtil->setArmInTaskPosition();
 
	// this prevent the run() method to be executed between the taskThread->start() and the taskThread->suspend() calls during the PlantIdentificationModule initialization
	runEnabled = false;

    cout << dbgTag << "Initialised correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


void TaskThread::initializeGrasping(){

	controllersUtil->init(taskData->commonData.jointToMove);
	controllersUtil->saveCurrentControlMode();
	controllersUtil->setTaskControlMode();
	runEnabled = true;
}


/* *********************************************************************************************************************** */
/* ******* Run thread                                                       ********************************************** */
void TaskThread::run(void) {

	if (runEnabled){

		while (currentTaskIndex < taskList.size()){

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

void TaskThread::openHand(){

	controllersUtil->restorePreviousControlMode();
	controllersUtil->openHand();
	runEnabled = false;

}

/* *********************************************************************************************************************** */
/* ******* Release thread                                                   ********************************************** */
void TaskThread::threadRelease() {
    cout << dbgTag << "Releasing. \n";
    
    // closing ports
	portsUtil->release();

	controllersUtil->restorePreviousControlMode();
	controllersUtil->restorePreviousArmPosition();
	controllersUtil->release();

	delete(portsUtil);
	delete(controllersUtil);
	delete(taskData);

    cout << dbgTag << "Released. \n";
}
/* *********************************************************************************************************************** */


void TaskThread::set(iCub::plantIdentification::RPCSetCmdArgName paramName,std::string paramValue){

	switch (paramName){

	// common data
	case FINGER_TO_MOVE:
		taskData->commonData.fingerToMove = atoi(paramValue.c_str());
		break;
	case JOINT_TO_MOVE:
		taskData->commonData.jointToMove = atoi(paramValue.c_str());
		break;
	case PWM_SIGN:
		taskData->commonData.pwmSign = atoi(paramValue.c_str());
		break;

	// step task data
	case STEP_LIFESPAN:
		taskData->stepData.lifespan = atoi(paramValue.c_str());
		break;

	// control task data
	case CTRL_PID_KPF:
		taskData->controlData.pidKpf = atof(paramValue.c_str());
		break;
	case CTRL_PID_KIF:
		taskData->controlData.pidKif = atof(paramValue.c_str());
		break;
	case CTRL_PID_KDF:
		taskData->controlData.pidKdf = atof(paramValue.c_str());
		break;
	case CTRL_PID_KPB:
		taskData->controlData.pidKpb = atof(paramValue.c_str());
		break;
	case CTRL_PID_KIB:
		taskData->controlData.pidKib = atof(paramValue.c_str());
		break;
	case CTRL_PID_KDB:
		taskData->controlData.pidKdb = atof(paramValue.c_str());
		break;
	case CTRL_OP_MODE:
		taskData->controlData.controlMode = atoi(paramValue.c_str());
		break;
	case CTRL_LIFESPAN:
		taskData->controlData.lifespan = atoi(paramValue.c_str());
		break;

	// ramp task data
	case RAMP_SLOPE:
		taskData->rampData.slope = atof(paramValue.c_str());
		break;
	case RAMP_INTERCEPT:
		taskData->rampData.intercept = atof(paramValue.c_str());
		break;
	case RAMP_LIFESPAN:
		taskData->rampData.lifespan = atoi(paramValue.c_str());
		break;
	case RAMP_LIFESPAN_AFTER_STAB:
		taskData->rampData.lifespanAfterStabilization = atoi(paramValue.c_str());
		break;
	}
}

void TaskThread::task(iCub::plantIdentification::RPCTaskCmdArgName paramName,iCub::plantIdentification::TaskName taskName,string paramValue){

	switch (paramName){

	case ADD:

		switch (taskName){
		case STEP:
			taskList.push_back(new StepTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->stepData,atof(paramValue.c_str())));
			break;

		case CONTROL:
			taskList.push_back(new ControlTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->controlData,atof(paramValue.c_str())));
			break;

		case RAMP:
			taskList.push_back(new RampTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->rampData,atof(paramValue.c_str())));
			break;
		}
		break;

	case EMPTY:
		for (size_t i = 0; i < taskList.size(); i++){
			delete(taskList[i]);
		}
		taskList.clear();
		break;

	case POP:
		delete(taskList[taskList.size() - 1]);
		taskList.pop_back();
		break;
	}
}

void TaskThread::view(iCub::plantIdentification::RPCViewCmdArgName paramName,iCub::plantIdentification::RPCCommandsData &rpcCmdData){

	switch (paramName){

	case SETTINGS:
		cout << "-------- SETTINGS --------" << "\n" <<
		cout << "\n" <<
		cout << "--- TASK COMMON DATA -----" << "\n" <<
		cout << rpcCmdData.getFullDescription(FINGER_TO_MOVE) << ": " << taskData->commonData.fingerToMove << "\n" <<
		cout << rpcCmdData.getFullDescription(JOINT_TO_MOVE) << ": " << taskData->commonData.jointToMove << "\n" <<
		cout << rpcCmdData.getFullDescription(PWM_SIGN) << ": " << taskData->commonData.pwmSign << "\n" <<
		cout << "\n" <<
		cout << "--- STEP TASK DATA -------" << "\n" <<
		cout << rpcCmdData.getFullDescription(STEP_LIFESPAN) << ": " << taskData->stepData.lifespan << "\n" <<
		cout << "\n" <<
		cout << "--- CONTROL TASK DATA ----" << "\n" <<
		cout << rpcCmdData.getFullDescription(CTRL_PID_KPF) << ": " << taskData->controlData.pidKpf << "\n" <<
		cout << rpcCmdData.getFullDescription(CTRL_PID_KIF) << ": " << taskData->controlData.pidKif << "\n" <<
		cout << rpcCmdData.getFullDescription(CTRL_PID_KDF) << ": " << taskData->controlData.pidKdf << "\n" <<
		cout << rpcCmdData.getFullDescription(CTRL_PID_KPB) << ": " << taskData->controlData.pidKpb << "\n" <<
		cout << rpcCmdData.getFullDescription(CTRL_PID_KIB) << ": " << taskData->controlData.pidKib << "\n" <<
		cout << rpcCmdData.getFullDescription(CTRL_PID_KDB) << ": " << taskData->controlData.pidKdb << "\n" <<
		cout << rpcCmdData.getFullDescription(CTRL_OP_MODE) << ": " << taskData->controlData.controlMode << "\n" <<
		cout << rpcCmdData.getFullDescription(CTRL_LIFESPAN) << ": " << taskData->controlData.lifespan << "\n" <<
		cout << "\n" <<
		cout << "--- RAMP TASK DATA ---" << "\n" <<
		cout << rpcCmdData.getFullDescription(RAMP_SLOPE) << ": " << taskData->rampData.slope << "\n" <<
		cout << rpcCmdData.getFullDescription(RAMP_INTERCEPT) << ": " << taskData->rampData.intercept << "\n" <<
		cout << rpcCmdData.getFullDescription(RAMP_LIFESPAN) << ": " << taskData->rampData.lifespan << "\n" <<
		cout << rpcCmdData.getFullDescription(RAMP_LIFESPAN_AFTER_STAB) << ": " << taskData->rampData.lifespanAfterStabilization << "\n";
		break;

	case TASKS:
		cout << "------- TASKS LIST -------" << "\n" <<
		cout << "\n";

		for (size_t i = 0; i < taskList.size(); i++){
			
			cout << "- ";
			switch (taskList[i]->getTaskName()){

			case STEP:
				cout << "STEP\t" << ((StepTask*)taskList[i])->getConstantPwm() << "\n";
				break;
			case CONTROL:
				cout << "CONTROL\t" << ((ControlTask*)taskList[i])->getPressureTargetValue() << "\n";
				break;
			case RAMP:
				cout << "RAMP\t" << ((RampTask*)taskList[i])->getPressureTargetValue() << "\n";
				break;

			}

		}

	}

}