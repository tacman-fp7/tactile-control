#include "iCub/objectGrasping/thread/TaskThread.h"
#include "iCub/objectGrasping/task/StepTask.h"
#include "iCub/objectGrasping/task/ControlTask.h"
#include "iCub/objectGrasping/task/RampTask.h"

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

using iCub::objectGrasping::TaskThread;
using iCub::objectGrasping::RPCSetCmdArgName;
using iCub::objectGrasping::RPCTaskCmdArgName;
using iCub::objectGrasping::TaskName;
using iCub::objectGrasping::RPCViewCmdArgName;
using iCub::objectGrasping::RPCCommandsData;

using yarp::os::RateThread;
using yarp::os::Value;


/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
TaskThread::TaskThread(iCub::objectGrasping::ControllersUtil *controllersUtil,iCub::objectGrasping::PortsUtil *portsUtil,const int period, const yarp::os::ResourceFinder &aRf) 
    : RateThread(period) {
		this->period = period;
        rf = aRf;

		this->controllersUtil = controllersUtil;
		this->portsUtil = portsUtil;

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

	// initialize task data
	taskData = new TaskData(rf,period);

 	// this prevent the run() method to be executed between the taskThread->start() and the taskThread->suspend() calls during the ObjectGraspingModule initialization
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

bool TaskThread::openHand(){

	if (!controllersUtil->restorePreviousControlMode()) return false;
	if (!controllersUtil->openHand()) return false;
	runEnabled = false;
    currentTaskIndex = 0;

	return true;
}

/* *********************************************************************************************************************** */
/* ******* Release thread                                                   ********************************************** */
void TaskThread::threadRelease() {
    cout << dbgTag << "Releasing. \n";

	delete(taskData);

	cout << dbgTag << "Released. \n";
}
/* *********************************************************************************************************************** */


void TaskThread::set(RPCSetCmdArgName paramName,Value paramValue,RPCCommandsData &rpcCmdData){

	switch (paramName){

	// common data
	case PWM_SIGN:
		taskData->commonData.pwmSign = paramValue.asInt();
		break;

	// step task data
	case STEP_LIFESPAN:
		taskData->stepData.lifespan = paramValue.asDouble();
		break;

	// control task data
	case CTRL_PID_KPF:
		taskData->controlData.pidKpf = paramValue.asDouble();
		break;
	case CTRL_PID_KIF:
		taskData->controlData.pidKif = paramValue.asDouble();
		break;
	case CTRL_PID_KDF:
		taskData->controlData.pidKdf = paramValue.asDouble();
		break;
	case CTRL_PID_KPB:
		taskData->controlData.pidKpb = paramValue.asDouble();
		break;
	case CTRL_PID_KIB:
		taskData->controlData.pidKib = paramValue.asDouble();
		break;
	case CTRL_PID_KDB:
		taskData->controlData.pidKdb = paramValue.asDouble();
		break;
	case CTRL_PID_RESET_ENABLED:
		taskData->controlData.pidResetEnabled = paramValue.asInt() != 0;
		break;
	case CTRL_OP_MODE:
		taskData->controlData.controlMode = static_cast<ControlTaskOpMode>(paramValue.asInt());
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
	}

	cout << "\n" <<
            "\n" << 
            "'" << rpcCmdData.setCmdArgMap[paramName] << "' SET TO " << (paramValue.isDouble() ? paramValue.asDouble() : paramValue.asInt()) << "\n";
}

void TaskThread::task(RPCTaskCmdArgName paramName,TaskName taskName,Value paramValue,RPCCommandsData &rpcCmdData){


	switch (paramName){

	case ADD:
		{
			string targetsString = paramValue.asString();
			char *targetsChar = new char[targetsString.length() + 1];
			strcpy(targetsChar,targetsString.c_str());
			std::vector<double> targetsList;
			char *target;
			
			target = strtok(targetsChar,"_");
			while(target != NULL){
				targetsList.push_back(atof(target));
			}

			switch (taskName){
			case STEP:
				taskList.push_back(new StepTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->stepData,targetsList));
				break;

			case CONTROL:
				taskList.push_back(new ControlTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->controlData,targetsList));
				break;

			case RAMP:
				taskList.push_back(new RampTask(controllersUtil,portsUtil,&taskData->commonData,&taskData->rampData,targetsList));
				break;
			}
			cout << "\n" <<
					"\n" << 
					"ADDED '" << rpcCmdData.taskMap[taskName] << "' TASK" << "\n";
		}
		break;

	case EMPTY:
		for (size_t i = 0; i < taskList.size(); i++){
			delete(taskList[i]);
		}
		taskList.clear();
		cout << "\n" <<
				"\n" << 
				"'" << "TASKS LIST CLEARED" << "\n";
		break;

	case POP:
		delete(taskList[taskList.size() - 1]);
		taskList.pop_back();
		cout << "\n" <<
				"\n" << 
				"'" << "LAST TASK REMOVED" << "\n";
		break;
	}
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
		        "\n" <<
		        "--- STEP TASK DATA -------" << "\n" <<
		        rpcCmdData.getFullDescription(STEP_LIFESPAN) << ": " << taskData->stepData.lifespan << "\n" <<
		        "\n" <<
		        "--- CONTROL TASK DATA ----" << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KPF) << ": " << taskData->controlData.pidKpf << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KIF) << ": " << taskData->controlData.pidKif << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KDF) << ": " << taskData->controlData.pidKdf << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KPB) << ": " << taskData->controlData.pidKpb << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KIB) << ": " << taskData->controlData.pidKib << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_PID_KDB) << ": " << taskData->controlData.pidKdb << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_OP_MODE) << ": " << taskData->controlData.controlMode << "\n" <<
				rpcCmdData.getFullDescription(CTRL_PID_RESET_ENABLED) << ": " << (taskData->controlData.pidResetEnabled ? "true" : "false") << "\n" <<
		        rpcCmdData.getFullDescription(CTRL_LIFESPAN) << ": " << taskData->controlData.lifespan << "\n" <<
		        "\n" <<
		        "--- RAMP TASK DATA ---" << "\n" <<
		        rpcCmdData.getFullDescription(RAMP_SLOPE) << ": " << taskData->rampData.slope << "\n" <<
		        rpcCmdData.getFullDescription(RAMP_INTERCEPT) << ": " << taskData->rampData.intercept << "\n" <<
		        rpcCmdData.getFullDescription(RAMP_LIFESPAN) << ": " << taskData->rampData.lifespan << "\n" <<
		        rpcCmdData.getFullDescription(RAMP_LIFESPAN_AFTER_STAB) << ": " << taskData->rampData.lifespanAfterStabilization << "\n";
		break;

	case TASKS:
		cout << "\n" <<
				"\n" << 
				"------- TASKS LIST -------" << "\n" <<
					"\n";

		for (size_t i = 0; i < taskList.size(); i++){
			
			cout << "- ";
			switch (taskList[i]->getTaskName()){

			case STEP:
				cout << "STEP\t" << ((StepTask*)taskList[i])->getConstantPwmDescription() << "\n";
				break;
			case CONTROL:
				cout << "CONTROL\t" << ((ControlTask*)taskList[i])->getPressureTargetValueDescription() << "\n";
				break;
			case RAMP:
				cout << "RAMP\t" << ((RampTask*)taskList[i])->getPressureTargetValueDescription() << "\n";
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
		    rpcCmdData.getFullDescription(QUIT) << "\n";
	
}			