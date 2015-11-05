#include "iCub/plantIdentification/PlantIdentificationModule.h"

using iCub::plantIdentification::PlantIdentificationModule;

using std::cout;

using yarp::os::ResourceFinder;
using yarp::os::Value;

/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
PlantIdentificationModule::PlantIdentificationModule() 
    : RFModule() {
        closing = false;
		tasksRunning = false;

        dbgTag = "PlantIdentificationModule: ";
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Destructor                                                       ********************************************** */   
PlantIdentificationModule::~PlantIdentificationModule() {}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Get Period                                                       ********************************************** */   
double PlantIdentificationModule::getPeriod() { return period; }
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Configure module                                                 ********************************************** */   
bool PlantIdentificationModule::configure(ResourceFinder &rf) {
    using std::string;
    using yarp::os::Property;

    cout << dbgTag << "Starting. \n";

    /* ****** Configure the Module                            ****** */
    moduleName = rf.check("name", Value("plantIdentification")).asString().c_str();
    period = rf.check("period", 1.0).asDouble();

    /* ******* Open ports                                       ******* */
    portPlantIdentificationRPC.open("/plantIdentification/cmd:i");
    attach(portPlantIdentificationRPC);

    /* ******* Threads                                          ******* */
    taskThread = new TaskThread(20, rf);
    if (!taskThread->start()) {
        cout << dbgTag << "Could not start the task thread. \n";
        return false;
    }
    taskThread->suspend();

	rpcCmdUtil.init(&rpcCmdData);

    cout << dbgTag << "Started correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Update    module                                                 ********************************************** */   
bool PlantIdentificationModule::updateModule() { 

	// check events
	if (taskThread->eventTriggered(FINGERTIP_PUSHED,3)){ // pinky
		if (tasksRunning){
			open();
		} else {
			task(EMPTY,NONE,Value(0));
			task(ADD,APPROACH,Value(0));
			task(ADD,CONTROL,Value(0));
			start();
		}
	}
	if (taskThread->eventTriggered(FINGERTIP_PUSHED,2)){ // ring finger
		int positionTrackingMode = taskThread->taskData->commonData.tempParameters[18].asInt();
		if (positionTrackingMode == 0){
			taskThread->taskData->commonData.tempParameters[18] = Value(2);
		} else {
			taskThread->taskData->commonData.tempParameters[18] = Value(0);
		}
	}


    return !closing; 
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Interrupt module                                                 ********************************************** */   
bool PlantIdentificationModule::interruptModule() {
    cout << dbgTag << "Interrupting. \n";
    
    // Interrupt port
    portPlantIdentificationRPC.interrupt();

    cout << dbgTag << "Interrupted correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Manage commands coming from RPC Port                             ********************************************** */   
bool PlantIdentificationModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){

	rpcCmdUtil.processCommand(command);

	switch (rpcCmdUtil.mainCmd){

	case HELP:
		help();
		break;
	case SET:
		set(rpcCmdUtil.setCmdArg,rpcCmdUtil.argValue);
		break;
	case TASK:
		task(rpcCmdUtil.taskCmdArg,rpcCmdUtil.task,rpcCmdUtil.argValue);
		break;
	case VIEW:
		view(rpcCmdUtil.viewCmdArg);
		break;
	case START:
		start();
		break;
	case STOP:
		stop();
		break;
	case OPEN:
		open();
		break;
	case ARM:
		arm();
		break;
	case QUIT:
        quit();

        break;
    }
	return true;
}

/* *********************************************************************************************************************** */
/* ******* Close module                                                     ********************************************** */   
bool PlantIdentificationModule::close() {
    cout << dbgTag << "Closing. \n";
    
	if (taskThread->isRunning()){
		taskThread->suspend();
	}

    // Stop thread
    taskThread->stop();

    // Close port
    portPlantIdentificationRPC.close();

    cout << dbgTag << "Closed. \n";
    
    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Stop task execution without opening the hand                 ********************************************** */
bool PlantIdentificationModule::stop() {
    
	tasksRunning = false;

	taskThread->suspend();

	taskThread->afterRun(false);

	return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* RPC Stop task execution and open the hand                        ********************************************** */
bool PlantIdentificationModule::open() {
    
	tasksRunning = false;

	taskThread->suspend();

	taskThread->afterRun(true);

	return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool PlantIdentificationModule::start() {

	tasksRunning = true;

	if (!taskThread->initializeGrasping()) return false;

	taskThread->resume();

    return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* RPC Set arm in task position                                     ********************************************** */
bool PlantIdentificationModule::arm() {

	return taskThread->setArmInTaskPosition();
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Quit module                                                  ********************************************** */
bool PlantIdentificationModule::quit() {
    return closing = true;
}
/* *********************************************************************************************************************** */


void PlantIdentificationModule::set(iCub::plantIdentification::RPCSetCmdArgName paramName,Value paramValue){
	taskThread->set(paramName,paramValue,rpcCmdData);
	view(SETTINGS);
}

void PlantIdentificationModule::task(iCub::plantIdentification::RPCTaskCmdArgName paramName,iCub::plantIdentification::TaskName taskName,Value paramValue){
	taskThread->task(paramName,taskName,paramValue,rpcCmdData);
	view(TASKS);
}

void PlantIdentificationModule::view(iCub::plantIdentification::RPCViewCmdArgName paramName){
	taskThread->view(paramName,rpcCmdData);
}

void PlantIdentificationModule::help(){
	taskThread->help(rpcCmdData);
}
