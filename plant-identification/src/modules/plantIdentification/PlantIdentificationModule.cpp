#include "iCub/plantIdentification/PlantIdentificationModule.h"

using iCub::plantIdentification::PlantIdentificationModule;

using iCub::plantIdentification::EventsThread;

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
double PlantIdentificationModule::getPeriod() { return moduleThreadPeriod/1000.0; }
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Configure module                                                 ********************************************** */   
bool PlantIdentificationModule::configure(ResourceFinder &rf) {
    using std::string;
    using yarp::os::Property;

    cout << dbgTag << "Starting. \n";

    /* ****** Configure the Module                            ****** */
    moduleName = rf.check("name", Value("plantIdentification")).asString().c_str();
    moduleThreadPeriod = rf.check("moduleThreadPeriod", 1000).asInt();

    /* ******* Open ports                                       ******* */
    portPlantIdentificationRPC.open("/plantIdentification/cmd:i");
    attach(portPlantIdentificationRPC);


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
	taskData = new TaskData(rf,controllersUtil);
 

    /* ******* Threads                                          ******* */
	taskThread = new TaskThread(taskData->commonData.taskThreadPeriod,rf,controllersUtil,portsUtil,taskData);
    if (!taskThread->start()) {
        cout << dbgTag << "Could not start the task thread. \n";
        return false;
    }
    taskThread->suspend();

	eventsThread = new EventsThread(taskData->commonData.eventsThreadPeriod,controllersUtil,portsUtil,&taskData->commonData);
    if (!eventsThread->start()) {
        cout << dbgTag << "Could not start the events thread. \n";
        return false;
    }
    //eventsThread->suspend();

	rpcCmdUtil.init(&rpcCmdData);

    cout << dbgTag << "Started correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Update    module                                                 ********************************************** */   
bool PlantIdentificationModule::updateModule() { 

	

	// manage event triggers
	if (eventsThread->eventTriggered(FINGERTIP_PUSHED,3)){ // pinky
		if (tasksRunning){
			open();
		} else {
			grasp();
		}
	}
	if (eventsThread->eventTriggered(FINGERTIP_PUSHED,2)){ // ring finger
		int positionTrackingMode = taskData->commonData.tempParameters[18].asInt();
		if (positionTrackingMode == 0){
			taskData->commonData.tempParameters[18] = Value(2);
			taskData->commonData.tempParameters[29] = Value(0);
		} else {
			taskData->commonData.tempParameters[18] = Value(0);
			taskData->commonData.tempParameters[29] = Value(1);
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
	case GRASP:
		grasp();
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
	if (eventsThread->isRunning()){
		eventsThread->suspend();
	}

    // Stop thread
    taskThread->stop();
    eventsThread->stop();

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
/* ******* RPC Execute the grasp task                                       ********************************************** */
bool PlantIdentificationModule::grasp() {

	task(EMPTY,NONE,Value(0));
	task(ADD,APPROACH,Value(0));
	task(ADD,CONTROL,Value(0));
	start();

	return true;
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
