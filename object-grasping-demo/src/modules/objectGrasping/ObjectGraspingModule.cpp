#include "iCub/objectGrasping/ObjectGraspingModule.h"

#include <yarp/os/Time.h>

using iCub::objectGrasping::ObjectGraspingModule;

using std::cout;

using yarp::os::ResourceFinder;
using yarp::os::Value;


/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
ObjectGraspingModule::ObjectGraspingModule() 
    : RFModule() {
        closing = false;

        dbgTag = "ObjectGraspingModule: ";
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Destructor                                                       ********************************************** */   
ObjectGraspingModule::~ObjectGraspingModule() {}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Get Period                                                       ********************************************** */   
double ObjectGraspingModule::getPeriod() { return period; }
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Configure module                                                 ********************************************** */   
bool ObjectGraspingModule::configure(ResourceFinder &rf) {
    using std::string;
    using yarp::os::Property;

    cout << dbgTag << "Starting. \n";

    /* ****** Configure the Module                            ****** */
    moduleName = rf.check("name", Value("objectGrasping")).asString().c_str();
    period = rf.check("period", 1.0).asDouble();

    /* ******* Open ports                                       ******* */
    portObjectGraspingRPC.open("/objectGrasping/cmd:io");
    attach(portObjectGraspingRPC);

	rpcCmdUtil.init(&rpcCmdData);


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

	// save current arm position, to be restored when the thread ends
	if (!controllersUtil->saveCurrentArmPosition()) {
        cout << dbgTag << "failed to store current arm position\n";
        return false;
    }

	/* ******* Threads                                          ******* */
    taskThread = new TaskThread(controllersUtil,portsUtil,20, rf);
    if (!taskThread->start()) {
        cout << dbgTag << "Could not start the task thread. \n";
        return false;
    }
    taskThread->suspend();

	taskState = WAIT_FOR_DEMO;

    cout << dbgTag << "Started correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Update    module                                                 ********************************************** */   
bool ObjectGraspingModule::updateModule() {

	switch(taskState){

	case WAIT_TO_START:
		// do nothing
		break;

	case SET_ARM_IN_GRASP_POSITION:
		if (controllersUtil->setArmInGraspPosition()) {
			taskState = BEGIN_GRASP_THREAD;
		} else{
	        cout << dbgTag << "failed to set the arm in grasp position\n";
	        return false;
		}
		break;

	case BEGIN_GRASP_THREAD:
		if (start()){
			taskState = WAIT_FOR_GRASP_THREAD;
		} else {
			cout << dbgTag << "failed to start the grasp thread\n";
	        return false;
		}
		break;
	
	case WAIT_FOR_GRASP_THREAD:
		yarp::os::Time::delay(5);
		break;

	case RAISE_ARM:
		if (controllersUtil->setArmInGraspPosition()) {
			taskState = WAIT_FOR_CLOSURE;
		} else{
	        cout << dbgTag << "failed to raise the arm\n";
	        return false;
		}
		break;

	case WAIT_FOR_CLOSURE:
		// do nothing
		break;

	default:
		break;
	}

    return !closing; 
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Interrupt module                                                 ********************************************** */   
bool ObjectGraspingModule::interruptModule() {
    cout << dbgTag << "Interrupting. \n";
    
    // Interrupt port
    portObjectGraspingRPC.interrupt();

    cout << dbgTag << "Interrupted correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Manage commands coming from RPC Port                             ********************************************** */   
bool ObjectGraspingModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){

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
	case QUIT:
		quit();
		break;
	}

	return true;
}

/* *********************************************************************************************************************** */
/* ******* Close module                                                     ********************************************** */   
bool ObjectGraspingModule::close() {
    cout << dbgTag << "Closing. \n";
    
	if (taskThread->isRunning()){
		taskThread->suspend();
	}

    // Stop thread
    taskThread->stop();

	portsUtil->release();

	controllersUtil->release();

	delete(portsUtil);
	delete(controllersUtil);

    // Close port
    portObjectGraspingRPC.close();

    cout << dbgTag << "Closed. \n";
    
    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Open hand                                                    ********************************************** */
bool ObjectGraspingModule::stop() {
    
	taskThread->suspend();
    
	taskThread->openHand();

	return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool ObjectGraspingModule::start() {

	if (!taskThread->initializeGrasping()) return false;

	taskThread->resume();

    return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool ObjectGraspingModule::demo() {

	taskState = SET_ARM_IN_GRASP_POSITION;

    return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* RPC Quit module                                                  ********************************************** */
bool ObjectGraspingModule::quit() {
    return closing = true;
}
/* *********************************************************************************************************************** */


void ObjectGraspingModule::set(iCub::objectGrasping::RPCSetCmdArgName paramName,Value paramValue){
	taskThread->set(paramName,paramValue,rpcCmdData);
	view(SETTINGS);
}

void ObjectGraspingModule::task(iCub::objectGrasping::RPCTaskCmdArgName paramName,iCub::objectGrasping::TaskName taskName,Value paramValue){
	taskThread->task(paramName,taskName,paramValue,rpcCmdData);
	view(TASKS);
}

void ObjectGraspingModule::view(iCub::objectGrasping::RPCViewCmdArgName paramName){
	taskThread->view(paramName,rpcCmdData);
}

void ObjectGraspingModule::help(){
	taskThread->help(rpcCmdData);
}