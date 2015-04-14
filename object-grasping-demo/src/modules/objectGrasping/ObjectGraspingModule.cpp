#include "iCub/objectGrasping/ObjectGraspingModule.h"

#include <yarp/os/Time.h>

#include <sstream>

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
    portIncomingCommandsRPC.open("/objectGrasping/incomingCmd:i");
    portOutgoingCommandsRPC.open("/objectGrasping/graspTaskCmd:o");
    attach(portIncomingCommandsRPC);

	rpcCmdUtil.init(&rpcCmdData);

	configData = new ConfigData(rf);

	// initialize controllers
	controllersUtil = new ControllersUtil();
	if (!controllersUtil->init(rf)) {
        cout << dbgTag << "failed to initialize controllers utility\n";
        return false;
    }

	// save current arm position, to be restored when the thread ends
	if (!controllersUtil->saveCurrentArmPosition()) {
        cout << dbgTag << "failed to store current arm position\n";
        return false;
    }

	taskState = SET_ARM_IN_START_POSITION;

    cout << dbgTag << "Started correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Update    module                                                 ********************************************** */   
bool ObjectGraspingModule::updateModule() {

	std::stringstream cmd;

	switch(taskState){

	case SET_ARM_IN_START_POSITION:
		if (controllersUtil->setArmInStartPosition()) {
			taskState = WAIT_TO_START;
		} else{
	        cout << dbgTag << "failed to set the arm in start position\n";
	        return false;
		}
		break;

	case WAIT_TO_START:
		// do nothing
		break;

	case SET_ARM_IN_GRASP_POSITION:

//        controllersUtil->testCartesianController();
//        taskState = WAIT_TO_START;

		if (controllersUtil->setArmInGraspPosition()) {
//			taskState = WAIT_TO_START;

			taskState = BEGIN_GRASP_THREAD;
		} else{
	        cout << dbgTag << "failed to set the arm in grasp position\n";
	        return false;
		}
		break;

	case BEGIN_GRASP_THREAD:
        
		controllersUtil->moveFingers();
		
        sendCommand("set kp_pe",configData->pidKp);
		
        sendCommand("set ki_pe",configData->pidKi);

		sendCommand("task empty");
		
        sendCommand("task add ctrl",configData->targetPressure);

		sendCommand("start");
		
		taskState = WAIT_FOR_GRASP_THREAD;
		break;
	
	case WAIT_FOR_GRASP_THREAD:
		yarp::os::Time::delay(4);
        taskState = RAISE_ARM;
		break;

	case RAISE_ARM:
		if (controllersUtil->raiseArm()) {
			taskState = SET_ARM_BACK_IN_GRASP_POSITION;
		} else{
	        cout << dbgTag << "failed to raise the arm\n";
	        return false;
		}
		break;

	case WAIT_WITH_ARM_RAISED:
		// do nothing
		break;

	case SET_ARM_BACK_IN_GRASP_POSITION:
		if (controllersUtil->setArmInGraspPosition()) {
			taskState = OPEN_HAND;
		} else{
	        cout << dbgTag << "failed to set the arm in grasp position\n";
	        return false;
		}
		break;

	case OPEN_HAND:
		sendCommand("stop");
        yarp::os::Time::delay(1);
		sendCommand("task empty");
        taskState = SET_ARM_BACK_IN_START_POSITION;
	break;

	case SET_ARM_BACK_IN_START_POSITION:
		if (controllersUtil->setArmInStartPosition()) {
			taskState = WAIT_FOR_CLOSURE;
		} else{
	        cout << dbgTag << "failed to set the arm in start position\n";
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
    portIncomingCommandsRPC.interrupt();
    portOutgoingCommandsRPC.interrupt();

    cout << dbgTag << "Interrupted correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Manage commands coming from RPC Port                             ********************************************** */   
bool ObjectGraspingModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){

	rpcCmdUtil.processCommand(command);

	switch (rpcCmdUtil.mainCmd){

	case DEMO:
		demo();
		break;
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

    controllersUtil->restorePreviousArmPosition();

	controllersUtil->release();

	delete(controllersUtil);

    // Close port
    portIncomingCommandsRPC.close();
    portOutgoingCommandsRPC.close();

    cout << dbgTag << "Closed. \n";
    
    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Open hand                                                    ********************************************** */
bool ObjectGraspingModule::stop() {
    
	return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool ObjectGraspingModule::start() {

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
//	taskThread->set(paramName,paramValue,rpcCmdData);
//	view(SETTINGS);
}

void ObjectGraspingModule::task(iCub::objectGrasping::RPCTaskCmdArgName paramName,iCub::objectGrasping::TaskName taskName,Value paramValue){
//	taskThread->task(paramName,taskName,paramValue,rpcCmdData);
//	view(TASKS);
}

void ObjectGraspingModule::view(iCub::objectGrasping::RPCViewCmdArgName paramName){
//	taskThread->view(paramName,rpcCmdData);
}

void ObjectGraspingModule::help(){
//	taskThread->help(rpcCmdData);
}



void ObjectGraspingModule::sendCommand(std::string command){
	
	yarp::os::Bottle message;

	rpcCmdUtil.createBottleMessage(command,message);

	portOutgoingCommandsRPC.write(message);

}

void ObjectGraspingModule::sendCommand(std::string command,double value){
	
	yarp::os::Bottle message;

	rpcCmdUtil.createBottleMessage(command,value,message);

	portOutgoingCommandsRPC.write(message);

}
