#include "iCub/plantIdentification/PlantIdentificationModule.h"

#include <iCub/iKin/iKinFwd.h>

using iCub::plantIdentification::PlantIdentificationModule;

using std::cout;

using yarp::os::ResourceFinder;
using yarp::os::Value;

/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
PlantIdentificationModule::PlantIdentificationModule() 
    : RFModule() {
        closing = false;

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
    using iCub::iKin::iCubFinger;

    iCubFinger *thumbA = new iCubFinger("right_thumb_a");
    iCubFinger *thumbB = new iCubFinger("right_thumb_b");
    iCubFinger *index = new iCubFinger("right_index_na");
    iCubFinger *middle = new iCubFinger("right_middle_na");

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
        //quit();
        yarp::sig::Vector thumbAPose = thumbA->EndEffPose();
        yarp::sig::Vector thumbBPose = thumbB->EndEffPose();
        yarp::sig::Vector indexPose = index->EndEffPose();
        yarp::sig::Vector middlePose = middle->EndEffPose();

        for(size_t i = 0; i < thumbAPose.length(); i++){
            cout << dbgTag << thumbAPose[i] << " ";
        }
        cout << "/n";

        for(size_t i = 0; i < thumbBPose.length(); i++){
            cout << dbgTag << thumbBPose[i] << " ";
        }
        cout << "/n";

        for(size_t i = 0; i < indexPose.length(); i++){
            cout << dbgTag << indexPose[i] << " ";
        }
        cout << "/n";

        for(size_t i = 0; i < middlePose.length(); i++){
            cout << dbgTag << middlePose[i] << " ";
        }
        cout << "/n";



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
    
	taskThread->suspend();

	taskThread->afterRun(false);

	return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* RPC Stop task execution and open the hand                        ********************************************** */
bool PlantIdentificationModule::open() {
    
	taskThread->suspend();

	taskThread->afterRun(true);

	return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool PlantIdentificationModule::start() {

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
