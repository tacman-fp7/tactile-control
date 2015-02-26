#include "iCub/plantIdentification/PlantIdentificationModule.h"

using iCub::plantIdentification::PlantIdentificationModule;

using std::cout;

using yarp::os::ResourceFinder;
using yarp::os::Value;


/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
PlantIdentificationModule::PlantIdentificationModule() 
    : RFModule(), plantIdentification_IDLServer() {
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
    portPlantIdentificationRPC.open("/plantIdentification/cmd:io");
    attach(portPlantIdentificationRPC);

    /* ******* Threads                                          ******* */
    taskThread = new TaskThread(20, rf);
    if (!taskThread->start()) {
        cout << dbgTag << "Could not start the grasp thread. \n";
        return false;
    }
    taskThread->suspend();

    cout << dbgTag << "Started correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Attach RPC port                                                  ********************************************** */   
bool PlantIdentificationModule::attach(yarp::os::RpcServer &source) {
    return this->yarp().attachAsServer(source);
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
/* ******* RPC Open hand                                                    ********************************************** */
bool PlantIdentificationModule::open(void) {
    
	taskThread->suspend();
    
	taskThread->openHand();

	return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool PlantIdentificationModule::grasp(void) {

	taskThread->initializeGrasping();

	taskThread->resume();

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Quit module                                                  ********************************************** */
bool PlantIdentificationModule::quit(void) {
    return closing = true;
}
/* *********************************************************************************************************************** */


void PlantIdentificationModule::set(iCub::plantIdentification::SetParamName paramName,std::string paramValue){
	taskThread->set(paramName,paramValue);
	taskThread->view(SETTINGS);
}

void PlantIdentificationModule::task(iCub::plantIdentification::TaskParamName paramName,double targetValue = 0){
	taskThread->task(paramName,targetValue);
	taskThread->view(TASKS);
}

void PlantIdentificationModule::view(iCub::plantIdentification::ViewParamName paramName){
	taskThread->view(paramName);
}