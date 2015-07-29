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

    iCubFinger *thumbA = new iCubFinger("left_thumb_a");
    iCubFinger *thumbB = new iCubFinger("left_thumb_b");
    iCubFinger *index = new iCubFinger("left_index_na");
    iCubFinger *middle = new iCubFinger("left_middle_na");

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
//        yarp::sig::Vector thumbAPose = thumbA->EndEffPose();
//        yarp::sig::Vector thumbBPose = thumbB->EndEffPose();
//        yarp::sig::Vector indexPose = index->EndEffPose();
//        yarp::sig::Vector middlePose = middle->EndEffPose();
//        yarp::sig::Vector thumbAPosition = thumbA->EndEffPosition();
//        yarp::sig::Vector thumbBPosition = thumbB->EndEffPosition();
//        yarp::sig::Vector indexPosition = index->EndEffPosition();
//        yarp::sig::Vector middlePosition = middle->EndEffPosition();

        yarp::sig::Vector fingEnc;

        yarp::sig::Vector thumbACJ;
        yarp::sig::Vector thumbBCJ;
        yarp::sig::Vector indexCJ;
        yarp::sig::Vector middleCJ;

        thumbA->getChainJoints(fingEnc,thumbACJ);
        thumbB->getChainJoints(fingEnc,thumbBCJ);
        index->getChainJoints(fingEnc,indexCJ);
        middle->getChainJoints(fingEnc,middleCJ);

        yarp::sig::Vector thumbAPose = thumbA->EndEffPose(thumbACJ);
        yarp::sig::Vector thumbBPose = thumbB->EndEffPose(thumbBCJ);
        yarp::sig::Vector indexPose = index->EndEffPose(indexCJ);
        yarp::sig::Vector middlePose = middle->EndEffPose(middleCJ);
        yarp::sig::Vector thumbAPosition = thumbA->EndEffPosition(thumbACJ);
        yarp::sig::Vector thumbBPosition = thumbB->EndEffPosition(thumbBCJ);
        yarp::sig::Vector indexPosition = index->EndEffPosition(indexCJ);
        yarp::sig::Vector middlePosition = middle->EndEffPosition(middleCJ);

        yarp::sig::Matrix thumbAH = thumbA->getH(thumbACJ);
        yarp::sig::Matrix thumbBH = thumbA->getH(thumbBCJ);
        yarp::sig::Matrix indexH = thumbA->getH(indexCJ);
        yarp::sig::Matrix middleH = thumbA->getH(middleCJ);


        cout << dbgTag << "thumb A pose: ";
        for(size_t i = 0; i < thumbAPose.length(); i++){
            cout << thumbAPose[i] << " ";
        }
        cout << "\n";
        cout << dbgTag << "thumb A position: ";
        for(size_t i = 0; i < thumbAPosition.length(); i++){
            cout << thumbAPosition[i] << " ";
        }
        cout << "\n";

        cout << dbgTag << "thumb B pose: ";
        for(size_t i = 0; i < thumbBPose.length(); i++){
            cout << thumbBPose[i] << " ";
        }
        cout << "\n";
        cout << dbgTag << "thumb B position: ";
        for(size_t i = 0; i < thumbBPosition.length(); i++){
            cout << thumbBPosition[i] << " ";
        }
        cout << "\n";

        cout << dbgTag << "index pose: ";
        for(size_t i = 0; i < indexPose.length(); i++){
            cout << indexPose[i] << " ";
        }
        cout << "\n";
        cout << dbgTag << "index position: ";
        for(size_t i = 0; i < indexPosition.length(); i++){
            cout << indexPosition[i] << " ";
        }
        cout << "\n";

        cout << dbgTag << "middle pose: ";
        for(size_t i = 0; i < middlePose.length(); i++){
            cout << middlePose[i] << " ";
        }
        cout << "\n";
        cout << dbgTag << "middle position: ";
        for(size_t i = 0; i < middlePosition.length(); i++){
            cout << middlePosition[i] << " ";
        }
        cout << "\n";

        cout << "------------";

        yarp::sig::Vector thumbAPos = thumbAH.getCol(3);
        yarp::sig::Vector thumbADir = thumbAH.getCol(1);

        yarp::sig::Vector thumbBPos = thumbBH.getCol(3);
        yarp::sig::Vector thumbBDir = thumbBH.getCol(1);

        yarp::sig::Vector indexPos = indexH.getCol(3);
        yarp::sig::Vector indexDir = indexH.getCol(1);

        yarp::sig::Vector middlePos = middleH.getCol(3);
        yarp::sig::Vector middleDir = middleH.getCol(1);


        cout << dbgTag << "thumbA pose: ";
        for(size_t i = 0; i < thumbAPos.length(); i++){
            cout << thumbAPos[i] << " ";
        }
        cout << "\n";
        cout << dbgTag << "thumbA dir: ";
        for(size_t i = 0; i < thumbADir.length(); i++){
            cout << thumbADir[i] << " ";
        }
        cout << "\n";

        cout << dbgTag << "thumbB pose: ";
        for(size_t i = 0; i < thumbBPos.length(); i++){
            cout << thumbBPos[i] << " ";
        }
        cout << "\n";
        cout << dbgTag << "thumbB dir: ";
        for(size_t i = 0; i < thumbBDir.length(); i++){
            cout << thumbBDir[i] << " ";
        }
        cout << "\n";

        cout << dbgTag << "index pose: ";
        for(size_t i = 0; i < indexPos.length(); i++){
            cout << indexPos[i] << " ";
        }
        cout << "\n";
        cout << dbgTag << "index dir: ";
        for(size_t i = 0; i < indexDir.length(); i++){
            cout << indexDir[i] << " ";
        }
        cout << "\n";

        cout << dbgTag << "middle pose: ";
        for(size_t i = 0; i < middlePos.length(); i++){
            cout << middlePos[i] << " ";
        }
        cout << "\n";
        cout << dbgTag << "middle dir: ";
        for(size_t i = 0; i < middleDir.length(); i++){
            cout << middleDir[i] << " ";
        }
        cout << "\n";

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
