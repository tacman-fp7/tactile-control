#include "iCub/plantIdentification/PlantIdentificationModule.h"

#include <yarp/os/Time.h>

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
    moduleName = rf.check("name", Value("stableGrasp")).asString().c_str();
    string whichHand = rf.check("whichHand", Value("right")).asString().c_str();
    moduleThreadPeriod = rf.check("moduleThreadPeriod", 1000).asInt();
    bool specifyHand = rf.check("specifyHand",Value(0)).asInt() != 0;
    string portPrefix;
    if (specifyHand){
        portPrefix = "/" + moduleName + "/" + whichHand + "_hand";
    } else {
        portPrefix = "/" + moduleName;
    }


    /* ******* Open ports                                       ******* */
    portPlantIdentificationRPC.open(portPrefix + "/cmd:i");
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

    // initialize machine learning utililty
    mlUtil.init(rf,portsUtil);

    // initialize task data
    taskData = new TaskData(rf,controllersUtil);


    /* ******* Threads                                          ******* */
    taskThread = new TaskThread(taskData->commonData.taskThreadPeriod,rf,controllersUtil,portsUtil,&mlUtil,taskData);
    if (!taskThread->start()) {
        cout << dbgTag << "Could not start the task thread. \n";
        return false;
    }
    taskThread->suspend();

    eventsThread = new EventsThread(rf,taskData->commonData.eventsThreadPeriod,controllersUtil,portsUtil,&taskData->commonData);
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
/*    if (eventsThread->eventTriggered(FINGERTIP_PUSHED,3)){ // pinky
        if (tasksRunning){
            open(yarp::os::Value(""));
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
*/

    if (taskData->commonData.requestOpen == true){
        taskData->commonData.requestOpen = false;
        open(yarp::os::Value(""));
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

    bool ok = rpcCmdUtil.processCommand(command);

    if (ok){

        switch (rpcCmdUtil.mainCmd){

        case HELP:
            ok = help();
            break;
        case SET:
            ok = set(rpcCmdUtil.setCmdArg,rpcCmdUtil.argValue);
            break;
        case TASK:
            ok = task(rpcCmdUtil.taskCmdArg,rpcCmdUtil.task,rpcCmdUtil.argValue);
            break;
        case VIEW:
            view(rpcCmdUtil.viewCmdArg);
            break;
        case START:
            ok = start();
            break;
        case STOP:
            ok = stop();
            break;
        case OPEN:
            ok = open(rpcCmdUtil.argValue);
            break;
        case ARM:
            ok = arm(rpcCmdUtil.argValue);
            break;
        case GRASP:
            ok = grasp();
            break;
        case ML:
            ok = ml(rpcCmdUtil.mlCmdArg,rpcCmdUtil.argValue);
            break;
        case WAVE:
            ok = wave();
            break;
        case QUIT:
            ok = quit();
            break;
        }
    }

    if (ok){
        reply.addString("ack");
    } else {
        reply.addString("nack");
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
    
    if (tasksRunning == true){

        tasksRunning = false;

        taskThread->suspend();

        taskThread->afterRun(false);

        return true;

    } else {

        return false;

    }

}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* RPC Stop task execution and open the hand                        ********************************************** */
bool PlantIdentificationModule::open(Value paramValue) {

    bool fingersAreStraight;
    if (paramValue.asString() == "wide"){
        fingersAreStraight = true;
    } else {
        fingersAreStraight = false;
    }

    if (tasksRunning == true){
        tasksRunning = false;

        taskThread->suspend();

        taskThread->afterRun(true);
    } else {
        controllersUtil->openHand(fingersAreStraight);
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool PlantIdentificationModule::start() {

    if (tasksRunning == true){
         
        return false;

    } else {

        tasksRunning = true;

        if (!taskThread->initializeGrasping()) return false;

        taskThread->resume();

        return true;

    }
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* RPC Set arm in task position                                     ********************************************** */
bool PlantIdentificationModule::arm(Value paramValue) {

    return taskThread->setArmPosition(paramValue);
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Execute the grasp task                                       ********************************************** */
bool PlantIdentificationModule::grasp() {

    using yarp::os::Time;

    if (tasksRunning == true){
         
        return false;

    } else {

        bool hysteresisThresholdsStorageEnabled = taskData->commonData.tpInt(82) != 0;

        if (hysteresisThresholdsStorageEnabled){

            std::cout << "Setting hysteresis thresholds...\n"; 
            double rate = 1.0 + taskData->commonData.tpDbl(81)/100.0;
            Time::delay(1);
            taskData->commonData.tempParameters[78] = rate * taskData->commonData.overallFingerPressureMedian[4]; // thumb
            taskData->commonData.tempParameters[79] = rate * taskData->commonData.overallFingerPressureMedian[0]; // index finger
            taskData->commonData.tempParameters[80] = rate * taskData->commonData.overallFingerPressureMedian[1]; // middle finger
            std::cout << "Done.\n"; 
        }

        task(EMPTY,NONE,Value(0));
        task(ADD,APPROACH,Value(0));
        task(ADD,CONTROL,Value(0));
        start();

        return true;

    }
}
/* *********************************************************************************************************************** */

/* ******* RPC Execute the wave action                                       ********************************************** */
bool PlantIdentificationModule::wave() {
    using iCub::plantIdentification::Wave;

    // TODO read parameters from keyboard
    Wave waveType = static_cast<Wave>(taskData->commonData.tpInt(60));
    double waveAmplitude = taskData->commonData.tpDbl(61);
    double wavePeriod = taskData->commonData.tpDbl(62);
    int armJoint = taskData->commonData.tpInt(63);
    double actionDuration = taskData->commonData.tpDbl(64);

    eventsThread->setWaveAction(actionDuration,armJoint,wavePeriod,waveAmplitude,waveType);

    return true;
}
/* *********************************************************************************************************************** */

/* ******* RPC Manage machine learning related commands                                       ********************************************** */
bool PlantIdentificationModule::ml(iCub::plantIdentification::RPCMlCmdArgName paramName,Value paramValue) {

    if (paramName == SAVE_MODEL || 
        paramName == LOAD_MODEL || 
        paramName == LOAD_TRAINING_SET || 
        paramName == LOAD_TEST_SET || 
        paramName == LOAD_OBJECT_NAMES || 
        paramName == LOAD_TRAINING_SET_AND_OBJECT_NAMES || 
        paramName == SAVE_TRAINING_SET || 
        paramName == SAVE_OBJECT_NAMES || 
        paramName == SAVE_TRAINING_SET_AND_OBJECT_NAMES || 
        paramName == GET_READY){

            if (paramValue.toString() == "default"){
                paramValue = Value(taskData->commonData.objRecDataDefaultSuffix);
            }
    }

    bool ok = true;

    switch(paramName){

    case VIEW_DATA:
        // view data for debugging
        ok = mlUtil.viewData();
        break;
    case TRAIN:
        // train the classifier
        ok = mlUtil.trainClassifier();
        break;
    case MODE:
        // set classification mode
        if (paramValue.asString() == "off"){
            taskData->commonData.tempParameters[95] = 0;
        } else if (paramValue.asString() == "std"){
            taskData->commonData.tempParameters[95] = 1;
        } else if (paramValue.asString() == "avg"){
            taskData->commonData.tempParameters[95] = 2;
        } else if (paramValue.asString() == "max"){
            taskData->commonData.tempParameters[95] = 3;
        } else { // default
            taskData->commonData.tempParameters[95] = 1;
        }
        break;
    case TEST:
        // test the classifier
        ok = mlUtil.testClassifier();
        break;
    case SAVE_MODEL:
        // save the learned model to file
        ok = mlUtil.saveModelToFile(paramValue.asString());
        break;
    case LOAD_MODEL:
        // load model from file
        ok = mlUtil.loadModelFromFile(paramValue.asString());
        break;
    case LOAD_TRAINING_SET:
        // load training set from files
        ok = mlUtil.loadTrainingSetFromFile(paramValue.asString());
        break;
    case LOAD_TEST_SET:
        // load test set from files
        ok = mlUtil.loadTestSetFromFile(paramValue.asString());
        break;
    case LOAD_OBJECT_NAMES:
        // load object names list
        ok = mlUtil.loadObjectNamesFromFile(paramValue.asString());
        break;
    case LOAD_TRAINING_SET_AND_OBJECT_NAMES:
        // load training set and object names list
        ok = mlUtil.loadTrainingSetFromFile(paramValue.asString());
        ok = mlUtil.loadObjectNamesFromFile(paramValue.asString());
        break;
    case SAVE_TRAINING_SET:
        // save training set from files
        ok = mlUtil.saveTrainingSetToFile(paramValue.asString());
        break;
    case SAVE_OBJECT_NAMES:
        // save object names list
        ok = mlUtil.loadObjectNamesFromFile(paramValue.asString());
        break;
    case SAVE_TRAINING_SET_AND_OBJECT_NAMES:
        // save training set and object names list
        ok = mlUtil.saveTrainingSetToFile(paramValue.asString());
        ok = mlUtil.saveObjectNamesToFile(paramValue.asString());
        break;
    case LEARN_NEW_OBJECT:
        // start the mode "learning a new object"
        ok = mlUtil.initNewObjectLearning(paramValue.asString(),false);
        break;
    case REFINE_NEW_OBJECT:
        // learn more features for the previous learned object
        ok = mlUtil.initNewObjectLearning(paramValue.asString(),true);
        break;    
    case DISCARD_LAST_FEATURES:
        // discard last collected features
        ok = mlUtil.discardLastCollectedFeatures();
        break;
    case PROCESS_COLLECTED_DATA:
        // process the collected data
        ok = mlUtil.processCollectedData();
        break;
    case GET_READY:
        // get ready
        ok = mlUtil.loadObjectNamesFromFile(paramValue.asString());
        if (ok){
            // load training set from files
            ok = mlUtil.loadTrainingSetFromFile(paramValue.asString());
            if (ok){
                // train the classifier
                ok = mlUtil.trainClassifier();
            }
        }
        break;
    case RESET:
        // reset the collected data
        ok = mlUtil.reset();
        break;
    }

    return ok;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Quit module                                                  ********************************************** */
bool PlantIdentificationModule::quit() {

    closing = true;
    return true;
}
/* *********************************************************************************************************************** */


bool PlantIdentificationModule::set(iCub::plantIdentification::RPCSetCmdArgName paramName,Value paramValue){
    taskThread->set(paramName,paramValue,rpcCmdData);
    view(SETTINGS);
    return true;
}

bool PlantIdentificationModule::task(iCub::plantIdentification::RPCTaskCmdArgName paramName,iCub::plantIdentification::TaskName taskName,Value paramValue){
    taskThread->task(paramName,taskName,paramValue,rpcCmdData);
    view(TASKS);
    return true;
}

bool PlantIdentificationModule::view(iCub::plantIdentification::RPCViewCmdArgName paramName){
    taskThread->view(paramName,rpcCmdData);
    return true;
}

bool PlantIdentificationModule::help(){
	controllersUtil->setArmHomeAsCurrent();
    taskThread->help(rpcCmdData);
    return true;
}
