#include "iCub/plantIdentification/PlantIdentificationModule.h"

#include <yarp/os/Time.h>

#include <sstream>
#include <fstream>

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

        if (taskData->commonData.classificationState == 1){
            taskData->commonData.classificationState = 2;
        }

        taskData->commonData.requestOpen = false;
        open(yarp::os::Value("wide"));
    }

    if (taskData->commonData.classificationState == 2){

        // TODO choose the proper function according to the task: completeClassification() is used with "classify", while getTactileClassifierOutput_step2() is used with "ml get_tactile_classifier_output"
        //completeClassification();
        getTactileClassifierOutput_step2();
        taskData->commonData.classificationState = 0;
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
        case CLASSIFY:
            ok = classify();
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

        //TODO TO REMOVE
        open(yarp::os::Value(""));
        Time::delay(3);

        task(EMPTY,NONE,Value(0));
        task(ADD,APPROACH,Value(0));
        task(ADD,CONTROL,Value(0));
        start();

        return true;

    }
}
/* *********************************************************************************************************************** */


bool PlantIdentificationModule::classify() {

    using yarp::os::Time;

    // wait a few seconds while putting the object in front of the robot
    Time::delay(4);

    // read the visual recognition average scores
    bool ok = portsUtil->readVisualClassifierAvgScores(taskData->commonData.vcAvgScores);

    if (!ok){
        std::cout << "<<< could not read visual recognition average scores (out of time) >>>" << std::endl;
        return false;
    }

    if (taskData->commonData.vcAvgScores.size() == 0){
        std::cout << "<<< could not read visual recognition average scores (returned 0 classes) >>>" << std::endl;
        return false;
    }

    taskData->commonData.classificationState = 1;
    // run the tactile classification (it will store the data in taskData->commonData.tactAvgScores)
    grasp();

    return true;
}



bool PlantIdentificationModule::completeClassification() {

    using std::string;

    int combiningMethod = 0;
    double combSumScale = 1.0;
    double combMaxmaxScale = 1.0;

    if (taskData->commonData.tactAvgScores.size() != taskData->commonData.vcAvgScores.size()){
        std::cout << "<<< tact num: " << taskData->commonData.tactAvgScores.size() << " - vc num: " << taskData->commonData.vcAvgScores.size() << " >>>" << std::endl;
        return false;
    }

    std::cout << "TACTILE CLASSIFIER SCORES" << std::endl;
    for (int i = 0; i < taskData->commonData.tactAvgScores.size(); i++){
        std::cout << "[ " << i + 1 << " : " << taskData->commonData.tactAvgScores[i] << " ] ";
    }
    std::cout << std::endl;

    std::cout << "VISUAL CLASSIFIER SCORES" << std::endl;
    for (int i = 0; i < taskData->commonData.vcAvgScores.size(); i++){
        std::cout << "[ " << i + 1 << " : " << taskData->commonData.vcAvgScores[i] << " ] ";
    }
    std::cout << std::endl;

    /*** compare the scores ***/
    int winningClass,sumWinningClass,maxmaxWinningClass;

    // avg method
    std::vector<double> scoresSum;
    scoresSum.resize(taskData->commonData.tactAvgScores.size());
    double currMax = -1000.0;
    double currMax_ind = -1;
    for (int i = 0; i < scoresSum.size(); i++){
        scoresSum[i] = combSumScale*taskData->commonData.tactAvgScores[i] + taskData->commonData.vcAvgScores[i];
        if (scoresSum[i] > currMax){
            currMax = scoresSum[i];
            currMax_ind = i;
        }
    }
    std::cout << "SCORES SUM" << std::endl;
    for (int i = 0; i < scoresSum.size(); i++){
        std::cout << "[ " << i + 1 << " : " << scoresSum[i] << " ] ";
    }
    std::cout << std::endl;

    sumWinningClass = currMax_ind;
    
    // maxmax method
    currMax = -1000.0;
    currMax_ind = -1;
    for (int i = 0; i < taskData->commonData.tactAvgScores.size(); i++){
        if (taskData->commonData.tactAvgScores[i] > currMax){
            currMax = taskData->commonData.tactAvgScores[i];
            currMax_ind = i;
        }
    }
    currMax = combMaxmaxScale*currMax;
    for (int i = 0; i < taskData->commonData.vcAvgScores.size(); i++){
        if (taskData->commonData.vcAvgScores[i] > currMax){
            currMax = taskData->commonData.vcAvgScores[i];
            currMax_ind = i;
        }
    }
    maxmaxWinningClass = currMax_ind;
    
    if (combiningMethod == 0){
        winningClass = sumWinningClass;
    }
    else if (combiningMethod == 1){
        winningClass = maxmaxWinningClass;
    }

    std::cout << "<<<<< WINNING CLASS: " << winningClass << "  sum/maxmax(" << sumWinningClass << "/" << maxmaxWinningClass << ") >>>>>" << std::endl;
    std::cout << "<<<<< WINNING CLASS: " << winningClass << "  sum/maxmax(" << sumWinningClass << "/" << maxmaxWinningClass << ") >>>>>" << std::endl;
    std::cout << "<<<<< WINNING CLASS: " << winningClass << "  sum/maxmax(" << sumWinningClass << "/" << maxmaxWinningClass << ") >>>>>" << std::endl;
    std::cout << "<<<<< WINNING CLASS: " << winningClass << "  sum/maxmax(" << sumWinningClass << "/" << maxmaxWinningClass << ") >>>>>" << std::endl;

    // store everything
    int objectIDInt = taskData->commonData.tpInt(100);
    int iterationIDInt = taskData->commonData.tpInt(101);
    string objectID = static_cast<std::ostringstream*>(&(std::ostringstream() << objectIDInt))->str();
    string iterationID = static_cast<std::ostringstream*>(&(std::ostringstream() << iterationIDInt))->str();
    if (objectIDInt < 10){
        objectID = "0" + objectID;
    }
    if (iterationIDInt < 10){
        iterationID = "0" + iterationID;
    }

    std::ofstream multiRecDataFileTouch;
    string fileNameMultiRecTouch = "obj" + objectID + "_task01_iter" + iterationID + ".dat";
    multiRecDataFileTouch.open(fileNameMultiRecTouch.c_str());
    for (int i = 0; i < taskData->commonData.tactAvgScores.size(); i++){
        multiRecDataFileTouch << taskData->commonData.tactAvgScores[i] << " ";
    }
    multiRecDataFileTouch << "\n";
    multiRecDataFileTouch.close();

    std::ofstream multiRecDataFileVision;
    string fileNameMultiRecVision = "obj" + objectID + "_task02_iter" + iterationID + ".dat";
    multiRecDataFileVision.open(fileNameMultiRecVision.c_str());
    for (int i = 0; i < taskData->commonData.vcAvgScores.size(); i++){
        multiRecDataFileVision << taskData->commonData.vcAvgScores[i] << " ";
    }
    multiRecDataFileVision << "\n";
    multiRecDataFileVision.close();

    return true;
}


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
    case GET_TACTILE_CLASSIFIER_OUTPUT:
        // get and store tactile classifier output
        getTactileClassifierOutput_step1();
        break;
    case GET_VISUAL_CLASSIFIER_OUTPUT:
        // get and store visual classifier output
        getVisualClassifierOutput();
        break;
    }

    return ok;
}
/* *********************************************************************************************************************** */

bool  PlantIdentificationModule::getTactileClassifierOutput_step1(){

    taskData->commonData.classificationState = 1;
    // run the tactile classification (it will store the data in taskData->commonData.tactAvgScores)
    grasp();

    return true;
}

bool  PlantIdentificationModule::getTactileClassifierOutput_step2(){

    using std::string;


    std::cout << "TACTILE CLASSIFIER SCORES" << std::endl;
    for (int i = 0; i < taskData->commonData.tactAvgScores.size(); i++){
        std::cout << "[ " << i + 1 << " : " << taskData->commonData.tactAvgScores[i] << " ] ";
    }
    std::cout << std::endl;

    // store everything
    int objectIDInt = taskData->commonData.tpInt(100);
    int iterationIDInt = taskData->commonData.tpInt(101);
    string objectID = static_cast<std::ostringstream*>(&(std::ostringstream() << objectIDInt))->str();
    string iterationID = static_cast<std::ostringstream*>(&(std::ostringstream() << iterationIDInt))->str();
    if (objectIDInt < 10){
        objectID = "0" + objectID;
    }
    if (iterationIDInt < 10){
        iterationID = "0" + iterationID;
    }

    std::ofstream multiRecDataFileTouch;
    string fileNameMultiRecTouch = "output/obj" + objectID + "_task01_iter" + iterationID + ".dat";
    multiRecDataFileTouch.open(fileNameMultiRecTouch.c_str());
    for (int i = 0; i < taskData->commonData.tactAvgScores.size(); i++){
        multiRecDataFileTouch << taskData->commonData.tactAvgScores[i] << " ";
    }
    multiRecDataFileTouch << "\n";
    multiRecDataFileTouch.close();

    return true;
}

bool  PlantIdentificationModule::getVisualClassifierOutput(){

    using yarp::os::Time;
    using std::string;

    // read the visual recognition average scores
    bool ok = portsUtil->readVisualClassifierAvgScores(taskData->commonData.vcAvgScores);

    if (!ok){
        std::cout << "<<< could not read visual recognition average scores (out of time) >>>" << std::endl;
        return false;
    }

    if (taskData->commonData.vcAvgScores.size() == 0){
        std::cout << "<<< could not read visual recognition average scores (returned 0 classes) >>>" << std::endl;
        return false;
    }


    std::cout << "VISUAL CLASSIFIER SCORES" << std::endl;
    for (int i = 0; i < taskData->commonData.vcAvgScores.size(); i++){
        std::cout << "[ " << i + 1 << " : " << taskData->commonData.vcAvgScores[i] << " ] ";
    }
    std::cout << std::endl;


    // store everything
    int objectIDInt = taskData->commonData.tpInt(100);
    int iterationIDInt = taskData->commonData.tpInt(101);
    string objectID = static_cast<std::ostringstream*>(&(std::ostringstream() << objectIDInt))->str();
    string iterationID = static_cast<std::ostringstream*>(&(std::ostringstream() << iterationIDInt))->str();
    if (objectIDInt < 10){
        objectID = "0" + objectID;
    }
    if (iterationIDInt < 10){
        iterationID = "0" + iterationID;
    }

    std::ofstream multiRecDataFileVision;
    string fileNameMultiRecVision = "obj" + objectID + "_task02_iter" + iterationID + ".dat";
    multiRecDataFileVision.open(fileNameMultiRecVision.c_str());
    for (int i = 0; i < taskData->commonData.vcAvgScores.size(); i++){
        multiRecDataFileVision << taskData->commonData.vcAvgScores[i] << " ";
    }
    multiRecDataFileVision << "\n";
    multiRecDataFileVision.close();

    return true;
    return true;
}

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
