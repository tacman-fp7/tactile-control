#include "iCub/plantIdentification/util/ControllersUtil.h"

#include <iCub/iKin/iKinFwd.h>

#include <vector>
#include <ctime>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::FingerJoint;

using std::string;
using std::cout;
using yarp::os::Property;


ControllersUtil::ControllersUtil(){

	dbgTag = "ControllersUtil: ";
}

bool ControllersUtil::init(yarp::os::ResourceFinder &rf){
	using std::vector;
	using yarp::os::Value;

	//TODO use constants
	storedJointsControlMode.resize(8,VOCAB_CM_POSITION);
	storedHandJointsMaxPwmLimits.resize(8);
	
	string robotName = rf.check("robot", Value("icub"), "The robot name.").asString().c_str();
    whichHand = rf.check("whichHand", Value("right"), "The hand to be used for the grasping.").asString().c_str();
    whichICub = rf.check("whichICub", Value("purple"), "The iCub used for the task.").asString().c_str();
    whichTask = rf.check("whichTask", Value("grasp"), "The code of the task [grasp/objrec]").asString().c_str();
    numFingers = rf.check("numFingers", Value(2), "Number of fingers used").asInt();
    headEnabled = rf.check("headEnabled",Value(0)).asInt() != 0;
    std::cout << whichHand << " " << whichICub << " " << whichTask << "\n";
	 /* ******* Joint interfaces                     ******* */
    string arm = whichHand + "_arm";
    Property options;
    options.put("robot", robotName.c_str()); 
    options.put("device", "remote_controlboard");
    options.put("part", arm.c_str());
    options.put("local", ("/plantIdentification/" + arm).c_str());
    options.put("remote", ("/" + robotName + "/" + arm).c_str());
    
    // Open driver
    if (!clientArm.open(options)) {
        cout << dbgTag << "could not open arm driver\n";
		return false;
    }
    // Open interfaces
    clientArm.view(iEncs);
    if (!iEncs) {
		cout << dbgTag << "could not open encoders interface\n";
        return false;
    }
	clientArm.view(iOLC);
    if (!iOLC) {
		cout << dbgTag << "could not open open-loop interface\n";
        return false;
    }
	clientArm.view(iCtrl);
    if (!iCtrl) {
		cout << dbgTag << "could not open control mode interface\n";
        return false;
    }
	clientArm.view(iPos);
    if (!iPos) {
		cout << dbgTag << "could not open position interface\n";
        return false;
    }
	clientArm.view(iVel);
    if (!iVel) {
		cout << dbgTag << "could not open velocity interface\n";
        return false;
    }
	clientArm.view(iPosDir);
    if (!iPosDir) {
		cout << dbgTag << "could not open position direct interface\n";
        return false;
    }
	clientArm.view(iPid);
    if (!iPid) {
		cout << dbgTag << "could not open pid interface\n";
        return false;
    }

	iPos->getAxes(&armJointsNum);
	
	// Set reference speeds
    vector<double> refSpeeds(armJointsNum, 0);
    iPos->getRefSpeeds(&refSpeeds[0]);
    for (int i = 11; i < 15; ++i) {
        refSpeeds[i] = 50;
    }
    iPos->setRefSpeeds(&refSpeeds[0]);

	if (headEnabled){

		Property gazeCtrlOptions;
		gazeCtrlOptions.put("device","gazecontrollerclient");
		gazeCtrlOptions.put("remote","/iKinGazeCtrl");
		gazeCtrlOptions.put("local","/plantIdentification/gaze");
 
		if (!clientGazeCtrl.open(gazeCtrlOptions)) {
			cout << dbgTag << "could not open gaze controller driver\n";
			return false;
		}
	
		clientGazeCtrl.view(iGaze);
		if (!iGaze) {
			cout << dbgTag << "could not open gaze controller interface\n";
			return false;
		}

		// store fixation point
		iGaze->getFixationPoint(storedFixationPoint);


	   
		Property cartCtrlOptions;
		cartCtrlOptions.put("device","cartesiancontrollerclient");
		cartCtrlOptions.put("remote","/" + robotName + "/cartesianController/" + arm);
		cartCtrlOptions.put("local","/plantIdentification/cartContr/" + arm);
    
		clientArmCartCtrl.open(cartCtrlOptions);

		if (clientArmCartCtrl.isValid()) {
		   clientArmCartCtrl.view(iCart);
		}
		if (!iCart) {
			cout << dbgTag << "could not open cartesian controller interface\n";
		}

	}
	return true;
}

bool ControllersUtil::sendPwm(int joint,double pwm){

	if (!iOLC->setRefOutput(joint,pwm)){
		cout << dbgTag << "could not send pwm\n";
		return false;
	}
	return true;
}

bool ControllersUtil::sendVelocity(int joint,double velocity){

	if (!iVel->velocityMove(joint,velocity)){
		cout << dbgTag << "could not send velocity\n";
		return false;
	}
	return true;
}

bool ControllersUtil::saveCurrentArmPosition(){
	using yarp::os::Time;

    armStoredPosition.resize(armJointsNum);
    
	bool encodersDataAcquired = false;
    while(!encodersDataAcquired) {

        encodersDataAcquired = iEncs->getEncoders(armStoredPosition.data());

//#ifndef NODEBUG
        cout << "DEBUG: " << dbgTag << "Encoder data is not available yet. \n";
//#endif

		Time::delay(0.1);
    }

#ifndef NODEBUG
    cout << "\n";
    cout << "DEBUG: " << dbgTag << "Stored initial arm positions are: ";
    for (size_t i = 0; i < armStoredPosition.size(); ++i) {
        cout << armStoredPosition[i] << " ";
    }
    cout << "\n";
#endif

	return true;
}


bool ControllersUtil::getArmEncodersAngles(std::vector<double> &armEncodersAngles,bool wait){
	using yarp::os::Time;
	
	yarp::sig::Vector armEncodersAnglesVector;
    armEncodersAnglesVector.resize(armJointsNum);

	bool encodersDataAcquired = false;

	encodersDataAcquired = iEncs->getEncoders(armEncodersAnglesVector.data());

	while(wait && !encodersDataAcquired) {

		cout << "DEBUG: " << dbgTag << "Encoder data is not available yet. \n";
		Time::delay(0.1);

		encodersDataAcquired = iEncs->getEncoders(armEncodersAnglesVector.data());
    
	}

	if (encodersDataAcquired){
		for(size_t i = 0; i < armEncodersAngles.size(); i++){
			armEncodersAngles[i] = armEncodersAnglesVector[i];
		}

		return true;
	}

	return false;
}


bool ControllersUtil::saveCurrentControlMode(){

	// save control mode of joints 8 9 10 11 12 13 14 15
	for(size_t i = 0; i < 8; i++){
		if (!iCtrl->getControlMode(8 + i,&storedJointsControlMode[i])){
			cout << dbgTag << "could not get current control mode\n";
			return false;
		}
	}
	return true;
}

bool ControllersUtil::setTaskControlModes(std::vector<int> &jointsList,int controlMode){

	for(size_t i = 0; i < jointsList.size(); i++){
		if (!setControlMode(jointsList[i],controlMode,true)){
			cout << dbgTag << "could not set all control modes\n";
			return false;
		}
	}
	return true;
}


/* ******* Place arm in grasping position                                   ********************************************** */ 
bool ControllersUtil::setArmInTaskPosition() {

    cout << dbgTag << "Reaching arm task position ... \t";
    
	iVel->stop();

    if (whichTask == "grasp"){

        // Arm
        iPos->positionMove(0 ,-35);
        iPos->positionMove(1 , 29);
        iPos->positionMove(2 , 21);
        iPos->positionMove(3 , 33);
        
        iPos->positionMove(4 , -20);// 0
        iPos->positionMove(5 , 2);// 1
        iPos->positionMove(6 , -20);// 1
        iPos->positionMove(7 , 12);
        
        // Hand
        if (whichICub == "black"){
            if (whichHand == "right"){
                iPos->positionMove(8 , 46);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 37);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 52);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 43);
                iPos->positionMove(15, 1);
            } else {
                iPos->positionMove(8 , 46);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 15);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            }
        } else {
            if (whichHand == "right"){
                iPos->positionMove(8 , 38);    
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 15);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            } else {
                iPos->positionMove(8 , 72);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 15);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            }
        }


    } else {

    // Arm
        iPos->positionMove(0 ,-35);
        iPos->positionMove(1 , 29);
        iPos->positionMove(2 , 21);
        iPos->positionMove(3 , 33);
        
        iPos->positionMove(4 , -20);// 0
        iPos->positionMove(5 , 2);// 1
        iPos->positionMove(6 , -20);// 1
        iPos->positionMove(7 , 12);
        
        // Hand
        if (whichICub == "black"){
            if (whichHand == "right"){
                iPos->positionMove(8 , 82);    
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 0);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            } else {
                iPos->positionMove(8 , 82);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 0);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            }
        } else {
            if (whichHand == "right"){
                iPos->positionMove(8 , 82);    
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 0);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            } else {
                iPos->positionMove(8 , 82);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 0);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            }
        }


    }

    // Check motion done
    waitMoveDone(10, 1);
	cout << "Done. \n";

	return true;
}
/* *********************************************************************************************************************** */

bool ControllersUtil::restorePreviousArmPosition(){
	
	// Stop interfaces
    if (iVel) {
        iVel->stop();
    }
    if (iPos) {
        iPos->stop();
        // Restore initial robot position
        iPos->positionMove(armStoredPosition.data());
    }

	return true;
}

bool ControllersUtil::restorePreviousControlMode(){

	// restore control modes from joints 8 9 10 11 12 13 14 15
	for(size_t i = 0; i < 8; i++){
		if (!setControlMode(8 + i,storedJointsControlMode[i],true)){
			cout << dbgTag << "could not set all control modes\n";
			return false;
		}
	}

	return true;
}

bool ControllersUtil::setControlMode(int joint,int controlMode,bool checkCurrent){

	if (checkCurrent){
		int currentControlMode = -1;
		if (iCtrl->getControlMode(joint,&currentControlMode)){

			if (currentControlMode != controlMode){
				if  (iCtrl->setControlMode(joint,controlMode)){
					cout << dbgTag << "CONTROL MODE SET TO " << controlMode << " ON JOINT " << joint << "   PREV: " << currentControlMode << "  OPEN: " << VOCAB_CM_OPENLOOP << "\n";
					return true;
				} else {
                    cout << dbgTag << "failed to SET control mode on joint " << joint << "\n";                   
                    return false;
                }	
			} else {
                cout << dbgTag << "open loop control mode already set (" << currentControlMode << ")\n";
                return true;
            }
		} else {
            cout << dbgTag << "failed to GET control mode from joint " << joint << " (controlMode appears to be " << currentControlMode << ")\n";           
            return false;
        }
	} else return iCtrl->setControlMode(joint,controlMode);
	
    return true;
}

/* ******* Wait for motion to be completed.                                 ********************************************** */
bool ControllersUtil::waitMoveDone(const double &i_timeout, const double &i_delay) {
    using yarp::os::Time;
    
    bool ok = false;
    
    double start = Time::now();

    while (!ok && (start - Time::now() <= i_timeout)) {
        iPos->checkMotionDone(&ok);
        Time::delay(i_delay);
    }

    return ok;
}
/* *********************************************************************************************************************** */


bool ControllersUtil::getEncoderAngle(int joint,double *encoderData){
	
	bool ok;

	ok = iEncs->getEncoder(joint,encoderData);
	
	if (!ok){
		cout << dbgTag << "could not get encoder value\n";
	}
	return ok;
}

//TODO restore getRealPwmValue
//bool ControllersUtil::getRealPwmValue(FingerJoint fingerJoint,double *pwmValue){
//
//	bool ok;
//
//	switch (fingerJoint){
//
//	case PROXIMAL:
//		ok = iOLC->getOutput(jointToMove,pwmValue);
//		break;
//
//	case DISTAL:
//		ok = iOLC->getOutput(jointToMove + 1,pwmValue);
//		break;
//	}
//
//	if (!ok){
//		cout << dbgTag << "could not get pwm value\n";
//	}
//	return ok;
//}

bool ControllersUtil::release(){

    // Close drivers
    if (!clientArm.close()){
		cout << dbgTag << "could not close arm driver\n";	
		return false;
	}
    if (!clientGazeCtrl.close()){
		cout << dbgTag << "could not close gaze controller driver\n";	
		return false;
	}
	return true;
}

/* *********************************************************************************************************************** */
/* ******* Open hand                                                        ********************************************** */
bool ControllersUtil::openHand() {
    
	cout << dbgTag << "Opening hand ... \t";
    
    iVel->stop();

    cout << dbgTag << whichICub << " " << whichHand << "\n";

    if (whichTask == "grasp"){

        // Hand
        if (whichICub == "black"){
            if (whichHand == "right"){
                iPos->positionMove(8 , 46);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 37);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 52);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 43);
                iPos->positionMove(15, 1);
            } else {
                iPos->positionMove(8 , 46);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 15);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            }
        } else {
            if (whichHand == "right"){
                iPos->positionMove(8 , 38);    
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 15);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            } else {
                iPos->positionMove(8 , 72);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 15);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            }
        }


    } else {

        // Hand
        if (whichICub == "black"){
            if (whichHand == "right"){
                iPos->positionMove(8 , 82);    
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 0);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            } else {
                iPos->positionMove(8 , 82);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 0);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            }
        } else {
            if (whichHand == "right"){
                iPos->positionMove(8 , 82);    
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 0);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            } else {
                iPos->positionMove(8 , 82);
                iPos->positionMove(9 , 3);
                iPos->positionMove(10, 0);
                iPos->positionMove(11, 2);
                iPos->positionMove(12, numFingers == 2? 0 : 30);
                iPos->positionMove(13, 3);
                iPos->positionMove(14, 30);
                iPos->positionMove(15, 1);
            }
        }


    }


    // Check motion done
    waitMoveDone(10, 1);
    cout << "Done. \n";

	return true;
}
/* *********************************************************************************************************************** */

bool ControllersUtil::setJointMaxPwmLimit(int joint,double maxPwm){

	yarp::dev::Pid pid;

	if (iPid->getPid(joint,&pid)){
		pid.max_output = maxPwm;
		iPid->setPid(joint,pid);
		return true;
	}
	return false;
}

bool ControllersUtil::setJointsMaxPwmLimit(std::vector<int> &jointsList,std::vector<double> &maxPwmList){

	bool ok = true;

	for(size_t i = 0; i < jointsList.size(); i++){
		ok = ok && setJointMaxPwmLimit(jointsList[i],maxPwmList[i]);
	}

	return ok;
}

bool ControllersUtil::saveHandJointsMaxPwmLimits(){

	//TODO use constants
	for(size_t i = 0; i < 8; i++){
		yarp::dev::Pid pid;
		if (iPid->getPid(8 + i,&pid)){
			storedHandJointsMaxPwmLimits[i] = pid.max_output;
		}
	}

	return true;
}

bool ControllersUtil::restoreHandJointsMaxPwmLimits(){

	//TODO use constants
	for(size_t i = 0; i < 8; i++){
		yarp::dev::Pid pid;
		if (iPid->getPid(8 + i,&pid)){
			pid.max_output = storedHandJointsMaxPwmLimits[i];
			iPid->setPid(8 + i,pid);
		}
	}

	return true;
}

void ControllersUtil::testShowEndEffectors(){

    using iCub::iKin::iCubFinger;

    saveCurrentArmPosition();

    iCubFinger *thumbA = new iCubFinger("right_thumb_a");
    iCubFinger *thumbB = new iCubFinger("right_thumb_b");
    iCubFinger *index = new iCubFinger("right_index_na");
    iCubFinger *middle = new iCubFinger("right_middle_na");

//        yarp::sig::Vector thumbAPose = thumbA->EndEffPose();
//        yarp::sig::Vector thumbBPose = thumbB->EndEffPose();
//        yarp::sig::Vector indexPose = index->EndEffPose();
//        yarp::sig::Vector middlePose = middle->EndEffPose();
//        yarp::sig::Vector thumbAPosition = thumbA->EndEffPosition();
//        yarp::sig::Vector thumbBPosition = thumbB->EndEffPosition();
//        yarp::sig::Vector indexPosition = index->EndEffPosition();
//        yarp::sig::Vector middlePosition = middle->EndEffPosition();

    yarp::sig::Vector thumbAEnc;
    yarp::sig::Vector thumbBEnc;
    yarp::sig::Vector indexEnc;
    yarp::sig::Vector middleEnc;

    yarp::sig::Vector thumbACJ;
    yarp::sig::Vector thumbBCJ;
    yarp::sig::Vector indexCJ;
    yarp::sig::Vector middleCJ;

    thumbA->getChainJoints(armStoredPosition,thumbACJ);
    thumbB->getChainJoints(armStoredPosition,thumbBCJ);
    index->getChainJoints(armStoredPosition,indexCJ);
    middle->getChainJoints(armStoredPosition,middleCJ);
    
    cout << dbgTag << "thumbA encoders: ";
    for(size_t i = 0; i < thumbAEnc.length(); i++){
        cout << thumbAEnc[i] << " ";
    }
    cout << "\n";
    cout << dbgTag << "thumbA chain joints: ";
    for(size_t i = 0; i < thumbACJ.length(); i++){
        cout << thumbACJ[i] << " ";
    }
    cout << "\n";
    
    cout << dbgTag << "thumbB encoders: ";
    for(size_t i = 0; i < thumbBEnc.length(); i++){
        cout << thumbBEnc[i] << " ";
    }
    cout << "\n";
    cout << dbgTag << "thumbB chain joints: ";
    for(size_t i = 0; i < thumbBCJ.length(); i++){
        cout << thumbBCJ[i] << " ";
    }
    cout << "\n";
    
    cout << dbgTag << "index encoders: ";
    for(size_t i = 0; i < indexEnc.length(); i++){
        cout << indexEnc[i] << " ";
    }
    cout << "\n";
    cout << dbgTag << "index chain joints: ";
    for(size_t i = 0; i < indexCJ.length(); i++){
        cout << indexCJ[i] << " ";
    }
    cout << "\n";
    
    cout << dbgTag << "middle encoders: ";
    for(size_t i = 0; i < middleEnc.length(); i++){
        cout << middleEnc[i] << " ";
    }
    cout << "\n";
    cout << dbgTag << "middle chain joints: ";
    for(size_t i = 0; i < middleCJ.length(); i++){
        cout << middleCJ[i] << " ";
    }
    cout << "\n";
    

    yarp::sig::Vector thumbAPose = thumbA->EndEffPose(thumbACJ);
    yarp::sig::Vector thumbBPose = thumbB->EndEffPose(thumbBCJ);
    yarp::sig::Vector indexPose = index->EndEffPose(indexCJ);
    yarp::sig::Vector middlePose = middle->EndEffPose(middleCJ);
    yarp::sig::Vector thumbAPosition = thumbA->EndEffPosition(thumbACJ);
    yarp::sig::Vector thumbBPosition = thumbB->EndEffPosition(thumbBCJ);
    yarp::sig::Vector indexPosition = index->EndEffPosition(indexCJ);
    yarp::sig::Vector middlePosition = middle->EndEffPosition(middleCJ);

    yarp::sig::Vector empty;

    thumbA->setAng(thumbACJ);
    thumbB->setAng(thumbBCJ);
    index->setAng(indexCJ);
    middle->setAng(middleCJ);



    yarp::sig::Matrix thumbAH = thumbA->getH(empty);
    yarp::sig::Matrix thumbBH = thumbA->getH(empty);
    yarp::sig::Matrix indexH = thumbA->getH(empty);
    yarp::sig::Matrix middleH = thumbA->getH(empty);

/*
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
*/

    cout << "------------\n";

    yarp::sig::Vector thumbAPos = thumbAH.getCol(3);
    yarp::sig::Vector thumbADir = thumbAH.getCol(1);

    yarp::sig::Vector thumbBPos = thumbBH.getCol(3);
    yarp::sig::Vector thumbBDir = thumbBH.getCol(1);

    yarp::sig::Vector indexPos = indexH.getCol(3);
    yarp::sig::Vector indexDir = indexH.getCol(1);

    yarp::sig::Vector middlePos = middleH.getCol(3);
    yarp::sig::Vector middleDir = middleH.getCol(1);


    

    yarp::sig::Vector thumbAFB = thumbA->getAng();
    yarp::sig::Vector thumbBFB = thumbB->getAng();
    yarp::sig::Vector indexFB = index->getAng();
    yarp::sig::Vector middleFB = middle->getAng();

    cout << dbgTag << "thumbAFB pose: ";
    for(size_t i = 0; i < thumbAFB.length(); i++){
        cout << thumbAFB[i] << " ";
    }
    cout << "\n";
    cout << dbgTag << "thumbBFB pose: ";
    for(size_t i = 0; i < thumbBFB.length(); i++){
        cout << thumbBFB[i] << " ";
    }
    cout << "\n";
    cout << dbgTag << "indexFB pose: ";
    for(size_t i = 0; i < indexFB.length(); i++){
        cout << indexFB[i] << " ";
    }
    cout << "\n";
    cout << dbgTag << "middleFB pose: ";
    for(size_t i = 0; i < middleFB.length(); i++){
        cout << middleFB[i] << " ";
    }
    cout << "\n";

/*
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

*/

}

bool ControllersUtil::resetPIDIntegralGain(double joint){

	yarp::dev::Pid pid;
	//std::cout << "resetting\n";
	if (iPid->getPid(joint,&pid)){
		storedPIDIntegralGain = pid.ki;
		//std::cout << "old ki " << pid.ki << "\n";
		pid.setKi(0);
		//std::cout << "new ki " << pid.ki << "\n";
		iPid->setPid(joint,pid);
		return true;
	}
	return false;
}

bool ControllersUtil::restorePIDIntegralGain(double joint){

	yarp::dev::Pid pid;
	//std::cout << "restoring\n";
	if (iPid->getPid(joint,&pid)){
		//std::cout << "old ki " << pid.ki << "\n";
		pid.setKi(storedPIDIntegralGain);
		//std::cout << "stored: " << storedPIDIntegralGain << "  new ki " << pid.ki << "\n";
		iPid->setPid(joint,pid);
		return true;
	}
	return false;
}

bool ControllersUtil::lookAtTheHand(){

	if (headEnabled){

		yarp::sig::Vector handPosition,handOrientation;
		handPosition.resize(3);
		handOrientation.resize(4);

		iCart->getPose(handPosition,handOrientation);

		iGaze->lookAtFixationPoint(handPosition);
		//iGaze->waitMotionDone();

	}
	return true;
}

bool ControllersUtil::restoreFixationPoint(){

	if (headEnabled) {
		iGaze->lookAtFixationPoint(storedFixationPoint);
	}

	return true;
}

bool ControllersUtil::setJointAngle(int joint,double angle){

	iPos->positionMove(joint,angle);

	return true;
}

bool ControllersUtil::setJointAnglePositionDirect(int joint,double angle){

	iPosDir->setPosition(joint,angle);

	return true;
}
