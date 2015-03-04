#include "iCub/plantIdentification/util/ControllersUtil.h"

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

	string robotName = rf.check("robot", Value("icub"), "The robot name.").asString().c_str();
    string whichHand = rf.check("whichHand", Value("right"), "The hand to be used for the grasping.").asString().c_str();

	 /* ******* Joint interfaces                     ******* */
    string arm = whichHand + "_arm";
    Property options;
    options.put("robot", robotName.c_str()); 
    options.put("device", "remote_controlboard");
    options.put("part", arm.c_str());
    options.put("local", ("/PlantIdentification/" + arm).c_str());
    options.put("remote", ("/" + robotName + "/" + arm).c_str());
    
    // Open driver
    if (!clientArm.open(options)) {
        cout << dbgTag << "could not open driver\n";
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

	iPos->getAxes(&armJointsNum);
	
	// Set reference speeds
    vector<double> refSpeeds(armJointsNum, 0);
    iPos->getRefSpeeds(&refSpeeds[0]);
    for (int i = 11; i < 15; ++i) {
        refSpeeds[i] = 50;
    }
    iPos->setRefSpeeds(&refSpeeds[0]);

	return true;
}

bool ControllersUtil::sendPwm(double pwm){

	if (!iOLC->setRefOutput(jointToMove,pwm)){
		cout << dbgTag << "could not send pwm\n";
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

bool ControllersUtil::saveCurrentControlMode(){

	if (!iCtrl->getControlMode(jointToMove,&jointStoredControlMode)){
		cout << dbgTag << "could not get current control mode\n";
		return false;
	}
	return true;
}

bool ControllersUtil::setTaskControlMode(){

	return setControlMode(VOCAB_CM_OPENLOOP,true);
}


/* ******* Place arm in grasping position                                   ********************************************** */ 
bool ControllersUtil::setArmInTaskPosition() {

    cout << dbgTag << "Reaching arm task position ... \t";
    
	iVel->stop();

    // Set the arm in the starting position
	// Arm
	iPos->positionMove(0 ,-38);
    iPos->positionMove(1 , 23);
    iPos->positionMove(2 , 0);
    iPos->positionMove(3 , 19);
    
    iPos->positionMove(4 ,-12);
    iPos->positionMove(5 , 0);
    iPos->positionMove(6 , 0);
    iPos->positionMove(7 , 15);
    
	// Hand
    iPos->positionMove(8 , 45);
    iPos->positionMove(9 , 0);
    iPos->positionMove(10, 0);
    iPos->positionMove(11, 0);
    iPos->positionMove(12, 10);
    iPos->positionMove(13, 0);
    iPos->positionMove(14, 10);
    iPos->positionMove(15, 0);

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

	return setControlMode(jointStoredControlMode,true);
}

bool ControllersUtil::setControlMode(int controlMode,bool checkCurrent){

	if (checkCurrent){
		int currentControlMode = -1;
		if (iCtrl->getControlMode(jointToMove,&currentControlMode)){

			if (currentControlMode != controlMode){
				if  (iCtrl->setControlMode(jointToMove,controlMode)){
					cout << dbgTag << "CONTROL MODE SET TO " << controlMode << " ON JOINT " << jointToMove << "   PREV: " << currentControlMode << "  OPEN: " << VOCAB_CM_OPENLOOP << "\n";
					return true;
				} else {
                    cout << dbgTag << "failed to SET control mode on joint " << jointToMove << "\n";                   
                    return false;
                }	
			} else {
                cout << dbgTag << "open loop control mode already set (" << currentControlMode << ")\n";
                return true;
            }
		} else {
            cout << dbgTag << "failed to GET control mode from joint " << jointToMove << " (controlMode appears to be " << currentControlMode << ")\n";           
            return false;
        }
	} else return iCtrl->setControlMode(jointToMove,controlMode);
	
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

bool ControllersUtil::getEncoderAngle(FingerJoint fingerJoint,double *encoderData){
	
	bool ok;

	switch (fingerJoint){

	case PROXIMAL:
		ok = iEncs->getEncoder(jointToMove,encoderData);
		break;

	case DISTAL:
		ok = iEncs->getEncoder(jointToMove + 1,encoderData);
		break;
	}
	
	if (!ok){
		cout << dbgTag << "could not get encoder value\n";
	}
	return ok;
}

bool ControllersUtil::getRealPwmValue(FingerJoint fingerJoint,double *pwmValue){

	bool ok;

	switch (fingerJoint){

	case PROXIMAL:
		ok = iOLC->getOutput(jointToMove,pwmValue);
		break;

	case DISTAL:
		ok = iOLC->getOutput(jointToMove + 1,pwmValue);
		break;
	}

	if (!ok){
		cout << dbgTag << "could not get pwm value\n";
	}
	return ok;
}

bool ControllersUtil::release(){

    // Close driver
    if (!clientArm.close()){
		cout << dbgTag << "could not close driver\n";	
		return false;
	}
	return true;
}

/* *********************************************************************************************************************** */
/* ******* Open hand                                                        ********************************************** */
bool ControllersUtil::openHand() {
    
	cout << dbgTag << "Opening hand ... \t";
    
    iVel->stop();

	// Hand
    iPos->positionMove(8 , 45);
    iPos->positionMove(9 , 0);
    iPos->positionMove(10, 0);
    iPos->positionMove(11, 0);
    iPos->positionMove(12, 10);
    iPos->positionMove(13, 0);
    iPos->positionMove(14, 10);
    iPos->positionMove(15, 0);

    // Check motion done
    waitMoveDone(10, 1);
    cout << "Done. \n";

	return true;
}

/* *********************************************************************************************************************** */

