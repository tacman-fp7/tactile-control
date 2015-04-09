#include "iCub/objectGrasping/util/ControllersUtil.h"

#include <vector>
#include <ctime>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

using iCub::objectGrasping::ControllersUtil;
using iCub::objectGrasping::FingerJoint;

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
	jointsStoredControlMode.resize(8,VOCAB_CM_POSITION);

	string robotName = rf.check("robot", Value("icub"), "The robot name.").asString().c_str();
    string whichHand = rf.check("whichHand", Value("right"), "The hand to be used for the grasping.").asString().c_str();

	 /* ******* Joint interfaces                     ******* */
    string arm = whichHand + "_arm";
    Property options;
    options.put("robot", robotName.c_str()); 
    options.put("device", "remote_controlboard");
    options.put("part", arm.c_str());
    options.put("local", ("/ObjectGrasping/" + arm).c_str());
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

bool ControllersUtil::sendPwm(int joint,double pwm){

	if (!iOLC->setRefOutput(joint,pwm)){
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

	// save control mode of joints 8 9 10 11 12 13 14 15
	for(size_t i = 0; i < 8; i++){
		if (!iCtrl->getControlMode(8 + i,&jointsStoredControlMode[i])){
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
bool ControllersUtil::setArmInStartPosition() {

    cout << dbgTag << "Reaching arm grasp position ... \t";
    
	iVel->stop();

	// Arm
	iPos->positionMove(0 ,-3);
    iPos->positionMove(1 , 41);
    iPos->positionMove(2 , 16);
    iPos->positionMove(3 , 76);
        
    iPos->positionMove(4 , -20);
    iPos->positionMove(5 , -8);
    iPos->positionMove(6 , -0);
    iPos->positionMove(7 , 33);
        
	// Hand
    iPos->positionMove(8 , 79);
    iPos->positionMove(9 , 2);
    iPos->positionMove(10, 29);
    iPos->positionMove(11, 0);
    iPos->positionMove(12, 0);
    iPos->positionMove(13, 0);
    iPos->positionMove(14, 15);
    iPos->positionMove(15, 1);

    // Check motion done
    waitMoveDone(10, 1);
	cout << "Done. \n";

	return true;
}
/* *********************************************************************************************************************** */

/* ******* Place arm in grasping position                                   ********************************************** */ 
bool ControllersUtil::setArmInGraspPosition() {

    cout << dbgTag << "Reaching arm grasp position ... \t";
    
	iVel->stop();

	// Arm
	iPos->positionMove(0 ,-39);
    iPos->positionMove(1 , 25);
    iPos->positionMove(2 , 0);
    iPos->positionMove(3 , 48);
        
    iPos->positionMove(4 , -6);
    iPos->positionMove(5 , -5);
    iPos->positionMove(6 , 0);
    iPos->positionMove(7 , 33);
        
	// Hand
    iPos->positionMove(8 , 79);
    iPos->positionMove(9 , 2);
    iPos->positionMove(10, 29);
    iPos->positionMove(11, 0);
    iPos->positionMove(12, 0);
    iPos->positionMove(13, 0);
    iPos->positionMove(14, 15);
    iPos->positionMove(15, 1);

    // Check motion done
    waitMoveDone(10, 1);
	cout << "Done. \n";

	return true;
}
/* *********************************************************************************************************************** */


/* ******* Place arm in grasping position                                   ********************************************** */ 
bool ControllersUtil::raiseArm() {

    cout << dbgTag << "Reaching arm grasp position ... \t";
    
//	iVel->stop();

	// Arm
	iPos->positionMove(0 ,-40);
    iPos->positionMove(1 , 41);
    iPos->positionMove(2 , 16);
    iPos->positionMove(3 , 76);
        
    iPos->positionMove(4 , -20);
    iPos->positionMove(5 , -8);
    iPos->positionMove(6 , -0);
    iPos->positionMove(7 , 33);
        
	// Hand
    //iPos->positionMove(8 , 79);
    //iPos->positionMove(9 , 2);
    //iPos->positionMove(10, 29);
    //iPos->positionMove(11, 0);
    //iPos->positionMove(12, 0);
    //iPos->positionMove(13, 25);
    //iPos->positionMove(14, 15);
    //iPos->positionMove(15, 1);

    // Check motion done
//    waitMoveDone(10, 1);
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
		if (!setControlMode(8 + i,jointsStoredControlMode[i],true)){
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


bool ControllersUtil::release(){

    // Close driver
    if (!clientArm.close()){
		cout << dbgTag << "could not close driver\n";	
		return false;
	}
	return true;
}

/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Open hand                                                        ********************************************** */
bool ControllersUtil::openHand() {
    
	cout << dbgTag << "Opening hand ... \t";
    
    iVel->stop();

    iPos->positionMove(8 , 79);
    iPos->positionMove(9 , 2);
    iPos->positionMove(10, 29);
    iPos->positionMove(11, 0);
    iPos->positionMove(12, 0);
    iPos->positionMove(13, 25);
    iPos->positionMove(14, 25);
    iPos->positionMove(15, 1);

    // Check motion done
    waitMoveDone(10, 1);
    cout << "Done. \n";

	return true;
}

/* *********************************************************************************************************************** */

