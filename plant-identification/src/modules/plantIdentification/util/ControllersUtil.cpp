#include "iCub/plantIdentification/util/ControllersUtil.h"

#include <vector>
#include <ctime>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

using std::string;
using std::cout;
using iCub::plantIdentification::ControllersUtil;
using yarp::os::Property;


ControllersUtil::ControllersUtil(yarp::os::ResourceFinder &rf){
	using std::vector;
	using yarp::os::Value;

	dbgTag = "ControllersUtil: ";

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
//        return false;
    }
    // Open interfaces
    clientArm.view(iEncs);
    if (!iEncs) {
//        return false;
    }
	clientArm.view(iOLC);
    if (!iOLC) {
//        return false;
    }
	clientArm.view(iCtrl);
    if (!iCtrl) {
//        return false;
    }
	clientArm.view(iPos);
    if (!iPos) {
//        return false;
    }
	clientArm.view(iVel);
    if (!iVel) {
//        return false;
    }

	iPos->getAxes(&armJointsNum);
	
	// Set reference speeds
    vector<double> refSpeeds(armJointsNum, 0);
    iPos->getRefSpeeds(&refSpeeds[0]);
    for (int i = 11; i < 15; ++i) {
        refSpeeds[i] = 50;
    }
    iPos->setRefSpeeds(&refSpeeds[0]);
}

void ControllersUtil::init(int jointToMove){

	this->jointToMove = jointToMove;
}

void ControllersUtil::sendPwm(double pwm){

	iOLC->setRefOutput(jointToMove,pwm);
}

void ControllersUtil::saveCurrentArmPosition(){
	using yarp::os::Time;

    armStoredPosition.resize(armJointsNum);
    
	bool encodersDataAcquired = false;
    while(!encodersDataAcquired) {

        encodersDataAcquired = iEncs->getEncoders(armStoredPosition.data());

#ifndef NODEBUG
        cout << "DEBUG: " << dbgTag << "Encoder data is not available yet. \n";
#endif

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
}

void ControllersUtil::saveCurrentControlMode(){

	iCtrl->getControlMode(jointToMove,&jointStoredControlMode);
}

void ControllersUtil::setTaskControlMode(){

	setControlMode(VOCAB_CM_OPENLOOP,true);
}


/* ******* Place arm in grasping position                                   ********************************************** */ 
void ControllersUtil::setArmInTaskPosition() {

    cout << dbgTag << "Reaching for grasp ... \t";
    
    iVel->stop();

    // Set the arm in the starting position
	// Arm
	//iPos->positionMove(0 ,-25);
    //iPos->positionMove(1 , 35);
    //iPos->positionMove(2 , 18);
    //iPos->positionMove(3 , 22);
    iPos->positionMove(4 ,-13);
    //iPos->positionMove(5 , 9);
    //iPos->positionMove(6 , -5);
    //iPos->positionMove(7 , 20);
    
	// Hand
    //iPos->positionMove(8 , 90);
    //iPos->positionMove(9 , 30);
    //iPos->positionMove(10, 30);
    iPos->positionMove(11, 5);
    iPos->positionMove(12, 19);
    iPos->positionMove(13, 3);
    iPos->positionMove(14, 20);
    iPos->positionMove(15, 17);

    // Check motion done
    waitMoveDone(10, 1);
	cout << "Done. \n";

}
/* *********************************************************************************************************************** */

void ControllersUtil::restorePreviousArmPosition(){
	
	// Stop interfaces
    if (iVel) {
        iVel->stop();
    }
    if (iPos) {
        iPos->stop();
        // Restore initial robot position
        iPos->positionMove(armStoredPosition.data());
    }
}

void ControllersUtil::restorePreviousControlMode(){

	setControlMode(jointStoredControlMode,true);

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

void ControllersUtil::getProximalEncoderAngle(double *encoderData){
	
	iEncs->getEncoder(jointToMove,encoderData);
}

void ControllersUtil::getDistalEncoderAngle(double *encoderData){
	
	iEncs->getEncoder(jointToMove + 1,encoderData);
}

void ControllersUtil::getProximalRealPwmValue(double *pwmValue){

	iOLC->getOutput(jointToMove,pwmValue);
}

void ControllersUtil::getDistalRealPwmValue(double *pwmValue){

	iOLC->getOutput(jointToMove + 1,pwmValue);
}

void ControllersUtil::release(){

    // Close driver
    clientArm.close();
}

/* *********************************************************************************************************************** */
/* ******* Open hand                                                        ********************************************** */
void ControllersUtil::openHand() {
    
	cout << dbgTag << "Opening hand ... \t";
    
    iVel->stop();

    // Set the fingers to the original position
    //iPos->positionMove(11, 5);
    //iPos->positionMove(12, 0);
    //iPos->positionMove(13, 0);
    //iPos->positionMove(14, 0);
    //iPos->positionMove(15, 40);

	iPos->positionMove(11, 5);
    iPos->positionMove(12, 19);
    iPos->positionMove(13, 3);
    iPos->positionMove(14, 20);
    iPos->positionMove(15, 17);

    // Check motion done
    waitMoveDone(10, 1);
    cout << "Done. \n";
}

/* *********************************************************************************************************************** */

