#ifndef __ICUB_PLANTIDENTIFICATION_CONTROLLERSUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_CONTROLLERSUTIL_H__

#include <iCub/plantIdentification/PlantIdentificationEnums.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <vector>
#include <deque>

namespace iCub {
    namespace plantIdentification {

        class ControllersUtil {

            public:

                int armJointsNum;

            private:

                yarp::dev::PolyDriver clientArm;
                yarp::dev::PolyDriver clientGazeCtrl;
                yarp::dev::PolyDriver clientArmCartCtrl;
                yarp::dev::IEncoders *iEncs;
                yarp::dev::IOpenLoopControl *iOLC;
                yarp::dev::IControlMode2 *iCtrl;
                yarp::dev::IPositionControl *iPos;
                yarp::dev::IVelocityControl *iVel;
                yarp::dev::IPositionDirect *iPosDir;
                yarp::dev::IPositionControl2 *iPosCtrl;
                yarp::dev::IPidControl *iPid;
                yarp::dev::IGazeControl *iGaze;
                yarp::dev::ICartesianControl *iCart;
                yarp::dev::IControlLimits *iLim;
                std::deque<yarp::dev::IControlLimits*> jointLimits;

                yarp::sig::Vector armStoredPosition;
                yarp::sig::Vector storedFixationPoint;
                std::vector<int> storedJointsControlMode;
                std::vector<int> handJointsToMove;
                std::vector<double> storedHandJointsMaxPwmLimits;

                //TODO TEMPORARY WORKAROUND
                double storedPIDIntegralGain;

                std::string whichHand;
                std::string whichICub;
                std::string whichTask;
                int numFingers;
                bool headEnabled;

                std::vector<double> wholeArmJointsHome;
                std::vector<double> wholeArmJointsDown;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

                ControllersUtil();

                bool init(yarp::os::ResourceFinder &rf);

                bool buildWholeArmJointsHome(const std::vector<double> armJointsHome,const std::vector<double> handJointsHome);

                bool buildWholeArmJointsDown(const std::vector<double> wholeArmJointsDown);

                bool sendPwm(int joint,double pwm);

                bool sendVelocity(int joint,double velocity);

                bool saveCurrentArmPosition();

                bool getArmEncodersAngles(std::vector<double> &armEncodersAngles,bool wait = false);

                bool getArmEncodersAnglesReferences(std::vector<double> &armEncodersAnglesReferences,bool wait = false);

                bool saveCurrentControlMode();

                bool setTaskControlModes(std::vector<int> &jointsList,int controlMode);

                bool setArmInTaskPosition();

                bool setArmDown();

                bool restorePreviousArmPosition();

                bool setArmHomeAsCurrent();

                bool restorePreviousControlMode();

                bool openHand(bool fingersAreStraight);

                bool getEncoderAngle(int joint,double *encoderData);

//				bool getRealPwmValue(iCub::plantIdentification::FingerJoint fingerJoint,double *pwmValue);

                bool release();

                bool setJointMaxPwmLimit(int joint,double maxPwm);

                bool setJointsMaxPwmLimit(std::vector<int> &jointsList,std::vector<double> &maxPwmList);

                bool saveHandJointsMaxPwmLimits();

                bool restoreHandJointsMaxPwmLimits();

                void testShowEndEffectors();    

                bool setControlMode(int joint,int controlMode,bool checkCurrent);

                bool resetPIDIntegralGain(double joint);
                bool restorePIDIntegralGain(double joint);

                bool lookAtTheHand();

                bool restoreFixationPoint();

                bool setJointAngle(int joint,double angle);

                bool setJointAnglePositionDirect(int joint,double angle);

                std::deque<yarp::dev::IControlLimits*> getArmJointLimits();


            private:

                bool waitMoveDone(const double &i_timeout, const double &i_delay);

};
    } //namespace plantIdentification
} //namespace iCub

#endif

