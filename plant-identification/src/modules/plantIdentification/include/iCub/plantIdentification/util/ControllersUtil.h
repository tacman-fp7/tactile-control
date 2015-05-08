#ifndef __ICUB_PLANTIDENTIFICATION_CONTROLLERSUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_CONTROLLERSUTIL_H__

#include <iCub/plantIdentification/PlantIdentificationEnums.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <vector>

namespace iCub {
    namespace plantIdentification {

        class ControllersUtil {

            private:

                yarp::dev::PolyDriver clientArm;
                yarp::dev::IEncoders *iEncs;
				yarp::dev::IOpenLoopControl *iOLC;
				yarp::dev::IControlMode2 *iCtrl;
				yarp::dev::IPositionControl *iPos;
				yarp::dev::IVelocityControl *iVel;
				yarp::dev::IPidControl *iPid;

				yarp::sig::Vector armStoredPosition;
				int armJointsNum;
				std::vector<int> storedJointsControlMode;
				std::vector<int> handJointsToMove;
				std::vector<double> storedHandJointsMaxPwmLimits;

                bool graspEnabled;

				/* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

				ControllersUtil();

				bool init(yarp::os::ResourceFinder &rf);

				bool sendPwm(int joint,double pwm);

				bool sendVelocity(int joint,double velocity);

				bool saveCurrentArmPosition();

				bool getArmEncodersAngles(std::vector<double> &armEncodersAngles,bool wait = false);

				bool saveCurrentControlMode();

				bool setTaskControlModes(std::vector<int> &jointsList,int controlMode);

				bool setArmInTaskPosition();

				bool restorePreviousArmPosition();

				bool restorePreviousControlMode();

				bool openHand();

				bool getEncoderAngle(int joint,double *encoderData);

//				bool getRealPwmValue(iCub::plantIdentification::FingerJoint fingerJoint,double *pwmValue);

				bool release();

				bool setJointMaxPwmLimit(int joint,double maxPwm);

				bool setJointsMaxPwmLimit(std::vector<int> &jointsList,std::vector<double> &maxPwmList);

				bool saveHandJointsMaxPwmLimits();

				bool restoreHandJointsMaxPwmLimits();

			private:

				bool waitMoveDone(const double &i_timeout, const double &i_delay);

				bool setControlMode(int joint,int controlMode,bool checkCurrent);
};
    } //namespace plantIdentification
} //namespace iCub

#endif

