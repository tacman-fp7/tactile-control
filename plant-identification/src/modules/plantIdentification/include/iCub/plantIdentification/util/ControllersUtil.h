#ifndef __ICUB_PLANTIDENTIFICATION_CONTROLLERSUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_CONTROLLERSUTIL_H__

#include <iCub/plantIdentification/PlantIdentificationEnums.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

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

				yarp::sig::Vector armStoredPosition;
				int armJointsNum;
				int jointStoredControlMode;
				int jointToMove;

				/* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

				ControllersUtil();

				bool init(yarp::os::ResourceFinder &rf);

				void setJoint(int jointToMove){ this->jointToMove = jointToMove; }

				bool sendPwm(double pwm);

				bool saveCurrentArmPosition();

				bool saveCurrentControlMode();

				bool setTaskControlMode();

				bool setArmInTaskPosition();

				bool restorePreviousArmPosition();

				bool restorePreviousControlMode();

				bool openHand();

				bool getEncoderAngle(iCub::plantIdentification::FingerJoint fingerJoint,double *encoderData);

				bool getRealPwmValue(iCub::plantIdentification::FingerJoint fingerJoint,double *pwmValue);

				bool release();

			private:

				bool waitMoveDone(const double &i_timeout, const double &i_delay);

				bool setControlMode(int controlMode,bool checkCurrent);
};
    } //namespace plantIdentification
} //namespace iCub

#endif

