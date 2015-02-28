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

				ControllersUtil(yarp::os::ResourceFinder &rf);

				void init(int jointToMove);

				void sendPwm(double pwm);

				void saveCurrentArmPosition();

				void saveCurrentControlMode();

				void setTaskControlMode();

				void setArmInTaskPosition();

				void restorePreviousArmPosition();

				void restorePreviousControlMode();

				void openHand();

				void getEncoderAngle(iCub::plantIdentification::FingerJoint fingerJoint,double *encoderData);

				void getRealPwmValue(iCub::plantIdentification::FingerJoint fingerJoint,double *pwmValue);

				void release();

			private:

				bool waitMoveDone(const double &i_timeout, const double &i_delay);

				bool setControlMode(int controlMode,bool checkCurrent);
};
    } //namespace plantIdentification
} //namespace iCub

#endif

