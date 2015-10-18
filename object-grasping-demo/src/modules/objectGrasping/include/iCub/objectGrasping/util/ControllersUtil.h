#ifndef __ICUB_OBJECTGRASPING_CONTROLLERSUTIL_H__
#define __ICUB_OBJECTGRASPING_CONTROLLERSUTIL_H__

#include <iCub/objectGrasping/ObjectGraspingEnums.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <vector>

namespace iCub {
    namespace objectGrasping {

        class ControllersUtil {

            private:

                yarp::dev::PolyDriver clientArm;
                yarp::dev::PolyDriver clientArmCartContr;
                
                yarp::dev::IEncoders *iEncs;
				yarp::dev::IControlMode2 *iCtrl;
				yarp::dev::IPositionControl *iPos;
				yarp::dev::IVelocityControl *iVel;

                yarp::dev::ICartesianControl *iCart;

				yarp::sig::Vector armStoredPosition;
				int armJointsNum;
				std::vector<int> jointsStoredControlMode;
				std::vector<int> handJointsToMove;

				/* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

				ControllersUtil();

				bool init(yarp::os::ResourceFinder &rf);

				bool saveCurrentArmPosition();

				bool saveCurrentControlMode();

				bool testCartesianController();

				bool setArmInStartPosition(bool cartesianMode);

				bool setArmInGraspPosition(bool cartesianMode);

				bool raiseArm(bool cartesianMode);

				bool restorePreviousArmPosition();

				bool restorePreviousControlMode();

				bool release();

				bool openHand();

				bool moveFingers();

				bool incrementEndEffectorPosition(double incrementValue,int coordinate,double seconds);

			private:

				bool waitMoveDone(const double &i_timeout, const double &i_delay);

				bool waitMoveDone(const double &i_timeout, const double &i_delay,bool excludeHand);

				bool setControlMode(int joint,int controlMode,bool checkCurrent);
};
    } //namespace objectGrasping
} //namespace iCub

#endif

