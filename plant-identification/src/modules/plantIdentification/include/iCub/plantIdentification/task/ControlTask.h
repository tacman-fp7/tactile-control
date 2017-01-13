#ifndef __ICUB_PLANTIDENTIFICATION_CONTROLTASK_H__
#define __ICUB_PLANTIDENTIFICATION_CONTROLTASK_H__

#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"

#include <iCub/ctrl/neuralNetworks.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

#include <string>
#include <fstream>

namespace iCub {
    namespace plantIdentification {

        class ControlTask : public Task {

            private:

                iCub::plantIdentification::ControlTaskData *controlData;
                std::vector<iCub::ctrl::parallelPID*> pid;
                // pid options bottle when error >= 0
                std::vector<yarp::os::Bottle> pidOptionsPE;
                // pid options bottle when error < 0
                std::vector<yarp::os::Bottle> pidOptionsNE;
                std::vector<double> pressureTargetValue;

                iCub::plantIdentification::PortsUtil *portsUtil;

                /* variables used for error integral reset mode */
                bool resetErrOnContact;
                std::vector<bool> fingerIsInContact;

                /* variables used for supervisor mode */
                iCub::ctrl::parallelPID *svPid;
                yarp::os::Bottle svPidOptions;
                bool supervisorControlMode;
                double svKp,svKi,svKd;
                std::vector<double> initialPressureTargetValue;
                double trajectoryInitialTime;
                double trajectoryInitialPose;
                bool trackingModeEnabled;
                bool minJerkTrackingModeEnabled;
                bool gmmJointsMinJerkTrackingModeEnabled;
                iCub::ctrl::minJerkTrajGen* minJerkTrajectory;
                iCub::ctrl::minJerkTrajGen* thAbdMinJerkTrajectory;
                iCub::ctrl::minJerkTrajGen* thDistMinJerkTrajectory;
                iCub::ctrl::minJerkTrajGen* indDistMinJerkTrajectory;
                iCub::ctrl::minJerkTrajGen* midDistMinJerkTrajectory;

                // TODO TEMPORARY WORKAROUND, variable used to disable the PID integral gain of the joint 8 while the hand grasps the object
                bool disablePIDIntegralGain;

                /* variables used for the policy learning mode */
                std::vector<double> policyActionsData;
                double indMidPressureBalance;
                int policyState; // 0: not active | 1: rest | 2: action
                double currentActionFinalTargetPose;

                // neural network
                iCub::ctrl::ff2LayNN_tansig_purelin neuralNetwork;

                // TODO to be removed
                std::vector<double> currentKp;
                std::vector<double> kpPe;
                std::vector<double> kpNe;
                std::vector<double> previousError;

                bool gmmCtrlModeIsSet;

                bool handPositionSet;
                double initialHandPosition;

                // related to object recognition
                bool closingHand;
                bool objectRecognitionEnabled;
                bool handClosurePerformed;
                std::ofstream objRecDataFile;
                bool objRecFileExists;
                bool snapshotSaved;
                bool squeezingStarted;


            public:

                ControlTask(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,iCub::plantIdentification::ControlTaskData *controlData,std::vector<double> &targetList,bool resetErrOnContact = false);

                std::string getPressureTargetValueDescription();

                void setTargetListRealTime(std::vector<double> &targetList);

                virtual void init();

                virtual void buildLogData(LogData &logData);

                virtual void calculateControlInput();

                virtual void release();

            private :

                void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValue);

                void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValue1,yarp::os::Value paramValue2);

                void scaleGains(double scaleFactor);

                void changeSVGain(std::string gainName,double newGainValue);

                void replaceBottle(yarp::os::Bottle &oldBottle,yarp::os::Bottle &newBottle,double scaleFactor);
                
                void replaceBottle(yarp::os::Bottle &oldBottle,yarp::os::Bottle &newBottle,std::string gainName,double newGainValue);

                double calculateTt(double kp,double ki,double kd);

                void setGMMJointsControlMode(int controlMode);

                void manageObjectRecognitionTask(yarp::os::Bottle &objectRecognitionBottle);

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

