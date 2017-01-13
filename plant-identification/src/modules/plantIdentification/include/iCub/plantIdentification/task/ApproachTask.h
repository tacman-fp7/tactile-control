#ifndef __ICUB_PLANTIDENTIFICATION_APPROACHTASK_H__
#define __ICUB_PLANTIDENTIFICATION_APPROACHTASK_H__

#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"

#include <iCub/ctrl/pids.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

namespace iCub {
    namespace plantIdentification {

        class ApproachTask : public Task {

            private:

                iCub::plantIdentification::ApproachTaskData *approachData;
                
                int controlMode; // 0:velocity  1:openloop
                int stopCondition; // 0:none  1:tactile  2:position  3:both tactile and position
                bool stopFingers;
                bool manageFingers;
                std::vector<bool> fingerIsInContact;
                std::vector<bool> fingerSetInPosition;
                int callsNumberForAvarage;
                int callsNumberForMovementTimeout;
                double thresholdScaleFactor;

                // stop condition by tactile feedback data
                std::vector<bool> thresholdExceeded;
                std::vector<double> tactileAvarage;
                std::vector<double> tactileMaximum;
                std::vector<double> tactileThreshold;

                // stop condition by position data
                std::vector<int> fingerState; // 0: before moving; 1:while moving; 2:after moving
                std::vector<std::vector<double> > fingerPositions;
                int windowSize;
                double finalCheckThreshold;
                int positionIndex;

                void moveFinger(int finger);
                void stopFinger(int finger);

            public:

                ApproachTask(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,iCub::plantIdentification::ApproachTaskData *approachData);

                virtual void init();

                virtual void buildLogData(LogData &logData);

                virtual void calculateControlInput();

                virtual void sendCommands();

                virtual void release();

                virtual bool taskIsOver();

                bool eachFingerIsInContact();

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

