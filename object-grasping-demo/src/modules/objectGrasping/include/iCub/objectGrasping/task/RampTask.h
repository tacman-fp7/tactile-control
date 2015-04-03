#ifndef __ICUB_OBJECTGRASPING_RAMPTASK_H__
#define __ICUB_OBJECTGRASPING_RAMPTASK_H__

#include "iCub/objectGrasping/ObjectGraspingEnums.h"
#include "iCub/objectGrasping/task/Task.h"
#include "iCub/objectGrasping/data/TaskData.h"
#include "iCub/objectGrasping/util/ControllersUtil.h"
#include "iCub/objectGrasping/util/PortsUtil.h"

namespace iCub {
    namespace objectGrasping {

        class RampTask : public Task {

            private:

				iCub::objectGrasping::RampTaskData *rampData;
				std::vector<iCub::objectGrasping::RampTaskState> internalState;
				std::vector<double> pressureTargetValue;
				int callsNumberAfterStabilization;
				int maxCallsNumberAfterStabilization;

            public:

                RampTask(iCub::objectGrasping::ControllersUtil *controllersUtil,iCub::objectGrasping::PortsUtil *portsUtil,iCub::objectGrasping::TaskCommonData *commonData,iCub::objectGrasping::RampTaskData *rampData,double pressureTargetValue);

				virtual void init();

				std::string getPressureTargetValueDescription();

				virtual void buildLogData(iCub::objectGrasping::LogData &logData);

				virtual void calculatePwm();

				virtual void saveProgress();

				virtual bool taskIsOver();

			private:

				bool areAllJointsSteady();
				
        };
    } //namespace objectGrasping
} //namespace iCub

#endif

