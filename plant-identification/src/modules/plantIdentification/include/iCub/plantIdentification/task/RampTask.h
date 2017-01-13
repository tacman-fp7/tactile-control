#ifndef __ICUB_PLANTIDENTIFICATION_RAMPTASK_H__
#define __ICUB_PLANTIDENTIFICATION_RAMPTASK_H__

#include "iCub/plantIdentification/PlantIdentificationEnums.h"
#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"

namespace iCub {
    namespace plantIdentification {

        class RampTask : public Task {

            private:

                iCub::plantIdentification::RampTaskData *rampData;
                std::vector<iCub::plantIdentification::RampTaskState> internalState;
                std::vector<double> pressureTargetValue;
                int callsNumberAfterStabilization;
                int maxCallsNumberAfterStabilization;

            public:

                RampTask(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,iCub::plantIdentification::RampTaskData *rampData,std::vector<double> &targetList);

                virtual void init();

                std::string getPressureTargetValueDescription();

                virtual void buildLogData(iCub::plantIdentification::LogData &logData);

                virtual void calculateControlInput();

                virtual void saveProgress();

                virtual bool taskIsOver();

            private:

                bool areAllJointsSteady();
                
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

