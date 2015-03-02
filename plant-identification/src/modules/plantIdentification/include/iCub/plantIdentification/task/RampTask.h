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
				iCub::plantIdentification::RampTaskState internalState;
				double pressureTargetValue;
				int callsNumberAfterStabilization;
				int maxCallsNumberAfterStabilization;

            public:

                RampTask(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,iCub::plantIdentification::RampTaskData *rampData,double pressureTargetValue);

				virtual void init();

				double getPressureTargetValue(){ return pressureTargetValue; }

				virtual void buildLogData(iCub::plantIdentification::LogData &logData);

				virtual void calculatePwm();

				virtual void saveProgress();

				virtual bool taskIsOver();
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

