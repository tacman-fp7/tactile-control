#ifndef __ICUB_PLANTIDENTIFICATION_CONTROLTASK_H__
#define __ICUB_PLANTIDENTIFICATION_CONTROLTASK_H__

#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"
#include <iCub/ctrl/pids.h>

namespace iCub {
    namespace plantIdentification {

        class ControlTask : public Task {

            private:

				iCub::plantIdentification::ControlTaskData *controlData;
				iCub::ctrl::parallelPID *pid;
				double pressureTargetValue;

            public:

                ControlTask(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,iCub::plantIdentification::ControlTaskData *controlData,double pressureTargetValue);

				double getPressureTargetValue(){ return pressureTargetValue; }

				virtual void buildLogData(LogData &logData);

				virtual void calculatePwm();

				virtual void release();
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

