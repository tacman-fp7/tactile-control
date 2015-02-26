#ifndef __ICUB_PLANTIDENTIFICATION_STEPTASK_H__
#define __ICUB_PLANTIDENTIFICATION_STEPTASK_H__

#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"

namespace iCub {
    namespace plantIdentification {

        class StepTask : public Task {

            private:
                
				iCub::plantIdentification::StepTaskData *stepData;
				double constantPwm;

            public:

                StepTask(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,iCub::plantIdentification::StepTaskData *stepData,double constantPwm);

				double getConstantPwm(){ return constantPwm; }

				virtual void buildLogData(iCub::plantIdentification::LogData &logData);

				virtual void calculatePwm();
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

