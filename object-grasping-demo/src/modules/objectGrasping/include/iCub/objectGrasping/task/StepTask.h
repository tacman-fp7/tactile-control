#ifndef __ICUB_OBJECTGRASPING_STEPTASK_H__
#define __ICUB_OBJECTGRASPING_STEPTASK_H__

#include "iCub/objectGrasping/task/Task.h"
#include "iCub/objectGrasping/data/TaskData.h"
#include "iCub/objectGrasping/util/ControllersUtil.h"
#include "iCub/objectGrasping/util/PortsUtil.h"

namespace iCub {
    namespace objectGrasping {

        class StepTask : public Task {

            private:
                
				iCub::objectGrasping::StepTaskData *stepData;
				std::vector<double> constantPwm;

            public:

                StepTask(iCub::objectGrasping::ControllersUtil *controllersUtil,iCub::objectGrasping::PortsUtil *portsUtil,iCub::objectGrasping::TaskCommonData *commonData,iCub::objectGrasping::StepTaskData *stepData,std::vector<double> &constantPwm);

				virtual void init();

				std::string getConstantPwmDescription();

				virtual void buildLogData(iCub::objectGrasping::LogData &logData);

				virtual void calculatePwm();
        };
    } //namespace objectGrasping
} //namespace iCub

#endif

