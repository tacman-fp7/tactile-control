#ifndef __ICUB_PLANTIDENTIFICATION_CONTROLTASK_H__
#define __ICUB_PLANTIDENTIFICATION_CONTROLTASK_H__

#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"

#include <iCub/ctrl/pids.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

namespace iCub {
    namespace plantIdentification {

        class ControlTask : public Task {

            private:

				iCub::plantIdentification::ControlTaskData *controlData;
				iCub::ctrl::parallelPID *pid;
				// pid options bottle when error >= 0
				yarp::os::Bottle pidOptionsPE;
				// pid options bottle when error < 0
				yarp::os::Bottle pidOptionsNE;
				double pressureTargetValue;

				// TODO to be removed
				double currentKp;
				double kpPe;
				double kpNe;
				double previousError;

            public:

                ControlTask(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,iCub::plantIdentification::ControlTaskData *controlData,double pressureTargetValue);

				double getPressureTargetValue(){ return pressureTargetValue; }

				virtual void init();

				virtual void buildLogData(LogData &logData);

				virtual void calculatePwm();

				virtual void release();

			private :

				void addOption(yarp::os::Bottle &bottle,char *paramName,yarp::os::Value paramValue);

				void addOption(yarp::os::Bottle &bottle,char *paramName,yarp::os::Value paramValue1,yarp::os::Value paramValue2);

				double calculateTt(iCub::plantIdentification::ControlTaskOpMode gainsSet);

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

