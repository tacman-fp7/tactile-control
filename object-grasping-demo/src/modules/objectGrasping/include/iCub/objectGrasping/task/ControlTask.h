#ifndef __ICUB_OBJECTGRASPING_CONTROLTASK_H__
#define __ICUB_OBJECTGRASPING_CONTROLTASK_H__

#include "iCub/objectGrasping/task/Task.h"
#include "iCub/objectGrasping/data/TaskData.h"
#include "iCub/objectGrasping/util/ControllersUtil.h"
#include "iCub/objectGrasping/util/PortsUtil.h"

#include <iCub/ctrl/pids.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

namespace iCub {
    namespace objectGrasping {

        class ControlTask : public Task {

            private:

				iCub::objectGrasping::ControlTaskData *controlData;
				std::vector<iCub::ctrl::parallelPID*> pid;
				// pid options bottle when error >= 0
				yarp::os::Bottle pidOptionsPE;
				// pid options bottle when error < 0
				yarp::os::Bottle pidOptionsNE;
				std::vector<double> pressureTargetValue;

				// TODO to be removed
				std::vector<double> currentKp;
				std::vector<double> kpPe;
				std::vector<double> kpNe;
				std::vector<double> previousError;

            public:

                ControlTask(iCub::objectGrasping::ControllersUtil *controllersUtil,iCub::objectGrasping::PortsUtil *portsUtil,iCub::objectGrasping::TaskCommonData *commonData,iCub::objectGrasping::ControlTaskData *controlData,double pressureTargetValue);

				std::string getPressureTargetValueDescription();

				virtual void init();

				virtual void buildLogData(LogData &logData);

				virtual void calculatePwm();

				virtual void release();

			private :

				void addOption(yarp::os::Bottle &bottle,char *paramName,yarp::os::Value paramValue);

				void addOption(yarp::os::Bottle &bottle,char *paramName,yarp::os::Value paramValue1,yarp::os::Value paramValue2);

				double calculateTt(iCub::objectGrasping::ControlTaskOpMode gainsSet);

        };
    } //namespace objectGrasping
} //namespace iCub

#endif

