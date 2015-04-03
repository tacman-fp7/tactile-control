#ifndef __ICUB_OBJECTGRASPING_GRASPTHREAD_H__
#define __ICUB_OBJECTGRASPING_GRASPTHREAD_H__

#include "iCub/objectGrasping/task/Task.h"
#include "iCub/objectGrasping/data/TaskData.h"
#include "iCub/objectGrasping/data/RPCCommandsData.h"
#include "iCub/objectGrasping/data/LogData.h"
#include "iCub/objectGrasping/util/ControllersUtil.h"
#include "iCub/objectGrasping/util/PortsUtil.h"
#include "iCub/objectGrasping/ObjectGraspingEnums.h"

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>

#include <fstream>
#include <string>
#include <vector>
#include <deque>

namespace iCub {
    namespace objectGrasping {

        class TaskThread : public yarp::os::RateThread {

            private:

                /* ****** Thread attributes                             ****** */
                int period;
				bool runEnabled;
                yarp::os::ResourceFinder rf;

				/* ****** Tasks management								****** */
				std::vector<Task*> taskList;
				int currentTaskIndex;

				/* ****** Task data										****** */
				iCub::objectGrasping::TaskData *taskData;

				/* ******* Controllers utility                          ******* */
                iCub::objectGrasping::ControllersUtil *controllersUtil;

				/* ****** Ports utility                                 ****** */
				iCub::objectGrasping::PortsUtil *portsUtil;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

			public:

                TaskThread(iCub::objectGrasping::ControllersUtil *controllersUtil,iCub::objectGrasping::PortsUtil *portsUtil,const int aPeriod, const yarp::os::ResourceFinder &aRf);
                virtual ~TaskThread();

                virtual bool threadInit();
                virtual void run();
                virtual void threadRelease();

				bool initializeGrasping();
				bool openHand();

				void set(iCub::objectGrasping::RPCSetCmdArgName paramName,yarp::os::Value paramValue,iCub::objectGrasping::RPCCommandsData &rpcCmdData);
				void task(iCub::objectGrasping::RPCTaskCmdArgName paramName,iCub::objectGrasping::TaskName taskName,yarp::os::Value paramValue,iCub::objectGrasping::RPCCommandsData &rpcCmdData);
				void view(iCub::objectGrasping::RPCViewCmdArgName paramName,iCub::objectGrasping::RPCCommandsData &rpcCmdData);
				void help(iCub::objectGrasping::RPCCommandsData &rpcCmdData);
        };
    }
}

#endif

