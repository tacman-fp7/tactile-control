#ifndef __ICUB_PLANTIDENTIFICATION_GRASPTHREAD_H__
#define __ICUB_PLANTIDENTIFICATION_GRASPTHREAD_H__

#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/data/RPCCommandsData.h"
#include "iCub/plantIdentification/data/LogData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"
#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <fstream>
#include <string>
#include <vector>
#include <deque>

namespace iCub {
    namespace plantIdentification {

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
				iCub::plantIdentification::TaskData *taskData;

				/* ******* Controllers utility                          ******* */
                iCub::plantIdentification::ControllersUtil *controllersUtil;

				/* ****** Ports utility                                 ****** */
				iCub::plantIdentification::PortsUtil *portsUtil;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

			public:

                TaskThread(const int aPeriod, const yarp::os::ResourceFinder &aRf);
                virtual ~TaskThread();

                virtual bool threadInit(void);
                virtual void run(void);
                virtual void threadRelease(void);

				bool initializeGrasping();
				bool openHand();

				void set(iCub::plantIdentification::RPCSetCmdArgName paramName,std::string paramValue,iCub::plantIdentification::RPCCommandsData &rpcCmdData);
				void task(iCub::plantIdentification::RPCTaskCmdArgName paramName,iCub::plantIdentification::TaskName taskName,std::string paramValue,iCub::plantIdentification::RPCCommandsData &rpcCmdData);
				void view(iCub::plantIdentification::RPCViewCmdArgName paramName,iCub::plantIdentification::RPCCommandsData &rpcCmdData);
        };
    }
}

#endif

