#ifndef __ICUB_PLANTIDENTIFICATION_GRASPTHREAD_H__
#define __ICUB_PLANTIDENTIFICATION_GRASPTHREAD_H__

#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/data/RPCCommandsData.h"
#include "iCub/plantIdentification/data/LogData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"
#include "iCub/plantIdentification/util/EventsUtil.h"
#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
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

				/* ******* Controllers utility                          ******* */
                iCub::plantIdentification::ControllersUtil *controllersUtil;

				/* ****** Ports utility                                 ****** */
				iCub::plantIdentification::PortsUtil *portsUtil;

				/* ****** Events utility                                 ****** */
				iCub::plantIdentification::EventsUtil *eventsUtil;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;


			public:

				/* ****** Task data										****** */
				iCub::plantIdentification::TaskData *taskData;

                TaskThread(const int aPeriod, const yarp::os::ResourceFinder &aRf);
                virtual ~TaskThread();

                virtual bool threadInit();
                virtual void run();
                virtual void threadRelease();

				bool initializeGrasping();
				bool afterRun(bool openHand);

				bool setArmInTaskPosition();

				void set(iCub::plantIdentification::RPCSetCmdArgName paramName,yarp::os::Value paramValue,iCub::plantIdentification::RPCCommandsData &rpcCmdData);
				void task(iCub::plantIdentification::RPCTaskCmdArgName paramName,iCub::plantIdentification::TaskName taskName,yarp::os::Value paramValue,iCub::plantIdentification::RPCCommandsData &rpcCmdData);
				void view(iCub::plantIdentification::RPCViewCmdArgName paramName,iCub::plantIdentification::RPCCommandsData &rpcCmdData);
				void help(iCub::plantIdentification::RPCCommandsData &rpcCmdData);

                void testShowEndEffectors();
				bool eventTriggered(EventToTrigger eventToTrigger,int index);
        };
    }
}

#endif

