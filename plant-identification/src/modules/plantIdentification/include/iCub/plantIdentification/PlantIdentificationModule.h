#ifndef __PLANTIDENTIFICATION_MODULE_H__
#define __PLANTIDENTIFICATION_MODULE_H__

#include "iCub/plantIdentification/thread/TaskThread.h"
#include "iCub/plantIdentification/thread/EventsThread.h"
#include "iCub/plantIdentification/util/RPCCommandsUtil.h"
#include "iCub/plantIdentification/data/RPCCommandsData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"
#include "iCub/plantIdentification/util/MLUtil.h"

#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

namespace iCub {
    namespace plantIdentification {

        class PlantIdentificationModule : public yarp::os::RFModule {

            private:

                /* ****** Module attributes                             ****** */
                int moduleThreadPeriod;
                std::string moduleName;
                std::string robotName;
                bool closing;
                bool tasksRunning;

                /* ****** RPC Ports                                     ****** */
                yarp::os::RpcServer portPlantIdentificationRPC;

                iCub::plantIdentification::RPCCommandsUtil rpcCmdUtil;
                iCub::plantIdentification::RPCCommandsData rpcCmdData;

                /* ******* Controllers utility                          ******* */
                iCub::plantIdentification::ControllersUtil *controllersUtil;

                /* ****** Ports utility                                 ****** */
                iCub::plantIdentification::PortsUtil *portsUtil;

                /* ****** Events utility                                 ****** */
                iCub::plantIdentification::EventsThread *eventsThread;

                /* ****** Task data										****** */
                iCub::plantIdentification::TaskData *taskData;

                /* ****** Threads			                            ****** */
                iCub::plantIdentification::TaskThread *taskThread;
         
                /* ****** Machine Learning utility                      ******* */
                iCub::plantIdentification::MLUtil mlUtil;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;
           
            public:
               
                PlantIdentificationModule();
                virtual ~PlantIdentificationModule();
                virtual double getPeriod();
                virtual bool configure(yarp::os::ResourceFinder &rf);
                virtual bool updateModule();
                virtual bool interruptModule();
                virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
                virtual bool close();

                /* ****** RPC Methods                                  ****** */
                bool start();
                bool stop();
                bool open(yarp::os::Value paramValue);
                bool arm(yarp::os::Value paramValue);
                bool grasp();
                bool classify();
                bool wave();
                bool ml(iCub::plantIdentification::RPCMlCmdArgName paramName,yarp::os::Value paramValue);
                bool quit();
                bool set(iCub::plantIdentification::RPCSetCmdArgName paramName,yarp::os::Value paramValue);
                bool task(iCub::plantIdentification::RPCTaskCmdArgName paramName,iCub::plantIdentification::TaskName taskName,yarp::os::Value paramValue);
                bool view(iCub::plantIdentification::RPCViewCmdArgName paramName);
                bool help();
        };
    }
}

#endif

