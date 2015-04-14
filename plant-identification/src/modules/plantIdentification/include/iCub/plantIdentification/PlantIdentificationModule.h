#ifndef __PLANTIDENTIFICATION_MODULE_H__
#define __PLANTIDENTIFICATION_MODULE_H__

#include "iCub/plantIdentification/thread/TaskThread.h"
#include "iCub/plantIdentification/util/RPCCommandsUtil.h"
#include "iCub/plantIdentification/data/RPCCommandsData.h"

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
				double period;
				std::string moduleName;
				std::string robotName;
				bool closing;
                
				/* ****** RPC Ports                                     ****** */
				yarp::os::RpcServer portPlantIdentificationRPC;

				iCub::plantIdentification::RPCCommandsUtil rpcCmdUtil;
				iCub::plantIdentification::RPCCommandsData rpcCmdData;


				/* ****** Threads			                            ****** */
				iCub::plantIdentification::TaskThread *taskThread;
         
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
				bool open();
				bool arm();
				bool quit();
				void set(iCub::plantIdentification::RPCSetCmdArgName paramName,yarp::os::Value paramValue);
				void task(iCub::plantIdentification::RPCTaskCmdArgName paramName,iCub::plantIdentification::TaskName taskName,yarp::os::Value paramValue);
				void view(iCub::plantIdentification::RPCViewCmdArgName paramName);
				void help();
        };
    }
}

#endif

