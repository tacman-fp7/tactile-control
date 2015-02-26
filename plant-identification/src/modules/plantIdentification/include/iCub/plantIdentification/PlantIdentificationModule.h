#ifndef __PLANTIDENTIFICATION_MODULE_H__
#define __PLANTIDENTIFICATION_MODULE_H__

#include "plantIdentification_IDLServer.h"
#include "iCub/plantIdentification/thread/TaskThread.h"

#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>

namespace iCub {
    namespace plantIdentification {

        class PlantIdentificationModule : public yarp::os::RFModule, public plantIdentification_IDLServer {

			private:

				/* ****** Module attributes                             ****** */
				double period;
				std::string moduleName;
				std::string robotName;
				bool closing;
                
				/* ****** RPC Ports                                     ****** */
				yarp::os::RpcServer portPlantIdentificationRPC;

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
				virtual bool close();
				virtual bool attach(yarp::os::RpcServer &source);

				/* ****** RPC Methods                                  ****** */
				virtual bool open(void);
				virtual bool grasp(void);
				virtual bool quit(void);
				//TODO methods to add using thrift
				virtual void set(iCub::plantIdentification::SetParamName paramName,std::string paramValue);
				virtual void task(iCub::plantIdentification::TaskParamName paramName,double targetValue);
				virtual void view(iCub::plantIdentification::ViewParamName paramName);
        };
    }
}

#endif

