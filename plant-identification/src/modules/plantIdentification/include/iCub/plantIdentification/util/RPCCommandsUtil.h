#ifndef __ICUB_PLANTIDENTIFICATION_RPCCOMMANDSUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_RPCCOMMANDSUTIL_H__

#include "iCub/plantIdentification/PlantIdentificationEnums.h"
#include "iCub/plantIdentification/data/RPCCommandsData.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

#include <map>

namespace iCub {
    namespace plantIdentification {

        class RPCCommandsUtil {

            private:

                iCub::plantIdentification::RPCCommandsData *rpcData;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

                iCub::plantIdentification::RPCMainCmdName mainCmd;
                iCub::plantIdentification::RPCSetCmdArgName setCmdArg;
                iCub::plantIdentification::RPCTaskCmdArgName taskCmdArg;
                iCub::plantIdentification::RPCViewCmdArgName viewCmdArg;
                iCub::plantIdentification::RPCMlCmdArgName mlCmdArg;
                iCub::plantIdentification::TaskName task;

                yarp::os::Value argValue;


                RPCCommandsUtil();

                void init(iCub::plantIdentification::RPCCommandsData *rpcData);

                void processCommand(const yarp::os::Bottle &rpcCmdBottle);

            private:

                void processTaskCommand(const yarp::os::Bottle &rpcCmdBottle);

};
    } //namespace plantIdentification
} //namespace iCub

#endif

