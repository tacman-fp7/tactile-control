#ifndef __ICUB_PLANTIDENTIFICATION_RPCCOMMANDSDATA_H__
#define __ICUB_PLANTIDENTIFICATION_RPCCOMMANDSDATA_H__

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/Value.h>

#include <map>
#include <string>
#include <vector>

namespace iCub {
    namespace plantIdentification {

        class RPCCommandsData {

            private:

				/* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

				std::map<iCub::plantIdentification::RPCMainCmdName,std::string> mainCmdMap;
				std::map<iCub::plantIdentification::RPCMainCmdName,std::string> mainCmdDescMap;
				std::map<std::string,iCub::plantIdentification::RPCMainCmdName> mainCmdRevMap;

				std::map<iCub::plantIdentification::RPCSetCmdArgName,std::string> setCmdArgMap;
				std::map<iCub::plantIdentification::RPCSetCmdArgName,std::string> setCmdArgDescMap;
				std::map<std::string,iCub::plantIdentification::RPCSetCmdArgName> setCmdArgRevMap;

				std::map<iCub::plantIdentification::RPCTaskCmdArgName,std::string> taskCmdArgMap;
				std::map<iCub::plantIdentification::RPCTaskCmdArgName,std::string> taskCmdArgDescMap;
				std::map<std::string,iCub::plantIdentification::RPCTaskCmdArgName> taskCmdArgRevMap;

				std::map<iCub::plantIdentification::RPCViewCmdArgName,std::string> viewCmdArgMap;
				std::map<iCub::plantIdentification::RPCViewCmdArgName,std::string> viewCmdArgDescMap;
				std::map<std::string,iCub::plantIdentification::RPCViewCmdArgName> viewCmdArgRevMap;

				std::map<iCub::plantIdentification::TaskName,std::string> taskMap;
				std::map<iCub::plantIdentification::TaskName,std::string> taskDescMap;
				std::map<std::string,iCub::plantIdentification::TaskName> taskRevMap;


				RPCCommandsData();

				std::string getFullDescription(iCub::plantIdentification::RPCSetCmdArgName setCmdArgName);

				std::string getFullDescription(iCub::plantIdentification::RPCMainCmdName mainCmdName);

				void setValues(yarp::os::Value &value,std::vector<double> &valueList);

				std::string printValue(yarp::os::Value &value);

			private:

				void add(std::string mapKey,iCub::plantIdentification::RPCMainCmdName mapValue,std::string description);

				void add(std::string mapKey,iCub::plantIdentification::RPCSetCmdArgName mapValue,std::string description);

				void add(std::string mapKey,iCub::plantIdentification::RPCTaskCmdArgName mapValue,std::string description);

				void add(std::string mapKey,iCub::plantIdentification::RPCViewCmdArgName mapValue,std::string description);

				void add(std::string mapKey,iCub::plantIdentification::TaskName mapValue,std::string description);

				
};
    } //namespace plantIdentification
} //namespace iCub

#endif

