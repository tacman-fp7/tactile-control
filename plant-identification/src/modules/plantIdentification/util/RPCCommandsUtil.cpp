#include "iCub/plantIdentification/util/RPCCommandsUtil.h"

#include <stdexcept>

using iCub::plantIdentification::RPCCommandsUtil;
using iCub::plantIdentification::RPCCommandsData;
using iCub::plantIdentification::RPCMainCmdName;
using iCub::plantIdentification::RPCSetCmdArgName;
using iCub::plantIdentification::RPCTaskCmdArgName;
using iCub::plantIdentification::RPCViewCmdArgName;
using iCub::plantIdentification::TaskName;

using yarp::os::Bottle;

using std::string;
using std::pair;

RPCCommandsUtil::RPCCommandsUtil(){}

void RPCCommandsUtil::init(RPCCommandsData *rpcData){

    this->rpcData = rpcData;

    dbgTag = "RPCCommandsUtil: ";
}

bool RPCCommandsUtil::processCommand(const Bottle &rpcCmdBottle){

    try {
        mainCmd = rpcData->mainCmdRevMap.at(rpcCmdBottle.get(0).asString());
    } catch(const std::out_of_range& oor){
        return false;
    }

    bool ok = true;

    switch (mainCmd){

    case HELP: // do nothing
        break;
    case SET:
        if (rpcCmdBottle.size() < 3){
            return false;
        }
        try {
            setCmdArg = rpcData->setCmdArgRevMap[rpcCmdBottle.get(1).asString()];
        } catch(const std::out_of_range& oor){
            return false;
        }
        argValue = rpcCmdBottle.get(2);
        break;
    case TASK:
        return processTaskCommand(rpcCmdBottle);
        break;
    case VIEW:
        if (rpcCmdBottle.size() < 2){
            return false;
        }
        try {
            viewCmdArg = rpcData->viewCmdArgRevMap[rpcCmdBottle.get(1).asString()];
        } catch(const std::out_of_range& oor){
            return false;
        }
        break;
    case START: // do nothing
        break;
    case STOP: // do nothing
        break;
    case OPEN:
        if (rpcCmdBottle.size() > 1 ){
            argValue = rpcCmdBottle.get(1);
        } else {
            argValue = yarp::os::Value("");
        }
        break;
    case ARM:
        if (rpcCmdBottle.size() > 1 ){
            argValue = rpcCmdBottle.get(1);
        } else {
            argValue = yarp::os::Value("");
        }
        break;
    case GRASP: // do nothing
        break;
    case WAVE: // do nothing
        break;
    case ML:
        if (rpcCmdBottle.size() < 2){
            return false;
        }
        try {
            mlCmdArg = rpcData->mlCmdArgRevMap[rpcCmdBottle.get(1).asString()];
        } catch(const std::out_of_range& oor){
            return false;
        }
        if (rpcCmdBottle.size() > 2){
            argValue = rpcCmdBottle.get(2);
        } else {
            argValue = yarp::os::Value("");
        }
        break;
    case QUIT: // do nothing
        break;

    };

    return true;
}

bool RPCCommandsUtil::processTaskCommand(const Bottle &rpcCmdBottle){

    try {
        taskCmdArg = rpcData->taskCmdArgRevMap[rpcCmdBottle.get(1).asString()];
    } catch(const std::out_of_range& oor){
        return false;
    }
        
    switch (taskCmdArg){

    case ADD:
        if (rpcCmdBottle.size() < 4){
            return false;
        }
        try {
            task = rpcData->taskRevMap[rpcCmdBottle.get(2).asString()];
        } catch(const std::out_of_range& oor){
            return false;
        }
        argValue = rpcCmdBottle.get(3);
        break;
    case EMPTY: // do nothing
        break;
    case POP: // do nothing
        break;
    }

    return true;
}
