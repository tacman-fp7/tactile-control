#include "iCub/plantIdentification/util/RPCCommandsUtil.h"

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

void RPCCommandsUtil::processCommand(const Bottle &rpcCmdBottle){

	mainCmd = rpcData->mainCmdRevMap[rpcCmdBottle.get(0).asString()];
	
	switch (mainCmd){

	case SET:
		setCmdArg = rpcData->setCmdArgRevMap[rpcCmdBottle.get(1).asString()];
		argValue = rpcCmdBottle.get(2).asString();
		break;
	case TASK:
		processTaskCommand(rpcCmdBottle);
		break;
	case VIEW:
		viewCmdArg = rpcData->viewCmdArgRevMap[rpcCmdBottle.get(1).asString()];
		break;
	case START: // do nothing
		break;
	case STOP:
		break;
	case QUIT: // do nothing
		break;

	};

}


void RPCCommandsUtil::processTaskCommand(const Bottle &rpcCmdBottle){

	taskCmdArg = rpcData->taskCmdArgRevMap[rpcCmdBottle.get(1).asString()];
	switch (taskCmdArg){

	case ADD:
		task = rpcData->taskRevMap[rpcCmdBottle.get(2).asString()];
		argValue = rpcCmdBottle.get(3).asString();
		break;
	case EMPTY: // do nothing
		break;
	case POP: // do nothing
		break;
	}
}
