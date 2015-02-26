#include "iCub/plantIdentification/util/PortsUtil.h"

#include <yarp/os/Network.h>
#include <yarp/os/Value.h>

using std::string;

using yarp::os::Value;

using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::LogData;

PortsUtil::PortsUtil(yarp::os::ResourceFinder &rf){
	using yarp::os::Network;

	string whichHand = rf.check("whichHand", Value("right")).asString().c_str();

	// opening ports
	portSkinCompIn.open("/PlantIdentification/skin/" + whichHand + "_hand_comp:i");
	portLogDataOut.open("/PlantIdentification/log:o");

	// connecting ports
	Network::connect(("/icub/skin/" + whichHand + "_hand_comp"), ("/PlantIdentification/skin/" + whichHand + "_hand_comp:i"));

	dbgTag = "PortsUtil: ";
}

void PortsUtil::sendLogData(LogData &logData){

	using yarp::os::Bottle;

	Bottle& logBottle = portLogDataOut.prepare();
	logBottle.clear();
	logData.toBottle(logBottle);
	portLogDataOut.write();
}

void PortsUtil::readFingerSkinCompData(int finger,std::vector<double> &fingerTaxelsData){

	using yarp::sig::Vector;

	Vector *iCubSkinData = portSkinCompIn.read(false);

    if (iCubSkinData) {
		for (size_t i = 0; i < fingerTaxelsData.size(); i++){
			fingerTaxelsData[i] = (*iCubSkinData)[12*finger + i];
		}
	}
}

void PortsUtil::release(){

	portLogDataOut.interrupt();
	portSkinCompIn.interrupt();

	portLogDataOut.close();
	portSkinCompIn.close();
}

/* *********************************************************************************************************************** */

