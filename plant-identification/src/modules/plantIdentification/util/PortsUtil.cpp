#include "iCub/plantIdentification/util/PortsUtil.h"

#include <yarp/os/Network.h>
#include <yarp/os/Value.h>

using std::string;

using yarp::os::Value;

using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::LogData;

PortsUtil::PortsUtil(){

	dbgTag = "PortsUtil: ";
}

bool PortsUtil::init(yarp::os::ResourceFinder &rf){
	using yarp::os::Network;
    using std::cout;

	string whichHand = rf.check("whichHand", Value("right")).asString().c_str();
    string moduleSkinCompPortName = "/PlantIdentification/skin/" + whichHand + "_hand_comp:i";
    string icubSkinCompPortName = "/icub/skin/" + whichHand + "_hand_comp";
    string logDataPortName = "/PlantIdentification/log:o";

    // opening ports
	if (!portSkinCompIn.open(moduleSkinCompPortName)){
        cout << dbgTag << "could not open " << moduleSkinCompPortName << " port \n";
        return false;
    }
	if (!portLogDataOut.open(logDataPortName)){
        cout << dbgTag << "could not open " << logDataPortName << " port \n";
        return false;
    }

	// connecting ports
	if (!Network::connect(icubSkinCompPortName,moduleSkinCompPortName)){
        cout << dbgTag << "could not connect ports: " << icubSkinCompPortName << " -> " <<  moduleSkinCompPortName << "\n";
        return false;
    }


	return true;
}

bool PortsUtil::sendLogData(LogData &logData){

	using yarp::os::Bottle;

	Bottle& logBottle = portLogDataOut.prepare();
	logBottle.clear();
	logData.toBottle(logBottle);
	portLogDataOut.write();

	return true;
}

bool PortsUtil::readFingerSkinCompData(int finger,std::vector<double> &fingerTaxelsData){

	using yarp::sig::Vector;

	Vector *iCubSkinData = portSkinCompIn.read(false);
    
    if (iCubSkinData) {

		for (size_t i = 0; i < fingerTaxelsData.size(); i++){
			fingerTaxelsData[i] = (*iCubSkinData)[12*finger + i];
		}
	}

	return true;
}

bool PortsUtil::release(){

	portLogDataOut.interrupt();
	portSkinCompIn.interrupt();

	portLogDataOut.close();
	portSkinCompIn.close();

	return true;
}

/* *********************************************************************************************************************** */

