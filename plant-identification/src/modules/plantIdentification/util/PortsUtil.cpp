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
    string moduleSkinCompPortName = "/plantIdentification/skin/" + whichHand + "_hand_comp:i";
    string icubSkinCompPortName = "/icub/skin/" + whichHand + "_hand_comp";
    string logDataPortName = "/plantIdentification/log:o";
    string infoDataPortName = "/plantIdentification/info";
    string controlDataPortName = "/plantIdentification/control";

    // opening ports
	if (!portSkinCompIn.open(moduleSkinCompPortName)){
        cout << dbgTag << "could not open " << moduleSkinCompPortName << " port \n";
        return false;
    }
	if (!portLogDataOut.open(logDataPortName)){
        cout << dbgTag << "could not open " << logDataPortName << " port \n";
        return false;
    }
	if (!portInfoDataOut.open(infoDataPortName)){
        cout << dbgTag << "could not open " << infoDataPortName << " port \n";
        return false;
    }
	if (!portControlDataOut.open(controlDataPortName)){
        cout << dbgTag << "could not open " << controlDataPortName << " port \n";
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

bool PortsUtil::sendInfoData(iCub::plantIdentification::TaskCommonData *commonData){

	using yarp::os::Bottle;

	Bottle& infoBottle = portInfoDataOut.prepare();
	infoBottle.clear();

	for(size_t i; i < commonData->overallFingerPressureByWeightedSum.size(); i++){
		infoBottle.add(commonData->overallFingerPressureByWeightedSum[i]);
	}
	for(size_t i; i < commonData->overallFingerPressureBySimpleSum.size(); i++){
		infoBottle.add(commonData->overallFingerPressureBySimpleSum[i]);
	}

	portInfoDataOut.write();

	return true;
}

bool PortsUtil::sendControlData(string taskId,string experimentDescription,string previousExperimentDescription,double s,double u,double error,double svKp,double svKi,double svKd,double thumbEnc,double indexEnc,double middleEnc,double enc8,std::vector<double> &pressureTarget,std::vector<double> &actualPressure,std::vector<double> &pwm,std::vector<int> &fingersList){

	using yarp::os::Bottle;

	Bottle& ctrlBottle = portControlDataOut.prepare();
	ctrlBottle.clear();

	double fingerJoint;

	ctrlBottle.addString(taskId);//1
	ctrlBottle.addString(experimentDescription);//2
	ctrlBottle.addString(previousExperimentDescription);//3
	ctrlBottle.addInt(pressureTarget.size());//4
	ctrlBottle.addDouble(s);//5
	ctrlBottle.addDouble(u);//6
	ctrlBottle.addDouble(error);//7
	ctrlBottle.addDouble(svKp);//8
	ctrlBottle.addDouble(svKi);//9
	ctrlBottle.addDouble(svKd);//10
	ctrlBottle.addDouble(thumbEnc);//11
	ctrlBottle.addDouble(indexEnc);//12
	ctrlBottle.addDouble(middleEnc);//13
	ctrlBottle.addDouble(enc8);//14
	for(int i = 0; i < pressureTarget.size(); i++){
		// TODO use function getProximalJointFromFingerNumber
//		if (fingersList[i] == 0) fingerJoint == 11;
//		else if (fingersList[i] == 1) fingerJoint == 13;
//		else fingerJoint == 9;
		
		ctrlBottle.addInt(fingersList[i]);// 15 ... 19 ...
//		ctrlBottle.addDouble(armEncodersAngles[fingerJoint]);// 8 ... 12 ...
		ctrlBottle.addDouble(pwm[i]);// 16 ... 20 ...
		ctrlBottle.addDouble(pressureTarget[i]);// 17 ... 21 ...
		ctrlBottle.addDouble(actualPressure[fingersList[i]]);// 18 ... 22...
	}

	portControlDataOut.write();

	return true;
}


bool PortsUtil::readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData){

	using yarp::sig::Vector;

	Vector *iCubSkinData = portSkinCompIn.read(false);
    
	//TODO generalize fingers number
    if (iCubSkinData) {
		for(size_t i = 0; i < 5; i++){
			for (size_t j = 0; j < fingerTaxelsData[i].size(); j++){
				fingerTaxelsData[i][j] = (*iCubSkinData)[12*i + j];
			}
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

