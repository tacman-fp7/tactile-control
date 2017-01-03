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

	string moduleName = "stableGrasp";
	string whichHand = rf.check("whichHand", Value("right")).asString().c_str();
    bool specifyHand = rf.check("specifyHand",Value(0)).asInt() != 0;
    string portPrefix;
    if (specifyHand){
        portPrefix = "/" + moduleName + "/" + whichHand + "_hand";
    } else {
        portPrefix = "/" + moduleName;
    }

    // icub ports
    string icubSkinRawPortName = "/icub/skin/" + whichHand + "_hand";
    string icubSkinCompPortName = "/icub/skin/" + whichHand + "_hand_comp";
    string icubHandEncodersRawPortName = "/icub/" + whichHand + "_hand/analog:o";

    // input ports
    string moduleSkinRawPortName = portPrefix + "/tactile_raw:i";
    string moduleSkinCompPortName = portPrefix + "/tactile_comp:i";
    string moduleHandEncodersRawPortName = portPrefix + "/encoders_raw:i";
    string policyActionsPortName = portPrefix + "/policy_actions:i";
    string forceSensorPortName = portPrefix + "/force_sensor:i";
	string thumbRealForcePortName = portPrefix + "/real_force/thumb:i";
	string indexFingerRealForcePortName = portPrefix + "/real_force/index_finger:i";
	string middleFingerRealForcePortName = portPrefix + "/real_force/middle_finger:i";

    // output ports
    string logDataPortName = portPrefix + "/log:o";
    string infoDataPortName = portPrefix + "/info";
    string controlDataPortName = portPrefix + "/control";
    string gmmDataPortName = portPrefix + "/gmm:o";
    string gmmRegressionDataPortName = portPrefix + "/gmmRegression:o";
    string objectRecognitionDataPortName = portPrefix + "/object_recognition_log:o";
    string gripStrengthDataPortName = portPrefix + "/grip_strength:o";


//	string middleFingerRealForcePortName = "/" + moduleName + "/real_force/thumb:i";
//	string indexFingerRealForcePortName = "/" + moduleName + "/real_force/index_finger:i";
//	string thumbRealForcePortName = "/" + moduleName + "/real_force/middle_finger:i";


    // opening ports
	if (!portSkinRawIn.open(moduleSkinRawPortName)){
        cout << dbgTag << "could not open " << moduleSkinRawPortName << " port \n";
        return false;
    }
	if (!portSkinCompIn.open(moduleSkinCompPortName)){
        cout << dbgTag << "could not open " << moduleSkinCompPortName << " port \n";
        return false;
    }
	if (!portHandEncodersRawIn.open(moduleHandEncodersRawPortName)){
        cout << dbgTag << "could not open " << moduleHandEncodersRawPortName << " port \n";
        return false;
    }
	if (!portPolicyActionsIn.open(policyActionsPortName)){
        cout << dbgTag << "could not open " << policyActionsPortName << " port \n";
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
	if (!portGMMDataOut.open(gmmDataPortName)){
        cout << dbgTag << "could not open " << gmmDataPortName << " port \n";
        return false;
    }
	if (!portGMMRegressionDataOut.open(gmmRegressionDataPortName)){
        cout << dbgTag << "could not open " << gmmRegressionDataPortName << " port \n";
        return false;
    }
    if (!portObjRecognDataOut.open(objectRecognitionDataPortName)){
        cout << dbgTag << "could not open " << objectRecognitionDataPortName << " port \n";
        return false;
    }
    if (!portGripStrengthDataOut.open(gripStrengthDataPortName)){
        cout << dbgTag << "could not open " << gripStrengthDataPortName << " port \n";
        return false;
    }
	if (!portForceSensorIn.open(forceSensorPortName)){
        cout << dbgTag << "could not open " << forceSensorPortName << " port \n";
        return false;
    }
	if (!portThumbRealForceIn.open(thumbRealForcePortName)){
        cout << dbgTag << "could not open " << thumbRealForcePortName << " port \n";
        return false;
    }
	if (!portIndexFingerRealForceIn.open(indexFingerRealForcePortName)){
        cout << dbgTag << "could not open " << indexFingerRealForcePortName << " port \n";
        return false;
    }
	if (!portMiddleFingerRealForceIn.open(middleFingerRealForcePortName)){
        cout << dbgTag << "could not open " << middleFingerRealForcePortName << " port \n";
        return false;
    }



	// connecting ports
	if (!Network::connect(icubSkinRawPortName,moduleSkinRawPortName)){
        cout << dbgTag << "could not connect ports: " << icubSkinRawPortName << " -> " <<  moduleSkinRawPortName << "\n";
        return false;
    }
	if (!Network::connect(icubSkinCompPortName,moduleSkinCompPortName)){
        cout << dbgTag << "could not connect ports: " << icubSkinCompPortName << " -> " <<  moduleSkinCompPortName << "\n";
        return false;
    }
	if (!Network::connect(icubHandEncodersRawPortName,moduleHandEncodersRawPortName)){
        cout << dbgTag << "could not connect ports: " << icubHandEncodersRawPortName << " -> " <<  moduleHandEncodersRawPortName << "\n";
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

	for(size_t i = 0; i < commonData->overallFingerPressureByWeightedSum.size(); i++){
		infoBottle.add(commonData->overallFingerPressureByWeightedSum[i]);
	}
	for(size_t i = 0; i < commonData->overallFingerPressureBySimpleSum.size(); i++){
		infoBottle.add(commonData->overallFingerPressureBySimpleSum[i]);
	}

	portInfoDataOut.write();

	return true;
}

bool PortsUtil::sendControlData(string taskId,string experimentDescription,string previousExperimentDescription,double targetGripStrength,double actualGripStrength,double u,double error,double svCurrentPosition,double actualCurrentTargetPose,double finalTargetPose,double estimatedFinalPose,double svKp,double svKi,double svKd,double thumbEnc,double indexEnc,double middleEnc,double enc8,std::vector<double> &pressureTarget,std::vector<double> &actualPressure,std::vector<double> &pwm,std::vector<int> &fingersList){

	using yarp::os::Bottle;

	Bottle& ctrlBottle = portControlDataOut.prepare();
	ctrlBottle.clear();

	double fingerJoint;

	ctrlBottle.addString(taskId);//1
	ctrlBottle.addString(experimentDescription);//2
	ctrlBottle.addString(previousExperimentDescription);//3
	ctrlBottle.addInt(pressureTarget.size());//4
	ctrlBottle.addDouble(targetGripStrength);//5
	ctrlBottle.addDouble(actualGripStrength);//6
	ctrlBottle.addDouble(u);//7
	ctrlBottle.addDouble(error);//8
	ctrlBottle.addDouble(actualCurrentTargetPose);//9
	ctrlBottle.addDouble(finalTargetPose);//10
	ctrlBottle.addDouble(svCurrentPosition);//11
	ctrlBottle.addDouble(estimatedFinalPose);//12
	ctrlBottle.addDouble(svKp);//13
	ctrlBottle.addDouble(svKi);//14
	ctrlBottle.addDouble(svKd);//15
	ctrlBottle.addDouble(thumbEnc);//16
	ctrlBottle.addDouble(indexEnc);//17
	ctrlBottle.addDouble(middleEnc);//18
	ctrlBottle.addDouble(enc8);//19
	for(int i = 0; i < pressureTarget.size(); i++){
		// TODO use function getProximalJointFromFingerNumber
//		if (fingersList[i] == 0) fingerJoint == 11;
//		else if (fingersList[i] == 1) fingerJoint == 13;
//		else fingerJoint == 9;
		
		ctrlBottle.addInt(fingersList[i]);// 20 ... 24 ...
//		ctrlBottle.addDouble(armEncodersAngles[fingerJoint]);// 8 ... 12 ...
		ctrlBottle.addDouble(pwm[i]);// 21 ... 25 ...
		ctrlBottle.addDouble(pressureTarget[i]);// 22 ... 26 ...
		ctrlBottle.addDouble(actualPressure[fingersList[i]]);// 23 ... 27...
	}

	portControlDataOut.write();

	return true;
}


bool PortsUtil::sendGripStrengthData(std::string experimentDescription,std::string previousExperimentDescription,double targetGripStrength,iCub::plantIdentification::TaskCommonData *commonData){

    using yarp::os::Bottle;

    Bottle& gripStrengthBottle = portGripStrengthDataOut.prepare();
    gripStrengthBottle.clear();

    gripStrengthBottle.addString(experimentDescription);//1
    gripStrengthBottle.addString(previousExperimentDescription);//2
    gripStrengthBottle.addDouble(targetGripStrength);//3

    double thumbForce = commonData->overallFingerPressure[4];
    double indexFingerForce = commonData->overallFingerPressure[0];
    double middleFingerForce = commonData->overallFingerPressure[1];


    double actualGripStrength = 2.0/3.0 * thumbForce + 1.0/3.0 * (indexFingerForce + middleFingerForce);

    gripStrengthBottle.addDouble(actualGripStrength);//4

    // fingers overall pressure (5) (5-9)
    for(size_t i = 0; i < commonData->overallFingerPressure.size(); i++){
        gripStrengthBottle.addDouble(commonData->overallFingerPressure[i]);
    }

    // compensated taxels feedback (60) (10-69)
    for(size_t i = 0; i < commonData->fingerTaxelsData.size(); i++){
        for(size_t j = 0; j < commonData->fingerTaxelsData[i].size(); j++){
            gripStrengthBottle.addDouble(commonData->fingerTaxelsData[i][j]);
        }
    }

    portGripStrengthDataOut.write();

    return true;
}

bool PortsUtil::sendGMMData(double gripStrength, double indexMiddleFingerPressureBalance, iCub::plantIdentification::TaskCommonData *commonData){

	using yarp::os::Bottle;

	bool thAbdRefEnabled = commonData->tpInt(83) != 0;

	Bottle& objGMMBottle = portGMMDataOut.prepare();
	objGMMBottle.clear();


    // grip strength (1) (1)
	objGMMBottle.addDouble(gripStrength);

	// index / midle finger pressure balance (1) (2)
	objGMMBottle.addDouble(indexMiddleFingerPressureBalance);

	// compensated taxels feedback (60) (3-62)
	for(size_t i = 0; i < commonData->fingerTaxelsData.size(); i++){
		for(size_t j = 0; j < commonData->fingerTaxelsData[i].size(); j++){
			objGMMBottle.addDouble(commonData->fingerTaxelsData[i][j]);
		}
	}

	// fingers overall pressure (5) (63-67)
    for(size_t i = 0; i < commonData->overallFingerPressureByWeightedSum.size(); i++){
        objGMMBottle.addDouble(commonData->overallFingerPressureByWeightedSum[i]);
	}

	// arm encoders (16) (68-83)
	for(size_t i = 0; i < commonData->armEncodersAngles.size(); i++){
		if (i == 8 && thAbdRefEnabled){
			objGMMBottle.addDouble(commonData->armEncodersAnglesReferences[i]);
		} else {
			objGMMBottle.addDouble(commonData->armEncodersAngles[i]);
		}
	}

	// raw taxels feedback (60) (84-143)
	for(size_t i = 0; i < commonData->fingerTaxelsRawData.size(); i++){
		for(size_t j = 0; j < commonData->fingerTaxelsRawData[i].size(); j++){
			objGMMBottle.addDouble(commonData->fingerTaxelsRawData[i][j]);
		}
	}

	portGMMDataOut.write();

	return true;

}

bool PortsUtil::sendGMMRegressionData(double handAperture,double indMidPosDiff,double targetHandPosition,double actualHandPosition,double filteredHandPosition,double targetThumbDistalJoint,double filteredThumbDistalJoint,double targetIndexDistalJoint,double filteredIndexDistalJoint,double targetMiddleDistalJoint,double filteredMiddleDistalJoint,double targetThumbAbductionJoint,double filteredThumbAbductionJoint, double targetIndMidForceBalance, double actualIndMidForceBalance,double targetGripStrength,double actualGripStrength,iCub::plantIdentification::TaskCommonData *commonData){


	using yarp::os::Bottle;

	Bottle& objGMMRegressionBottle = portGMMRegressionDataOut.prepare();
	objGMMRegressionBottle.clear();

	// experiment description (1)
	objGMMRegressionBottle.addString(commonData->tpStr(16));
	// previous experiment description (2)
	objGMMRegressionBottle.addString(commonData->tpStr(17));

    // hand aperture (query variable) (3)
	objGMMRegressionBottle.addDouble(handAperture);

    // index/middle finger position difference (query variable) (4)
	objGMMRegressionBottle.addDouble(indMidPosDiff);

    // target hand position (output variable) (5)
	objGMMRegressionBottle.addDouble(targetHandPosition);
	// filtered hand position (6)
	objGMMRegressionBottle.addDouble(filteredHandPosition);
    // actual hand position (7)
	objGMMRegressionBottle.addDouble(actualHandPosition);

    // target thumb distal joint (output variable) (8)
	objGMMRegressionBottle.addDouble(targetThumbDistalJoint);
	// filtered thumb distal joint (9)
	objGMMRegressionBottle.addDouble(filteredThumbDistalJoint);
    // actual thumb distal joint (10)
	objGMMRegressionBottle.addDouble(commonData->armEncodersAngles[10]);

    // target index distal joint (output variable) (11)
	objGMMRegressionBottle.addDouble(targetIndexDistalJoint);
	// filtered index distal joint (12)
	objGMMRegressionBottle.addDouble(filteredIndexDistalJoint);
    // actual index distal joint (13)
	objGMMRegressionBottle.addDouble(commonData->armEncodersAngles[12]);

    // target middle distal joint (output variable) (14)
	objGMMRegressionBottle.addDouble(targetMiddleDistalJoint);
	// filtered middle distal joint (15)
	objGMMRegressionBottle.addDouble(filteredMiddleDistalJoint);
    // actual middle distal joint (16)
	objGMMRegressionBottle.addDouble(commonData->armEncodersAngles[14]);

    // target thumb abduction joint (output variable) (17)
	objGMMRegressionBottle.addDouble(targetThumbAbductionJoint);
	// filtered thumb abduction joint (18)
	objGMMRegressionBottle.addDouble(filteredThumbAbductionJoint);
    // actual thumb abduction joint (19)
	objGMMRegressionBottle.addDouble(commonData->armEncodersAngles[8]);

    // target index/middle force balance (output variable) (20)
	objGMMRegressionBottle.addDouble(targetIndMidForceBalance);
    // actual index/middle force balance (21)
	objGMMRegressionBottle.addDouble(actualIndMidForceBalance);

	// target grip strength (output variable) (22)
	objGMMRegressionBottle.addDouble(targetGripStrength);
    // actual grip strength (23)
	objGMMRegressionBottle.addDouble(actualGripStrength);

	// arm encoders [16] (24-39)
	for(size_t i = 0; i < commonData->armEncodersAngles.size(); i++){
		objGMMRegressionBottle.addDouble(commonData->armEncodersAngles[i]);
	}

	// fingers overall pressure [5] (40-44)
    for(size_t i = 0; i < commonData->overallFingerPressureByWeightedSum.size(); i++){
        objGMMRegressionBottle.addDouble(commonData->overallFingerPressureByWeightedSum[i]);
	}

	// compensated taxels feedback [60] (45-104)
	for(size_t i = 0; i < commonData->fingerTaxelsData.size(); i++){
		for(size_t j = 0; j < commonData->fingerTaxelsData[i].size(); j++){
			objGMMRegressionBottle.addDouble(commonData->fingerTaxelsData[i][j]);
		}
	}
	
	// compensated taxels feedback [60] (105-164)
	//for(size_t i = 0; i < commonData->fingerTaxelsRawData.size(); i++){
	//	for(size_t j = 0; j < commonData->fingerTaxelsRawData[i].size(); j++){
	//		objGMMRegressionBottle.addDouble(commonData->fingerTaxelsRawData[i][j]);
	//	}
	//}

	portGMMRegressionDataOut.write();

	return true;

}


bool PortsUtil::sendObjectRecognitionData(string taskId,int objectId,iCub::plantIdentification::ObjectRecognitionTask objRecTask,int extraCode1,int extraCode2,int skipPreviousRepetition,string experimentDescription,string previousExperimentDescription,iCub::plantIdentification::TaskCommonData *commonData){

	using yarp::os::Bottle;

	Bottle& objRecognBottle = portObjRecognDataOut.prepare();
	objRecognBottle.clear();

    // text data
	objRecognBottle.addString(experimentDescription);
	objRecognBottle.addString(previousExperimentDescription);

	// no text data
	// general information (6) (1-6)
	objRecognBottle.addInt(objectId);
	objRecognBottle.addInt(objRecTask);
	objRecognBottle.addInt(extraCode1);
	objRecognBottle.addInt(extraCode2);
	objRecognBottle.addString(taskId);
	objRecognBottle.addInt(skipPreviousRepetition);

	// logging raw tactile data (24) (7-30)
//    objRecognBottle.addString("midRawTact");
	for(size_t j = 0; j < commonData->fingerTaxelsRawData[1].size(); j++){
		objRecognBottle.addDouble(commonData->fingerTaxelsRawData[1][j]);
	}
//    objRecognBottle.addString("thmbRawTact");
	for(size_t j = 0; j < commonData->fingerTaxelsRawData[4].size(); j++){
		objRecognBottle.addDouble(commonData->fingerTaxelsRawData[4][j]);
	}
	// logging compensated tactile data (24) (31-54)
//    objRecognBottle.addString("midCompTact");
	for(size_t j = 0; j < commonData->fingerTaxelsData[1].size(); j++){
		objRecognBottle.addDouble(commonData->fingerTaxelsData[1][j]);
	}
//    objRecognBottle.addString("thmbCompTact");
	for(size_t j = 0; j < commonData->fingerTaxelsData[4].size(); j++){
		objRecognBottle.addDouble(commonData->fingerTaxelsData[4][j]);
	}
	// logging processed tactile data (2) (55-56)
//    objRecognBottle.addString("overallTactVal");
	objRecognBottle.addDouble(commonData->overallFingerPressureByWeightedSum[1]);
	objRecognBottle.addDouble(commonData->overallFingerPressureByWeightedSum[4]);


	// logging raw encoders (16) (57-72)
//    objRecognBottle.addString("handRawAng");
	for(size_t i = 0; i < commonData->fingerEncodersRawData.size(); i++){
		objRecognBottle.addInt(commonData->fingerEncodersRawData[i]);
	}
//    objRecognBottle.addString("armAng");
	// logging encoders (16) (73-88)
	for(size_t i = 0; i < commonData->armEncodersAngles.size(); i++){
		objRecognBottle.addDouble(commonData->armEncodersAngles[i]);
	}

	portObjRecognDataOut.write();

	return true;
}


bool PortsUtil::readFingerSkinRawData(std::vector<std::vector<double> > &fingerTaxelsRawData,std::vector<double> &fingersSensitivityScale){

	using yarp::sig::Vector;

	Vector *iCubSkinData = portSkinRawIn.read(false);
    
	//TODO generalize fingers number
    if (iCubSkinData) {
		for(size_t i = 0; i < 5; i++){
			for (size_t j = 0; j < fingerTaxelsRawData[i].size(); j++){
				fingerTaxelsRawData[i][j] = fingersSensitivityScale[i] * (*iCubSkinData)[12*i + j];
			}
		}
	}

	return true;
}

bool PortsUtil::readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData,std::vector<double> &fingersSensitivityScale){

	using yarp::sig::Vector;

	Vector *iCubSkinData = portSkinCompIn.read(false);
    
	//TODO generalize fingers number
    if (iCubSkinData) {
		for(size_t i = 0; i < 5; i++){
			for (size_t j = 0; j < fingerTaxelsData[i].size(); j++){
				fingerTaxelsData[i][j] = fingersSensitivityScale[i] * (*iCubSkinData)[12*i + j];
			}
		}
	}

	return true;
}

bool PortsUtil::readFingerEncodersRawData(std::vector<double> &fingerEncodersRawData){

	using yarp::sig::Vector;

	Vector *iCubEncRawData = portHandEncodersRawIn.read(false);
    
    if (iCubEncRawData) {
		for (size_t i = 0; i < fingerEncodersRawData.size(); i++){
			fingerEncodersRawData[i] = (*iCubEncRawData)[i];
		}
	}

	return true;
}

bool PortsUtil::readPolicyActionsData(std::vector<double> &policyActionsData){

	using yarp::sig::Vector;

	Vector *iCubPolicyActionsData = portPolicyActionsIn.read(false);
    
    if (iCubPolicyActionsData) {
		for (size_t i = 0; i < policyActionsData.size(); i++){
			policyActionsData[i] = (*iCubPolicyActionsData)[i];
		}
		return true;
	} else {
		return false;
	}
}

bool PortsUtil::readForceSensorData(std::vector<double> &forceSensorData,std::vector<double> &forceSensorBias){

	using yarp::sig::Vector;

	Vector *portData = portForceSensorIn.read(false);
    
    if (portData) {
		for (size_t i = 0; i < forceSensorData.size(); i++){
			forceSensorData[i] = (*portData)[i] - forceSensorBias[i];
		}
		return true;
	} else {
		return false;
	}
}

bool PortsUtil::readRealForceData(std::vector<double> &realForceData){

	using yarp::sig::Vector;

	Vector *portData;// =  portThumbRealForceIn.read(false);


	portData =  portThumbRealForceIn.read(false);

    if (portData) {
		realForceData[4] = (*portData)[0];
	} else {
		return false;
	}

	portData =  portIndexFingerRealForceIn.read(false);
    
    if (portData) {
		realForceData[0] = (*portData)[0];
	} else {
		return false;
	}

	portData =  portMiddleFingerRealForceIn.read(false);
    
    if (portData) {
		realForceData[1] = (*portData)[0];
	} else {
		return false;
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

