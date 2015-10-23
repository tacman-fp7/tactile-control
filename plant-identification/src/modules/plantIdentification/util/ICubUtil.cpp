#include "iCub/plantIdentification/util/ICubUtil.h"

#include <cmath>
#include <string>
#include <sstream>

#define N_HID_NODES_2F 3
#define N_HID_NODES_3F 3

using iCub::plantIdentification::ICubUtil;

using yarp::os::Bottle;
using yarp::os::Value;




double ICubUtil::getForce(std::vector<double>& tactileData,iCub::plantIdentification::ForceCalculationMode forceCalculationMode){

	switch(forceCalculationMode){

	case SIMPLE_SUM:
		return getForceBySimpleSum(tactileData);
		break;
	case WEIGHTED_SUM:
		return getForceByWeightedSum(tactileData);
		break;
	case MAPPING:
		return getForceByLearntMapping(tactileData);
		break;
	default:
		return 0;
	}

}

double ICubUtil::getForceBySimpleSum(std::vector<double>& tactileData){

	double partialSum = 0.0;

	for(int i = 0; i < tactileData.size(); i++){
		partialSum += tactileData[i];
	}

	return partialSum;
}

double ICubUtil::getForceByWeightedSum(std::vector<double>& tactileData){

	double partialXSum = 0.0;
	double partialYSum = 0.0;
	double partialZSum = 0.0;
	std::vector<double> unitVector(3);

	for(int i = 0; i < tactileData.size(); i++){
		
		getUnitVector(i,unitVector);

		partialXSum += tactileData[i]*unitVector[0];
		partialYSum += tactileData[i]*unitVector[1];
		partialZSum += tactileData[i]*unitVector[2];
	}

	return sqrt(partialXSum*partialXSum + partialYSum*partialYSum + partialZSum*partialZSum);
}

void ICubUtil::getUnitVector(int index,std::vector<double>& unitVector){

	switch(index){

	case 0:
		unitVector[0] = -1.0;
		unitVector[1] = 0.0;
		unitVector[2] = 0.0;
		break;

	case 1:
		unitVector[0] = -0.39956;
		unitVector[1] = 0.0;
		unitVector[2] = 0.91671;
		break;

	case 2:
		unitVector[0] = -0.39956;
		unitVector[1] = 0.0;
		unitVector[2] = 0.91671;
		break;

	case 3:
		unitVector[0] = -1.0;
		unitVector[1] = 0.0;
		unitVector[2] = 0.0;
		break;

	case 4:
		unitVector[0] = -0.78673;
		unitVector[1] = 0.60316;
		unitVector[2] = 0.13140;
		break;

	case 5:
		unitVector[0] = -0.30907;
		unitVector[1] = 0.47765;
		unitVector[2] = 0.82239;
		break;

	case 6:
		unitVector[0] = 0.0;
		unitVector[1] = 1.0;
		unitVector[2] = 0.0;
		break;

	case 7:
		unitVector[0] = 0.30907;
		unitVector[1] = 0.47765;
		unitVector[2] = 0.82239;
		break;

	case 8:
		unitVector[0] = 0.78673;
		unitVector[1] = 0.60316;
		unitVector[2] = 0.13140;
		break;

	case 9:
		unitVector[0] = 1.0;
		unitVector[1] = 0.0;
		unitVector[2] = 0.0;
		break;

	case 10:
		unitVector[0] = 0.39956;
		unitVector[1] = 0.0;
		unitVector[2] = 0.91671;
		break;

	case 11:
		unitVector[0] = 0.39956;
		unitVector[1] = 0.0;
		unitVector[2] = 0.91671;
		break;
	}

}


double ICubUtil::getForceByLearntMapping(std::vector<double>& tactileData){

	return 0;
}

void ICubUtil::getNNOptionsForErrorPrediction2Fingers(Bottle& neuralNetworkOptions){

	// values taken from net2F3 in 2-3fingManifoldsNeuralNetworks_dist.mat, where distance from best position and current position is learnt, and
	// where the definition of position is "MID - TH". Used in September/October 2015.
	// if uncommented, N_HID_NODES_2F = 3 and N_HID_NODES_3F = 3 have to be assigned
	//int numInputNodes = 2;
	//int numHiddenNodes = N_HID_NODES_2F;
	//int numOutputNodes = 1;
	//double inputWeights_0[N_HID_NODES_2F] = {0.9475,-0.3622,1.1300};
	//double inputWeights_1[N_HID_NODES_2F] = {0.1268,9.6247,0.3733};
	//double outputWeights[N_HID_NODES_2F] = {1.8636,-0.0991,0.8172};
	//double inputBiases[N_HID_NODES_2F] = {-1.6453,1.2764,0.3451};
	//double outputBiases[1] = {1.3827};
	//double inMinMaxX_0[2] = {-45.3710,41.5327};
	//double inMinMaxX_1[2] = {25.9258,64.9437};
	//double inMinMaxY_0[2] = {-1,1};
	//double inMinMaxY_1[2] = {-1,1};
	//double outMinMaxY[2] = {-1,1};
	//double outMinMaxX[2] = {-55.7832,57.9475};

	// values taken from net2F1 in 2-3fingManifoldsNeuralNetworks_pos.mat, where best position is learnt, and
	// where the definition of position is "(MID - TH)/2". Used in from the ond of October 2015.
	// if uncommented, N_HID_NODES_2F = 1 and N_HID_NODES_3F = 3 have to be assigned
	int numInputNodes = 2;
	int numHiddenNodes = N_HID_NODES_2F;
	int numOutputNodes = 1;
	double inputWeights_0[N_HID_NODES_2F] = {-0.1748};
	double inputWeights_1[N_HID_NODES_2F] = {0.3504};
	double outputWeights[N_HID_NODES_2F] = {1.7567};
	double inputBiases[N_HID_NODES_2F] = {0.2792};
	double outputBiases[1] = {-0.5383};
	double inMinMaxX_0[2] = {-45.3710,41.5327};
	double inMinMaxX_1[2] = {25.9258,64.9437};
	double inMinMaxY_0[2] = {-1,1};
	double inMinMaxY_1[2] = {-1,1};
	double outMinMaxY[2] = {-1,1};
	double outMinMaxX[2] = {-5.9985,6.7621};

	std::stringstream ss;
	
	addOption(neuralNetworkOptions,"numInputNodes",numInputNodes);
	addOption(neuralNetworkOptions,"numHiddenNodes",numHiddenNodes);
	addOption(neuralNetworkOptions,"numOutputNodes",numOutputNodes);

	for(int i = 0; i < N_HID_NODES_2F; i++){
		ss.str("");
		ss << "IW_" << i;
		addOption(neuralNetworkOptions,ss.str().c_str(),inputWeights_0[i],inputWeights_1[i]);
	}
	addOption(neuralNetworkOptions,"LW_0",outputWeights,N_HID_NODES_2F);

	addOption(neuralNetworkOptions,"b1",inputBiases,N_HID_NODES_2F);
	addOption(neuralNetworkOptions,"b2",outputBiases,1);

	addOption(neuralNetworkOptions,"inMinMaxX_0",inMinMaxX_0[0],inMinMaxX_0[1]);
	addOption(neuralNetworkOptions,"inMinMaxX_1",inMinMaxX_1[0],inMinMaxX_1[1]);
	
	addOption(neuralNetworkOptions,"inMinMaxY_0",inMinMaxY_0[0],inMinMaxY_0[1]);
	addOption(neuralNetworkOptions,"inMinMaxY_1",inMinMaxY_1[0],inMinMaxY_1[1]);

	addOption(neuralNetworkOptions,"outMinMaxY_0",outMinMaxY[0],outMinMaxY[1]);
	addOption(neuralNetworkOptions,"outMinMaxX_0",outMinMaxX[0],outMinMaxX[1]);

}

void ICubUtil::getNNOptionsForErrorPrediction3Fingers(Bottle& neuralNetworkOptions){

	// values taken from net3F3 in 2-3fingManifoldsNeuralNetworks_dist.mat, where distance from best position and current position is learnt, and
	// where the definition of position is "(IND + MID)/2 - TH". Used in September/October 2015.
	// if uncommented, N_HID_NODES_2F = 3 and N_HID_NODES_3F = 3 have to be assigned
	//int numInputNodes = 3;
	//int numHiddenNodes = N_HID_NODES_3F;
	//int numOutputNodes = 1;
	//double inputWeights_0[N_HID_NODES_3F] = {0.6257,-0.1632,1.3617};
	//double inputWeights_1[N_HID_NODES_3F] = {0.2470,0.0537,15.1220};
	//double inputWeights_2[N_HID_NODES_3F] = {2.0194,-0.0460,3.2803};
	//double outputWeights[N_HID_NODES_3F] = {0.2803,6.8676,0.0386};
	//double inputBiases[N_HID_NODES_3F] = {-1.4054,0.3551,-6.7669};
	//double outputBiases[1] = {-2.2777};
	//double inMinMaxX_0[2] = {-21.1274,88.1176};
	//double inMinMaxX_1[2] = {25.3521,75.3450};
	//double inMinMaxX_2[2] = {-1.0962,6.3491};
	//double inMinMaxY_0[2] = {-1,1};
	//double inMinMaxY_1[2] = {-1,1};
	//double inMinMaxY_2[2] = {-1,1};
	//double outMinMaxY[2] = {-1,1};
	double outMinMaxX[2] = {-55.7063,62.0727};

	// values taken from net3F3 in 2-3fingManifoldsNeuralNetworks_dist.mat, where distance from best position and current position is learnt, and
	// where the definition of position is "(IND + MID)/2 - TH". Used in September/October 2015.
	// if uncommented, N_HID_NODES_2F = 1 and N_HID_NODES_3F = 3 have to be assigned
	int numInputNodes = 3;
	int numHiddenNodes = N_HID_NODES_3F;
	int numOutputNodes = 1;
	double inputWeights_0[N_HID_NODES_3F] = {2.2680, 0.1827, -2.1783};
	double inputWeights_1[N_HID_NODES_3F] = {1.2908, 0.3028, -3.3035};
	double inputWeights_2[N_HID_NODES_3F] = {9.1037, -0.1681, -6.4834};
	double outputWeights[N_HID_NODES_3F] = {0.1558, 4.1576, -0.1811};
	double inputBiases[N_HID_NODES_3F] = {-6.3656, 0.7623, 3.7774};
	double outputBiases[1] = {-2.2479};
	double inMinMaxX_0[2] = {-21.1274,88.1176};
	double inMinMaxX_1[2] = {25.3521,75.3450};
	double inMinMaxX_2[2] = {-1.0962,6.3491};
	double inMinMaxY_0[2] = {-1,1};
	double inMinMaxY_1[2] = {-1,1};
	double inMinMaxY_2[2] = {-1,1};
	double outMinMaxY[2] = {-1,1};
	double outMinMaxX[2] = { 0.3040, 21.8555};

	std::stringstream ss;
	
	addOption(neuralNetworkOptions,"numInputNodes",numInputNodes);
	addOption(neuralNetworkOptions,"numHiddenNodes",numHiddenNodes);
	addOption(neuralNetworkOptions,"numOutputNodes",numOutputNodes);

	for(int i = 0; i < N_HID_NODES_3F; i++){
		ss.str("");
		ss << "IW_" << i;
		addOption(neuralNetworkOptions,ss.str().c_str(),inputWeights_0[i],inputWeights_1[i],inputWeights_2[i]);
	}
	addOption(neuralNetworkOptions,"LW_0",outputWeights,N_HID_NODES_3F);

	addOption(neuralNetworkOptions,"b1",inputBiases,N_HID_NODES_3F);
	addOption(neuralNetworkOptions,"b2",outputBiases,1);

	addOption(neuralNetworkOptions,"inMinMaxX_0",inMinMaxX_0[0],inMinMaxX_0[1]);
	addOption(neuralNetworkOptions,"inMinMaxX_1",inMinMaxX_1[0],inMinMaxX_1[1]);
	addOption(neuralNetworkOptions,"inMinMaxX_2",inMinMaxX_2[0],inMinMaxX_2[1]);
	
	addOption(neuralNetworkOptions,"inMinMaxY_0",inMinMaxY_0[0],inMinMaxY_0[1]);
	addOption(neuralNetworkOptions,"inMinMaxY_1",inMinMaxY_1[0],inMinMaxY_1[1]);
	addOption(neuralNetworkOptions,"inMinMaxY_2",inMinMaxY_2[0],inMinMaxY_2[1]);

	addOption(neuralNetworkOptions,"outMinMaxY_0",outMinMaxY[0],outMinMaxY[1]);
	addOption(neuralNetworkOptions,"outMinMaxX_0",outMinMaxX[0],outMinMaxX[1]);

}

void ICubUtil::rotateFingersData(std::vector<double>& fingersAngles,std::vector<double>& rotatedFingersAngles){

	if (fingersAngles.size() == 2){
		rotatedFingersAngles.resize(2);

		rotatedFingersAngles[0] = fingersAngles[0]*0.7226 + fingersAngles[1]*(-0.6913);
		rotatedFingersAngles[1] = fingersAngles[0]*0.6913 + fingersAngles[1]*(0.7226);

	} else { // fingersAngles.size() == 3
		rotatedFingersAngles.resize(3);

		rotatedFingersAngles[0] = fingersAngles[0]*(-0.4686) + fingersAngles[1]*0.6483 + fingersAngles[2]*0.6001;
		rotatedFingersAngles[1] = fingersAngles[0]*0.8831 + fingersAngles[1]*0.3630 + fingersAngles[2]*0.2973;
		rotatedFingersAngles[2] = fingersAngles[0]*0.0251 + fingersAngles[1]*(-0.6693) + fingersAngles[2]*0.7426;

	}
}

void ICubUtil::addOption(Bottle &bottle,const char *paramName,Value paramValue){

	Bottle paramBottle;

	paramBottle.add(paramName);
	paramBottle.add(paramValue);

	bottle.addList() = paramBottle;
}

void ICubUtil::addOption(Bottle &bottle,const char *paramName,Value paramValue1,Value paramValue2){

	Value valueArray[2];
	valueArray[0] = paramValue1;
	valueArray[1] = paramValue2;

	addOption(bottle,paramName,valueArray,2);
}

void ICubUtil::addOption(Bottle &bottle,const char *paramName,Value paramValue1,Value paramValue2,Value paramValue3){

	Value valueArray[3];
	valueArray[0] = paramValue1;
	valueArray[1] = paramValue2;
	valueArray[2] = paramValue3;

	addOption(bottle,paramName,valueArray,3);
}

void ICubUtil::addOption(Bottle &bottle,const char *paramName,double paramValueList[],int numElem){

	Bottle valueBottle,paramBottle;

	for(int i = 0; i < numElem; i++){
		valueBottle.add(Value(paramValueList[i]));
	}

	paramBottle.add(paramName);
	paramBottle.addList() = valueBottle;

	bottle.addList() = paramBottle;
}

void ICubUtil::addOption(Bottle &bottle,const char *paramName,Value paramValueList[],int numElem){

	Bottle valueBottle,paramBottle;

	for(int i = 0; i < numElem; i++){
		valueBottle.add(paramValueList[i]);
	}

	paramBottle.add(paramName);
	paramBottle.addList() = valueBottle;

	bottle.addList() = paramBottle;
}


static void addOption(yarp::os::Bottle &bottle,const char *paramName,double paramValueList[],double numElem);
