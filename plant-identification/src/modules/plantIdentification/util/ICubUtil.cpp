#include "iCub/plantIdentification/util/ICubUtil.h"

#include <cmath>
#include <string>
#include <sstream>

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

Bottle* ICubUtil::getNNOptionsForErrorPrediction2Fingers(){

	Bottle neuralNetworkOptions;

	int numInputNodes = 2;
	int numHiddenNodes = 10;
	int numOutputNodes = 1;
	double inputWeights_0[10] = {0};
	double inputWeights_1[10] = {0};
	double outputWeights[10] = {0};
	double inputBiases[10] = {0};
	double outputBiases[1] = {0};
	double inMinMaxX_0[2] = {0,0};
	double inMinMaxX_1[2] = {0,0};
	double inMinMaxY[2] = {-1,1};
	double outMinMaxY[2] = {-1,1};
	double outMinMaxX[2] = {0,0};
	std::stringstream ss;
	
	addOption(neuralNetworkOptions,"numInputNodes",numInputNodes);
	addOption(neuralNetworkOptions,"numHiddenNodes",numHiddenNodes);
	addOption(neuralNetworkOptions,"numOutputNodes",numOutputNodes);

	for(int i = 0; i < 10; i++){
		ss.clear();
		ss << "IW_" << i;
		addOption(neuralNetworkOptions,ss.str().c_str(),inputWeights_0[i],inputWeights_1[i]);
	}
	addOption(neuralNetworkOptions,"LW_0",outputWeights,10);

	addOption(neuralNetworkOptions,"b1",inputBiases,10);
	addOption(neuralNetworkOptions,"b2",outputBiases,1);

	addOption(neuralNetworkOptions,"inMinMaxX_0",inMinMaxX_0[0],inMinMaxX_0[1]);
	addOption(neuralNetworkOptions,"inMinMaxX_1",inMinMaxX_1[0],inMinMaxX_1[1]);
	
	addOption(neuralNetworkOptions,"inMinMaxY_0",inMinMaxY[0],inMinMaxY[1]);
	addOption(neuralNetworkOptions,"inMinMaxY_1",inMinMaxY[0],inMinMaxY[1]);

	addOption(neuralNetworkOptions,"outMinMaxY_0",outMinMaxY[0],outMinMaxY[1]);
	addOption(neuralNetworkOptions,"outMinMaxX_0",outMinMaxX[0],outMinMaxX[1]);

	return &neuralNetworkOptions;
}

Bottle* ICubUtil::getNNOptionsForErrorPrediction3Fingers(){

	Bottle neuralNetworkOptions;

	int numInputNodes = 3;
	int numHiddenNodes = 10;
	int numOutputNodes = 1;
	double inputWeights_0[10] = {0};
	double inputWeights_1[10] = {0};
	double inputWeights_2[10] = {0};
	double outputWeights[10] = {0};
	double inputBiases[10] = {0};
	double outputBiases[1] = {0};
	double inMinMaxX_0[2] = {0,0};
	double inMinMaxX_1[2] = {0,0};
	double inMinMaxX_2[2] = {0,0};
	double inMinMaxY[2] = {-1,1};
	double outMinMaxY[2] = {-1,1};
	double outMinMaxX[2] = {0,0};
	std::stringstream ss;
	
	addOption(neuralNetworkOptions,"numInputNodes",numInputNodes);
	addOption(neuralNetworkOptions,"numHiddenNodes",numHiddenNodes);
	addOption(neuralNetworkOptions,"numOutputNodes",numOutputNodes);

	for(int i = 0; i < 10; i++){
		ss.clear();
		ss << "IW_" << i;
		addOption(neuralNetworkOptions,ss.str().c_str(),inputWeights_0[i],inputWeights_1[i],inputWeights_2[i]);
	}
	addOption(neuralNetworkOptions,"LW_0",outputWeights,10);

	addOption(neuralNetworkOptions,"b1",inputBiases,10);
	addOption(neuralNetworkOptions,"b2",outputBiases,1);

	addOption(neuralNetworkOptions,"inMinMaxX_0",inMinMaxX_0[0],inMinMaxX_0[1]);
	addOption(neuralNetworkOptions,"inMinMaxX_1",inMinMaxX_1[0],inMinMaxX_1[1]);
	addOption(neuralNetworkOptions,"inMinMaxX_2",inMinMaxX_2[0],inMinMaxX_2[1]);
	
	addOption(neuralNetworkOptions,"inMinMaxY_0",inMinMaxY[0],inMinMaxY[1]);
	addOption(neuralNetworkOptions,"inMinMaxY_1",inMinMaxY[0],inMinMaxY[1]);
	addOption(neuralNetworkOptions,"inMinMaxY_2",inMinMaxY[0],inMinMaxY[1]);

	addOption(neuralNetworkOptions,"outMinMaxY_0",outMinMaxY[0],outMinMaxY[1]);
	addOption(neuralNetworkOptions,"outMinMaxX_0",outMinMaxX[0],outMinMaxX[1]);

	return &neuralNetworkOptions;
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