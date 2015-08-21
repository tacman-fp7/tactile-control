#include "iCub/plantIdentification/util/ICubUtil.h"

#include <cmath>

using iCub::plantIdentification::ICubUtil;





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

