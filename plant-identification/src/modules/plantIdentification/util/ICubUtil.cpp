#include "iCub/plantIdentification/util/ICubUtil.h"

#include <cmath>
#include <string>
#include <sstream>

#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>

#define N_HID_NODES_2F 1
#define N_HID_NODES_3F 1

using iCub::plantIdentification::ICubUtil;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;

using yarp::os::Bottle;
using yarp::os::Value;

using std::string;




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
    // where the definition of position is "(MID - TH)/2". Used from the ond of October 2015.
    // if uncommented, N_HID_NODES_2F = 1 has to be assigned
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

bool ICubUtil::updateExternalData(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,bool xyzCoordEnabled,int &forceSensorBiasCounter,std::vector<double> &forceSensorBiasPartial){

    using yarp::sig::Vector;
    
    std::vector<double> fingersSensitivityScale(5,1.0);
    fingersSensitivityScale[4] = commonData->tpDbl(52); // thumb
    fingersSensitivityScale[0] = commonData->tpDbl(53); // index finger
    fingersSensitivityScale[1] = commonData->tpDbl(54); // middle finger
    bool forceSensorReadingEnabled = commonData->tpInt(70) != 0;
    bool realForceMappingEnabled = commonData->tpInt(76) != 0;


    if (realForceMappingEnabled){
            portsUtil->readRealForceData(commonData->realForceData);
    }


    if (!portsUtil->readFingerSkinRawData(commonData->fingerTaxelsRawData,fingersSensitivityScale)){
        return false;
    }

    if (!portsUtil->readFingerSkinCompData(commonData->fingerTaxelsData,fingersSensitivityScale)){
        return false;
    }

    if (!portsUtil->readFingerEncodersRawData(commonData->fingerEncodersRawData)){
        return false;
    }

    controllersUtil->getArmEncodersAngles(commonData->armEncodersAngles);
//    controllersUtil->getArmEncodersAnglesReferences(commonData->armEncodersAnglesReferences);

    processTactileData(commonData,realForceMappingEnabled);
    
    // index finger
    controllersUtil->getEncoderAngle(11,&commonData->proximalJointAngle[0]);
    controllersUtil->getEncoderAngle(12,&commonData->distalJointAngle[0]);


    // middle finger
    controllersUtil->getEncoderAngle(13,&commonData->proximalJointAngle[1]);
    controllersUtil->getEncoderAngle(14,&commonData->distalJointAngle[1]);


    // ring finger
    double enc15;
    controllersUtil->getEncoderAngle(15,&enc15);
    commonData->proximalJointAngle[2] = commonData->distalJointAngle[2] = enc15/2;
        

    // TODO pinky
    commonData->proximalJointAngle[3] = commonData->distalJointAngle[3] = enc15/2;


    // thumb
    controllersUtil->getEncoderAngle(9,&commonData->proximalJointAngle[4]);
    controllersUtil->getEncoderAngle(10,&commonData->distalJointAngle[4]);


    if (xyzCoordEnabled){
        /* calculate the cartesian position of the fingertips respect to the palm */
        Vector thumbJoints;
        Vector indexJoints;
        Vector middleJoints;
        
        // std::vector<double> to yarp::sig::Vector
        Vector armEncoders(commonData->armEncodersAngles.size());
        for (size_t i = 0; i < armEncoders.size(); i++){
            armEncoders[i] = commonData->armEncodersAngles[i];
        }


        commonData->iCubThumb->getChainJoints(armEncoders,thumbJoints);
        commonData->iCubIndexFinger->getChainJoints(armEncoders,indexJoints);
        commonData->iCubMiddleFinger->getChainJoints(armEncoders,middleJoints);
        


        // angle correction. When the fingers are pushing against an object, the angle of the first underactuated joint is always zero
        thumbJoints[thumbJoints.size()-1] = thumbJoints[thumbJoints.size()-1] + thumbJoints[thumbJoints.size()-2];
        thumbJoints[thumbJoints.size()-2] = 0;
        indexJoints[indexJoints.size()-1] = indexJoints[indexJoints.size()-1] + indexJoints[indexJoints.size()-2];
        indexJoints[indexJoints.size()-2] = 0;
        middleJoints[middleJoints.size()-1] = middleJoints[middleJoints.size()-1] + middleJoints[middleJoints.size()-2];
        middleJoints[middleJoints.size()-2] = 0;
        

        yarp::sig::Matrix thumbTipFrame = commonData->iCubThumb->getH(thumbJoints);
        yarp::sig::Matrix indexFingertipFrame = commonData->iCubIndexFinger->getH(indexJoints);
        yarp::sig::Matrix middleFingertipFrame = commonData->iCubMiddleFinger->getH(middleJoints);

        std::cout << indexFingertipFrame.toString();
        //for (size_t i = 0; i < indexJoints.size(); i++){
        //    std::cout << indexJoints[i] << " \t";
        //}
        std::cout << "\n";
        std::cout << "\n";
        std::cout << "\n";



        commonData->thumbXYZ = thumbTipFrame.getCol(3).subVector(0,2);
        commonData->indexXYZ = indexFingertipFrame.getCol(3).subVector(0,2);
        commonData->middleXYZ = middleFingertipFrame.getCol(3).subVector(0,2);
    }

    // reading force sensor data
    if (forceSensorReadingEnabled){

        if (!portsUtil->readForceSensorData(commonData->forceSensorData,commonData->forceSensorBias)){
            return false;
        }
        processForceSensorData(commonData,forceSensorBiasCounter,forceSensorBiasPartial);

    }

    return true;
}

void ICubUtil::processTactileData(TaskCommonData *commonData,bool realForceMappingEnabled){

    double partialOverallFingerPressure;
    
    for(size_t i = 0; i < commonData->fingerTaxelsData.size(); i++){
        
        commonData->overallFingerPressureBySimpleSum[i] = ICubUtil::getForce(commonData->fingerTaxelsData[i],SIMPLE_SUM);
        commonData->overallFingerPressureByWeightedSum[i] = ICubUtil::getForce(commonData->fingerTaxelsData[i],WEIGHTED_SUM);
        
        if (realForceMappingEnabled){

                        //if (i == 2 || i == 3){ // either ring or little finger
                        //        commonData->overallFingerPressure[i] = commonData->overallFingerPressureByWeightedSum[i];
                        //} else {
                        //        commonData->overallFingerPressure[i] = commonData->realForceData[i];
                        //}
                        commonData->overallFingerPressure[i] = commonData->realForceData[i];

        } else {

            commonData->overallFingerPressure[i] = commonData->overallFingerPressureByWeightedSum[i];
            //commonData->overallFingerPressure[i] = commonData->overallFingerPressureBySimpleSum[i];
        }

                // force cannot be negative
                commonData->overallFingerPressure[i] = std::max(0.0,commonData->overallFingerPressure[i]);

        commonData->previousOverallFingerPressures[i][commonData->previousPressuresIndex[i]] = commonData->overallFingerPressure[i];
        commonData->previousPressuresIndex[i] = (commonData->previousPressuresIndex[i] + 1)%commonData->previousOverallFingerPressures[i].size();

        std::vector<double> previousOverallFingerPressuresCopy(commonData->previousOverallFingerPressures[i]);

        gsl_sort(&previousOverallFingerPressuresCopy[0],1,previousOverallFingerPressuresCopy.size());
        commonData->overallFingerPressureMedian[i] = gsl_stats_median_from_sorted_data(&previousOverallFingerPressuresCopy[0],1,previousOverallFingerPressuresCopy.size());

    }

}

void ICubUtil::processForceSensorData(TaskCommonData *commonData,int &forceSensorBiasCounter,std::vector<double> &forceSensorBiasPartial){

    double fX,fY,fZ,tX,tY,tZ;
    double forceSensorDiscountRate = commonData->tpDbl(71);
    int fingerToConsider = commonData->tpInt(72);
    bool forceSensorCalibrationTriggered = commonData->tpInt(73) != 0;
    int numStepsForAverage = commonData->tpInt(74);

    fX = commonData->forceSensorData[0];
    fY = commonData->forceSensorData[1];
    fZ = commonData->forceSensorData[2];
    tX = commonData->forceSensorData[3];
    tY = commonData->forceSensorData[4];
    tZ = commonData->forceSensorData[5];

    commonData->procForceSensorData[0] = -fZ;
    commonData->procForceSensorData[1] = forceSensorDiscountRate*commonData->procForceSensorData[0] + (1-forceSensorDiscountRate)*commonData->procForceSensorData[1];
    commonData->procForceSensorData[2] = tZ;
    commonData->procForceSensorData[3] = forceSensorDiscountRate*commonData->procForceSensorData[2] + (1-forceSensorDiscountRate)*commonData->procForceSensorData[3];
    if (abs(commonData->procForceSensorData[0]) > 0.1){
        commonData->procForceSensorData[4] = commonData->overallFingerPressure[fingerToConsider] / commonData->procForceSensorData[0];
        commonData->procForceSensorData[5] = forceSensorDiscountRate*commonData->procForceSensorData[4] + (1-forceSensorDiscountRate)*commonData->procForceSensorData[5];
    } else {
        commonData->procForceSensorData[4] = commonData->procForceSensorData[5] = 0.0;
    }

    // check if the force calibration has been triggered
    if (forceSensorCalibrationTriggered){
        forceSensorBiasCounter = numStepsForAverage;
        commonData->tempParameters[73] = 0;
                std::cout << "ACTIVATED\n";
    }
    if (forceSensorBiasCounter > 0){
        forceSensorBiasCounter--;
        for(size_t i = 0; i < forceSensorBiasPartial.size(); i++){
            forceSensorBiasPartial[i] += commonData->forceSensorData[i];
        }
                std::cout << "partial " << forceSensorBiasPartial[2] << "\n";
    } else if (forceSensorBiasCounter == 0){
        for(size_t i = 0; i < commonData->forceSensorBias.size(); i++){
            commonData->forceSensorBias[i] = forceSensorBiasPartial[i]/numStepsForAverage;
        }
                std::cout << "DONE!! " << commonData->forceSensorBias[2] << "\n";
        forceSensorBiasCounter = -1;
        forceSensorBiasPartial.resize(6,0);
    }
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
    //double outMinMaxX[2] = {-55.7063,62.0727};

    // values taken from net3F1 in 2-3fingManifoldsNeuralNetworks_pos2.mat, where best position is learnt, and
    // where the definition of position is "((IND + MID)/2 - TH)/2". Used from the end of October 2015.
    // if uncommented, N_HID_NODES_3F = 1 has to be assigned
    int numInputNodes = 3;
    int numHiddenNodes = N_HID_NODES_3F;
    int numOutputNodes = 1;
    double inputWeights_0[N_HID_NODES_3F] = {0.4308};
    double inputWeights_1[N_HID_NODES_3F] = {0.5713};
    double inputWeights_2[N_HID_NODES_3F] = {-0.3086};
    double outputWeights[N_HID_NODES_3F] = {1.6268};
    double inputBiases[N_HID_NODES_3F] = {0.7023};
    double outputBiases[1] = {-0.8966};
    double inMinMaxX_0[2] = {-21.1274,88.1176};
    double inMinMaxX_1[2] = {25.3521,75.3450};
    double inMinMaxX_2[2] = {-1.0962,6.3491};
    double inMinMaxY_0[2] = {-1,1};
    double inMinMaxY_1[2] = {-1,1};
    double inMinMaxY_2[2] = {-1,1};
    double outMinMaxY[2] = {-1,1};
    double outMinMaxX[2] = { -6.4613, 7.5325};

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

    paramBottle.addString(paramName);
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

    paramBottle.addString(paramName);
    paramBottle.addList() = valueBottle;

    bottle.addList() = paramBottle;
}

void ICubUtil::addOption(Bottle &bottle,const char *paramName,Value paramValueList[],int numElem){

    Bottle valueBottle,paramBottle;

    for(int i = 0; i < numElem; i++){
        valueBottle.add(paramValueList[i]);
    }

    paramBottle.addString(paramName);
    paramBottle.addList() = valueBottle;

    bottle.addList() = paramBottle;
}


void ICubUtil::putDataIntoVector(const double *dataIn,int size,yarp::sig::Vector &dataOut){

    dataOut.resize(size);

    for(size_t i = 0; i < size; i++){
        dataOut[i] = dataIn[i];
    }
}

void ICubUtil::putDataIntoMatrix(const double *dataIn,int rows,int columns,yarp::sig::Matrix &dataOut){

    dataOut.resize(rows,columns);

    for(size_t i = 0; i < rows; i++){
        for(size_t j = 0; j < columns; j++){
            dataOut[i][j] = dataIn[columns*i + j];
        }
    }
}


void ICubUtil::putSelectedElementsIntoVector(const yarp::sig::Vector &dataIn,const std::vector<int> &selectedIndexes,yarp::sig::Vector &dataOut){

    dataOut.resize(selectedIndexes.size());

    for(size_t i = 0; i < dataOut.size(); i++){
        dataOut[i] = dataIn[selectedIndexes[i]];
    }
}

void ICubUtil::putSelectedElementsIntoMatrix(const yarp::sig::Matrix &dataIn,const std::vector<int> &selectedRowIndexes,const std::vector<int> &selectedColumnIndexes,yarp::sig::Matrix &dataOut){

    dataOut.resize(selectedRowIndexes.size(),selectedColumnIndexes.size());

    for(size_t i = 0; i < dataOut.rows(); i++){
        for(size_t j = 0; j < dataOut.cols(); j++){
            dataOut[i][j] = dataIn[selectedRowIndexes[i]][selectedColumnIndexes[j]];
        }
    }
}


 void ICubUtil::makeObjectRecognitionBottle(Bottle &objRecBottle,string taskId,int objectId,iCub::plantIdentification::ObjectRecognitionTask objRecTask,int extraCode1,int extraCode2,int skipPreviousRepetition,string experimentDescription,string previousExperimentDescription,double handAperture,double actualHandPosition,double targetHandPosition,double actualGripStrength,double targetGripStrength,std::vector<double> &pwm,iCub::plantIdentification::TaskCommonData *commonData){

     objRecBottle.clear();

    // text data
//    objRecognBottle.addString(experimentDescription);
//    objRecognBottle.addString(previousExperimentDescription);

    // no text data
    // general information (6) (1-6) | (4) (1-4)
    objRecBottle.addInt(objectId);
    objRecBottle.addInt(objRecTask);
    objRecBottle.addInt(extraCode1);
    objRecBottle.addInt(extraCode2);
//    objRecognBottle.addString(taskId);
//    objRecognBottle.addInt(skipPreviousRepetition);

//    double handAperture,double actualHandPosition,double targetHandPosition,double actualGripStrength,double targetGripStrength,

    // hand aperture | 5
    objRecBottle.addDouble(handAperture);

    // actual hand position | 6
    objRecBottle.addDouble(actualHandPosition);
    // target hand position | 7
    objRecBottle.addDouble(targetHandPosition);

    // actual grip strength | 8
    objRecBottle.addDouble(actualGripStrength);
    // target grip strength | 9
    objRecBottle.addDouble(targetGripStrength);

    // logging raw tactile data (24) (7-30) | (24) (10-33)
//    objRecognBottle.addString("midRawTact");
    for(size_t j = 0; j < commonData->fingerTaxelsRawData[1].size(); j++){
        objRecBottle.addDouble(commonData->fingerTaxelsRawData[1][j]);
    }
//    objRecognBottle.addString("thmbRawTact");
    for(size_t j = 0; j < commonData->fingerTaxelsRawData[4].size(); j++){
        objRecBottle.addDouble(commonData->fingerTaxelsRawData[4][j]);
    }
    // logging compensated tactile data (24) (31-54) | (24) (34-57)
//    objRecognBottle.addString("midCompTact");
    for(size_t j = 0; j < commonData->fingerTaxelsData[1].size(); j++){
        objRecBottle.addDouble(commonData->fingerTaxelsData[1][j]);
    }
//    objRecognBottle.addString("thmbCompTact");
    for(size_t j = 0; j < commonData->fingerTaxelsData[4].size(); j++){
        objRecBottle.addDouble(commonData->fingerTaxelsData[4][j]);
    }
    // logging processed tactile data (2) (55-56) | (2) (57-59)
//    objRecognBottle.addString("overallTactVal");
    objRecBottle.addDouble(commonData->overallFingerPressureByWeightedSum[1]);
    objRecBottle.addDouble(commonData->overallFingerPressureByWeightedSum[4]);


    // logging raw encoders (16) (57-72) | (16) (60-75)
//    objRecognBottle.addString("handRawAng");
    for(size_t i = 0; i < commonData->fingerEncodersRawData.size(); i++){
        objRecBottle.addInt(commonData->fingerEncodersRawData[i]);
    }
//    objRecognBottle.addString("armAng");
    // logging encoders (16) (73-88) | (16) (76-91)
    for(size_t i = 0; i < commonData->armEncodersAngles.size(); i++){
        objRecBottle.addDouble(commonData->armEncodersAngles[i]);
    }

    // logging pwm values (2) (89-90) | (2) (92-93)
    for(size_t i = 0; i < pwm.size(); i++){
        objRecBottle.addDouble(pwm[i]);
    }


 }

void ICubUtil::printBottleIntoFile(std::ofstream &file,yarp::os::Bottle &bottle){

    for(int i = 0; i < bottle.size(); i++){

        file << bottle.get(i).toString();
        if (i < bottle.size() - 1){
            file << " ";
        }
    }

    file << "\n";

}

void ICubUtil::normalizeVector(const std::vector<double> &inputVector,std::vector<double> &outputVector){

    double avarage = 0;

    for(int i = 0; i < inputVector.size(); i++){
        avarage += inputVector[i];
    }
    avarage /= inputVector.size();

    std::vector<double> zeroAvarage(inputVector.size());
    for(int i = 0; i < zeroAvarage.size(); i++){
        zeroAvarage[i] = inputVector[i] - avarage;
    }

    double stdDev = 0;
    for(int i = 0; i < zeroAvarage.size(); i++){
        stdDev += zeroAvarage[i]*zeroAvarage[i];
    }
    stdDev = sqrt(stdDev / zeroAvarage.size());

    outputVector.resize(zeroAvarage.size());
    for(int i = 0; i < outputVector.size(); i++){
        outputVector[i] = zeroAvarage[i] / stdDev;
    }
}
