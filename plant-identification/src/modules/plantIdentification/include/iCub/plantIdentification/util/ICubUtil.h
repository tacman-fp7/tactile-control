#ifndef __ICUB_PLANTIDENTIFICATION_ICUBUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_ICUBUTIL_H__

#include "iCub/plantIdentification/PlantIdentificationEnums.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>


#include <vector>
#include <fstream>

namespace iCub {
    namespace plantIdentification {

        class ICubUtil {
    
        public:

                static double getForce(std::vector<double>& tactileData,iCub::plantIdentification::ForceCalculationMode forceCalculationMode);

                static void getNNOptionsForErrorPrediction2Fingers(yarp::os::Bottle& bottle);
                static void getNNOptionsForErrorPrediction3Fingers(yarp::os::Bottle& bottle);
                static void rotateFingersData(std::vector<double>& fingersAngles,std::vector<double>& rotatedFingersAngles);

                static bool updateExternalData(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,bool xyzCoordEnabled,int &forceSensorBiasCounter,std::vector<double> &forceSensorBiasPartial);

                static void putDataIntoVector(const double *dataIn,int size,yarp::sig::Vector &dataOut);
                static void putDataIntoMatrix(const double *dataIn,int rows,int columns,yarp::sig::Matrix &dataOut);
                static void putSelectedElementsIntoVector(const yarp::sig::Vector &dataIn,const std::vector<int> &selectedIndexes,yarp::sig::Vector &dataOut);
                static void putSelectedElementsIntoMatrix(const yarp::sig::Matrix &dataIn,const std::vector<int> &selectedRowIndexes,const std::vector<int> &selectedColumnIndexes,yarp::sig::Matrix &dataOut);
                
                static void makeObjectRecognitionBottle(yarp::os::Bottle &objRecBottle,std::string taskId,int objectId,iCub::plantIdentification::ObjectRecognitionTask objRecTask,int extraCode1,int extraCode2,int skipPreviousRepetition,std::string experimentDescription,std::string previousExperimentDescription,double handAperture,double actualHandPosition,double targetHandPosition,double actualGripStrength,double targetGripStrength,std::vector<double> &pwm,iCub::plantIdentification::TaskCommonData *commonData); 

                static void printBottleIntoFile(std::ofstream &file,yarp::os::Bottle &bottle);

                static void normalizeVector(const std::vector<double> &inputVector,std::vector<double> &outputVector);

        private:

                static void processTactileData(iCub::plantIdentification::TaskCommonData *commonData,bool realForceMappingEnabled);

                static void processForceSensorData(iCub::plantIdentification::TaskCommonData *commonData,int &forceSensorBiasCounter,std::vector<double> &forceSensorBiasPartial);

                static double getForceBySimpleSum(std::vector<double>& tactileData);

                static double getForceByWeightedSum(std::vector<double>& tactileData);
                static void getUnitVector(int index,std::vector<double>& unitVector);

                static double getForceByLearntMapping(std::vector<double>& tactileData);

                static void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValue);

                static void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValue1,yarp::os::Value paramValue2);

                static void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValue1,yarp::os::Value paramValue2,yarp::os::Value paramValue3);

                static void addOption(yarp::os::Bottle &bottle,const char *paramName,double paramValueList[],int numElem);

                static void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValueList[],int numElem);

};
    } //namespace plantIdentification
} //namespace iCub

#endif

