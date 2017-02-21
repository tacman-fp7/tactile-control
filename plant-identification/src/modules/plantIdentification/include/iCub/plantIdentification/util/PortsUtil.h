#ifndef __ICUB_PLANTIDENTIFICATION_PORTSUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_PORTSUTIL_H__

#include "iCub/plantIdentification/data/LogData.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

#include <vector>
#include <string>

namespace iCub {
    namespace plantIdentification {
        
        class PortsUtil {
            
            private:
                
                /* ******* Ports			                ******* */
                yarp::os::BufferedPort<yarp::sig::Vector> portSkinRawIn;
                yarp::os::BufferedPort<yarp::sig::Vector> portSkinCompIn;
                yarp::os::BufferedPort<yarp::sig::Vector> portHandEncodersRawIn;
                yarp::os::BufferedPort<yarp::sig::Vector> portPolicyActionsIn;
                yarp::os::BufferedPort<yarp::sig::Vector> portForceSensorIn;
                yarp::os::BufferedPort<yarp::sig::Vector> portThumbRealForceIn;
                yarp::os::BufferedPort<yarp::sig::Vector> portIndexFingerRealForceIn;
                yarp::os::BufferedPort<yarp::sig::Vector> portMiddleFingerRealForceIn;

                yarp::os::BufferedPort<yarp::os::Bottle> portLogDataOut;
                yarp::os::BufferedPort<yarp::os::Bottle> portInfoDataOut;
                yarp::os::BufferedPort<yarp::os::Bottle> portControlDataOut;
                yarp::os::BufferedPort<yarp::os::Bottle> portGMMDataOut;
                yarp::os::BufferedPort<yarp::os::Bottle> portGMMRegressionDataOut;
                yarp::os::BufferedPort<yarp::os::Bottle> portObjRecognDataOut;
                yarp::os::BufferedPort<yarp::os::Bottle> portGripStrengthDataOut;
                yarp::os::BufferedPort<yarp::os::Bottle> portSpeakerOut;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

            public:

                PortsUtil();

                bool init(yarp::os::ResourceFinder &rf);

                bool sendLogData(iCub::plantIdentification::LogData &logData);

                bool sendInfoData(iCub::plantIdentification::TaskCommonData *commonData);

                bool sendControlData(std::string taskId,std::string experimentDescription,std::string previousExperimentDescription,double targetGripStrength,double actualGripStrength,double u,double error,double svCurrentPosition,double actualCurrentTargetPose,double finalTargetPose,double estimatedFinalPose,double svKp,double svKi,double svKd,double thumbEnc,double indexEnc,double middleEnc,double enc8,std::vector<double> &pressureTarget,std::vector<double> &actualPressure,std::vector<double> &pwm,std::vector<int> &fingersList);

                bool sendObjectRecognitionData(std::string taskId,int objectId,iCub::plantIdentification::ObjectRecognitionTask objRecTask,int extraCode1,int extraCode2,int skipPreviousRepetition,std::string experimentDescription,std::string previousExperimentDescription,std::vector<double> &pwm,iCub::plantIdentification::TaskCommonData *commonData);

                bool sendStringToSpeaker(std::string objectLabel);

                bool sendGMMData(double gripStrength, double indexMiddleFingerPressureBalance, iCub::plantIdentification::TaskCommonData *commonData);

                bool sendGMMRegressionData(double handAperture,double indMidPosDiff,double targetHandPosition,double actualHandPosition,double filteredHandPosition,double targetThumbDistalJoint,double filteredThumbDistalJoint,double targetIndexDistalJoint,double filteredIndexDistalJoint,double targetMiddleDistalJoint,double filteredMiddleDistalJoint,double targetThumbAbductionJoint,double filteredThumbAbductionJoint, double targetIndMidForceBalance, double actualIndMidForceBalance,double targetGripStrength,double actualGripStrength,iCub::plantIdentification::TaskCommonData *commonData);

                bool sendGripStrengthData(std::string experimentDescription,std::string previousExperimentDescription,double targetGripStrength,iCub::plantIdentification::TaskCommonData *commonData);

                bool readFingerSkinRawData(std::vector<std::vector<double> > &fingerTaxelsRawData,std::vector<double> &fingersSensitivityScale);

                bool readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData,std::vector<double> &fingersSensitivityScale);

                bool readFingerEncodersRawData(std::vector<double> &fingerEncodersRawData);

                bool readPolicyActionsData(std::vector<double> &policyActionsData);

                bool readForceSensorData(std::vector<double> &forceSensorData,std::vector<double> &forceSensorBias);

                bool readRealForceData(std::vector<double> &realForceData);

                bool release();
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

