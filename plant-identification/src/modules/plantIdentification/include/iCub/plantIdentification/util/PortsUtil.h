#ifndef __ICUB_PLANTIDENTIFICATION_PORTSUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_PORTSUTIL_H__

#include "iCub/plantIdentification/data/LogData.h"
#include "iCub/plantIdentification/data/TaskData.h"

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
                yarp::os::BufferedPort<yarp::sig::Vector> portSkinCompIn;
				yarp::os::BufferedPort<yarp::os::Bottle> portLogDataOut;
				yarp::os::BufferedPort<yarp::os::Bottle> portInfoDataOut;
				yarp::os::BufferedPort<yarp::os::Bottle> portControlDataOut;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

			public:

				PortsUtil();

				bool init(yarp::os::ResourceFinder &rf);

				bool sendLogData(iCub::plantIdentification::LogData &logData);

				bool sendInfoData(iCub::plantIdentification::TaskCommonData *commonData);

				bool sendControlData(std::string taskId,std::string experimentDescription,std::string previousExperimentDescription,double s,double u,double error,double svCurrentPosition,double svTarget,double svKp,double svKi,double svKd,double thumbEnc,double indexEnc,double middleEnc,double enc8,std::vector<double> &pressureTarget,std::vector<double> &actualPressure,std::vector<double> &pwm,std::vector<int> &fingersList);

				bool readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData);

				bool release();
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

