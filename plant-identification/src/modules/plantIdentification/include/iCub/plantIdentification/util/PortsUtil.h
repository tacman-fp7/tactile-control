#ifndef __ICUB_PLANTIDENTIFICATION_PORTSUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_PORTSUTIL_H__

#include "iCub/plantIdentification/data/LogData.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

namespace iCub {
    namespace plantIdentification {
        
		class PortsUtil {
            
			private:
				
                /* ******* Ports			                ******* */
                yarp::os::BufferedPort<yarp::sig::Vector> portSkinCompIn;
				yarp::os::BufferedPort<yarp::os::Bottle> portLogDataOut;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

			public:

				PortsUtil(yarp::os::ResourceFinder &rf);

				void sendLogData(iCub::plantIdentification::LogData &logData);

				void readFingerSkinCompData(int finger,std::vector<double> &fingerTaxelsData);

				void release();
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

