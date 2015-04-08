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

				PortsUtil();

				bool init(yarp::os::ResourceFinder &rf);

				bool sendLogData(iCub::plantIdentification::LogData &logData);

				bool readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData);

				bool release();
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

