#ifndef __ICUB_OBJECTGRASPING_PORTSUTIL_H__
#define __ICUB_OBJECTGRASPING_PORTSUTIL_H__

#include "iCub/objectGrasping/data/LogData.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

namespace iCub {
    namespace objectGrasping {
        
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

				bool sendLogData(iCub::objectGrasping::LogData &logData);

				bool readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData);

				bool release();
        };
    } //namespace objectGrasping
} //namespace iCub

#endif

