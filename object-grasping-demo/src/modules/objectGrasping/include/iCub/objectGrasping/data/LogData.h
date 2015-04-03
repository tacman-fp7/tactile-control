#ifndef __ICUB_OBJECTGRASPING_LOGDATA_H__
#define __ICUB_OBJECTGRASPING_LOGDATA_H__

#include <string>
#include <vector>

#include <yarp/os/Bottle.h>

namespace iCub {
    namespace objectGrasping {

        class LogData {

            private:

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

            public:

				/* ******* Module attributes.               ******* */
				std::string taskId;
				int taskType;
				int taskOperationMode;
				double targetValue;
				std::vector<double> fingerTaxelValues;
				double overallFingerPressure;
				double overallFingerPressureMedian;
				double pwm;
				double realProximalPwm;
				double realDistalPwm;
				double proximalJointAngle;
				double distalJointAngle;
				double pidKpf;
				double pidKif;
				double pidKdf;
				double pidKpb;
				double pidKib;
				double pidKdb;
				double error;
				double errorIntegral;
                

				LogData();
                
				void toBottle(yarp::os::Bottle &bottle);
        };
    } //namespace objectGrasping
} //namespace iCub

#endif
