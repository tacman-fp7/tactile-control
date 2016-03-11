#ifndef __ICUB_PLANTIDENTIFICATION_LOGDATA_H__
#define __ICUB_PLANTIDENTIFICATION_LOGDATA_H__

#include <string>
#include <vector>

#include <yarp/os/Bottle.h>

namespace iCub {
    namespace plantIdentification {

        class LogData {

            private:

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

            public:

				/* ******* Module attributes.               ******* */
				std::string taskId;
				int taskType;
				int taskOperationMode;
                int fingersNum;
                std::vector<double> targetValue;
                std::vector<std::vector<double> > fingerTaxelValues;
                std::vector<double> overallFingerPressure;
                std::vector<double> overallFingerPressureMedian;
                std::vector<double> pwm;
                std::vector<double> proximalJointAngle;
                std::vector<double> distalJointAngle;
                std::vector<double> pidKpf;
                std::vector<double> pidKif;
                std::vector<double> pidKdf;
                std::vector<double> pidKpb;
                std::vector<double> pidKib;
                std::vector<double> pidKdb;
                std::vector<double> error;
                std::vector<double> errorIntegral;
                

                LogData(int fingersNum);
                
				void toBottle(yarp::os::Bottle &bottle);
        };
    } //namespace plantIdentification
} //namespace iCub

#endif

