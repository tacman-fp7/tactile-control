#ifndef __ICUB_PLANTIDENTIFICATION_TASKDATA_H__
#define __ICUB_PLANTIDENTIFICATION_TASKDATA_H__

#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <vector>

namespace iCub {
    namespace plantIdentification {

		struct TaskCommonData {

			std::vector<std::vector<double>> fingerTaxelsData;
			std::vector<std::vector<double>> previousOverallFingerPressures;
			std::vector<int> previousPressuresIndex;
			std::vector<double> overallFingerPressure;
			std::vector<double> overallFingerPressureMedian;
//			double realProximalPwm;
//			double realDistalPwm;
//			double proximalJointAngle;
//			double distalJointAngle;

			int threadRate;
			int pwmSign;
			int screenLogStride;
		};
		typedef struct TaskCommonData TaskCommonData;

		struct StepTaskData {

			std::vector<int> jointsList;
			std::vector<int> fingersList;
			int lifespan;
		};
		typedef struct StepTaskData StepTaskData;

		struct ControlTaskData {

			std::vector<int> jointsList;
			std::vector<int> fingersList;
			double pidKpf;
			double pidKif;
			double pidKdf;
			double pidKpb;
			double pidKib;
			double pidKdb;
			double pidN;
			double pidMinSatLim;
			double pidMaxSatLim;
			double pidWp;
			double pidWi;
			double pidWd;
			double pidWindUpCoeff;
			iCub::plantIdentification::ControlTaskOpMode controlMode;
			bool pidResetEnabled;
			int lifespan;
		};
		typedef struct ControlTaskData ControlTaskData;

		struct RampTaskData {

			std::vector<int> jointsList;
			std::vector<int> fingersList;
            double slope;
			double intercept;
			int lifespan;
			int lifespanAfterStabilization;
		};
		typedef struct RampTaskData RampTaskData;

        class TaskData {

            private:
                         
                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

				TaskCommonData commonData;

				StepTaskData stepData;
				ControlTaskData controlData;
				RampTaskData rampData;
				
				TaskData(yarp::os::ResourceFinder &rf,int threadRate);
		
			private:

				int getFingerFromJoint(int joint);

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

