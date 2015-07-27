#ifndef __ICUB_PLANTIDENTIFICATION_TASKDATA_H__
#define __ICUB_PLANTIDENTIFICATION_TASKDATA_H__

#include "iCub/plantIdentification/PlantIdentificationEnums.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <vector>
#include <string>

namespace iCub {
    namespace plantIdentification {

		class TaskCommonData {
		
		public:
			std::vector<std::vector<double> > fingerTaxelsData;
			std::vector<std::vector<double> > previousOverallFingerPressures;
			std::vector<int> previousPressuresIndex;
			std::vector<double> overallFingerPressure;
			std::vector<double> overallFingerPressureMedian;
			std::vector<double> objDetectPressureThresholds;
			std::vector<double> armEncodersAngles;
            std::vector<double> realProximalPwm;
            std::vector<double> realDistalPwm;
            std::vector<double> proximalJointAngle;
            std::vector<double> distalJointAngle;

			std::vector<yarp::os::Value> tempParameters;

			int threadRate;
			int pwmSign;
			int screenLogStride;

			int tpInt(int index);

			double tpDbl(int index);

		};

		class StepTaskData {
		public:
			std::vector<int> jointsList;
			std::vector<int> fingersList;
			int lifespan;
		};

		class ControlTaskData {
		public:
			std::vector<int> jointsList;
			std::vector<int> fingersList;
			std::vector<double> pidKpf;
			std::vector<double> pidKif;
			std::vector<double> pidKpb;
			std::vector<double> pidKib;
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

		class RampTaskData {
		public:
			std::vector<int> jointsList;
			std::vector<int> fingersList;
            double slope;
			double intercept;
			int lifespan;
			int lifespanAfterStabilization;
		};

		struct ApproachTaskData {
		public:
			std::vector<int> jointsList;
			std::vector<int> fingersList;
			std::vector<double> velocitiesList;
			std::vector<double> jointsPwmLimitsList;
			bool jointsPwmLimitsEnabled;
			int lifespan;
		};

        class TaskData {

            private:
                         
                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

				TaskCommonData commonData;

				StepTaskData stepData;
				ControlTaskData controlData;
				RampTaskData rampData;
				ApproachTaskData approachData;
				
				TaskData(yarp::os::ResourceFinder &rf,int threadRate,iCub::plantIdentification::ControllersUtil *controllersUtil);

				std::string getValueDescription(iCub::plantIdentification::RPCSetCmdArgName cmdName);
		
				int getFingerFromJoint(int joint);

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

