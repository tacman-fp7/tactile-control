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
			std::vector<std::vector<double> > fingerTaxelsRawData;
			std::vector<std::vector<double> > previousOverallFingerPressures;
			std::vector<int> previousPressuresIndex;
			std::vector<double> overallFingerPressure;
			std::vector<double> overallFingerPressureBySimpleSum;
			std::vector<double> overallFingerPressureByWeightedSum;
			std::vector<double> overallFingerPressureMedian;
			std::vector<double> objDetectPressureThresholds;
			std::vector<double> armEncodersAngles;
			std::vector<double> fingerEncodersRawData;
            std::vector<double> realProximalPwm;
            std::vector<double> realDistalPwm;
            std::vector<double> proximalJointAngle;
            std::vector<double> distalJointAngle;
            
            /*  TEMP PARAMETERS USED DURING CONTROL TASKS (EXCEPT THE 15th, USED DURING STEP TASKS)
            *   0: supervisor mode off/on [0/1]
            *   1: supervisor Kp
            *   2: supervisor Ki
            *   3: supervisor Kd
            *   4: if set > 0, reset pid and returns to 0
            *   5: supervisor gains scale factor
            *   6: if != 0, set all voltages to 0 (maximum priority)
            *   7: grip strength
            *   8: starting grasp balance factor
            *   9: index/middle fingers balance factor
            *   10: if set > 0, scales low level PID gains and returns to 0
            *   11: square wave mode off/on [0/1] (to test LOW level PID)
            *   12: wave amplitude
            *   13: wave half period
            *   14: prints on screen extra data about low level pid tuning
            *   15: pinky mode on/off (control grasp balance with the pinky, settable during step tasks!)
			*   16: string describing the next experiment. it is logged in the control log
			*   17: string describing the previous experiment. it is logged in the control log
			*   18: supervisor hand pose square wave / sinusoid generator mode [0:off 1:square wave 2:sinusoid] (to test HIGH level PID)
            *   19: wave amplitude
            *   20: wave half period
			*   21: wave mean
			*   22: supervisor grip strength square wave / sinusoid generator mode [0:off 1:square wave 2:sinusoid] (to test HIGH level PID)
            *   23: wave amplitude
            *   24: wave half period
			*   25: wave mean
			*   26: supervisor tracker mode off/on [0/1]
			*   27: supervisor tracker velocity (degrees/second)
			*   28: supervisor tracker acceleration (degrees/(second*second))
			*   29: minimum jerk tracking mode off/on [0/1]
			*   30: minimum jerk trajectory reference time (90% of steady-state value in this time, transient estinguished after 150%) (restart minJerk to take effect)
			*   31: pinky control mode on/off (control grasp balance with the pinky, but working with control mode! it affects the balance factor)
			*	32: approach task control mode [0:velocity / 1:openloop]
			*	33: approach task stop condition [0:none / 1:tactile / 2:position / 3:both]
			*	34: approach threshold scale factor (valid for stop condition by tactile data)
			*	35: approach seconds for avarage (valid for stop condition by tactile data)
			*	36: approach window size (valid for stop condition by position)
			*	37: approach final check threshold (valid for stop condition by position)
			*	38: approach seconds for movement timeout (valid for stop condition by position)
			*	39: approach stop-fingers mode off/on [0/1] (fingers are stopped when in contact with the object)
			*   40: grasp task: disable PID integral gain from joint 8. no/yes [0/1]
			*   41: finger push event: time window in seconds
			*   42: finger push event: pressure increase threshold
			*   43: finger push event: min time (in seconds) between event triggers
			*   44: head enabled. no/yes [0/1]
			*	45: object recognition enabled. no/yes [0/1]
            *	46: object recognition: ID object used (for whatever task)
			*   47: object recognition: ID kind of task (for example squeezing)
			*	48: object recognition: extra info (abouth the object or the task)
			*	49: object recognition: extra info (abouth the object or the task)
			*	50: skip previous repetition [0/1]. Like property 17, it is reset to zero at the end of the control task
			*	51: policy learning enabled. no/yes [0/1]
			*   
			*
			*   Note: double values have to contain a dot, while strings have to start with '#'
			*
            */
			std::vector<yarp::os::Value> tempParameters;

			int taskThreadPeriod; // milliseconds between two consecutive task thread calls
			int eventsThreadPeriod; // milliseconds between two consecutive events thread calls
			int pwmSign;
			int screenLogStride;

			int tpInt(int index);

			double tpDbl(int index);

			std::string tpStr(int index);

			// time in seconds
			int getNumOfThreadCallsFromTime(iCub::plantIdentification::MyThread whichThread,double time);
			double getTimeFromNumOfThreadCalls(iCub::plantIdentification::MyThread whichThread,int numOfTaskThreadCalls);

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
			std::vector<double> pwmList;
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
				
				TaskData(yarp::os::ResourceFinder &rf,iCub::plantIdentification::ControllersUtil *controllersUtil);

				std::string getValueDescription(iCub::plantIdentification::RPCSetCmdArgName cmdName);
		
				int getFingerFromJoint(int joint);

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

