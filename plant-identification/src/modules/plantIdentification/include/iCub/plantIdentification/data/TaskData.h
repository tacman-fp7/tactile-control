#ifndef __ICUB_PLANTIDENTIFICATION_TASKDATA_H__
#define __ICUB_PLANTIDENTIFICATION_TASKDATA_H__

#include "iCub/plantIdentification/PlantIdentificationEnums.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/data/GMMData.h"

#include <iCub/iKin/iKinFwd.h>
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
            std::vector<double> armEncodersAnglesReferences;
            std::vector<double> armJointsHome;
            std::vector<double> wholeArmJointsDown;
            std::vector<double> handJointsHome;

            std::vector<double> fingerEncodersRawData;
            std::vector<double> realProximalPwm;
            std::vector<double> realDistalPwm;
            std::vector<double> proximalJointAngle;
            std::vector<double> distalJointAngle;
            std::vector<double> fingersSensitivityScale;
            yarp::sig::Vector thumbXYZ;
            yarp::sig::Vector indexXYZ;
            yarp::sig::Vector middleXYZ;
            
            iCub::iKin::iCubFinger* iCubThumb;
            iCub::iKin::iCubFinger* iCubIndexFinger;
            iCub::iKin::iCubFinger* iCubMiddleFinger;

            std::vector<double> forceSensorData;
            std::vector<double> forceSensorBias;
            std::vector<double> procForceSensorData;
            
            std::vector<double> realForceData;

            double currentThAbdJointAngleSetpoint;

            bool requestOpen;

            std::string objRecDataDefaultSuffix;

            // data related to tactile/visual combined classification
            std::vector<double> tactAvgScores;
            std::vector<double> vcAvgScores;
            int classificationState; // 0: not started; 1: running (classify() run but tactile recognition not finished); 2: finished (tactile recognition finished);



            /*  TEMP PARAMETERS USED DURING CONTROL TASKS (EXCEPT THE 15th, USED DURING STEP TASKS)
            *   0: supervisor mode off/on [0/1]
            *   1: supervisor Kp
            *   2: supervisor Ki
            *   3: supervisor Kd
            *   4: if set > 0, reset pid and returns to 0
            *   5: supervisor gains scale factor
            *   6: if != 0, set all voltages to 0 (maximum priority)
            *   7: grip strength
            *   8: initial grasp balance factor
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
            *   47: object recognition: ID kind of task (for example squeezing) // in the new object recognition task is the part of the object
            *	48: object recognition: ID iteration used
            *	49: object recognition: extra info (abouth the object or the task) // in the case of squeezing, 0 refers to the phase before the grip strength is incremented, 1 refers to phase after it is incremented
            *	50: skip previous repetition [0/1]. Like property 17, it is reset to zero at the end of the control task
            *	51: policy learning enabled. no/yes [0/1]
            *	52: thumb sensitivity scale
            *	53: index finger sensitivity scale
            *	54: middle finger sensitivity scale
            *   55: if set != 0, log current gmm data (one shot) and returns to 0
            *   56: best pose estimator [0: neural network / 1: gaussian mixture model / 2: gaussian mixture model without hand position regression]
            *	57: enable XYZ finger coordinates logging
            *	58: enable tactile data logging
            *	59: enable <not defined yet> logging
            *   60: wave action. wave type [0:sine / 1:square]
            *   61: wave action. amplitude
            *   62: wave action. period
            *   63: wave action. arm joint number
            *   64: wave action. action duration (sec)
            *   65: gmm joints minimum jerk tracking mode off/on [0/1]
            *   66: gmm joints minimum jerk trajectory reference time (90% of steady-state value in this time, transient estinguished after 150%) (restart minJerk to take effect)
            *	67: hand freeze enabled [0/1]. If enabled, as soon as the hand grasps the object, the hand doesn't move
            *	68: hand freeze automation enabled [0/1]. if enabled, it automatically set property 67 (that we can ignore) in order to freeze the hand the first x seconds, where x is specified by property 69
            *	69: hand freeze duration (sec). Check property 68 for the description
            *	70: force sensors: reading enabled. no/yes [0/1]
            *	71: force sensors: discount rate (in [0 1]), it regulates the moving average of all the printed values
            *	72: force sensors: finger to consider
            *	73: force sensors: if set != 0, calibration starts and it returns to 0.
            *	74: force sensors: number of steps for the calibration (in order to calculate the average)
            *	75: thumb abduction joint offset, applied when GMM regression is used
            *	76: real forces enabled (tactile data is ignored and only real force mapping is used). no/yes [0/1]
            *	77: hysteresis: force threshold enabled (when the force reference goes below a given threshold, the force reference becomes the threshold itself). no/yes [0/1]
            *	78: hysteresis: thumb threshold
            *	79: hysteresis: index finger threshold
            *	80: hysteresis: middle finger threshold
            *	81: hysteresis: threshold tollerance (percentage of tollerance, so if 10 is specified, the threshold will be 110% of the one specified in 78-80)
            *	82: hysteresis: automatic threshold storing enabled (threshold are stored after the "open" command). no/yes [0/1]
            *	83: if set == 1, when the current gmm data is logged (through #55) for the thumb abduction joint angle is used the setpoint and not the actual value
            *	84: object recognition: time for grasp stabilization
            *	85: object recognition: time from grasp stabilization to grip strength increment (to log something before squeezing)
            *	86: object recognition: squeezing time
            *	87: object recognition: proximals closure time
            *	88: object recognition: grip strength used for squeezing
            *	89: object recognition: pwm to request at the index finger proximal joint
            *	90: object recognition: pwm to request at the ring/pinky fingers
            *   91: object recognition: if set > 0, set handClosurePerformed to true and returns to 0 (it's an integer)
            *   92: object recognition: distal closure time
            *   93: object recognition: pwm to request at the index finger distal joint
            *   94: object recognition: disable object recognition logging. no/yes [0/1]
            *   95: object recognition: test mode status. 0: off 1: on (one shot) 2: on (refinement using avarage) 3: on (refinement using maxmax)
            *   96: neural network offeset
            *   97: combining classifiers: combining mode
            *   98: combining classifiers: combSumScale
            *   99: combining classifiers: combMaxMaxScale
            *   100: combining classifiers: objectTested
            *   101: combining classifiers: objectTestIteration
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
            iCub::plantIdentification::GMMData* gmmDataStandard;
            iCub::plantIdentification::GMMData* gmmDataObjectInclinedThumbUp;
            iCub::plantIdentification::GMMData* gmmDataObjectInclinedThumbDown;
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

