#ifndef __PLANTIDENTIFICATION_ENUMS_H__
#define __PLANTIDENTIFICATION_ENUMS_H__

namespace iCub {
    namespace plantIdentification {
     
        enum RPCMainCmdName {

            HELP,
            SET,
            TASK,
            VIEW,
            START,
            STOP,
            OPEN,
            ARM,
            GRASP,
            CLASSIFY,
            WAVE,
            ML,
            QUIT
        };

        enum RPCSetCmdArgName {

            PWM_SIGN,
            OBJ_DETECT_PRESS_THRESHOLDS,
            TEMPORARY_PARAM,

            STEP_JOINTS_LIST,
            STEP_LIFESPAN,
            
            CTRL_JOINTS_LIST,
            CTRL_PID_KPF,
            CTRL_PID_KIF,
            CTRL_PID_KPB,
            CTRL_PID_KIB,
            CTRL_OP_MODE,
            CTRL_PID_RESET_ENABLED,
            CTRL_TARGET_REAL_TIME,
            CTRL_LIFESPAN,

            RAMP_JOINTS_LIST,
            RAMP_SLOPE,
            RAMP_INTERCEPT,
            RAMP_LIFESPAN,
            RAMP_LIFESPAN_AFTER_STAB,

            APPR_JOINTS_LIST,
            APPR_JOINTS_VELOCITIES,
            APPR_JOINTS_PWM_LIMITS,
            APPR_JOINTS_PWM_LIMITS_ENABLED,
            APPR_LIFESPAN,

            ALIAS_GRIP_STRENGTH,
            ALIAS_OBJECT_POSITION,
            ALIAS_MINIMUM_FORCE
        };
    
        enum RPCTaskCmdArgName {

            ADD,
            EMPTY,
            POP
        };

        enum TaskName {

            STEP = 0,
            CONTROL = 1,
            RAMP = 2,
            APPROACH_AND_CONTROL = 3,
            APPROACH = 4,
            NONE
        };

        enum RPCViewCmdArgName {

            SETTINGS,
            TASKS
        };

        enum RPCMlCmdArgName {

            VIEW_DATA,
            MODE,
            TRAIN,
            TEST,
            SAVE_MODEL,
            LOAD_MODEL,
            LOAD_TRAINING_SET,
            LOAD_TEST_SET,
            LOAD_OBJECT_NAMES,
            LOAD_TRAINING_SET_AND_OBJECT_NAMES,
            SAVE_TRAINING_SET,
            SAVE_OBJECT_NAMES,
            SAVE_TRAINING_SET_AND_OBJECT_NAMES,
            LEARN_NEW_OBJECT,
            REFINE_NEW_OBJECT,
            DISCARD_LAST_FEATURES,
            PROCESS_COLLECTED_DATA,
            GET_READY,
            RESET,
            GET_TACTILE_CLASSIFIER_OUTPUT,
            GET_VISUAL_CLASSIFIER_OUTPUT
        };

        enum FingerJoint {

            PROXIMAL,
            DISTAL
        };

        enum RampTaskState {

            DECREASING,
            STEADY
        };

        enum ControlTaskOpMode {

            GAINS_SET_POS_ERR = 0,
            GAINS_SET_NEG_ERR = 1,
            BOTH_GAINS_SETS = 2
        };

        enum ForceCalculationMode {

            SIMPLE_SUM = 0,
            WEIGHTED_SUM = 1,
            MAPPING = 2
        };

        enum EventToTrigger {

            FINGERTIP_PUSHED
        };

        enum MyThread {

            TASK_THREAD,
            EVENTS_THREAD
        };

        enum ObjectRecognitionTask {

            SQUEEZING = 0
        };

        enum Wave {

            SINE = 0,
            SQUARE = 1
        };

        enum GMM {

            STANDARD = 0,
            INCLINED_THUMB_DOWN = 1,
            INCLINED_THUMB_UP = 2
        };

    }
}

#endif
