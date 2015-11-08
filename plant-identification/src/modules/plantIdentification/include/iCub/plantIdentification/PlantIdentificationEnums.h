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
			APPR_LIFESPAN
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

	}
}

#endif
