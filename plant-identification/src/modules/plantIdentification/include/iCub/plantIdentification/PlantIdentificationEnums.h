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
			QUIT
		};

		enum RPCSetCmdArgName {

			PWM_SIGN,
			OBJ_DETECT_PRESS_THRESHOLDS,

			STEP_JOINTS_LIST,
			STEP_LIFESPAN,
			
			CTRL_JOINTS_LIST,
			CTRL_PID_KPF,
			CTRL_PID_KIF,
			CTRL_PID_KPB,
			CTRL_PID_KIB,
			CTRL_OP_MODE,
			CTRL_PID_RESET_ENABLED,
			CTRL_LIFESPAN,

			RAMP_JOINTS_LIST,
			RAMP_SLOPE,
			RAMP_INTERCEPT,
			RAMP_LIFESPAN,
			RAMP_LIFESPAN_AFTER_STAB
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
			APPROACH_AND_CONTROL = 3
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

	}
}

#endif
