#ifndef __PLANTIDENTIFICATION_ENUMS_H__
#define __PLANTIDENTIFICATION_ENUMS_H__

namespace iCub {
    namespace plantIdentification {
     
		enum SetParamName {

			FINGER_TO_MOVE,
			JOINT_TO_MOVE,
			PWM_SIGN,

			STEP_LIFESPAN,
			
			CTRL_PID_KPF,
			CTRL_PID_KIF,
			CTRL_PID_KDF,
			CTRL_PID_KPB,
			CTRL_PID_KIB,
			CTRL_PID_KDB,
			CTRL_OP_MODE,
			CTRL_LIFESPAN,

			DB_SLOPE,
			DB_INTERCEPT,
			DB_LIFESPAN,
			DB_LIFESPAN_AFTER_STAB
		};
    
		enum TaskParamName {

			STEP,
			CONTROL,
			RAMP,

			EMPTY,
			POP
		};

		enum ViewParamName {

			SETTINGS,
			TASKS
		};

	}
}

#endif
