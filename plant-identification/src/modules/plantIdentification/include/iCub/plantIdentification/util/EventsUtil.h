#ifndef __ICUB_PLANTIDENTIFICATION_EVENTSUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_EVENTSUTIL_H__

#include "iCub/plantIdentification/data/TaskData.h"

#include <vector>
#include <string>

namespace iCub {
    namespace plantIdentification {
        
		class EventsUtil {
            
			private:

				iCub::plantIdentification::TaskCommonData *commonData;

				/* ******* Events data                      ******* */
				// event 'Finger Pushed' (FP)
				int fpWindowSize;
				int fpWindowIndex;
				double fpFinalCheckThreshold;
				std::vector<int> fpTimeFromLastEventReset;
				int fpMinTimeBetweenActivations;
				std::vector<std::vector<double> > fpPressureMemory;
				std::vector<bool> fpEventTriggered;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

			public:

				EventsUtil(iCub::plantIdentification::TaskCommonData *commonData);

				bool init();

				void checkEvents();

				bool eventFPTriggered(int whichFinger);


        };
    } //namespace plantIdentification
} //namespace iCub

#endif

