#ifndef __ICUB_PLANTIDENTIFICATION_EVENTSTHREAD_H__
#define __ICUB_PLANTIDENTIFICATION_EVENTSTHREAD_H__

#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"
#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/RateThread.h>

#include <vector>
#include <string>

namespace iCub {
    namespace plantIdentification {
        
		class EventsThread : public yarp::os::RateThread {
           

			private:

				/* ****** Thread attributes                             ****** */
                int period;

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
				bool fpEnabled;

				/* ******* Controllers utility                          ******* */
                iCub::plantIdentification::ControllersUtil *controllersUtil;

				/* ****** Ports utility                                 ****** */
				iCub::plantIdentification::PortsUtil *portsUtil;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

			
			public:

				EventsThread(const int period,iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData);
				virtual ~EventsThread();

                virtual bool threadInit();
                virtual void run();
                virtual void threadRelease();

				void checkEvents();

				bool eventTriggered(iCub::plantIdentification::EventToTrigger eventToTrigger,int index);
			
			private:

				bool eventFPTriggered(int whichFinger);

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

