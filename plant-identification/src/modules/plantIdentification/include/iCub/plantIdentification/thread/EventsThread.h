#ifndef __ICUB_PLANTIDENTIFICATION_EVENTSTHREAD_H__
#define __ICUB_PLANTIDENTIFICATION_EVENTSTHREAD_H__

#include "iCub/plantIdentification/action/WaveAction.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"
#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RateThread.h>

#include <vector>
#include <string>

namespace iCub {
    namespace plantIdentification {
        
		class EventsThread : public yarp::os::RateThread {
           

			private:

				/* ****** Thread attributes                             ****** */
                int period;

				unsigned long counter;

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

				iCub::plantIdentification::WaveAction waveAction;

				/* ******* Controllers utility                          ******* */
                iCub::plantIdentification::ControllersUtil *controllersUtil;

				/* ****** Ports utility                                 ****** */
				iCub::plantIdentification::PortsUtil *portsUtil;

				bool xyzCoordEnabled;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

			
			public:

				EventsThread(yarp::os::ResourceFinder &rf,const int period,iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData);
				virtual ~EventsThread();

                virtual bool threadInit();
                virtual void run();
                virtual void threadRelease();

				bool eventTriggered(iCub::plantIdentification::EventToTrigger eventToTrigger,int index);

                void setWaveAction(double actionDuration,double joint,double period,double amplitude,iCub::plantIdentification::Wave waveType);
			
			private:

				void checkEvents();

				void logData();

				void executeWaveAction();

				bool eventFPTriggered(int whichFinger);

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

