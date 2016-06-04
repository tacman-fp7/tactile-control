#ifndef __ICUB_PLANTIDENTIFICATION_WAVEACTION_H__
#define __ICUB_PLANTIDENTIFICATION_WAVEACTION_H__

#include "iCub/plantIdentification/PlantIdentificationEnums.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"

#include <string>
#include <vector>

#include <yarp/os/Bottle.h>

namespace iCub {
    namespace plantIdentification {

        class WaveAction {

            private:

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

				/* ******* Module attributes.               ******* */
				double actionDuration;
				int counter;
				bool waveMeanInitialized;
				bool enabled;
				double joint;
				double waveMean;
				double wavePeriod;
				double waveAmplitude;
                double threadPeriod; // milliseconds
				iCub::plantIdentification::Wave waveType;
                
                iCub::plantIdentification::ControllersUtil *controllersUtil;

				void release();

            public:

				WaveAction();
					
				void init(iCub::plantIdentification::ControllersUtil *controllersUtil,double actionDuration,double joint,double period,double amplitude,iCub::plantIdentification::Wave waveType,int threadPeriod);
                
				bool isEnabled();

				void executeNextStep();

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

