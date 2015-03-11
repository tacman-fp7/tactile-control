#ifndef __ICUB_PLANTIDENTIFICATION_TASK_H__
#define __ICUB_PLANTIDENTIFICATION_TASK_H__

#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/data/LogData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"
#include "iCub/plantIdentification/PlantIdentificationEnums.h"

#include <string>

namespace iCub {
    namespace plantIdentification {

        class Task {

            private:
				
				bool isFirstCall;

			protected:

				iCub::plantIdentification::TaskName taskName;
				iCub::plantIdentification::ControllersUtil *controllersUtil;
				iCub::plantIdentification::PortsUtil *portsUtil;

				iCub::plantIdentification::TaskCommonData *commonData;

				int callsNumber;
				int maxCallsNumber;
				double pwmToUse;

				std::string taskId;

				std::string optionalLogString;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

            public:
                
				Task(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,int maxCallsNumber);
            
				iCub::plantIdentification::TaskName getTaskName(){ return taskName; }

				bool manage(bool keepActive);

			protected:

				void addCommonLogData(iCub::plantIdentification::LogData &logData);

				void processTactileData();

			private:
			
				void createTaskId();

				virtual void init(){};

				virtual bool loadICubData();

				virtual void calculatePwm() = 0;

				virtual void buildLogData(iCub::plantIdentification::LogData &logData);

				virtual void printScreenLog();

				virtual void saveProgress();

				virtual bool taskIsOver();

				virtual void release(){};

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

