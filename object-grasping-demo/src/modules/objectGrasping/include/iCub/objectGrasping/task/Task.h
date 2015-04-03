#ifndef __ICUB_OBJECTGRASPING_TASK_H__
#define __ICUB_OBJECTGRASPING_TASK_H__

#include "iCub/objectGrasping/data/TaskData.h"
#include "iCub/objectGrasping/data/LogData.h"
#include "iCub/objectGrasping/util/ControllersUtil.h"
#include "iCub/objectGrasping/util/PortsUtil.h"
#include "iCub/objectGrasping/ObjectGraspingEnums.h"

#include <string>

namespace iCub {
    namespace objectGrasping {

        class Task {

            private:
				
				bool isFirstCall;

			protected:

				iCub::objectGrasping::TaskName taskName;
				iCub::objectGrasping::ControllersUtil *controllersUtil;
				iCub::objectGrasping::PortsUtil *portsUtil;

				iCub::objectGrasping::TaskCommonData *commonData;

				std::vector<int> jointsList;
				std::vector<int> fingersList;



				int callsNumber;
				int maxCallsNumber;
				std::vector<double> pwmToUse;

				std::string taskId;

				std::string optionalLogString;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

            public:
                
				Task(iCub::objectGrasping::ControllersUtil *controllersUtil,iCub::objectGrasping::PortsUtil *portsUtil,iCub::objectGrasping::TaskCommonData *commonData,int maxCallsNumber,std::vector<int> &jointsList,std::vector<int> &fingersList);
            
				iCub::objectGrasping::TaskName getTaskName(){ return taskName; }

				bool manage(bool keepActive);

			protected:

				void addCommonLogData(iCub::objectGrasping::LogData &logData);

				void processTactileData();

			private:
			
				void createTaskId();

				virtual void init(){};

				virtual bool loadICubData();

				virtual void calculatePwm() = 0;

				virtual void buildLogData(iCub::objectGrasping::LogData &logData);

				virtual void printScreenLog();

				virtual void saveProgress();

				virtual bool taskIsOver();

				virtual void release(){};

        };
    } //namespace objectGrasping
} //namespace iCub

#endif

