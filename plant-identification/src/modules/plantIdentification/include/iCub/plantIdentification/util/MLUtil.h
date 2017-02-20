#ifndef __ICUB_PLANTIDENTIFICATION_MLUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_MLUTIL_H__

#include "iCub/plantIdentification/util/PortsUtil.h"

#include <gurls++/kernelrlswrapper.h>

#include <yarp/os/ResourceFinder.h>

#include <string>
#include <map>

namespace iCub {
    namespace plantIdentification {

        class MLUtil {

            private:

                gurls::KernelRLSWrapper<double> *wrapper;
                std::string modelFileName;
                std::string objectNamesFileName;
                std::string trainingSetXFileName;
                std::string trainingSetYFileName;
                std::string testSetXFileName;
                std::string testSetYFileName;
                gurls::gMat2D<double> xTr,yTr,xTe,yTe;
                std::vector<std::vector<double> > outputsOverTime;
                std::map<int,std::string> objectsMap;
                iCub::plantIdentification::PortsUtil *portsUtil;

                // new object learning
                bool learningNewObjectMode;
                std::vector<std::vector<double> > collectedFeatures;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

                int getArgMin(const gurls::gMat2D<double> &mat,int rowNum);

                bool getStandardPredictionsFrom1vsAll(const gurls::gMat2D<double> &predictions1vsAll,std::vector<int> &predictions);

                bool checkAccuracy(const std::vector<int> &predictions,const std::vector<int> &groundTruth,int numObjects);

                bool sendDetectedObjectToPort(int objectCode);

            public:

                MLUtil();

                bool init(yarp::os::ResourceFinder &rf,iCub::plantIdentification::PortsUtil *portsUtil);

                bool trainClassifier();

                bool testClassifier();

                bool testClassifierOneShot(std::vector<double> &features,int predictionEvaluationMethod);

                bool saveModelToFile(std::string fileSuffix);

                bool loadModelFromFile(std::string fileSuffix);

                bool loadTrainingSetFromFile(std::string fileSuffix);

                bool loadTestSetFromFile(std::string fileSuffix);

                bool loadObjectNamesFromFile(std::string fileSuffix);

                bool saveObjectNamesToFile(std::string fileSuffix);

                bool saveTrainingSetToFile(std::string fileSuffix);

                // methods related to the "new object learning" mode
                bool initNewObjectLearning(std::string newObjectName,bool isRefinement);
                bool addCollectedFeatures(std::vector<double> &features);
                bool discardLastCollectedFeatures();
                bool processCollectedData();
                bool isNewObjectLearningModeEnabled();

                void viewData();

                bool reset();

                bool release();

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

