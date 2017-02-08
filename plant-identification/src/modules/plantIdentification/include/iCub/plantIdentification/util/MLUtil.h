#ifndef __ICUB_PLANTIDENTIFICATION_MLUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_MLUTIL_H__

#include <gurls++\kernelrlswrapper.hpp>

#include <yarp/os/ResourceFinder.h>

#include <string>

namespace iCub {
    namespace plantIdentification {
        
        class MLUtil {
            
            private:

                gurls::KernelRLSWrapper<double> *wrapper;
                std::string modelFileName;
                std::string trainingSetXFileName;
                std::string trainingSetYFileName;
                std::string testSetXFileName;
                std::string testSetYFileName;
                gurls::gMat2D<double> xTr,yTr,xTe,yTe;
                std::vector<std::vector<double> > outputsOverTime;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

                int getArgMin(const gurls::gMat2D<double> &mat,int rowNum);

                bool getStandardPredictionsFrom1vsAll(const gurls::gMat2D<double> &predictions1vsAll,std::vector<int> &predictions);

                bool checkAccuracy(const std::vector<int> &predictions,const std::vector<int> &groundTruth,int numObjects);

            public:

                MLUtil();

                bool init(yarp::os::ResourceFinder &rf);

                bool trainClassifier();

                bool testClassifier();

                bool testClassifierOneShot(std::vector<double> &features,int predictionEvaluationMethod);

                bool saveModelToFile();

                bool loadModelFromFile();

                bool loadTrainingAndTestSetsFromFile();

                bool release();

        };
    } //namespace plantIdentification
} //namespace iCub

#endif

