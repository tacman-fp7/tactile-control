#include "iCub/plantIdentification/util/MLUtil.h"

#include <iomanip>
#include <fstream>
#include <iostream>

using iCub::plantIdentification::MLUtil;


MLUtil::MLUtil(){

    learningNewObjectMode = false;

    xTr.resize(0,0);
    yTr.resize(0,0);
    xTe.resize(0,0);
    yTe.resize(0,0);

    dbgTag = "MLUtil: ";
}

bool MLUtil::init(yarp::os::ResourceFinder &rf,iCub::plantIdentification::PortsUtil *portsUtil){

    this->portsUtil = portsUtil;

    wrapper = new gurls::KernelRLSWrapper<double>("myWrapper");

    outputsOverTime.resize(0);

    std::string pathPrefix = "/home/icub/tmp_massimo/tactile-control/data/objectRecognition/";

    modelFileName = pathPrefix + "model_";
    objectNamesFileName = pathPrefix + "objectNames_";
    trainingSetXFileName = pathPrefix + "trainingSetX_";
    trainingSetYFileName = pathPrefix + "trainingSetY_";
    testSetXFileName = pathPrefix + "testSetX_";
    testSetYFileName = pathPrefix + "testSetY_";

    // build default objects map
    objectsMap.clear();
    objectsMap.insert(std::pair<int,std::string>(1,"sugar box"));
    objectsMap.insert(std::pair<int,std::string>(2,"tomato can"));
    objectsMap.insert(std::pair<int,std::string>(3,"brown block"));
    objectsMap.insert(std::pair<int,std::string>(4,"cat"));
    objectsMap.insert(std::pair<int,std::string>(5,"blue sponge"));
    objectsMap.insert(std::pair<int,std::string>(6,"blue ball"));
    objectsMap.insert(std::pair<int,std::string>(7,"yellow sponge"));
    objectsMap.insert(std::pair<int,std::string>(8,"small cube"));
    objectsMap.insert(std::pair<int,std::string>(9,"tennis ball"));
    objectsMap.insert(std::pair<int,std::string>(10,"soccer ball"));

    return true;
}

bool MLUtil::trainClassifier(){

    std::cout << "Training started...";
    wrapper->train(xTr,yTr);
    std::cout << "...finished!" << std::endl;

    return true;
}

bool MLUtil::testClassifier(){

    gurls::gMat2D<double> *output = wrapper->eval(xTe);

    std::vector<int> predictions, groundTruth;

    getStandardPredictionsFrom1vsAll(*output,predictions);

    getStandardPredictionsFrom1vsAll(yTe,groundTruth);

    checkAccuracy(predictions,groundTruth,output->cols());

    return true;
}

bool MLUtil::testClassifierOneShot(std::vector<double> &features,int predictionEvaluationMethod){

    gurls::gMat2D<double> input(1,features.size());

    for(int i = 0; i < features.size(); i++){
        input(0,i) = features[i];
        std::cout << "( " << input[0][i] << "/" << input(0,i) << "/" << features[i] << ")" << " ";
    }

    std::cout << std::endl;

    for(int i = 0; i < input.rows(); i++){
        for(int j = 0; j < input.cols(); j++){
            std::cout << input(i,j) << " ";
        }
        std::cout << std::endl;
    }

    gurls::gMat2D<double> *output = wrapper->eval(input);

    int numObjects = output->cols();
    int prediction;

    if (predictionEvaluationMethod > 1 && outputsOverTime.size() > 0){ // refinement using either avarage or maxmax

        std::vector<double> currentOutput(numObjects);
        for(int i = 0; i < currentOutput.size(); i++){
            currentOutput[i] = (*output)(0,i);
        }
        outputsOverTime.push_back(currentOutput);

        if (predictionEvaluationMethod == 2){ // using avarage method

            gurls::gMat2D<double> outputMean = gurls::gMat2D<double>::zeros(1,numObjects);
           
            for(int i = 0; i < outputMean.cols(); i++){
                for(int j = 0; j < outputsOverTime.size(); j++){
                    outputMean(0,i) += outputsOverTime[j][i];
                }
                outputMean(0,i) /= outputsOverTime.size();
            }

            std::vector<int> predictions;
            getStandardPredictionsFrom1vsAll(outputMean,predictions);
            prediction = predictions[0];

        } else { // using maxmax method

            double currMax = -1000.0;
            int currIndex = -1;
            for(int i = 0; i < outputsOverTime.size(); i++){
                for(int j = 0; j < outputsOverTime[i].size(); j++){
                    if (outputsOverTime[i][j] > currMax){
                        currMax = outputsOverTime[i][j];
                        currIndex = j;
                    }
                }
            }
            prediction = currIndex;
        }

    } else {

        outputsOverTime.resize(1);
        outputsOverTime[0].resize(output->cols());
        for(int i = 0; i < outputsOverTime[0].size(); i++){
            outputsOverTime[0][i] = (*output)(0,i);
        }

        std::vector<int> predictions;
        getStandardPredictionsFrom1vsAll(*output,predictions);
        prediction = predictions[0];


        for(int i = 0; i < output->rows(); i++){
            for(int j = 0; j < output->cols(); j++){
                std::cout << (*output)(i,j) << " ";
            }
            std::cout << std::endl;
        }

    }


    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;
    std::cout << "PREDICTION: <<<<<<<<<<<<<<       " << prediction + 1 << "       >>>>>>>>>>>>>>" << std::endl;

    sendDetectedObjectToPort(prediction + 1);

}

bool MLUtil::saveModelToFile(std::string fileSuffix){

    std::cout << "saving model...";

    wrapper->saveModel(modelFileName + fileSuffix + ".dat");

    std::cout << " ...done" << std::endl;

    return true;
}

bool MLUtil::loadModelFromFile(std::string fileSuffix){

    std::cout << "loading model...";

    wrapper->loadOpt(modelFileName + fileSuffix + ".dat");

    std::cout << " ...done" << std::endl;

    return true;
}

bool MLUtil::loadTrainingSetFromFile(std::string fileSuffix){

    std::cout << "loading training data...";

    xTr.readCSV(trainingSetXFileName + fileSuffix + ".dat");

    yTr.readCSV(trainingSetYFileName + fileSuffix + ".dat");

    std::cout << " ...done" << std::endl;

    return true;
}

bool MLUtil::loadTestSetFromFile(std::string fileSuffix){

    std::cout << "loading test data...";

    xTe.readCSV(testSetXFileName + fileSuffix + ".dat");

    yTe.readCSV(testSetYFileName + fileSuffix + ".dat");

    std::cout << " ...done" << std::endl;

    return true;
}

bool MLUtil::loadObjectNamesFromFile(std::string fileSuffix){

    std::cout << "loading object names list...";

    std::string objectName;
    std::ifstream objectNamesFile(objectNamesFileName + fileSuffix + ".dat");
    objectsMap.clear();
    int i = 1;
    while (std::getline(objectNamesFile,objectName).good()){
        objectsMap.insert(std::pair<int,std::string>(i,objectName));
        i++;
    }
    objectNamesFile.close();

    std::cout << " ...done" << std::endl;

    return true;
}

bool MLUtil::saveObjectNamesToFile(std::string fileSuffix){

    std::cout << "saving object names list...";

    std::ofstream objectNamesFile(objectNamesFileName + fileSuffix + ".dat");

    for(int i = 1; i <= objectsMap.size(); i++){

        objectNamesFile << objectsMap[i] << std::endl;

    }

    objectNamesFile.close();

    std::cout << " ...done" << std::endl;

    return true;

}

bool MLUtil::saveTrainingSetToFile(std::string fileSuffix){

    std::cout << "saving training data...";

    xTr.saveCSV(trainingSetXFileName + fileSuffix + ".dat");

    yTr.saveCSV(trainingSetYFileName + fileSuffix + ".dat");

    std::cout << " ...done" << std::endl;

    return true;
}

int MLUtil::getArgMin(const gurls::gMat2D<double> &mat,int rowNum){

    int currIndex = -1;
    double currMax = -1000.0;
    
    for(int i = 0; i < mat.cols(); i++){
        if (mat(rowNum,i) > currMax){
            currMax = mat(rowNum,i);
            currIndex = i;
        }
    }

    return currIndex;
}

bool MLUtil::getStandardPredictionsFrom1vsAll(const gurls::gMat2D<double> &predictions1vsAll,std::vector<int> &predictions){

    predictions.resize(predictions1vsAll.rows());
    
    for(int i = 0; i < predictions1vsAll.rows(); i++){
        
        predictions[i] = getArgMin(predictions1vsAll,i);

    }

    return true;
}

bool MLUtil::checkAccuracy(const std::vector<int> &predictions,const std::vector<int> &groundTruth,int numObjects){
    using std::cout;
    using std::endl;

    int numGuessed = 0,numSamples = predictions.size();
    double accuracy;
    std::vector<int> numGuessedByClass(numObjects,0);
    std::vector<int> numSamplesByClass(numObjects,0);
    std::vector<double> accuracyByClass(numObjects,0);
    std::vector<std::vector<int> > confusionMatrix(numObjects);
    for(int i = 0; i < confusionMatrix.size(); i++){
        confusionMatrix[i].resize(numObjects,0);
    }

    for(int i = 0; i < numSamples; i++){
        confusionMatrix[groundTruth[i]][predictions[i]]++;
        if (groundTruth[i] == predictions[i]){
            numGuessed++;
            numGuessedByClass[groundTruth[i]]++;
        }
        numSamplesByClass[groundTruth[i]]++;
    }

    accuracy = static_cast<double>(numGuessed)/numSamples;
    for(int i = 0; i < accuracyByClass.size(); i++){
        accuracyByClass[i] = static_cast<double>(numGuessedByClass[i])/numSamplesByClass[i];
    }

    cout << "Confusion matrix: " << endl;
    cout << "    ";
    for(int i = 0; i < confusionMatrix.size(); i++){
        cout << (i + 1)%10 << " ";
    }
    cout << endl;
    cout << "    ";
    for(int i = 0; i < confusionMatrix.size(); i++){
        cout << "--";
    }
    cout << endl;
    

    for(int i = 0; i < confusionMatrix.size(); i++){
        cout << (i + 1)%10 << " | ";
        for(int j = 0; j < confusionMatrix[i].size(); j++){
	
		if (confusionMatrix[i][j] > 0){
			cout << confusionMatrix[i][j];
		} else {
			cout << ".";
		}
		cout << " ";
        }
        cout << endl;
    }
    cout << endl;

    cout << "Accuracy by class: ";
    for(int i = 0; i < accuracyByClass.size(); i++){
//        cout << i + 1 << ": " << std::setprecision(2) << accuracyByClass[i]*100 << "%  - ";
        cout << i + 1 << ": " << accuracyByClass[i]*100 << "%  - ";
    }
    cout << endl;
    
//    cout << "Total accuracy: " << std::setprecision(2) << accuracy*100 << "%" << endl;
    cout << "Total accuracy: " << accuracy*100 << "%" << endl;

}

bool MLUtil::sendDetectedObjectToPort(int objectNum){

    portsUtil->sendObjectLabelToSpeaker(objectsMap[objectNum]);
}

bool MLUtil::initNewObjectLearning(std::string newObjectName,bool isRefinement){

    learningNewObjectMode = true;

    if (!isRefinement){
        collectedFeatures.clear();
    }

    int newKey = objectsMap.rbegin()->first + 1;
    objectsMap.insert(std::pair<int,std::string>(newKey,newObjectName));
}

bool MLUtil::addCollectedFeatures(std::vector<double> &features){

    collectedFeatures.push_back(features);
}

bool MLUtil::discardLastCollectedFeatures(){

    if (collectedFeatures.size() > 0){
        collectedFeatures.pop_back();
    }
}

bool MLUtil::processCollectedData(){

    if (collectedFeatures.size() > 0){

        // add the new features to xTr and yTr 

        int numPrevObjects = yTr.cols();
        int numPrevSamples = xTr.rows();

        xTr.resize(xTr.rows() + collectedFeatures.size(),collectedFeatures[0].size());
        yTr.resize(yTr.rows() + collectedFeatures.size(),yTr.cols() + 1);

        for(int i = 0; i < collectedFeatures.size(); i++){
            // update xTr
            for(int j = 0; j < collectedFeatures[i].size(); j++){
                xTr(numPrevSamples + i,j) = collectedFeatures[i][j];
            }
            // update yTr
            for(int j = 0; j < numPrevObjects + 1; j++){
                // in the added rows, the columns related to the old objects are set to -1, the one related to the new object is set to 1 (as requested from the learner)
                if (j < numPrevObjects){
                    yTr(numPrevSamples + i,j) = -1;
                } else {
                    yTr(numPrevSamples + i,j) = +1;
                }
            }
        }
        
        // set the last column to -1 (except for the added rows)
        for(int i = 0; i < numPrevSamples; i++){
            yTr(i,yTr.cols() - 1) = -1;
        }

        // re-train the model

        trainClassifier();

    }

    // disable the "learning new object" mode

    learningNewObjectMode = false;
}

bool MLUtil::isNewObjectLearningModeEnabled(){

    return learningNewObjectMode;
}

bool MLUtil::release(){

    delete(wrapper);

    return true;
}
