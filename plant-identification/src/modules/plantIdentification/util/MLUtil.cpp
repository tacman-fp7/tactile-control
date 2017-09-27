#include "iCub/plantIdentification/util/MLUtil.h"

#include <stdexcept>
#include <iomanip>
#include <fstream>
#include <iostream>

using iCub::plantIdentification::MLUtil;


MLUtil::MLUtil(){

    learningNewObjectMode = false;
    refineObjectMode = false;

    trainingSetLoaded = false;
    testSetLoaded = false;
    objectMapLoaded = false;
    classifierTrained = false;

    nameObjectToLearn = "";

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

/*    objectsMap.insert(std::pair<int,std::string>(1,"sugar box"));
    objectsMap.insert(std::pair<int,std::string>(2,"tomato can"));
    objectsMap.insert(std::pair<int,std::string>(3,"brown block"));
    objectsMap.insert(std::pair<int,std::string>(4,"cat"));
    objectsMap.insert(std::pair<int,std::string>(5,"blue sponge"));
    objectsMap.insert(std::pair<int,std::string>(6,"blue ball"));
    objectsMap.insert(std::pair<int,std::string>(7,"yellow sponge"));
    objectsMap.insert(std::pair<int,std::string>(8,"small cube"));
    objectsMap.insert(std::pair<int,std::string>(9,"tennis ball"));
    objectsMap.insert(std::pair<int,std::string>(10,"soccer ball"));
*/

    return true;
}

bool MLUtil::trainClassifier(){

    if (trainingSetLoaded){

        std::cout << "Training started...";
        wrapper->train(xTr,yTr);
        std::cout << "...finished!" << std::endl;

        classifierTrained = true;

        return true;

    } else {

        return false;

    }
}

bool MLUtil::testClassifier(){

    if (trainingSetLoaded && testSetLoaded && classifierTrained){

        gurls::gMat2D<double> *output = wrapper->eval(xTe);

        std::vector<int> predictions, groundTruth;

        getStandardPredictionsFrom1vsAll(*output,predictions);

        getStandardPredictionsFrom1vsAll(yTe,groundTruth);

        checkAccuracy(predictions,groundTruth,output->cols());

        return true;

    } else {

        return false;

    }
}

bool MLUtil::testClassifierOneShot(std::vector<double> &features, int predictionEvaluationMethod, std::vector<double> &outputScores){




    if (trainingSetLoaded && classifierTrained){

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


        std::cout << "REAL OUTPUT >>> ";
        for(int i = 0; i < output->cols(); i++){
            std::cout << (*output)(0,i) << " ";
        }

        // store output scores
        outputScores.resize(output->cols());
        for (int i = 0; i < output->cols(); i++){
            outputScores[i] = (*output)(0, i);
        }


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

                std::cout << "OUTPUT MEAN" << std::endl;
                for(int i = 0; i < outputMean.cols(); i++){
                    std::cout << outputMean(0,i) << " ";
                }
                std::cout << std::endl;

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
                std::cout << "CURRENT MAX: " << currMax << std::endl;

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

        sendDetectedObjectToPort(prediction + 1);

    } else {

        return false;

    }


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

    classifierTrained = true;

    return true;
}

bool MLUtil::loadTrainingSetFromFile(std::string fileSuffix){

    std::cout << "loading training data...";

    xTr.readCSV(trainingSetXFileName + fileSuffix + ".dat");

    yTr.readCSV(trainingSetYFileName + fileSuffix + ".dat");

    std::cout << " ...done" << std::endl;

    trainingSetLoaded = true;

    return true;
}

bool MLUtil::loadTestSetFromFile(std::string fileSuffix){

    std::cout << "loading test data...";

    xTe.readCSV(testSetXFileName + fileSuffix + ".dat");

    yTe.readCSV(testSetYFileName + fileSuffix + ".dat");

    std::cout << " ...done" << std::endl;

    testSetLoaded = true;

    return true;
}

bool MLUtil::loadObjectNamesFromFile(std::string fileSuffix){

    std::cout << "loading object names list...";

    std::string objectName;
    std::string fileName = objectNamesFileName + fileSuffix + ".dat";
    std::ifstream objectNamesFile(fileName.c_str());
    objectsMap.clear();
    int i = 1;
    while (std::getline(objectNamesFile,objectName).good()){
        objectsMap.insert(std::pair<int,std::string>(i,objectName));
        i++;
    }
    objectNamesFile.close();

    std::cout << " ...done" << std::endl;

    objectMapLoaded = true;

    return true;
}

bool MLUtil::saveObjectNamesToFile(std::string fileSuffix){

    std::cout << "saving object names list...";

    std::string fileName = objectNamesFileName + fileSuffix + ".dat";
    std::ofstream objectNamesFile(fileName.c_str());

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

    std::string objectName;

    try {

        objectName = objectsMap[objectNum];
        portsUtil->sendStringToSpeaker("I think this is the " + objectsMap[objectNum]);

    } catch(const std::out_of_range& oor){

        portsUtil->sendStringToSpeaker("I do not know this object");
    }

    return true;
}

bool MLUtil::initNewObjectLearning(std::string newObjectName,bool isRefinement){

    learningNewObjectMode = true;

    collectedFeatures.clear();

    if (isRefinement){

        refineObjectMode = true;
    }
    else {

        nameObjectToLearn = newObjectName;

    }

    return true;
}

bool MLUtil::addCollectedFeatures(std::vector<double> &features){

    collectedFeatures.push_back(features);
}

bool MLUtil::discardLastCollectedFeatures(){

    if (collectedFeatures.size() > 0){
        collectedFeatures.pop_back();
        return true;
    } else {
        return false;
    }

}

bool MLUtil::processCollectedData(){

    if (collectedFeatures.size() > 0){

        /// add the new features to xTr and yTr 

        int numPrevObjects = yTr.cols();
        int numPrevSamples = xTr.rows();

        std::vector<std::vector<double> > tempStorage;
        
        // resize xTr
        tempStorage.resize(xTr.rows());
        for(int i = 0; i < tempStorage.size(); i++){
            tempStorage[i].resize(xTr.cols());
            for(int j = 0; j < tempStorage[i].size(); j++){
                tempStorage[i][j] = xTr(i,j);
            }
        }
        xTr.resize(xTr.rows() + collectedFeatures.size(),collectedFeatures[0].size());
        for(int i = 0; i < tempStorage.size(); i++){
            for(int j = 0; j < tempStorage[i].size(); j++){
                xTr(i,j) = tempStorage[i][j];
            }
        }

        // resize yTr
        tempStorage.resize(yTr.rows());
        for(int i = 0; i < tempStorage.size(); i++){
            tempStorage[i].resize(yTr.cols());
            for(int j = 0; j < tempStorage[i].size(); j++){
                tempStorage[i][j] = yTr(i,j);
            }
        }
        if (refineObjectMode == true){
            yTr.resize(yTr.rows() + collectedFeatures.size(),yTr.cols());
        } else {
            yTr.resize(yTr.rows() + collectedFeatures.size(),yTr.cols() + 1);
        }
        for(int i = 0; i < tempStorage.size(); i++){
            for(int j = 0; j < tempStorage[i].size(); j++){
                yTr(i,j) = tempStorage[i][j];
            }
        }


        for(int i = 0; i < collectedFeatures.size(); i++){
            // update xTr
            for(int j = 0; j < collectedFeatures[i].size(); j++){
                xTr(numPrevSamples + i,j) = collectedFeatures[i][j];
            }
            // update yTr
            if (refineObjectMode == true){

                for(int j = 0; j < numPrevObjects; j++){
                    // in the added rows, just the last column is set to one
                    if (j < numPrevObjects - 1){
                        yTr(numPrevSamples + i,j) = -1;
                    } else {
                        yTr(numPrevSamples + i,j) = +1;
                    }
                }
            } else {

                for(int j = 0; j < numPrevObjects + 1; j++){
                    // in the added rows, the columns related to the old objects are set to -1, the one related to the new object is set to 1 (as requested from the learner)
                    if (j < numPrevObjects){
                        yTr(numPrevSamples + i,j) = -1;
                    } else {
                        yTr(numPrevSamples + i,j) = +1;
                    }
                }
            }
        }
        
        // set the last column to -1 (except for the added rows)
        if (!refineObjectMode){
            for(int i = 0; i < numPrevSamples; i++){
                yTr(i,yTr.cols() - 1) = -1;
            }
        }

        trainingSetLoaded = true;

        /// update the object names map

        if (!refineObjectMode){
		int newKey;
		if (!objectsMap.empty()){
		    newKey = objectsMap.rbegin()->first + 1;
		} else {
		    newKey = 1;
		}
		objectsMap.insert(std::pair<int,std::string>(newKey,nameObjectToLearn));

		objectMapLoaded = true;
        }

        /// re-train the model

        trainClassifier();

         // disable the "learning new object" mode

        learningNewObjectMode = false;
        if (refineObjectMode == true){
            refineObjectMode = false;
        }

        return true;

    } else {

        return false;

    }

}

bool MLUtil::isNewObjectLearningModeEnabled(){

    return learningNewObjectMode;
}

bool MLUtil::viewData(){

    std::cout << std::endl << std::endl;

    std::cout << "-- X training --" << std::endl;
    for(int i = 0; i < xTr.rows(); i++){
        for(int j = 0; j < xTr.cols(); j++){
            std::cout << xTr(i,j) << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


    std::cout << "-- Y training --" << std::endl;
    for(int i = 0; i < yTr.rows(); i++){
        for(int j = 0; j < yTr.cols(); j++){
            std::cout << yTr(i,j) << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


    std::cout << "-- Collected features --" << std::endl;
    for(int i = 0; i < collectedFeatures.size(); i++){
        for(int j = 0; j < collectedFeatures[i].size(); j++){
            std::cout << collectedFeatures[i][j] << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


    std::cout << "-- object names map --" << std::endl;
    for(std::map<int,std::string>::iterator it = objectsMap.begin(); it != objectsMap.end(); ++it){

       std::cout << it->first << "\t" << objectsMap[it->first] << std::endl;
    }

    std::cout << "-- Output over time --" << std::endl;
    for(int i = 0; i < outputsOverTime.size(); i++){
        for(int j = 0; j < outputsOverTime[i].size(); j++){
            std::cout << outputsOverTime[i][j] << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    std::cout << std::endl << std::endl;

    return true;
}

bool MLUtil::reset(){

//    xTr.resize(0,0);
//    yTr.resize(0,0);
    trainingSetLoaded = false;
    testSetLoaded = false;

    objectsMap.clear();
    objectMapLoaded = false;

    classifierTrained = false;

    collectedFeatures.clear();
    learningNewObjectMode = false;

    return true;
}

bool MLUtil::release(){

    delete(wrapper);

    return true;
}
