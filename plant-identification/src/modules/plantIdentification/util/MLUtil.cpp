#include "iCub/plantIdentification/util/MLUtil.h"

#include <iomanip>

using iCub::plantIdentification::MLUtil;


MLUtil::MLUtil(){

    dbgTag = "MLUtil: ";
}

bool MLUtil::init(yarp::os::ResourceFinder &rf){

    wrapper = new gurls::KernelRLSWrapper<double>("myWrapper");
    std::vector<double> cioa;
    outputsOverTime.resize(0);

    modelFileName = "model.dat";
    trainingSetXFileName = "trainingSetX.dat";
    trainingSetYFileName = "trainingSetY.dat";
    testSetXFileName = "testSetX.dat";
    testSetYFileName = "testSetY.dat";

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
        input[0][i] = features[i];
    }

    gurls::gMat2D<double> *output = wrapper->eval(input);

    int numObjects = output->cols();
    int prediction;

    if (predictionEvaluationMethod > 1 && outputsOverTime.size() > 0){ // refinement using either avarage or maxmax

        std::vector<double> currentOutput(numObjects);
        for(int i = 0; i < currentOutput.size(); i++){
            currentOutput[i] = (*output)[0][i];
        }
        outputsOverTime.push_back(currentOutput);

        if (predictionEvaluationMethod){ // using avarage method

            gurls::gMat2D<double> outputMean = gurls::gMat2D<double>::zeros(1,numObjects);
           
            for(int i = 0; i < outputMean.cols(); i++){
                for(int j = 0; j < outputsOverTime.size(); j++){
                    outputMean[i] += outputsOverTime[j][i];
                }
                outputMean[i] /= outputsOverTime.size();
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
            outputsOverTime[0][i] = (*output)[0][i];
        }

        std::vector<int> predictions;
        getStandardPredictionsFrom1vsAll(*output,predictions);
        prediction = predictions[0];
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

}

bool MLUtil::saveModelToFile(){

    wrapper->saveModel(modelFileName);

    return true;
}


bool MLUtil::loadModelFromFile(){

    wrapper->loadOpt(modelFileName);

    return true;
}

bool MLUtil::loadTrainingAndTestSetsFromFile(){

    xTr.readCSV(trainingSetXFileName);
    yTr.readCSV(trainingSetYFileName);
    xTe.readCSV(testSetXFileName);
    yTe.readCSV(testSetYFileName);

    return true;
}

int MLUtil::getArgMin(const gurls::gMat2D<double> &mat,int rowNum){

    int currIndex = -1;
    double currMax = -1000.0;
    
    for(int i = 0; i < mat.cols(); i++){
        if (mat[rowNum][i] > currMax){
            currMax = mat[rowNum][i];
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
    std::vector<std::vector<int>> confusionMatrix(numObjects);
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
    for(int i = 0; i < confusionMatrix.size(); i++){
        for(int j = 0; j < confusionMatrix[i].size(); j++){

            cout << confusionMatrix[i][j] << "\t";
        }
        cout << endl;
    }
    cout << endl;

    cout << "Accuracy by class: ";
    for(int i = 0; i < accuracyByClass.size(); i++){
        cout << i + 1 << ": " << std::setprecision(2) << accuracyByClass[i]*100 << "%  - ";
    }
    cout << endl;
    
    cout << "Total accuracy: " << std::setprecision(2) << accuracy << "%" << endl;

}

bool MLUtil::release(){

    delete(wrapper);

    return true;
}
