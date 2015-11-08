#include "iCub/plantIdentification/task/ControlTask.h"

#include "iCub/plantIdentification/PlantIdentificationEnums.h"
#include "iCub/plantIdentification/util/ICubUtil.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Property.h>
#include <yarp/os/ConstString.h>

#include <sstream>
#include <cmath>

using iCub::plantIdentification::ControlTask;
using iCub::plantIdentification::LogData;
using iCub::plantIdentification::ControllersUtil;
using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::TaskCommonData;
using iCub::plantIdentification::ControlTaskData;
using iCub::plantIdentification::ICubUtil;

using iCub::ctrl::parallelPID;
using iCub::ctrl::minJerkTrajGen;
using yarp::os::Bottle;
using yarp::os::Value;

using std::string;

ControlTask::ControlTask(ControllersUtil *controllersUtil,PortsUtil *portsUtil,TaskCommonData *commonData,ControlTaskData *controlData,std::vector<double> &targetList,bool resetErrOnContact):Task(controllersUtil,portsUtil,commonData,controlData->lifespan,controlData->jointsList,controlData->fingersList) {
	using yarp::sig::Vector;
	using yarp::sig::Matrix;
    this->controlData = controlData;

	this->portsUtil = portsUtil;

	this->resetErrOnContact = resetErrOnContact;
	fingerIsInContact.resize(commonData->objDetectPressureThresholds.size(),false);

	pressureTargetValue.resize(fingersList.size());
    initialPressureTargetValue.resize(fingersList.size());
	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		pressureTargetValue[i] = (i >= targetList.size() ? targetList[targetList.size()-1] : targetList[i]);
        initialPressureTargetValue[i] = pressureTargetValue[i];
	}
	
	pid.resize(jointsList.size());
	pidOptionsPE.resize(jointsList.size());
	pidOptionsNE.resize(jointsList.size());
	
	currentKp.resize(jointsList.size());
    kpPe.resize(jointsList.size());
	kpNe.resize(jointsList.size());
	previousError.resize(jointsList.size());

    double taskThreadPeriodSec = commonData->taskThreadPeriod/1000.0;
	std::vector<double> ttPeOption,ttNeOption;
	ttPeOption.resize(jointsList.size());
	ttNeOption.resize(jointsList.size());
	for(size_t i = 0; i < jointsList.size(); i++){
		ttPeOption[i] = calculateTt(controlData->pidKpf[i],controlData->pidKif[i],0.0);
		ttNeOption[i] = calculateTt(controlData->pidKpb[i],controlData->pidKib[i],0.0);
	}
	
	std::vector<Vector> kpPeOptionVect;
	kpPeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < kpPeOptionVect.size(); i++){
		kpPeOptionVect[i].resize(1,controlData->pidKpf[i]);
	}
	std::vector<Vector> kiPeOptionVect;
	kiPeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < kiPeOptionVect.size(); i++){
		kiPeOptionVect[i].resize(1,controlData->pidKif[i]);
	}
	Vector kdPeOptionVect(1,0.0);
	std::vector<Vector> ttPeOptionVect;
	ttPeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < ttPeOptionVect.size(); i++){
		ttPeOptionVect[i].resize(1,ttPeOption[i]);
	}

	
	std::vector<Vector> kpNeOptionVect;
	kpNeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < kpNeOptionVect.size(); i++){
		kpNeOptionVect[i].resize(1,controlData->pidKpb[i]);
	}
	std::vector<Vector> kiNeOptionVect;
	kiNeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < kiNeOptionVect.size(); i++){
		kiNeOptionVect[i].resize(1,controlData->pidKib[i]);
	}
	Vector kdNeOptionVect(1,0.0);
	std::vector<Vector> ttNeOptionVect;
	ttNeOptionVect.resize(jointsList.size());
	for(size_t i = 0; i < ttNeOptionVect.size(); i++){
		ttNeOptionVect[i].resize(1,ttNeOption[i]);
	}

	Vector wpOptionVect(1,controlData->pidWp);
	Vector wiOptionVect(1,controlData->pidWi);
	Vector wdOptionVect(1,controlData->pidWd);
	Vector nOptionVect(1,controlData->pidN);
	
	Matrix satLimMatrix(1,2);
	satLimMatrix[0][0] = controlData->pidMinSatLim;
	satLimMatrix[0][1] = controlData->pidMaxSatLim;

	Bottle commonOptions;
	
	addOption(commonOptions,"Wp",Value(controlData->pidWp));
	addOption(commonOptions,"Wi",Value(controlData->pidWi));
	addOption(commonOptions,"Wd",Value(controlData->pidWd));
	addOption(commonOptions,"N",Value(controlData->pidN));
	addOption(commonOptions,"satLim",Value(controlData->pidMinSatLim),Value(controlData->pidMaxSatLim));

	for(size_t i = 0; i < jointsList.size(); i++){
		addOption(pidOptionsPE[i],"Kp",Value(controlData->pidKpf[i]));
		addOption(pidOptionsPE[i],"Ki",Value(controlData->pidKif[i]));
		addOption(pidOptionsPE[i],"Kd",Value(0.0));
		addOption(pidOptionsPE[i],"Tt",Value(ttPeOption[i]));
		pidOptionsPE[i].append(commonOptions);

		addOption(pidOptionsNE[i],"Kp",Value(controlData->pidKpb[i]));
		addOption(pidOptionsNE[i],"Ki",Value(controlData->pidKib[i]));
		addOption(pidOptionsNE[i],"Kd",Value(0.0));
		addOption(pidOptionsNE[i],"Tt",Value(ttNeOption[i]));
		pidOptionsNE[i].append(commonOptions);
	}

	for(size_t i = 0; i < jointsList.size(); i++){

		// TODO to be removed
		kpPe[i] = controlData->pidKpf[i];
		kpNe[i] = controlData->pidKpb[i];
		previousError[i] = 0;

		switch (controlData->controlMode){

			case GAINS_SET_POS_ERR:
				pid[i] = new parallelPID(taskThreadPeriodSec,kpPeOptionVect[i],kiPeOptionVect[i],kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect[i],satLimMatrix);
				pid[i]->setOptions(pidOptionsPE[i]);
				currentKp[i] = kpPe[i];
				break;

			case GAINS_SET_NEG_ERR:
				pid[i] = new parallelPID(taskThreadPeriodSec,kpNeOptionVect[i],kiNeOptionVect[i],kdNeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttNeOptionVect[i],satLimMatrix);
				pid[i]->setOptions(pidOptionsNE[i]);
				currentKp[i] = kpNe[i];
				break;

			case BOTH_GAINS_SETS:
				pid[i] = new parallelPID(taskThreadPeriodSec,kpPeOptionVect[i],kiPeOptionVect[i],kdPeOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttPeOptionVect[i],satLimMatrix);
				pid[i]->setOptions(pidOptionsPE[i]);
				currentKp[i] = kpPe[i];
				break;
		}
	}


	/*** CODE RELATED TO SUPERVISOR MODE ***/
	svKp = commonData->tpDbl(1);
	svKi = commonData->tpDbl(2);
	svKd = commonData->tpDbl(3);
	supervisorControlMode = (commonData->tpInt(0) != 0);
	Vector kpSvOptionVect(1,svKp);
	Vector kiSvOptionVect(1,svKi);
	Vector kdSvOptionVect(1,svKd);
	Vector ttSvOptionVect(1,calculateTt(svKp,svKi,svKd));
	Matrix pvSatLimMatrix(1,2);
	pvSatLimMatrix[0][0] = -1000.0;
	pvSatLimMatrix[0][1] = 1000.0;
	Bottle svPidOptions;
	addOption(svPidOptions,"Wp",Value(controlData->pidWp));
	addOption(svPidOptions,"Wi",Value(controlData->pidWi));
	addOption(svPidOptions,"Wd",Value(controlData->pidWd));
	addOption(svPidOptions,"N",Value(controlData->pidN));
	addOption(svPidOptions,"satLim",Value(-100000.0),Value(100000.0));
	addOption(svPidOptions,"Kp",Value(svKp));
	addOption(svPidOptions,"Ki",Value(svKi));
	addOption(svPidOptions,"Kd",Value(svKd));
	addOption(svPidOptions,"Tt",Value(ttSvOptionVect[0]));
	svPid = new parallelPID(taskThreadPeriodSec,kpSvOptionVect,kiSvOptionVect,kdSvOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttSvOptionVect,pvSatLimMatrix);
	svPid->setOptions(svPidOptions);
	
	// create the neural network and configure it

	yarp::os::Property nnConfProperty;
	Bottle nnConfBottle;

	if (fingersList.size() == 2){
		ICubUtil::getNNOptionsForErrorPrediction2Fingers(nnConfBottle);
	} else {
		ICubUtil::getNNOptionsForErrorPrediction3Fingers(nnConfBottle);
	}

	nnConfProperty.fromString(nnConfBottle.toString());
	neuralNetwork.configure(nnConfProperty);
	
	trackingModeEnabled = false;
	minJerkTrackingModeEnabled = false;

	// initialize the minimum jerk trajectory class 
	minJerkTrajectory = new minJerkTrajGen(1,commonData->taskThreadPeriod/1000.0,4); //(dimensions,sample time in seconds, trajectory reference time)

    disablePIDIntegralGain = (commonData->tpInt(40) != 0);

	/*** END OF CODE RELATED TO SUPERVISOR MODE ***/

	if (resetErrOnContact){
		taskName = APPROACH_AND_CONTROL;
		dbgTag = "Approach&ControlTask: ";
	} else {
		taskName = CONTROL;
		dbgTag = "ControlTask: ";
	}

}

void ControlTask::init(){
	using std::cout;

	controllersUtil->setTaskControlModes(jointsList,VOCAB_CM_OPENLOOP);

    // TODO WORKAROUND TO REMOVE
    if (disablePIDIntegralGain) controllersUtil->resetPIDIntegralGain(8);

	cout << "\n\n" << dbgTag << "TASK STARTED - Target: ";
	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		cout << pressureTargetValue[i] << " ";
	}
	cout << "\n\n";
}

void ControlTask::calculateControlInput(){
	using yarp::sig::Vector;
	
	double TP_1S_scaleLowLevelPidGains = commonData->tpDbl(10);


	// scala i guadagni del pid di basso livello
	if (TP_1S_scaleLowLevelPidGains > 0.001){
		
		scaleGains(TP_1S_scaleLowLevelPidGains);

		commonData->tempParameters[10] = Value(0.0);

		std::stringstream gainsChangedLog("");
		gainsChangedLog << "[ Kpf: " << controlData->pidKpf[0] << " Kif: " << controlData->pidKif[0] << " Kpb: " << controlData->pidKpb[0] << " Kib: " << controlData->pidKib[0] << " ]";
		optionalLogString.append(gainsChangedLog.str());
	}


	/*** CODE RELATED TO SUPERVISOR MODE ***/
	double svResultValueScaled;
	double svErr;
	double svCurrentPosition;
    double estimatedFinalPose; // best pose estimated either by neural networks or by plane fitting
	double finalTargetPose; // pose equal to either estimatedFinalPose or the pose coming out from the wave generator, depending on which mode is activated
	double actualCurrentTargetPose; // actual pose used as target in the supervising controller, it can be either finalTargetPose or the filtered pose coming out from the pose tracking 
	double thumbEnc,indexEnc,middleEnc,interMiddleEnc,enc8,handPosition;
	double svTrackerVel = commonData->tpDbl(27);
	double svTrackerAcc = commonData->tpDbl(28);
	double gripStrength;
	if (supervisorControlMode){
		thumbEnc = commonData->armEncodersAngles[9];
		indexEnc = commonData->armEncodersAngles[11];
		middleEnc = commonData->armEncodersAngles[13];
		enc8 = commonData->armEncodersAngles[8];
		Vector svRef;
		Vector svFb;

		if (jointsList.size() == 2){

			// using the "simple" method
			//svErrOld = (1.417*thumbEnc - 12.22) - middleEnc;
			
			// using the neural network
			std::vector<double> actualAngles(2);
			std::vector<double> rotatedAngles;
			actualAngles[0] = thumbEnc;
			actualAngles[1] = middleEnc;
			ICubUtil::rotateFingersData(actualAngles,rotatedAngles);
			Vector rotatedAnglesVector;
			rotatedAnglesVector.resize(2);
			rotatedAnglesVector[0] = rotatedAngles[0];
			rotatedAnglesVector[1] = rotatedAngles[1];
			Vector estimatedBestPositionNNVector = neuralNetwork.predict(rotatedAnglesVector);
			estimatedFinalPose = estimatedBestPositionNNVector[0];

			// handPosition = middleEnc - thumbEnc;
			handPosition = (middleEnc - thumbEnc)/2;

		} else {

			// si calcola il valore dell'angolo prossimale del dito medio intersezione del piano delle pose migliori
			//double interMiddleEnc = -0.1046*thumbEnc + 0.8397*indexEnc + 10.05;
			//interMiddleEnc = 1.4667*thumbEnc -1.0*indexEnc + 34.96;
			
			// si applica la formula della distanza tenendo conto che la differenza per i giunti di pollice e indice e' nulla. In teoria si sarebbe potuta evitare la divisione per due, perche' a noi interessa l'errore a meno di una costante
			//svErrOld = (interMiddleEnc - middleEnc)/2;

			// using the neural network
			std::vector<double> actualAngles(3);
			std::vector<double> rotatedAngles;
			actualAngles[0] = thumbEnc;
			actualAngles[1] = indexEnc;
			actualAngles[2] = middleEnc;
			ICubUtil::rotateFingersData(actualAngles,rotatedAngles);
			Vector rotatedAnglesVector;
			rotatedAnglesVector.resize(3);
			rotatedAnglesVector[0] = rotatedAngles[0];
			rotatedAnglesVector[1] = rotatedAngles[1];
			rotatedAnglesVector[2] = rotatedAngles[2];
			Vector estimatedBestPositionNNVector = neuralNetwork.predict(rotatedAnglesVector);
			estimatedFinalPose = estimatedBestPositionNNVector[0];

			// handPosition = (middleEnc + indexEnc)/2 - thumbEnc;
			handPosition = ((middleEnc + indexEnc)/2 - thumbEnc)/2;
		}

		svCurrentPosition = handPosition;
		svErr = estimatedFinalPose - svCurrentPosition;

		// hand pose square wave / sinusoid generator
		if (commonData->tpInt(18) != 0){
			
			double waveMean = commonData->tpInt(21);

			if (commonData->tpInt(18) == 1){
				double halfStep = commonData->tpDbl(19);
				int div = (int)((callsNumber*commonData->taskThreadPeriod/1000.0)/commonData->tpDbl(20));
				if (div%2 == 1){
					finalTargetPose = waveMean + halfStep;
				} else {
					finalTargetPose = waveMean - halfStep;
				}
			} else if (commonData->tpInt(18) == 2){
				int numCallsPerPeriod = (int)(2*(commonData->tpDbl(20)/(commonData->taskThreadPeriod/1000.0)));
				int callsNumberMod = callsNumber%numCallsPerPeriod;
				double ratio = (1.0*callsNumberMod)/numCallsPerPeriod;
				finalTargetPose = waveMean + commonData->tpDbl(19) * sin(ratio*2*3.14159265);
			}
		} else {
			finalTargetPose = estimatedFinalPose;
		}


		// TRACKING MODE
		// if tracking mode is activated, trajectoryInitialPose is initialized, if tracking mode is disabled, trackingModeEnabled is set to false so that next time trajectoryInitialPose will be initialized again
		if (commonData->tpInt(26) != 0){
			if (trackingModeEnabled == false){
				trajectoryInitialPose = handPosition;
                trajectoryInitialTime = callsNumber;
				trackingModeEnabled = true;
			}

            double trajectoryFinalPose = finalTargetPose;
            double d = fabs(trajectoryFinalPose - trajectoryInitialPose);
            double v = svTrackerVel;
            double a = svTrackerAcc;
            double t = (callsNumber-trajectoryInitialTime)*commonData->taskThreadPeriod/1000.0;
            double s;

            if (d == 0){
                actualCurrentTargetPose = trajectoryFinalPose;    
            } else {
                if (0.5*v*v/a < 0.5*d){
                    if (t < v/a){
                        s = 0.5*a*t*t;
                    } else if (t < d/v){
                        s = 0.5*v*v/a + v*(t - v/a);
                    } else if (t < d/v + v/a){
                        s = d - 0.5*v*v/a + v*(t - d/v) - 0.5*a*(t - d/v)*(t - d/v);
                    } else {
                        s = d;
                    }
                } else {
                    if (t < sqrt(d/a)){
                        s = 0.5*a*t*t;
                    } else if (t < 2*sqrt(d/a)){
                        s = 0.5*d + sqrt(a*d)*(t-sqrt(d/a)) - 0.5*a*(t-sqrt(d/a))*(t-sqrt(d/a));
                    } else {
                        s = d;
                    }
                }
                actualCurrentTargetPose = trajectoryInitialPose + (s/d)*(trajectoryFinalPose - trajectoryInitialPose);        
            }

		} else {
			if (trackingModeEnabled == true){
				trackingModeEnabled = false;
			}
		}

		// MIN JERK TRACKING
		// if minJerkTracking mode is activated, minJerkTrackingModeEnabled is initialized, if minJerkTracking mode is disabled, minJerkTrackingModeEnabled is set to false so that next time initPosition will be initialized again
		if (commonData->tpInt(29) != 0){
			if (minJerkTrackingModeEnabled == false){

				Vector initPosition(1,handPosition);
				minJerkTrajectory->init(initPosition);
				minJerkTrajectory->setT(commonData->tpDbl(30));
				minJerkTrackingModeEnabled = true;
			}

			Vector targetPosition(1,finalTargetPose);
			minJerkTrajectory->computeNextValues(targetPosition);

			Vector currentTargetPosition = minJerkTrajectory->getPos();
			actualCurrentTargetPose = currentTargetPosition[0];


		} else {
			if (minJerkTrackingModeEnabled == true){
				minJerkTrackingModeEnabled = false;
			}
		}

		// if no tracking modes are active, use the finalTargetPose as target reference in the supervising controller
		if (commonData->tpInt(26) == 0 && commonData->tpInt(29) == 0){
			actualCurrentTargetPose = finalTargetPose;
		}

		svRef.resize(1,actualCurrentTargetPose);
		svFb.resize(1,svCurrentPosition);
		
		Vector svResult = svPid->compute(svRef,svFb);
	
		if (commonData->tpInt(4) != 0){
			svPid->reset(svResult);
			commonData->tempParameters[4] = Value(0);
			optionalLogString.append("[ PID RESET ] ");
		}

		double balanceFactor;

		balanceFactor = svResult[0];

		// if pinky control mode is active, overwrite balanceFactor
		double pinkyAngleReference = 45;
		if (commonData->tpInt(31) != 0){
				double pinkyEnc = commonData->armEncodersAngles[15];
				double diff = pinkyEnc - pinkyAngleReference;
			
				if (callsNumber%commonData->screenLogStride == 0){
					std::stringstream printLog("");
					printLog << "[ref " << pinkyAngleReference << " pinky " << pinkyEnc << " diff " << diff << "]";
					optionalLogString.append(printLog.str());
				}

				balanceFactor = diff;
		}


		// 2 dita: valore POSITIVO se il MEDIO deve INCREMENTARE l'angolo 
		// 3 dita: valore POSITIVO sempre
		svResultValueScaled = commonData->tpDbl(5)*balanceFactor;
		




		// if the grip strength wave generatore is not active, I get the grip strength from the temp params 
		gripStrength = commonData->tpDbl(7);

		// grip strength square wave / sinusoid generator
		if (commonData->tpInt(22) != 0){
			
			// if the grip strength wave generatore is active, the current position is the hand position
			double waveMean = commonData->tpInt(25);

			if (commonData->tpInt(22) == 1){
				double halfStep = commonData->tpDbl(23);
				int div = (int)((callsNumber*commonData->taskThreadPeriod/1000.0)/commonData->tpDbl(24));
				if (div%2 == 1){
					gripStrength = waveMean + halfStep;
				} else {
					gripStrength = waveMean - halfStep;
				}
			} else if (commonData->tpInt(22) == 2){
				int numCallsPerPeriod = (int)(2*(commonData->tpDbl(24)/(commonData->taskThreadPeriod/1000.0)));
				int callsNumberMod = callsNumber%numCallsPerPeriod;
				double ratio = (1.0*callsNumberMod)/numCallsPerPeriod;
				gripStrength = waveMean + commonData->tpDbl(23) * sin(ratio*2*3.14159265);
			}
		}




		if (jointsList.size() == 2){

//			pressureTargetValue[0] = gripStrength *(1 - (commonData->tpDbl(8)+svResultValueScaled));
//			pressureTargetValue[1] = gripStrength *(1 + (commonData->tpDbl(8)+svResultValueScaled));

			pressureTargetValue[0] = gripStrength - (commonData->tpDbl(8)+svResultValueScaled)/2.0;
			pressureTargetValue[1] = gripStrength + (commonData->tpDbl(8)+svResultValueScaled)/2.0;

			//// se il valore e' positivo e quindi devo muovere il medio, devo aumentare la pressione richiesta al giunto 13, che si trova in posizione uno, altrimenti al giunto 9, in posizione 0
			//if (svResultValueScaled >= 0){
			//	pressureTargetValue[0] = initialPressureTargetValue[0] - svResultValueScaled/2.0;
			//	pressureTargetValue[1] = initialPressureTargetValue[1] + svResultValueScaled/2.0;
			//} else {
			//	pressureTargetValue[0] = initialPressureTargetValue[0] - svResultValueScaled/2.0;
			//	pressureTargetValue[1] = initialPressureTargetValue[1] + svResultValueScaled/2.0;
			//}
		} else {
			// il valore temporaneo di indice 9 serve eventualmente ad equilibrare la calibratura di indice e medio
//			pressureTargetValue[0] = gripStrength*(1 - (commonData->tpDbl(8)+svResultValueScaled));
//			pressureTargetValue[1] = (1-commonData->tpDbl(9))*0.5*gripStrength*(1 + (commonData->tpDbl(8)+svResultValueScaled));
//			pressureTargetValue[2] = (1+commonData->tpDbl(9))*0.5*gripStrength*(1 + (commonData->tpDbl(8)+svResultValueScaled));
			
			pressureTargetValue[0] = gripStrength - (commonData->tpDbl(8)+svResultValueScaled)/3.0;
			pressureTargetValue[1] = (1-commonData->tpDbl(9))*0.5*(gripStrength + 2.0*(commonData->tpDbl(8)+svResultValueScaled)/3.0);
			pressureTargetValue[2] = (1+commonData->tpDbl(9))*0.5*(gripStrength + 2.0*(commonData->tpDbl(8)+svResultValueScaled)/3.0);
		}

    	if (callsNumber%commonData->screenLogStride == 0){
    		std::stringstream printLog("");
			if (jointsList.size() == 2){
	    		printLog << " [P " << pressureTargetValue[0] << " - " << pressureTargetValue[1] << "]" << " [J " << thumbEnc << " - " << middleEnc << " err " << svErr << " u " << commonData->tpDbl(8)+svResultValueScaled << "]" ;
			} else {
				printLog << " [P " << pressureTargetValue[0] << " - " << pressureTargetValue[1] << " - " << pressureTargetValue[2] << "]" << " [J " << thumbEnc << " - " << indexEnc << " - " << middleEnc << " err " << svErr << " u " << commonData->tpDbl(8)+svResultValueScaled << "]" ;
			}
			optionalLogString.append(printLog.str());
	    }
		
	}
	/*** END OF CODE RELATED TO SUPERVISOR MODE ***/

	// square wave generator, it overrides what done by the supervisor, if active
	if (commonData->tpInt(11) != 0){
		double halfStep = commonData->tpDbl(12);
		int div = (int)((callsNumber*commonData->taskThreadPeriod/1000.0)/commonData->tpDbl(13));
		if (div%2 == 1){
			pressureTargetValue[0] = gripStrength + halfStep;
		} else {
			pressureTargetValue[0] = gripStrength - halfStep;
		}
	}

	for(size_t i = 0; i < jointsList.size(); i++){

        double error = pressureTargetValue[i] - commonData->overallFingerPressure[fingersList[i]];

        if (controlData->controlMode == BOTH_GAINS_SETS){

			if (error >= 0 && previousError[i] < 0){
				pid[i]->setOptions(pidOptionsPE[i]);
				currentKp[i] = kpPe[i];
			} else if (error < 0 && previousError[i] >= 0){
				pid[i]->setOptions(pidOptionsNE[i]);
				currentKp[i] = kpNe[i];
			}
		}

		Vector ref(1,pressureTargetValue[i]);
		Vector fb(1,commonData->overallFingerPressure[fingersList[i]]);
		Vector result = pid[i]->compute(ref,fb);
	
		// TODO to be removed
		if (controlData->pidResetEnabled && error > 0 && result[0] < currentKp[i]*error - 1.0){
			pid[i]->reset(result);
			optionalLogString.append("[ PID RESET ] ");
		}

		// if a finger gets in touch with the object then reset its PID
		if (resetErrOnContact && !fingerIsInContact[fingersList[i]] && commonData->overallFingerPressureMedian[fingersList[i]] > commonData->objDetectPressureThresholds[fingersList[i]]){
			pid[i]->reset(result);
			fingerIsInContact[fingersList[i]] = true;
			std::stringstream output("");
			output << "[ Finger " << fingersList[i] << " PID RESET ] ";
			optionalLogString.append(output.str());
		}

		inputCommandValue[i] = result[0];

		previousError[i] = error;


		if (commonData->tpInt(6) != 0){
			inputCommandValue[i] = 0.0;
		}

	}
    if (callsNumber%commonData->screenLogStride == 0 && commonData->tpInt(14)!=0){
        std::stringstream gainsLog("");
		gainsLog << "[K " << controlData->pidKpf[0] << " " << controlData->pidKif[0] << "][T " << pressureTargetValue[0] << "]";
		optionalLogString.append(gainsLog.str());
    }
	
    double actualGripStrength = 0;
    if (pressureTargetValue.size() == 2){
        actualGripStrength = (commonData->overallFingerPressure[4] + commonData->overallFingerPressure[1])/2.0;
    } else if (pressureTargetValue.size() == 3){
        actualGripStrength = (2*commonData->overallFingerPressure[4] + (commonData->overallFingerPressure[0] + commonData->overallFingerPressure[1]))/3.0;
    }




    if (pressureTargetValue.size() == 2){
//        estimatedFinalPose = (pressureTargetValue[0] + pressureTargetValue[1])/2.0;
    } else if (pressureTargetValue.size() == 3){
//        estimatedFinalPose = (2*pressureTargetValue[0] + (pressureTargetValue[1] + pressureTargetValue[2]))/3.0;
    }



	// log control data
	portsUtil->sendControlData(taskId,commonData->tpStr(16),commonData->tpStr(17),gripStrength,actualGripStrength,commonData->tpDbl(8)+svResultValueScaled,svErr,svCurrentPosition,actualCurrentTargetPose,finalTargetPose,estimatedFinalPose,svKp*commonData->tpDbl(5),svKi*commonData->tpDbl(5),svKd*commonData->tpDbl(5),thumbEnc,indexEnc,middleEnc,enc8,pressureTargetValue,commonData->overallFingerPressure,inputCommandValue,fingersList);


	//TODO TO REMOVE if the suprvisor PID gains change (in the temperary variables), update them (in the PID object)
    if (fabs(svKp - commonData->tpDbl(1)) > 0.0001){
		svKp = commonData->tpDbl(1);
		changeSVGain("Kp",svKp);
		Bottle tmp; svPid->getOptions(tmp);
		optionalLogString.append(tmp.toString());
	}
	if (fabs(svKi - commonData->tpDbl(2)) > 0.0001){
		svKi = commonData->tpDbl(2);
		changeSVGain("Ki",svKi);
		Bottle tmp; svPid->getOptions(tmp);
		optionalLogString.append(tmp.toString());
	}
	if (fabs(svKd - commonData->tpDbl(3)) > 0.0001){
		svKd = commonData->tpDbl(3);
		changeSVGain("Kd",svKd);
		Bottle tmp; svPid->getOptions(tmp);
		optionalLogString.append(tmp.toString());
	}

}

void ControlTask::buildLogData(LogData &logData){

	addCommonLogData(logData);

	if (resetErrOnContact) {
		logData.taskType = APPROACH_AND_CONTROL;
	} else {
		logData.taskType = CONTROL;
	}
	logData.taskOperationMode = controlData->controlMode;
	//TODO only first elements are logged!

    for(size_t i = 0; i < fingersList.size(); i++){

        logData.targetValue[i] = pressureTargetValue[i];

        logData.pidKpf[i] = controlData->pidKpf[i];
        logData.pidKif[i] = controlData->pidKif[i];
        logData.pidKdf[i] = 0.0;
        logData.pidKpb[i] = controlData->pidKpb[i];
        logData.pidKib[i] = controlData->pidKib[i];
        logData.pidKdb[i] = 0.0;
    }
}

void ControlTask::release(){

	for(size_t i = 0; i < jointsList.size(); i++){
		delete(pid[i]);
	}

	delete(minJerkTrajectory);

	// TODO TO REMOVE serve solo in fase di test
	commonData->tempParameters[17] = Value("#");

    // TODO WORKAROUND TO REMOVE
    if (disablePIDIntegralGain) controllersUtil->restorePIDIntegralGain(8);
}

void ControlTask::addOption(Bottle &bottle,const char *paramName,Value paramValue){

	Bottle valueBottle,paramBottle;

	valueBottle.add(paramValue);

	paramBottle.add(paramName);
	paramBottle.addList() = valueBottle;

	bottle.addList() = paramBottle;
}

void ControlTask::addOption(Bottle &bottle,const char *paramName,Value paramValue1,Value paramValue2){

	Bottle valueBottle,paramBottle;

	valueBottle.add(paramValue1);
	valueBottle.add(paramValue2);

	paramBottle.add(paramName);
	paramBottle.addList() = valueBottle;

	bottle.addList() = paramBottle;
}

void ControlTask::scaleGains(double scaleFactor){

	for(int i = 0; i < pid.size(); i++){
		
        Bottle oldOptions;
		pid[i]->getOptions(oldOptions);

		Bottle newOptions;

		replaceBottle(oldOptions,newOptions,scaleFactor);

		pid[i]->setOptions(newOptions);

		Bottle newOptionPE,newOptionNE;

		replaceBottle(pidOptionsPE[i],newOptionPE,scaleFactor);
		replaceBottle(pidOptionsNE[i],newOptionNE,scaleFactor);
		pidOptionsPE[i] = newOptionPE;
		pidOptionsNE[i] = newOptionNE;

		controlData->pidKpf[i] = scaleFactor*controlData->pidKpf[i];
		controlData->pidKif[i] = scaleFactor*controlData->pidKif[i];
		controlData->pidKpb[i] = scaleFactor*controlData->pidKpb[i];
		controlData->pidKib[i] = scaleFactor*controlData->pidKib[i];

	}

}

void ControlTask::changeSVGain(string gainName,double newGainValue){

        Bottle oldOptions;
		Bottle newOptions;
		
		svPid->getOptions(oldOptions);

		replaceBottle(oldOptions,newOptions,gainName,newGainValue);

		svPid->setOptions(newOptions);

}


void ControlTask::replaceBottle(yarp::os::Bottle &oldBottle,yarp::os::Bottle &newBottle,double scaleFactor){

	for(int i = 0; i < oldBottle.size(); i++){
        
		Bottle *paramBottle = oldBottle.get(i).asList();

		string paramName = paramBottle->get(0).asString();

		if (paramName == "Kp" || paramName == "Ki" || paramName == "Kd"){
			double gain = paramBottle->get(1).asList()->get(0).asDouble();
			addOption(newBottle,paramName.c_str(),Value(gain*scaleFactor));
            if (commonData->tpInt(14)!=0){
                //std::cout << "old " << paramName << ": " << gain << " new: " << gain*scaleFactor << "\n";
            }
		} else {
			newBottle.addList() = *paramBottle;
		}

	}
}

void ControlTask::replaceBottle(yarp::os::Bottle &oldBottle,yarp::os::Bottle &newBottle,string gainName,double newGainValue){

	for(int i = 0; i < oldBottle.size(); i++){
        
		Bottle *paramBottle = oldBottle.get(i).asList();

		string paramName = paramBottle->get(0).asString();

		if (paramName == gainName){
			addOption(newBottle,paramName.c_str(),Value(newGainValue));
		} else {
			newBottle.addList() = *paramBottle;
		}

	}
}

double ControlTask::calculateTt(double kp,double ki,double kd){

	double tt,ti,td,minTt,maxTt;

	ti = kp/ki;
	td = kd/kp;

	// TODO check the Tt rule
	minTt = controlData->pidWindUpCoeff*ti;
	maxTt = ti;
	if (td < minTt){
		tt = minTt;
	} else if (td > maxTt){
		tt = maxTt;
	} else tt = td;

	return tt;
}

std::string ControlTask::getPressureTargetValueDescription(){

	std::stringstream description("");

	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		description << pressureTargetValue[i] << " ";
	}
	
	return description.str();
}

void ControlTask::setTargetListRealTime(std::vector<double> &targetList){

	for(size_t i = 0; i < pressureTargetValue.size(); i++){
		pressureTargetValue[i] = (i >= targetList.size() ? targetList[targetList.size()-1] : targetList[i]);
	}

}
