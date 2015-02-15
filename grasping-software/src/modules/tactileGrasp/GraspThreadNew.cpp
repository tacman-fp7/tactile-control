/* 
 * Copyright (C) 2014 Francesco Giovannini, iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Francesco Giovannini
 * email:   francesco.giovannini@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */



#include "iCub/tactileGrasp/GraspThread.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <ctime>
#include <cmath>

#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

using std::cerr;
using std::cout;
using std::string;

using iCub::tactileGrasp::GraspThread;

using yarp::os::RateThread;
using yarp::os::Value;


/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
GraspThread::GraspThread(const int aPeriod, const yarp::os::ResourceFinder &aRf) 
    : RateThread(aPeriod) {
        period = aPeriod;
        rf = aRf;

        nFingers = 0;

        dbgTag = "GraspThread: ";
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Destructor                                                       ********************************************** */   
GraspThread::~GraspThread() {}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Initialise thread                                                ********************************************** */
bool GraspThread::threadInit(void) {
    using yarp::os::Property;
    using yarp::os::Network;
    using yarp::os::Bottle;
    using yarp::os::Time;
    using std::vector;

    cout << dbgTag << "Initialising. \n";

    /* ******* Extract configuration files          ******* */
    string robotName = rf.check("robot", Value("icub"), "The robot name.").asString().c_str();
    string whichHand = rf.check("whichHand", Value("right"), "The hand to be used for the grasping.").asString().c_str();

	
	jointVelocity = 10.0;
	jointToMove = 13;
	fingerToMove = 1;
	usedVoltage = false;
	thresholdParam = 10.0;
	touchToReach = 40.0;
	sumToReach = 60.0;
	kBackParam = 15.0;
	kParam = 0.02;
	kVoltage = 0.8;
	counter = -1;
	dataCollection.resize(12);
	counterMax = 200;
	for(int i = 0; i < 12; i++){
		vector<double> a;
		a.resize(1000);
		dataCollection[i] = a;
	}
	stopCounter = false;
	counterLoop = false;
	stdLogging = true;
	maxVoltage = 550.0;
	minVoltage = 500.0;
	previousMaxContact = 0.0;
	previousSumContact = 0.0;

	refVoltage = 200;

	operationMode = 4;
	op1Mode = 0;
	op1UseVoltage = false;
	initialVoltage = 260.0;
	voltageCounter = 0;
	op1MaxVoltage = 500.0;
	op1NumMaxVoltage = 25;

	op2VoltageToUse;
	op2Mode = 0;
	op2UseVoltage = false;
	op2MaxVoltage = 460.0;
	op2VoltageStep = 5.0;
	
	op3Mode = -1; // 0: go forward for x sec. | 1: collect data | 2: switch V to 0 | 3: collect data | 4: before back V | 5: back V | -1: wait for commands
	op3UseVoltage = false;
	op3BackSteps = 1;
	op3BackVoltage = -50;
	op3CmdState = -1; // 0: start | 1: go back | 2: close file
	op3AutomaticMode = false;

	op4MaxVel = 10.0;
	sumContactsRif = 60.0;
	op4kp = 0.02;
	op4State = 0;
	op4Counter = 0;
	op4LoggingEnabled = true;
	op4ControlMode = 1;
	op4KPosDir = 1.0;
	op4KOL = 2000.0;
	op4Contact = false;
	op4ki = 0.0;
	op4IntegrErr = 0.0;
	op4MaxIntegrErr = 100000.0;

	op5TestNumber = 0;

	op6ContactState = 0;
	op6FirstTimeOpenLoop = true;
	op6FirstTimeVelocity = true;

	voltageDirection = -1;

	voltageVector.push_back(360.0);
	voltageVector.push_back(460.0);
	//voltageVector.push_back(450.0);
	//voltageVector.push_back(500.0);

	
    // Build grasp parameters
    Bottle &confGrasp = rf.findGroup("graspTh");
    if (!confGrasp.isNull()) {
        // Individual touch thresholds per fingertip
        Bottle *confTouchThr = confGrasp.find("touchThresholds").asList();

        if (!confTouchThr->isNull()) {
            // Generate parameter vectors
            nFingers = confTouchThr->size();
            for (int i = 0; i < confTouchThr->size(); ++i) {
                touchThresholds.push_back(confTouchThr->get(i).asDouble());
            }
        } else {
            cerr << dbgTag << "Could not find the touch thresholds in the specified configuration file under the [graspTh] parameter group. \n";
            return false;
        }
    } else {
        cerr << dbgTag << "Could not find grasp configuration [graspTh] group in the specified configuration file. \n";
        return false;
    }


    /* ******* Build finger to joint map.           ******* */
    generateJointMap(touchThresholds);


    // Print out debug information
#ifndef NODEBUG
    cout << "\n";
    cout << "DEBUG: " << dbgTag << " Configured joints and thresholds: \n";
    for (size_t i = 0; i < touchThresholds.size(); ++i) {
        cout << "DEBUG: " << dbgTag << "\tFinger ID: " << i << "\t Touch threshold: " << touchThresholds[i] << "\t Joints: ";
        vector<int> fingerJoints = jointMap[i];
        for (size_t j = 0; j < fingerJoints.size(); ++j) {
            cout << fingerJoints[j] << " ";
        }
        cout << "\n";
    }
    cout << "\n";
#endif


    /* ******* Initialise previous contacts.        ******* */
    previousContacts.resize(nFingers, false);


    /* ******* Ports                                ******* */
    portGraspThreadInSkinComp.open("/TactileGrasp/skin/" + whichHand + "_hand_comp:i");
    portGraspThreadInSkinRaw.open("/TactileGrasp/skin/" + whichHand + "_hand_raw:i");
    portGraspThreadInSkinContacts.open("/TactileGrasp/skin/contacts:i");


    /* ******* Joint interfaces                     ******* */
    string arm = whichHand + "_arm";
    Property options;
    options.put("robot", robotName.c_str()); 
    options.put("device", "remote_controlboard");
//    options.put("writeStrict", "on");
    options.put("part", arm.c_str());
    options.put("local", ("/TactileGrasp/" + arm).c_str());
    options.put("remote", ("/" + robotName + "/" + arm).c_str());
    
    // Open driver
    if (!clientArm.open(options)) {
        return false;
    }
    // Open interfaces
    clientArm.view(iEncs);
    if (!iEncs) {
        return false;
    }
    clientArm.view(iPos);
    if (!iPos) {
        return false;
    }
    clientArm.view(iVel);
    if (!iVel) {
        return false;
    }
    clientArm.view(iPosDir);
    if (!iPosDir) {
        return false;
    }
	clientArm.view(iPid);
	if (!iPid) {
		return false;
	}
	clientArm.view(iOLC);
    if (!iOLC) {
        return false;
    }
	clientArm.view(iCtrl);
    if (!iCtrl) {
        return false;
    }

    // Set velocity control parameters
    iVel->getAxes(&nJointsVel);
    std::vector<double> refAccels(nJointsVel, 10^6);
    iVel->setRefAccelerations(&refAccels[0]);

    
    /* ******* Store position prior to acquiring control.           ******* */
    int nnJoints;
    iPos->getAxes(&nnJoints);
    startPos.resize(nnJoints);
    bool ok = false;
    while(!ok) {
        ok = iEncs->getEncoders(startPos.data());
#ifndef NODEBUG
        cout << "DEBUG: " << dbgTag << "Encoder data is not available yet. \n";
#endif
        Time::delay(0.1);
    }
    // Set reference speeds
    vector<double> refSpeeds(nnJoints, 0);
    iPos->getRefSpeeds(&refSpeeds[0]);
    for (int i = 11; i < 15; ++i) {
        refSpeeds[i] = 50;
    }
    iPos->setRefSpeeds(&refSpeeds[0]);

#ifndef NODEBUG
    cout << "\n";
    cout << "DEBUG: " << dbgTag << "Stored initial arm positions are: ";
    for (size_t i = 0; i < startPos.size(); ++i) {
        cout << startPos[i] << " ";
    }
    cout << "\n";
#endif

    // Put arm in position
    reachArm();


    // Connecting ports
    Network::connect(("/icub/skin/" + whichHand + "_hand_comp"), ("/TactileGrasp/skin/" + whichHand + "_hand_comp:i"));

    
    cout << dbgTag << "Initialised correctly. \n";
    
	double errorLimit;
	yarp::dev::Pid pid;

	iPid->getErrorLimit(jointToMove,&errorLimit);

	cout << "PID VALUES:\n";
	cout << "error limit: " << errorLimit << "\n";

	if (iPid->getPid(jointToMove,&pid)){
		pid.max_output = lastPosVoltage = minVoltage;
		iPid->setPid(jointToMove,pid);
		cout << "kp kd ki kff: " << pid.kp << " " << pid.kd << " " << pid.ki << " " << pid.kff << "\n";
		cout << "max output: " << pid.max_output << "\n";
		cout << "scale: " << pid.scale << "\n";
	}


    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Run thread                                                       ********************************************** */
void GraspThread::run(void) {
    using std::deque;
    using std::vector;
	using std::setw;

	if (operationMode == -1){
	
		if (velocities.grasp.size() > 0) {
			deque<bool> contacts (false, nFingers);
			vector<double> graspVelocities(nJointsVel, 0);
			vector<double> maxContacts(nFingers);
			vector<double> sumContacts(nFingers,0.0);
			std::vector<double> fingerTaxelValues;

			if (sampleCounter == 0){
				setControlMode(VOCAB_CM_OPENLOOP,true);
			}
			iOLC->setRefOutput(jointToMove,voltageDirection*refVoltage);

			if (sampleCounter%10 == 0){
			detectContact(contacts,maxContacts,sumContacts,fingerTaxelValues);
				cout << sumContacts[fingerToMove] << "\t" << maxContacts[fingerToMove] << "\n";
			}
			sampleCounter++;
		}	
	} else if (operationMode == 0){

		bool useVoltage = false;
		double voltageToUse;

		// Check that the control thread is actually being run or if this is just the module::configure() acting.
		if (velocities.grasp.size() > 0) {
			deque<bool> contacts (false, nFingers);
			vector<double> graspVelocities(nJointsVel, 0);
			vector<double> maxContacts(nFingers);
			vector<double> sumContacts(nFingers,0.0);
			std::vector<double> fingerTaxelValues;
			if (detectContact(contacts,maxContacts,sumContacts,fingerTaxelValues)) {

					// Loop all contacts
					for (size_t i = 0; i < contacts.size(); ++i) {
						vector<int> fingerJoints = jointMap[i];
						if (!contacts[i]) {
							// Loop all joints in that finger
							for (size_t j = 0; j < fingerJoints.size(); ++j) {
								// FG: -8 is required as the velocities array contains only finger joints speeds i.e. joints with id >= 8.
								if (fingerJoints[j] == jointToMove) graspVelocities[fingerJoints[j]] = jointVelocity;
								else graspVelocities[fingerJoints[j]] = 0.0;
							
								//graspVelocities[fingerJoints[j]] = velScalingFactor * velocities.grasp[fingerJoints[j] - 8];
							}
						} else {
							double error = sumToReach - sumContacts[i];
							

							// Loop all joints in that finger
							for (size_t j = 0; j < fingerJoints.size(); ++j) {
								// FG: -8 is required as the velocities array contains only finger joints speeds i.e. joints with id >= 8.
								
								if (fingerJoints[j] == jointToMove){
									
									useVoltage = true;
									if (error >= 0.0){
										if (usedVoltage == false){
											yarp::dev::Pid pid;
											iPid->getPid(jointToMove,&pid);
											previousVoltage = pid.max_output;
										} else if (previousVoltage < 1.0){
											previousVoltage = lastPosVoltage * kVoltage;
										}

										voltageToUse = previousVoltage + kParam * error;
										
										if (voltageToUse > maxVoltage) voltageToUse = maxVoltage;
										else if (voltageToUse < minVoltage) voltageToUse = minVoltage;

										lastPosVoltage = voltageToUse;
									} else voltageToUse = kBackParam*error;

								} else graspVelocities[fingerJoints[j]] = velocities.stop[fingerJoints[j] - 8];
							}
						}
					}

			} else {
				cout << dbgTag << "No contact. \n";
			}

	#ifndef NODEBUG
			//cout << "DEBUG: " << dbgTag << "Moving joints at velocities: \t";
			//for (size_t i = 0; i < graspVelocities.size(); ++i) {
			//    cout << i << " " << graspVelocities[i] << "\t";
			//}
			//cout << "\n";
	#endif

			if (useVoltage != usedVoltage){
				if (useVoltage == true) setControlMode(VOCAB_CM_OPENLOOP,false);
				else setControlMode(previousControlMode,false);
				usedVoltage = useVoltage;
			}


			double indexProximalEnc;
			iEncs->getEncoder(jointToMove,&indexProximalEnc);
			if ((jointToMove == 15 && indexProximalEnc > 90.0) || (jointToMove != 15 && indexProximalEnc > 45.0)){
				graspVelocities[jointToMove] = 0.0;
				voltageToUse = 0.0;
			}

			double output;
			iPid->getOutput(jointToMove,&output);
			if (useVoltage){ 
			if (stdLogging) cout << setw(15) << "VOLT: " << voltageToUse << " LAST POS: " << lastPosVoltage << "\n";
			} else {
				if (stdLogging) cout << setw(15) << "VEL: " << graspVelocities[jointToMove] << "\n";
			}
		
			if (useVoltage){
				if (voltageToUse > maxVoltage) voltageToUse = maxVoltage;
				if (voltageToUse < -250.0) voltageToUse = -250.0;
				previousVoltage = voltageToUse;
				iOLC->setRefOutput(jointToMove,voltageDirection*voltageToUse);
			} else {
				// Send move command
				if (graspVelocities[jointToMove] > 20.0) graspVelocities[jointToMove] = 20.0;
				iVel->velocityMove(jointToMove,graspVelocities[jointToMove]);
			}
		
		} else {
	#ifndef NODEBUG
			cout << "DEBUG: " << dbgTag << "Module initialisation running. \n";
	#endif
		}

		if (counter == counterMax){
			stdLogging = false;

			cout << "Threshold: " << thresholdParam  << " Goal: " << sumToReach << " kForward: " << kParam << " kBackward: " << kBackParam << " kVoltage: " << kVoltage << "\n";

			for (int i = 0; i < 12; i++){
				double tmpMedia = 0.0;
				double tmpVar = 0.0;
				double tmpMediaDiff = 0.0;
				double tmpVarDiff = 0.0;
				double tmpMediaDiffPos = 0.0;
				double tmpVarDiffPos = 0.0;
				double tmpMediaDiffNeg = 0.0;
				double tmpVarDiffNeg = 0.0;
				int posCounter = 0;
				int negCounter = 0;

				for(int j = 0; j < counterMax; j++){		
					tmpMedia += dataCollection[i][j];
					if (j >= 1){
						double diff = dataCollection[i][j] - dataCollection[i][j-1];
						if (diff >= 0){ 
							tmpMediaDiff += diff;
							tmpMediaDiffPos += diff;
							posCounter++;
						} else {
							tmpMediaDiff += -diff;
							tmpMediaDiffNeg += -diff;
							negCounter++;
						}
					}
				}
				tmpMedia = tmpMedia / counterMax;
				tmpMediaDiff = tmpMediaDiff / (counterMax-1);
				tmpMediaDiffPos = posCounter != 0 ? (tmpMediaDiffPos / posCounter) : -1.0;
				tmpMediaDiffNeg = negCounter != 0 ? (tmpMediaDiffNeg / negCounter) : -1.0;

				for(int j = 0; j < counterMax; j++){		
					tmpVar += (dataCollection[i][j] - tmpMedia)*(dataCollection[i][j] - tmpMedia);
					if (j >= 1){
						double diff = dataCollection[i][j] - dataCollection[i][j-1];
						if (diff >= 0){ 
							tmpVarDiff += (diff - tmpMediaDiff)*(diff - tmpMediaDiff);
							tmpVarDiffPos += (diff - tmpMediaDiffPos)*(diff - tmpMediaDiffPos);
						} else {
							tmpVarDiff += (-diff - tmpMediaDiff)*(-diff - tmpMediaDiff);
							tmpVarDiffNeg += (-diff - tmpMediaDiffNeg)*(-diff - tmpMediaDiffNeg);
						}
					}
				}
				tmpVar = tmpVar / counterMax;
				tmpVarDiff = tmpVarDiff / (counterMax-1);
				tmpVarDiffPos = posCounter != 0 ? (tmpVarDiffPos / posCounter) : -1.0;
				tmpVarDiffNeg = negCounter != 0 ? (tmpVarDiffNeg / negCounter) : -1.0;

				if (tmpMedia > 10) cout << "M&S: " << tmpMedia << " " << std::sqrt(tmpVar) << " DIFF SPN: " << tmpMediaDiff << " " << std::sqrt(tmpVarDiff) << " | " << tmpMediaDiffPos << " " << std::sqrt(tmpVarDiffPos) << " | " << tmpMediaDiffNeg << " " << std::sqrt(tmpVarDiffNeg) <<  "\n";

			}

			cout << "-------------------------------------------------------\n";

			if (stopCounter == true || counterLoop == false){
				counter = -1;
				stopCounter = false;
			} else {
				counter = 0;
			}

		
		} 
		

		}else if (operationMode == 1){

			if (velocities.grasp.size() > 0){

				if (!op1UseVoltage){
					setControlMode(VOCAB_CM_OPENLOOP,true);
					op1UseVoltage = true;
				}

				deque<bool> contacts (false, nFingers);
				vector<double> maxContacts(nFingers);
				vector<double> sumContacts(nFingers,0.0);
				std::vector<double> fingerTaxelValues;
				if (detectContact(contacts,maxContacts,sumContacts,fingerTaxelValues)){
					
					if (op1Mode == -1){
						op1VoltageToUse = 0.0;
					} else if (op1Mode == 0){
						op1VoltageToUse = initialVoltage;
						if (contacts[fingerToMove]){
							op1Mode = 1;
							op1Counter = 0;
							cout << "CONTACT!\nwaiting 10 sec...\n";
						}
					} else if (op1Mode == 1){
						if (op1Counter < 50*10){
							op1Counter++;
							if (op1Counter%50 == 0){
								cout << op1Counter/50 << ": maxCont: " << maxContacts[fingerToMove] << "  sum: " << sumContacts[fingerToMove] << "\n";
							}
						} else {
							if (voltageVector.size() > 0){
								op1Mode = 2;
								op1Counter = 0;
							} else {
								op1Mode = -1;
								cout << "THE END\n";
							}
						}
					} else if (op1Mode == 2){
						if (op1Counter == 0){
							double newVoltage = voltageVector[voltageCounter];
						
							std::ostringstream fileName(std::ostringstream::ate);
							fileName.str("");
							fileName << (int)op1VoltageToUse << "_" << (int)newVoltage << ".csv";
						
						
							outputFile.open(fileName.str().c_str(), std::ofstream::out | std::ofstream::app);
						
							op1GlobalCounter = -1;

							cout << "FILE " << fileName.str() << " OPENED\nwaiting 1 sec...\n";

							op1Counter++;
						} else if (op1Counter < 50*1){
							op1Counter++;
						} else {
							op1Mode = 3;
							op1Counter = 0;
							cout << "CHANGING VOLTAGE FROM " << op1VoltageToUse << " TO " << voltageVector[voltageCounter] << "\nwaiting 5 sec...\n";
							op1VoltageToUse = voltageVector[voltageCounter];
						}
						op1GlobalCounter++;
						// SCRIVI SU FILE
						outputFile << op1GlobalCounter << ";" << op1VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << "\n";
					} else if (op1Mode == 3){
						if (op1Counter < 50*5){
							if (op1Counter < op1NumMaxVoltage && voltageCounter == 0){
								op1VoltageToUse = op1MaxVoltage;
							} else  {
								op1VoltageToUse = voltageVector[voltageCounter];
							}
							op1Counter++;
							op1GlobalCounter++;
							//SCRIVI SU FILE
							outputFile << op1GlobalCounter << ";" << op1VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << "\n";
						} else {
							voltageCounter++;
							if (voltageCounter < voltageVector.size()){
								op1Mode = 2;
								op1Counter = 0;
							} else {
								op1Mode = -1;
								cout << "THE END\n";
							}
							op1GlobalCounter++;
							//SCRIVI SU FILE
							outputFile << op1GlobalCounter << ";" << op1VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << "\n";
							outputFile.close();
						}
					}

					iOLC->setRefOutput(jointToMove,voltageDirection*op1VoltageToUse);
				
				}
		
			}
		} else if (operationMode == 2){
		
			if (velocities.grasp.size() > 0){

				if (!op2UseVoltage){
					setControlMode(VOCAB_CM_OPENLOOP,true);
					op2UseVoltage = true;
				}

				deque<bool> contacts (false, nFingers);
				vector<double> maxContacts(nFingers);
				vector<double> sumContacts(nFingers,0.0);
				std::vector<double> fingerTaxelValues;
				if (detectContact(contacts,maxContacts,sumContacts,fingerTaxelValues)){
					
					if (op2Mode == -1){
						op2VoltageToUse = 0.0;
					} else if (op2Mode == 0){
						op2VoltageToUse = initialVoltage;
						if (contacts[fingerToMove]){
							op2Mode = 1;
							op2Counter = 0;
							cout << "CONTACT!\nwaiting 5 sec...\n";
						}
					} else if (op2Mode == 1){
						if (op2Counter < 50*5){
							op2Counter++;
							if (op2Counter%50 == 0){
								cout << op2Counter/50 << ": maxCont: " << maxContacts[fingerToMove] << "  sum: " << sumContacts[fingerToMove] << "\n";
							}
						} else {
							op2Mode = 2;
							op2Counter = 0;
							std::ostringstream fileName(std::ostringstream::ate);
							fileName.str("");
							fileName << (int)initialVoltage << "_to_" << (int)op2MaxVoltage << "_by_" << (int)op2VoltageStep << ".csv";
							outputFile.open(fileName.str().c_str(), std::ofstream::out | std::ofstream::app);
							outputFile << "Volt;MaxF;SumF;Deg\n";
							cout << "FILE " << fileName.str() << " OPENED\n";

						}
					} else if (op2Mode == 2){
						if (op2Counter == 0){
							cout << "USING VOLTAGE: " << op2VoltageToUse << " / " << op2MaxVoltage << "\n";
						}

						if (op2Counter < 50*2){
							op2Counter++;
						} else {
							op2Mode = 3;
							op2Counter = 0;
							op2EncoderTot = 0.0;
							op2VoltageTot = 0.0;
							op2MaxContactTot = 0.0;
							op2SumContactTot = 0.0;
						}
					} else if (op2Mode == 3){
						if (op2Counter < 50*2){
							double jointToMoveEnc;
							iEncs->getEncoder(jointToMove,&jointToMoveEnc);
							op2EncoderTot += jointToMoveEnc;
							op2VoltageTot += op2VoltageToUse;
							op2MaxContactTot += maxContacts[fingerToMove];
							op2SumContactTot += sumContacts[fingerToMove];
							op2Counter++;
						} else {
							outputFile << op2VoltageTot/(50*2) << ";" << op2MaxContactTot/(50*2) << ";" << op2SumContactTot/(50*2) << ";" << op2EncoderTot/(50*2) << "\n";
							
							if (op2VoltageToUse + op2VoltageStep  > op2MaxVoltage + 0.01){
								outputFile.close();
								op2Mode = -1;
								cout << "THE END\n";
							} else {
								op2VoltageToUse += op2VoltageStep;
								op2Mode = 2;
								op2Counter = 0;
							}
						}
					}

					iOLC->setRefOutput(jointToMove,voltageDirection*op2VoltageToUse);
				
				}
			
			}
		
		} else if (operationMode == 3){
		
			if (velocities.grasp.size() > 0){

				if (!op3UseVoltage){
					setControlMode(VOCAB_CM_OPENLOOP,true);
					op3UseVoltage = true;
				}

				deque<bool> contacts (false, nFingers);
				vector<double> maxContacts(nFingers);
				vector<double> sumContacts(nFingers,0.0);
				std::vector<double> fingerTaxelValues;
				if (detectContact(contacts,maxContacts,sumContacts,fingerTaxelValues)){
					
					if (op3Mode == -1){
						op3VoltageToUse = 0.0;
						if (op3CmdState == 0){
							op3TrackingNum = 0;
							op3Mode = 0;
							op3Counter = 0;
							op3CmdState = -1;
						} else if (op3CmdState == 1){
							// OPEN T_BACK_X
							std::ostringstream trackFileName(std::ostringstream::ate);
							trackFileName.str("");
							trackFileName << op3TrackingNum << "_tracking_back_" << (int)(-op3BackVoltage) << "_for_" << op3BackSteps << "_steps.csv";
							trackFile.open(trackFileName.str().c_str(), std::ofstream::out | std::ofstream::app);
							trackFile << "Volt;MaxF;SumF;Deg\n";
							op3TrackingNum++;
							cout << "TRACKING FILE " << trackFileName.str() << " OPENED\n";
							
							op3Mode = 4;
							op3Counter = 0;
							op3CmdState = -1;
						} else if (op3CmdState == 2){
							averageFile.close();
							op3CmdState = -1;
						}
					} else if (op3Mode == 0){
						op3VoltageToUse = initialVoltage;
						if (op3Counter < 50*8){
							op3Counter ++;
							if (op3Counter == 50*8){
								// OPEN C
								std::ostringstream averageFileName(std::ostringstream::ate);
								averageFileName.str("");
								averageFileName << "average_from_" << (int)initialVoltage << ".csv";
								averageFile.open(averageFileName.str().c_str(), std::ofstream::out | std::ofstream::app);
								averageFile << "Volt;MaxF;SumF;Deg\n";
								cout << "FILE " << averageFileName.str() << " OPENED\n";

								// OPEN T_IV_0
								std::ostringstream trackFileName(std::ostringstream::ate);
								trackFileName.str("");
								trackFileName << "tracking_from_" << (int)initialVoltage << "_to_0.csv";
								trackFile.open(trackFileName.str().c_str(), std::ofstream::out | std::ofstream::app);
								trackFile << "Volt;MaxF;SumF;Deg\n";
								cout << "FILE " << trackFileName.str() << " OPENED\n";

								op3Mode = 1;
								op3Counter = 0;
							}
						}
					} else if (op3Mode == 1){
						op3VoltageToUse = initialVoltage;
						if (op3Counter < 50*2){
							if (op3Counter == 0){
								op3EncoderTot = 0.0;
								op3VoltageTot = 0.0;
								op3MaxContactTot = 0.0;
								op3SumContactTot = 0.0;
							}
							double jointToMoveEnc;
							iEncs->getEncoder(jointToMove,&jointToMoveEnc);
							// COLLECT DATA
							op3EncoderTot += jointToMoveEnc;
							op3VoltageTot += op3VoltageToUse;
							op3MaxContactTot += maxContacts[fingerToMove];
							op3SumContactTot += sumContacts[fingerToMove];
							
							if (op3Counter >= 50*1){
								// TRACK T
								trackFile << op3VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << ";" << jointToMoveEnc << "\n";
							}
							op3Counter++;
							if (op3Counter == 50*2){
								// STORE C
								averageFile << op3VoltageToUse << ";" << op3MaxContactTot/(50*2) << ";" << op3SumContactTot/(50*2) << ";" << op3EncoderTot/(50*2) << "\n";
								cout << "V: " << op3VoltageToUse << " Max: " << op3MaxContactTot/(50*2) << " Sum: " << op3SumContactTot/(50*2) << " Enc: " << op3EncoderTot/(50*2) << "\n";
								op3Mode = 2;
								op3Counter = 0;
								cout << "SWITCHING FROM " << initialVoltage << " V TO 0 V\n";
							}
						}
					} else if (op3Mode == 2){
						op3VoltageToUse = 0.0;
						if (op3Counter < 50*5){
							// TRACK T
							double jointToMoveEnc;
							iEncs->getEncoder(jointToMove,&jointToMoveEnc);
							trackFile << op3VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << ";" << jointToMoveEnc << "\n";
							op3Counter++;
							if (op3Counter == 50*5){
								// CLOSE T
								cout << "TRACKING FILE CLOSED\n";
								trackFile.close();
								op3Mode = 3;
								op3Counter = 0;
							}
						}
					} else if (op3Mode == 3){
						op3VoltageToUse = 0.0;
						if (op3Counter < 50*2){
							if (op3Counter == 0){
								op3EncoderTot = 0.0;
								op3VoltageTot = 0.0;
								op3MaxContactTot = 0.0;
								op3SumContactTot = 0.0;
							}
							double jointToMoveEnc;
							iEncs->getEncoder(jointToMove,&jointToMoveEnc);
							// COLLECT DATA
							op3EncoderTot += jointToMoveEnc;
							op3VoltageTot += op3VoltageToUse;
							op3MaxContactTot += maxContacts[fingerToMove];
							op3SumContactTot += sumContacts[fingerToMove];
							op3Counter++;
							if (op3Counter == 50*2){
								// STORE C
								averageFile << op3VoltageToUse << ";" << op3MaxContactTot/(50*2) << ";" << op3SumContactTot/(50*2) << ";" << op3EncoderTot/(50*2) << "\n";
								cout << "V: " << op3VoltageToUse << " Max: " << op3MaxContactTot/(50*2) << " Sum: " << op3SumContactTot/(50*2) << " Enc: " << op3EncoderTot/(50*2) << "\n";
								if (op3AutomaticMode){
									if (op3SumContactTot/(50*2) > 20.0){
										if (op3TrackingNum > 0 && op3TrackingNum%10 == 0){
											op3BackVoltage -= 50.0;
										}
										std::ostringstream trackFileName(std::ostringstream::ate);
										trackFileName.str("");
										trackFileName << op3TrackingNum << "_tracking_back_" << (int)(-op3BackVoltage) << "_for_" << op3BackSteps << "_steps.csv";
										trackFile.open(trackFileName.str().c_str(), std::ofstream::out | std::ofstream::app);
										trackFile << "Volt;MaxF;SumF;Deg\n";
										op3TrackingNum++;
										cout << "TRACKING FILE " << trackFileName.str() << " OPENED\n";
										op3Mode = 4;
										op3Counter = 0;
									} else {
										cout << "THE END!\n";
										averageFile.close();
										op3Mode = -1;
										op3Counter = 0;
									}
								} else {
									op3Mode = -1;
									op3Counter = 0;
								}
							}
						}
					} else if (op3Mode == 4){
						op3VoltageToUse = 0.0;
						if (op3Counter < 50*1){
							// TRACK T
							double jointToMoveEnc;
							iEncs->getEncoder(jointToMove,&jointToMoveEnc);
							trackFile << op3VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << ";" << jointToMoveEnc << "\n";
							op3Counter++;
							if (op3Counter == 50*1){
								op3Mode = 5;
								op3Counter = 0;
								cout << "SWITCHING FROM 0 V TO " << op3BackVoltage << " V\n";
							}
						}
					} else if (op3Mode == 5){
						op3VoltageToUse = op3BackVoltage;
						if (op3Counter < op3BackSteps){
							// TRACK T
							double jointToMoveEnc;
							iEncs->getEncoder(jointToMove,&jointToMoveEnc);
							trackFile << op3VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << ";" << jointToMoveEnc << "\n";
							op3Counter++;
							if (op3Counter == op3BackSteps){
								op3Mode = 2;
								op3Counter = 0;
								cout << "SWITCHING FROM " << op3BackVoltage << " V TO 0 V\n";
							}
						}
					
					}


					iOLC->setRefOutput(jointToMove,voltageDirection*op3VoltageToUse);
				
				}
			
			}
		
		} else if (operationMode == 4){

			if (velocities.grasp.size() > 0){
				bool a = true;
				int c = a;
				c = a + 2;

				deque<bool> contacts (false, nFingers);
				vector<double> maxContacts(nFingers);
				vector<double> sumContacts(nFingers,0.0);
				std::vector<double> fingerTaxelValues;
				if (detectContact(contacts,maxContacts,sumContacts,fingerTaxelValues)){

					if (op4State == 0){

						double jointToMoveEnc;
						iEncs->getEncoder(jointToMove,&jointToMoveEnc);
						op4PosRef = jointToMoveEnc;

						std::ostringstream fileName(std::ostringstream::ate);
						fileName.str("");
						if (op4ControlMode == 1){
							fileName << "grasping_data_maxvel" << (int)op4MaxVel << "_kp" << (int)(op4kp * 100) << "_ki" << (int)(op4ki * 100) << "_sum" << sumContactsRif << ".csv";
						} else if (op4ControlMode == 2) {
							fileName << "grasping_data_kposdir" << (int)(op4KPosDir * 100) << "_kp" << (int)(op4kp * 100) << "_ki" << (int)(op4ki * 100) << "_sum" << sumContactsRif << ".csv";
						} else if (op4ControlMode == 3){
							fileName << "grasping_data_kol" << (int)(op4KOL * 100) << "_kposdir" << (int)(op4KPosDir * 100) << "_kp" << (int)(op4kp * 100) << "_ki" << (int)(op4ki * 100) << "_sum" << sumContactsRif << ".csv";
						}

						outputFile.open(fileName.str().c_str(), std::ofstream::out | std::ofstream::app);

						cout << "FILE " << fileName.str() << " OPENED\n";

						outputFile << "N;kp;Vel;Max;Sum;Deg;IntErr" << "\n";

						std::ostringstream fileName2(std::ostringstream::ate);
						fileName2.str("");
						fileName2 << "grasping_data_detail" << (int)(op4kp * 100) << "_sum" << sumContactsRif << ".csv";
						
						detailFile.open(fileName2.str().c_str(), std::ofstream::out | std::ofstream::app);

						cout << "FILE " << fileName2.str() << " OPENED\n";

						detailFile << "N;kp;Vel;Max;Sum;Deg;IntErr" << "\n";

						op4State = 1;

					} else if (op4State == 1){
						
						double velOrPosDir;

						if (!contacts[fingerToMove]){
							
							op4VelToUse = op4MaxVel;

						} else {

							if (!op4Contact){
								op4Contact = true;
							}

							double error = sumContactsRif - sumContacts[fingerToMove];
							
							//if (op4IntegrErr + error > op4MaxIntegrErr){
							//	op4IntegrErr = op4MaxIntegrErr;
							//} else if (op4IntegrErr + error < op4MaxIntegrErr) {
							//	op4IntegrErr = -op4MaxIntegrErr;
							//} else {
							//	op4IntegrErr += error;
							//}

							op4IntegrErr += error;

							op4VelToUse = op4kp*error + op4ki*op4IntegrErr;
							
							if (op4VelToUse > op4MaxVel){
								op4VelToUse = op4MaxVel;
							}

						}

						double jointToMoveEnc;
						iEncs->getEncoder(jointToMove,&jointToMoveEnc);

						if (op4ControlMode == 1){
							iVel->velocityMove(jointToMove,op4VelToUse);
							velOrPosDir = op4VelToUse;
						} else if (op4ControlMode == 2) {
							double op4PosToReach;
							op4PosRef = op4PosRef + op4KPosDir*op4VelToUse/50.0;
							op4PosToReach = op4PosRef;
							if (op4PosToReach > 90.0) op4PosToReach = 90.0;
							else if (op4PosToReach < 0.0) op4PosToReach = 0.0;
							iPosDir->setPosition(jointToMove,op4PosToReach);
							velOrPosDir = op4PosToReach;
						} else if (op4ControlMode == 3){
							// double op4VoltageToUse = op4KOL*op4KPosDir*op4VelToUse/50.0;
							double op4VoltageToUse = op4KOL*op4VelToUse;
							iOLC->setRefOutput(jointToMove,voltageDirection*op4VoltageToUse);
							velOrPosDir = op4VoltageToUse;
						}

						
						if (op4LoggingEnabled){
							if (op4Counter%5 == 0) {
								outputFile << op4Counter << ";" << op4kp << ";" << velOrPosDir << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << ";" << jointToMoveEnc << ";" << op4IntegrErr << "\n";
							}
							detailFile << op4Counter << ";"  << op4kp << ";" << velOrPosDir << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << ";" << jointToMoveEnc << ";" << op4IntegrErr << "\n";
						}
						if (op4Counter%10 == 0){
							cout << sumContacts[fingerToMove] << "\t" << velOrPosDir << "\t" << maxContacts[fingerToMove] << "\t" << jointToMoveEnc << ";" << op4IntegrErr << "\n";
						}







						op4Counter++;

						if (op4Counter == 50*20){
							op4State = 2;
						}
					
					} else if (op4State == 2){

						if (op4ControlMode == 1){
							iVel->velocityMove(jointToMove,0.0);
						} else if (op4ControlMode == 2) {
							double jointToMoveEnc;
							iEncs->getEncoder(jointToMove,&jointToMoveEnc);
							iPosDir->setPosition(jointToMove,jointToMoveEnc);
						} else if (op4ControlMode == 3){
							iOLC->setRefOutput(jointToMove,0.0);
						}
						
						outputFile.close();
						detailFile.close();

						cout << "THE END!\n";
					}


				}
		
			}

		} else if (operationMode == 5){

			if (velocities.grasp.size() > 0){

				if (!op1UseVoltage){
					setControlMode(VOCAB_CM_OPENLOOP,true);
					op1UseVoltage = true;
				}

				deque<bool> contacts (false, nFingers);
				vector<double> maxContacts(nFingers);
				vector<double> sumContacts(nFingers,0.0);
				vector<double> fingerTaxelValues(12);
				if (detectContact(contacts,maxContacts,sumContacts,fingerTaxelValues)){
					
					if (op1Mode == -1){
						op1VoltageToUse = 0.0;
					} else if (op1Mode == 0){
						if (voltageVector.size() > 0){
							op1VoltageToUse = 0.0;
							op1Mode = 2;
							op1Counter = 0;
							voltageCounter = 0;						
						} else {
							op1Mode = -1;
							op5TestNumber++;
							cout << "THE END\n";
						}

					//} else if (op1Mode == 1){
					//	if (op1Counter < 50*10){
					//		op1Counter++;
					//		if (op1Counter%50 == 0){
					//			cout << op1Counter/50 << ": maxCont: " << maxContacts[fingerToMove] << "  sum: " << sumContacts[fingerToMove] << "\n";
					//		}
					//	} else {
					//		if (voltageVector.size() > 0){
					//			op1Mode = 2;
					//			op1Counter = 0;
					//		} else {
					//			op1Mode = -1;
					//			cout << "THE END\n";
					//		}
					//	}

					} else if (op1Mode == 2){
						if (op1Counter == 0){
							double newVoltage = voltageVector[voltageCounter];
						
							time_t now = time(0);
							tm *ltm = localtime(&now);
							char myDate[15];
							strftime(myDate,15,"%m%d%H%M%S",ltm);

							std::ostringstream fileName(std::ostringstream::ate);
							fileName.str("");
							fileName << myDate << "_T" << op5TestNumber << "_" << (int)op1VoltageToUse << "_" << (int)newVoltage << ".csv";
						
						
							outputFile.open(fileName.str().c_str(), std::ofstream::out | std::ofstream::app);
						
							op1GlobalCounter = -1;

							cout << "FILE " << fileName.str() << " OPENED\nwaiting 1 sec...\n";
							
							// first row
							int firstVoltage;
							if (voltageCounter >= 3){
								outputFile << "... -> ";
								firstVoltage = voltageCounter - 2;
							} else {
								firstVoltage = 0;
							}
							for (int i = firstVoltage; i <= voltageCounter; i++){
								outputFile << voltageVector[i];
								if (i < voltageCounter){
									outputFile << " -> ";
								}
							}
							outputFile << "\n";

							//headers row
							for (int i = 0; i < fingerTaxelValues.size(); i++){
								outputFile << i << " ";
							}
							outputFile << "realVoltageProx voltageProx voltageDist degreesProx degreesDist\n";



							op1Counter++;
						} else if (op1Counter < 50*1){
							op1Counter++;
						} else {
							op1Mode = 3;
							op1Counter = 0;
							cout << "CHANGING VOLTAGE FROM " << op1VoltageToUse << " TO " << voltageVector[voltageCounter] << "\nwaiting 5 sec...\n";
							op1VoltageToUse = voltageVector[voltageCounter];
						}
						op1GlobalCounter++;
						// SCRIVI SU FILE
						for (int i = 0; i < fingerTaxelValues.size(); i++){
							outputFile << fingerTaxelValues[i] << " ";
						}
						double fingerProximalEnc,fingerDistalEnc,fingerProximalOutput,fingerDistalOutput;
						iEncs->getEncoder(jointToMove,&fingerProximalEnc);
						iEncs->getEncoder(jointToMove + 1,&fingerDistalEnc);
						iOLC->getOutput(jointToMove,&fingerProximalOutput);
						iOLC->getOutput(jointToMove + 1,&fingerDistalOutput);
						outputFile << op1VoltageToUse << " " << fingerProximalOutput << " " << fingerDistalOutput << " " << fingerProximalEnc << " " << fingerDistalEnc << "\n";
						// outputFile << op1GlobalCounter << ";" << op1VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << "\n";
					} else if (op1Mode == 3){
						double fingerProximalEnc,fingerDistalEnc,fingerProximalOutput,fingerDistalOutput;
						iEncs->getEncoder(jointToMove,&fingerProximalEnc);
						iEncs->getEncoder(jointToMove + 1,&fingerDistalEnc);
						iOLC->getOutput(jointToMove,&fingerProximalOutput);
						iOLC->getOutput(jointToMove + 1,&fingerDistalOutput);
							
						if (op1Counter < 50*5){
							//if (op1Counter < op1NumMaxVoltage && voltageCounter == 0){
							//	op1VoltageToUse = op1MaxVoltage;
							//} else  {
								op1VoltageToUse = voltageVector[voltageCounter];
							//}
							op1Counter++;
							op1GlobalCounter++;
							//SCRIVI SU FILE
							for (int i = 0; i < fingerTaxelValues.size(); i++){
								outputFile << fingerTaxelValues[i] << " ";
							}
							outputFile << op1VoltageToUse << " " << fingerProximalOutput << " " << fingerDistalOutput << " " << fingerProximalEnc << " " << fingerDistalEnc << "\n";
							// outputFile << op1GlobalCounter << ";" << op1VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << "\n";
						} else {
							voltageCounter++;
							if (voltageCounter < voltageVector.size()){
								op1Mode = 2;
								op1Counter = 0;
							} else {
								op1Mode = -1;
								cout << "THE END\n";
							}
							op1GlobalCounter++;
							//SCRIVI SU FILE
							for (int i = 0; i < fingerTaxelValues.size(); i++){
								outputFile << fingerTaxelValues[i] << " ";
							}
							outputFile << op1VoltageToUse << " " << fingerProximalOutput << " " << fingerDistalOutput << " " << fingerProximalEnc << " " << fingerDistalEnc << "\n";
							// outputFile << op1GlobalCounter << ";" << op1VoltageToUse << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << "\n";
							outputFile.close();
						}
					}

					iOLC->setRefOutput(jointToMove,voltageDirection*op1VoltageToUse);
				
				}
		
			}
		} else if (operationMode == 6){

			op6ControlType = 1;

			if (velocities.grasp.size() > 0){

				deque<bool> contacts (false, nFingers);
				vector<double> maxContacts(nFingers);
				vector<double> sumContacts(nFingers,0.0);
				std::vector<double> fingerTaxelValues;
				if (detectContact(contacts,maxContacts,sumContacts,fingerTaxelValues)){

					if (op4State == 0){

						double jointToMoveEnc;
						iEncs->getEncoder(jointToMove,&jointToMoveEnc);
						op4PosRef = jointToMoveEnc;

						std::ostringstream fileName(std::ostringstream::ate);
						fileName.str("");
							
						fileName << "grasping_data_maxvel" << (int)op4MaxVel << "_kp" << (int)(op4kp * 100) << "_ki" << (int)(op4ki * 100) << "_sum" << sumContactsRif << ".csv";


						outputFile.open(fileName.str().c_str(), std::ofstream::out | std::ofstream::app);

						cout << "FILE " << fileName.str() << " OPENED\n";

						outputFile << "N;kp;Vel;Max;Sum;Deg;IntErr" << "\n";

						op4State = 1;

					} else if (op4State == 1){
						
						double velOrPosDir;

						if (!contacts[fingerToMove]){
							
							op6ControlType = 0;

						} else {

							if (!op4Contact){
								op4Contact = true;
							}

							double error = sumContactsRif - sumContacts[fingerToMove];
							
							//if (op4IntegrErr + error > op4MaxIntegrErr){
							//	op4IntegrErr = op4MaxIntegrErr;
							//} else if (op4IntegrErr + error < op4MaxIntegrErr) {
							//	op4IntegrErr = -op4MaxIntegrErr;
							//} else {
							//	op4IntegrErr += error;
							//}

							op4IntegrErr += error;

							op4VelToUse = op4kp*error + op4ki*op4IntegrErr;
							
							if (op4VelToUse > op4MaxVel){
								op4VelToUse = op4MaxVel;
							}

						}

						double jointToMoveEnc;
						iEncs->getEncoder(jointToMove,&jointToMoveEnc);

						if (op6ControlType == 0){
							iOLC->setRefOutput(jointToMove,op6InitialVoltage);
							velOrPosDir = initialVoltage;
						} else if (op6ControlType == 1){
							iVel->velocityMove(jointToMove,op4VelToUse);
							velOrPosDir = op4VelToUse;
						}
						
						
						if (op4LoggingEnabled){
							outputFile << op4Counter << ";" << op4kp << ";" << velOrPosDir << ";" << maxContacts[fingerToMove] << ";" << sumContacts[fingerToMove] << ";" << jointToMoveEnc << ";" << op4IntegrErr << "\n";
						}
						if (op4Counter%10 == 0){
							cout << sumContacts[fingerToMove] << "\t" << velOrPosDir << "\t" << maxContacts[fingerToMove] << "\t" << jointToMoveEnc << ";" << op4IntegrErr << "\n";
						}

						op4Counter++;

						if (op4Counter == 50*20){
							op4State = 2;
						}
					
					} else if (op4State == 2){

						iVel->velocityMove(jointToMove,0.0);
						
						outputFile.close();
						
						cout << "THE END!\n";
					}


				}
		
			}

		}

}  
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Release thread                                                   ********************************************** */
void GraspThread::threadRelease(void) {
    cout << dbgTag << "Releasing. \n";
    
    // Close ports
    portGraspThreadInSkinComp.interrupt();
    portGraspThreadInSkinRaw.interrupt();
    portGraspThreadInSkinContacts.interrupt();
    portGraspThreadInSkinComp.close();
    portGraspThreadInSkinRaw.close();
    portGraspThreadInSkinContacts.close();

    // Stop interfaces
    if (iVel) {
        iVel->stop();
    }
    if (iPos) {
        iPos->stop();
        // Restore initial robot position
        iPos->positionMove(startPos.data());
    }

    // Close driver
    clientArm.close();

    cout << dbgTag << "Released. \n";
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Detect contact on each finger.                                   ********************************************** */
bool GraspThread::detectContact(std::deque<bool> &o_contacts,std::vector<double> &maxContacts,std::vector<double> &sumContacts,std::vector<double> &fingerTaxelValues) {
    using yarp::sig::Vector;
    using std::deque;
    using std::vector;
	using std::setw;

    Vector *inComp = portGraspThreadInSkinComp.read(false);
    if (inComp) {
        // Convert yarp vector to stl vector
        vector<double> contacts(12*nFingers);
        for (size_t i = 0; i < contacts.size(); ++i) {
            contacts[i] = (*inComp)[i];
        }

        // Find maximum for each finger
        //vector<double> maxContacts(nFingers);
        vector<double>::iterator start;
        vector<double>::iterator end;
        for (int i = 0; i < nFingers; ++i) {
            start = contacts.begin() + 12*i;
            end = start + 11;
			
			for (int j = 12*i; j < 12*(i+1); j++){
				sumContacts[i] += contacts[j];
				if (i == fingerToMove){
					fingerTaxelValues[j - 12*i] = contacts[j];
				}
			}

			if (counter >= 0 && counter < counterMax && i == fingerToMove){
				for (int j = 12*i; j < 12*(i+1); j++){
					dataCollection[j - 12*i][counter] = contacts[j];
				}
				counter ++;
			}

            maxContacts[i] = *std::max_element(start, end);
        }

#ifndef NODEBUG
//        cout << "DEBUG: " << dbgTag << "Maximum contact detected: \t\t";
//        for (size_t i = 0; i < maxContacts.size(); ++i) {
//            cout << maxContacts[i] << " ";
//        }
//        cout << "\n";
		//if (stdLogging) cout << setw(10) << maxContacts[fingerToMove] << " ";

#endif
		
        // Check if contact is greater than threshold
        o_contacts.resize(nFingers, false);
        for (size_t i = 0; i < maxContacts.size(); ++i) {
            o_contacts[i] = usedVoltage == true ? (maxContacts[i] >= thresholdParam - 2) : (maxContacts[i] >= thresholdParam); //touchThresholds[i]);
        }

        // Store previous contacts
        previousContacts = o_contacts;
		previousMaxContact = maxContacts[fingerToMove];
		previousSumContact = sumContacts[fingerToMove];

    } else {
#ifndef NODEBUG
        if (stdLogging) cout << "DEBUG: " << dbgTag << "No skin data. \n";
        if (stdLogging) cout << "DEBUG: " << dbgTag << "Using previous skin value. \n";
#endif
        o_contacts = previousContacts;
		maxContacts[fingerToMove] = previousMaxContact;
		sumContacts[fingerToMove] = previousSumContact;
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Set touch threshold.                                             ********************************************** */
bool GraspThread::setTouchThreshold(const int aFinger, const double aThreshold) {
    
	
	if (aFinger == -1){
		yarp::dev::Pid pid;
		bool pidGot;
		pidGot = iPid->getPid(jointToMove,&pid);
	//	sss
		//	std::ostringstream fileName(std::ostringstream::ate);
			//				fileName.str("");
				//			fileName << (int)op1VoltageToUse << "_" << (int)newVoltage << ".csv";
						
						
					//		outputFile.open(fileName.str(), std::ofstream::out | std::ofstream::app);
						
		std::ostringstream voltageList(std::ostringstream::ate);
		voltageList.str("");
		for(int i = 0; i < voltageVector.size(); i++){
			voltageList << voltageVector[i];
			if (i < voltageVector.size() -1){
				voltageList << " ";
			}
		}


		cout << "-------- HELP ---------" << "\n" <<
				"-1)  this help" << "\n" <<
				"0-4) fingers tactile thresholds" << "\n" <<
				"5) joint velocity (" << jointVelocity << ")" << "\n" <<
				"6) PID max value (" << (pidGot ? pid.max_output : -1.0) << ")" << "\n" <<
				"7) joint to move (" << jointToMove << ")" << "\n" <<
				"8) touch threshold (" << thresholdParam << ")" << "\n" <<
				"9) pressure (" << sumToReach << ")" << "\n" <<
				"10) statistics:" << "\n" <<
				"\t- 0: start logging" << "\n" <<
				"\t- 1: stop logging" << "\n" <<
				"\t- 2: enable statistics loop" << "\n" <<
				"\t- 3: disable statistics loop" << "\n" <<
				"11) counter max value (" << counterMax << ")" << "\n" <<
				"12) logging (" << (stdLogging ? "1" : "0") << ")" << "\n" <<
				"13) kParam (" << kParam << ")" << "\n" <<
				"14) kBackParam (" << kBackParam << ")" << "\n" <<
				"15) kVoltage (" << kVoltage << ")" << "\n" <<
				"16) max voltage (" << maxVoltage << ")" << "\n" <<
				"17) operation mode (" << operationMode << ")" << "\n" <<
				"18) write file" << "\n" <<
				"19) print info" << "\n" <<
				"20) set ref voltage" << "\n" <<
				"21) back mode:" << "\n" <<
				"\t- 0: start" << "\n" <<
				"\t- 1: back" << "\n" <<
				"\t- 2: close" << "\n" <<
				"\t- <0: backVoltage (" << op3BackVoltage << ")" << "\n" <<
				"\t- >100: initialVoltage (" << initialVoltage << ")" << "\n" <<
				"\t- 5<x<100: 10 + backSteps (" << op3BackSteps << ")" << "\n" <<
				"22) vel control mode (11:enable|10:stop)" << "\n" <<
				"\t- 10: stop" << "\n" <<
				"\t- 11: enable vel" << "\n" <<
				"\t- 12: enable pos direct" << "\n" <<
				"\t- 13: enable open loop" << "\n" <<
				"\t- -10000<x<0: -kposdir (" << op4KPosDir << ")" << "\n" <<
				"\t- <-10000: -10000 -kol (" << op4KOL << ")" << "\n" <<
				"\t- 0<x<10: kpp (" << op4kp << ")" << "\n" <<
				"\t- >20: sumRif (" << sumContactsRif << ")" << "\n" <<
				"\t- 200 < x < 900: 200 + max vel (" << op4MaxVel << ")" << "\n" <<
				"\t- 900 < x < 1000: 900 + op4ki (" << op4ki << ")" << "\n" <<
				"\t- 1001/1000 logging enabled/disabled (" << (op4LoggingEnabled ? "enabled" : "disabled") << ")" << "\n" <<
				"\t- >1002: op4MaxIntegrErr (" << op4MaxIntegrErr << ")" << "\n" <<
				"23) voltage direction (+1|-1) (" << voltageDirection << ")" << "\n" <<
				"24) voltage vector ( " << voltageList.str() << ")" << "\n" <<
				"25) set test number ( " << op5TestNumber << ")" << "\n"
				"-----------------------" << "\n";
		return true;
	}
	else if ((aFinger >= 0) && (aFinger < nFingers)) {
        touchThresholds[aFinger] = aThreshold;
        return true;
    } else if (aFinger >= nFingers){
		if (aFinger == 5) {
			jointVelocity = aThreshold;
			cout << "new velocity: " << velocities.grasp[jointToMove - 8] << "\n";
		}
		if (aFinger == 6){
			yarp::dev::Pid pid;
			if (iPid->getPid(jointToMove,&pid)){
				pid.setMaxOut(aThreshold);
				minVoltage = aThreshold;
				if (iPid->setPid(jointToMove,pid)){
					cout << "new pid max output: " << aThreshold << "\n";
				}
			}
		}
		if (aFinger == 7){
			int joint = (int) aThreshold;
			jointToMove = joint;
			cout << "joint to move: " << jointToMove << "\n";
		}
		if (aFinger == 8){
			thresholdParam = aThreshold;
			cout << "new touch threshold: " << aThreshold << "\n";
		}
		if (aFinger == 9){
			sumToReach = aThreshold;
			cout << "new pressure: " << aThreshold << "\n";
		}
		if (aFinger == 10){
			int command = (int) aThreshold;
			if (command == 0){
				counter = 0;
				cout << "statistics logging started!\n";
			}
			if (command == 1){
				stopCounter = true;
				cout << "statistics logging stopped!\n";
			}
			if (command == 2){
				counterLoop = true;
				cout << "statistics loop enabled\n";
			}
			if (command == 3){
				counterLoop = false;
				cout << "statistics loop disabled\n";
			}
		}
		if (aFinger == 11){
			counterMax = (int)aThreshold;
			cout << "counterMax value changed to " << counterMax << "\n";
		}
		if (aFinger == 12){
			stdLogging = ((int)aThreshold) == 0 ? false : true;
			cout << (stdLogging ? "logging enabled" : "logging disabled") << "\n";
		}
		if (aFinger == 13){
			kParam = aThreshold;
			cout << "new kParam: " << aThreshold << "\n";
		}
		if (aFinger == 14){
			kBackParam = aThreshold;
			cout << "new kBackParam: " << aThreshold << "\n";
		}
		if (aFinger == 15){
			kVoltage = aThreshold;
			cout << "new kVoltage: " << aThreshold << "\n";
		}
		if (aFinger == 16){
			maxVoltage = aThreshold;
			cout << "max voltage set to: " << aThreshold << "\n";
		}
		if (aFinger == 17){
			operationMode = (int)aThreshold;
			if (operationMode == 1){
				op1UseVoltage = false;
				op1Mode = 0;
				voltageCounter = 0;
			}
			if (operationMode == 2){
				op2UseVoltage = false;
				op2Mode = 0;
			}
			if (operationMode == 3){
				op3UseVoltage = false;
				op3Mode = 0;
			}
			if (operationMode == 5){
				op1UseVoltage = false;
				op1Mode = 0;
				voltageCounter = 0;
			}
			cout << "operation mode set to: " << operationMode << "\n";
		}
		if (aFinger == 18){
			std::ostringstream fName(std::ostringstream::ate);
			fName.str("");
			fName << "nomeTest_" << (int)aThreshold << ".csv";
						
			outputFile.open(fName.str().c_str(), std::ofstream::out | std::ofstream::app);
			outputFile << "un due tre prova\n" << aThreshold << "\n";
			outputFile.close();
		}
		if (aFinger == 19){
			cout << "voltageVector.size(): " << voltageVector.size() << " values: " << voltageVector[0] << voltageVector[1] << voltageVector[2] << "\n";
		}
		if (aFinger == 20){
			operationMode = -1;
			sampleCounter = 0;
			refVoltage = aThreshold;
			cout << "new voltage set to: " << refVoltage << "\n";
		}
		if (aFinger == 21){
			if (aThreshold > -1 && aThreshold < 5){
				op3CmdState = (int)aThreshold;
				if (op3CmdState == 0){
					op3UseVoltage = false;
				}
				cout << "op3 state set to: " << op3CmdState << "\n";
			} else if (aThreshold < -1){
				op3BackVoltage = aThreshold;
				cout << "back V set to: " << op3BackVoltage << "\n";
			} else if (aThreshold > 100){
				initialVoltage = aThreshold;
				cout << "initial V set to: " << initialVoltage << "\n";
			} else if (aThreshold > 5 && aThreshold < 100){
				op3BackSteps = ((int)aThreshold) - 10;
				cout << "back steps set to: " << op3BackSteps << "\n";
			}
			operationMode = 3;
			
		}
		if (aFinger == 22){
			if (aThreshold > 0 && aThreshold < 5){
				op4kp = aThreshold;
				cout << "op4Kp set to: " << aThreshold << "\n";
			} else if (aThreshold < 0 && aThreshold > -10000){
				op4KPosDir = -aThreshold;
				cout << "op4KPosDir set to: " << op4KPosDir << "\n";
			}  else if (aThreshold < -10000){
				op4KOL = -aThreshold - 10000;
				cout << "op4KOL set to: " << op4KOL << "\n";
			} else if (aThreshold > 20 && aThreshold < 200){
				sumContactsRif = aThreshold;
				cout << "sumContactsRif set to: " << sumContactsRif << "\n";
			} else if (aThreshold > 200 && aThreshold < 900){
				op4MaxVel = aThreshold - 200;
				cout << "op4MaxVel set to: " << op4MaxVel << "\n";
			} else if (aThreshold > 5 && aThreshold < 20){
				int input = ((int)aThreshold) - 10;
				if (input == 3){
					setControlMode(VOCAB_CM_OPENLOOP,true);
					op4ControlMode = 3;
					operationMode = 4;
					op4State = 0;
					op4Counter = 0;
					op4IntegrErr = 0;
					cout << "open loop control mode enabled!" << "\n";
				} else if (input == 2){
					setControlMode(VOCAB_CM_POSITION_DIRECT,true);
					op4ControlMode = 2;
					operationMode = 4;
					op4State = 0;
					op4Counter = 0;
					op4IntegrErr = 0;
					cout << "pos direct control mode enabled!" << "\n";
				} else if (input == 1){
					setControlMode(previousControlMode,true);
					op4ControlMode = 1;
					operationMode = 4;
					op4State = 0;
					op4Counter = 0;
					op4IntegrErr = 0;
					cout << "vel control mode enabled!" << "\n";
				} else if (input == 0){
					op4State = 2;
					cout << "control mode stopped!" << "\n";
				}
			} else if (aThreshold >= 900 && aThreshold < 999){
				op4ki = aThreshold - 900;
				cout << "op4ki set to " << op4ki << "\n";
			} else if (aThreshold > 999 && aThreshold < 1002){
				int input = ((int)aThreshold) - 1000;
				if (input == 1){
					op4LoggingEnabled = true;
					cout << "op4 logging enabled!" << "\n";
				} else if (input == 0){
					op4LoggingEnabled = false;
					cout << "op4 logging disabled!" << "\n";
				}
			} else if (aThreshold > 1002){
				op4MaxIntegrErr = aThreshold;
				cout << "op4MaxIntegrErr set to " << op4MaxIntegrErr << "\n";
			}

		}

		if (aFinger == 23){
			voltageDirection = aThreshold < 0 ? -1 : 1;
			cout << "voltage direction set to: " << voltageDirection << "\n";
		}
		if (aFinger == 24){
			if (aThreshold < 0){
				voltageVector.clear();
			} else {
				voltageVector.push_back(aThreshold);
			}
			
			std::ostringstream voltageList(std::ostringstream::ate);
			voltageList.str("");
			for(int i = 0; i < voltageVector.size(); i++){
				voltageList << voltageVector[i];
				if (i < voltageVector.size() -1){
					voltageList << " ";
				}
			}

			cout << "voltage vectors values: " << voltageList.str() << "\n";
		}
		if (aFinger == 25){
			op5TestNumber = aThreshold;
			cout << "new test number: " << op5TestNumber << "\n";
		}

		return true;
	} else {
        cerr << dbgTag << "RPC::setTouchThreshold() - The specified finger is out of range. \n";
        return false;
    }
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Open hand                                                        ********************************************** */
bool GraspThread::openHand(void) {
    cout << dbgTag << "Opening hand ... \t";
    
    iVel->stop();

    // Set the fingers to the original position
    //iPos->positionMove(11, 5);
    //iPos->positionMove(12, 0);
    //iPos->positionMove(13, 0);
    //iPos->positionMove(14, 0);
    //iPos->positionMove(15, 40);
	
	iPos->positionMove(11, 5);
    iPos->positionMove(12, 19);
    iPos->positionMove(13, 3);
    iPos->positionMove(14, 0);
    iPos->positionMove(15, 17);

    // Check motion done
    waitMoveDone(10, 1);
    cout << "Done. \n";

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Place arm in grasping position                                   ********************************************** */ 
bool GraspThread::reachArm(void) {
    cout << dbgTag << "Reaching for grasp ... \t";
    
    iVel->stop();

    // Set the arm in the starting position
    // Arm
    //iPos->positionMove(0 ,-25);
    //iPos->positionMove(1 , 35);
    //iPos->positionMove(2 , 18);
    //iPos->positionMove(3 , 22);
    //iPos->positionMove(4 ,-67);
    //iPos->positionMove(5 , 9);
    //iPos->positionMove(6 , -5);
    //iPos->positionMove(7 , 20);
    // Hand
    //iPos->positionMove(8 , 90);
    //iPos->positionMove(9 , 30);
    //iPos->positionMove(10, 30);
    //iPos->positionMove(11, 5);
    //iPos->positionMove(12, 0);
    //iPos->positionMove(13, 0);
    //iPos->positionMove(14, 0);
    //iPos->positionMove(15, 40);

	//iPos->positionMove(8 , 60);
	//iPos->positionMove(9 , 36);
	//iPos->positionMove(10, 7);
    iPos->positionMove(11, 5);
    iPos->positionMove(12, 19);
    iPos->positionMove(13, 3);
    iPos->positionMove(14, 0);
    iPos->positionMove(15, 17);

    // Check motion done
    waitMoveDone(10, 1);
    cout << "Done. \n";

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Set the given velocity                                           ********************************************** */
bool GraspThread::setVelocities(const int &i_type, const std::vector<double> &i_vel) {
    switch (i_type) {
        case GraspType::Stop :
            velocities.stop = i_vel;
            break;
        case GraspType::Grasp :
            velocities.grasp = i_vel;
            break;

        default:
            cerr << dbgTag << "Unknown velocity type specified. \n";
            return false;
            break;
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Set the velocity for the given joint.                            ********************************************** */
bool GraspThread::setVelocity(const int &i_type, const int &i_joint, const double &i_vel) {
    if ((i_joint > 0) && (i_joint < nJointsVel)) {
        switch (i_type) {
            case GraspType::Stop :
                velocities.stop[i_joint - 8] = i_vel;
                break;
            case GraspType::Grasp :
                velocities.grasp[i_joint - 8] = i_vel;
                break;

            default:
                cerr << dbgTag << "Unknown velocity type specified. \n";
                return false;
                break;
        }
    } else {
        cerr << dbgTag << "Invalid joint specified. \n";
        return false;
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Generate the mapping of each finger into the controllable joints it contains.  ******************************** */
bool GraspThread::generateJointMap(std::vector<double> &i_thresholds) {
    // FG:  This is probably one of the dirtiest pieces of code I have ever concocted. I found no clean why to map finger IDs
    //      (0,1,2,3,4) into their respective joints in the robotic kinematic chain. I chose this one.
    //
    using std::vector;

    // Initialise map
    jointMap.resize(i_thresholds.size());

    // Loop thresholds
    for (size_t i = 0; i < i_thresholds.size(); ++i) {
        vector<int> tmp;
        if (i_thresholds[i] >= 0) {
            // Create joint list
            if (i == 0) {
                tmp.push_back(11);
                tmp.push_back(12);
            } else if (i == 1) {
                tmp.push_back(13);
                tmp.push_back(14);
            } else if (i == 2) {
                tmp.push_back(15);
            } else if (i == 3) {
                tmp.push_back(15);
            } else if (i == 4) {
                tmp.push_back(8);
                tmp.push_back(9);
                tmp.push_back(10);
            } 
        }
        // Add joint list to map
        jointMap[i] = tmp;
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Wait for motion to be completed.                                 ********************************************** */
bool GraspThread::waitMoveDone(const double &i_timeout, const double &i_delay) {
    using yarp::os::Time;
    
    bool ok = false;
    
    double start = Time::now();
    while (!ok && (start - Time::now() <= i_timeout)) {
        iPos->checkMotionDone(&ok);
        Time::delay(i_delay);
    }

    return ok;
}
/* *********************************************************************************************************************** */

bool GraspThread::getControlMode(int *controlMode){

	return iCtrl->getControlMode(jointToMove,controlMode);
}

bool GraspThread::setControlMode(int controlMode,bool checkCurrent){

	if (controlMode != VOCAB_CM_OPENLOOP){
		op1UseVoltage = false;
		op2UseVoltage = false;
		op3UseVoltage = false;
	}

	if (checkCurrent){
		int currentControlMode;
		if (iCtrl->getControlMode(jointToMove,&currentControlMode)){
			if (currentControlMode != controlMode){
				if  (iCtrl->setControlMode(jointToMove,controlMode)){
					cout << "CONTROL MODE SET TO " << controlMode << "    PREV: " << previousControlMode << "  OPEN: " << VOCAB_CM_OPENLOOP << "\n";
					return true;
				} else return false;	
			} else return true;
		} else return false;
	} else return iCtrl->setControlMode(jointToMove,controlMode);
	
//	iCtrl->setControlMode(12,controlMode);
	//iCtrl->setControlMode(13,controlMode);
//	iCtrl->setControlMode(14,controlMode);
}

void GraspThread::setPreviousControlMode(int previousControlMode){
	this->previousControlMode = previousStepControlMode = previousControlMode;
}
