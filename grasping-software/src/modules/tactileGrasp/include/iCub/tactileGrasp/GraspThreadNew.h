
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


#ifndef __ICUB_TACTILEGRASP_GRASPTHREAD_H__
#define __ICUB_TACTILEGRASP_GRASPTHREAD_H__

#include <iCub/tactileGrasp/TactileGraspEnums.h>

#include <fstream>
#include <string>
#include <vector>
#include <deque>

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/sig/Vector.h>

#include <iCub/skinDynLib/skinContactList.h>

namespace iCub {
    namespace tactileGrasp {
        /**
         * Structure containing the velocities to be used for each grasping movement.
         */
        struct GraspVelocity {
            std::vector<double> grasp;
            std::vector<double> stop;
        };

        /** Typedef representing the mapping of each finger into the controllable joints it contains. */
        typedef std::vector<std::vector<int> > FingerJointMap;

        class GraspThread : public yarp::os::RateThread {
            private:
                /* ****** Module attributes                             ****** */
                int period;
                yarp::os::ResourceFinder rf;


                /* ******* Controllers                                  ******* */
                yarp::dev::PolyDriver clientArm;
                yarp::dev::IEncoders *iEncs;
                yarp::dev::IPositionControl *iPos;
                yarp::dev::IVelocityControl *iVel;
				yarp::dev::IPositionDirect *iPosDir;
				yarp::dev::IPidControl *iPid;
				yarp::dev::IOpenLoopControl *iOLC;
				yarp::dev::IControlMode2 *iCtrl;


                /** Robot arm start position. */
                yarp::sig::Vector startPos;


                /* ******* Contact detection configuration              ******* */
                /** The previous skin contacts. This is stored to avoid herratic behaviour when clocking this thread faster than the skin threads. */
                std::deque<bool> previousContacts;
                /** The touch threshold for each fingertip. */
                std::vector<double> touchThresholds;

                /* ******* Grasp configuration                          ******* */
                /** The grasp velocities. */
                GraspVelocity velocities;
                /** Number of fingers used for the grasping movement. */
                int nFingers;
                /** Total number of joints to be controlled by the velocity interface. This is set by yarp::dev::IVelocityControl::getAxes(). */
                int nJointsVel;
                /** IDs of the joints to be used for the grasping movement. */
                std::vector<int> graspJoints;
                /** Mapping of each finger into the controllable joints it contains. */
                FingerJointMap jointMap;

                
                /* ****** Ports                                         ****** */
                yarp::os::BufferedPort<yarp::sig::Vector> portGraspThreadInSkinComp;
                yarp::os::BufferedPort<yarp::sig::Vector> portGraspThreadInSkinRaw;
                yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> portGraspThreadInSkinContacts;
                

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

				/* ****** Temporary attributes							****** */
				double jointVelocity;
				int jointToMove;
				int previousControlMode;
				int previousStepControlMode;
				bool usedVoltage;
				double previousVoltage;
				double lastPosVoltage;
				double touchToReach;
				double sumToReach;
				double kParam;
				double thresholdParam;
				double kBackParam;
				double kVoltage;
				int counter;
				std::vector<std::vector<double> > dataCollection;
				bool stopCounter,counterLoop;
				bool stdLogging;
				int counterMax;
				int fingerToMove;
				double maxVoltage;
				double minVoltage;
				double previousMaxContact;
				double previousSumContact;

				int sampleCounter;
				double refVoltage;
				int operationMode; // 0: normal | 1: convergency | 2: interpolation | 3: go back smoothly | 4: vel control mode | 5: convergency modified | 6: vel control mode modified | -1: do nothing
				int op1Mode; // 0: no contact | 1: first contact | 2: get element | 3: change voltage | -1: do nothing
				bool op1UseVoltage; // false
				bool op1Impulse; // false
				double initialVoltage; // 250
				double op1MaxVoltage;
				int op1NumMaxVoltage;
				int voltageCounter; // 0
				std::vector<double> voltageVector; //
				int op1Counter;
				int op1GlobalCounter;
				double op1VoltageToUse;
				std::ofstream  outputFile;

				int op2Mode; // 0: no contact | 1: first contact | 2: save data, change voltage and wait for stabilization | 3: collect data for x seconds | -1: do nothing
				double op2VoltageToUse;
				int op2Counter;
				bool op2UseVoltage;
				double op2MaxVoltage;
				double op2VoltageStep;
				double op2EncoderTot;
				double op2VoltageTot;
				double op2MaxContactTot;
				double op2SumContactTot;

				int op3Mode; // 0: go forward for x sec. | 1: collect data | 2: switch V to 0 | 3: collect data | 4: before back V | 5: back V | -1: wait for commands
				bool op3UseVoltage;
				double op3VoltageToUse;
				double op3Counter;
				int op3BackSteps;
				int op3BackVoltage;
				std::ofstream  trackFile;
				std::ofstream averageFile;
				double op3EncoderTot;
				double op3VoltageTot;
				double op3MaxContactTot;
				double op3SumContactTot;
				int op3TrackingNum;
				int op3CmdState; // 0: start | 1: go back | 2: close file
				bool op3AutomaticMode;

				double op4MaxVel;
				double op4VelToUse;
				double sumContactsRif;
				double op4kp;
				double op4KPosDir;
				double op4KOL;
				int op4State;
				int op4Counter;
				std::ofstream detailFile;
				bool op4LoggingEnabled;
				//bool op4VelControlMode; // true: vel control mode, false: position direct control mode
				int op4ControlMode; // 1: vel | 2: position direct | 3: open loop
				bool op4Contact;
				double op4ki;
				double op4IntegrErr;
				double op4MaxIntegrErr;
				double op4PosRef;

				int op5TestNumber;

				int op6ContactState;
				bool op6FirstTimeOpenLoop;
				bool op6FirstTimeVelocity;
				bool op6ControlType; // 0 open-loop | 1 velocity
				double op6InitialVoltage; 

				int voltageDirection; // +1 | -1

            public:
                GraspThread(const int aPeriod, const yarp::os::ResourceFinder &aRf);
                virtual ~GraspThread();

                virtual bool threadInit(void);
                virtual void run(void);
                virtual void threadRelease(void);

                bool setTouchThreshold(const int aFinger, const double aThreshold);

                /**
                 * Set velocities of all joints.
                 *
                 * \param i_type The velocity type (grasp or stop).
                 * \param i_vel The vector of joint velocities
                 * \return True upon success
                 */
                bool setVelocities(const int &i_type, const std::vector<double> &i_vel);

                /**
                 * Set the velocity of the given joint.
                 *
                 * \param i_type The velocity type (grasp or stop).
                 * \param i_joint The join to be set
                 * \param i_vel The vector of joint velocities
                 * \return True upon success
                 */
                bool setVelocity(const int &i_type, const int &i_joint, const double &i_vel);
                bool openHand(void); 

				bool getControlMode(int *controlMode);
				bool setControlMode(int controlMode,bool checkCurrent);
				void setPreviousControlMode(int previousControlMode);

            private:
                bool generateJointMap(std::vector<double> &i_thresholds);

                bool detectContact(std::deque<bool> &o_contacts,std::vector<double> &maxContacts,std::vector<double> &sumContacts,std::vector<double> &fingerTaxelValues);

                bool reachArm(void);

                bool waitMoveDone(const double &i_timeout, const double &i_delay);
        };
    }
}

#endif

