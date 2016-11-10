import iCubInterface
import numpy as np
import yarp
import time
import sys



def main():

 

    joint1StartPos = 18

    #                    0               1   2   3   4   5   6   7   8   9  10  11  12                    13                  14  15
    startingPosEncs = [-44, 18, -4, 39,-14,  2,  2, 18, 10,  0,0,  0,  0,0,0,  0]   
    actionEnabled = True

    VOCAB_CM_POSITION_DIRECT = 1685286768

    dataDumperPortName = "/gpc/log:i"
    iCubIconfigFileName = "iCubInterface.txt"
    

    jointToActuate = 5
    jointsToActuate = [jointToActuate]

    wavePeriod = 0.3
    waveAmplitude = 10
    waveDuration = 0.9
    threadPeriod = 0.02

    #isNewExperiment = False
    #if len(sys.argv) > 1:
    #    if sys.argv[1] == 'new':
    #        isNewExperiment = True 
    


    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()


    encData = iCubI.readEncodersData()

    startingPosition = encData[jointToActuate]

 

    # wait for the user
    raw_input("- press enter to change the control mode -")

    
    # initialize control mode
    iCubI.setControlMode(jointsToActuate,VOCAB_CM_POSITION_DIRECT)



    # generate wave


    waveMean = startingPosition


    timeStep = 0

    while timeStep < waveDuration:
        ratio = timeStep/wavePeriod
        jointTargetPosition = waveMean + waveAmplitude * np.sin(ratio*2*3.14159265)
        timeStep = timeStep + threadPeriod
        #print jointTargetPosition
        time.sleep(threadPeriod)
        if actionEnabled:
            iCubI.setJointPositionDirect(jointToActuate,jointTargetPosition)





    # restore position mode and close iCubInterface
    if actionEnabled:
        iCubI.setPositionMode(jointsToActuate)
        iCubI.setJointPosition(jointToActuate,startingPosition)

    iCubI.closeInterface()
    
		
if __name__ == "__main__":
    main()
