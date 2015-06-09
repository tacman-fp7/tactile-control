import iCubInterface
import numpy as np
import yarp
import time
import sys

def reachPosition(jointToMove,pwm,targetPos,maxIterations,logEnabled):
   
    fullEncodersData = iCubI.readEncodersData()
    startingEncoderValue = fullEncodersData[jointToMove]

    if logEnabled:
        print 0,0,startingEncoderValue
    
    iCubI.openLoopCommand(jointToMove,pwm)

    iterCounter = 0
    while iterCounter < maxIterations and encoderValue < targetPosition:

        time.sleep(0.01)

        fullEncodersData = iCubI.readEncodersData()
        encoderValue = fullEncodersData[jointToMove]

        if logEnabled:
            print iterCounter*0.10,encoderValue-startingEncoderValue,encoderValue

        iterCounter = iterCounter + 1

    iCubI.openLoopCommand(jointToMove,0.0)

    if iterCounter == maxIterations
        print 'reaching ',targetPos,' failed failed'
        sys.exit()



def main():

    # module parameters
    dataDumperPortName = "/gpc/log:i"
    iCubIconfigFileName = "iCubInterface"
    jointToMove = 13
    # startinPosition1 < targetPosition < startingPosition2
    startingPosition1 = 10
    startingPosition2 = 50
    targetPosition = 30
    maxIterations = 500
    pwm = 200
    

    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()

    # set position mode
    iCubI.setPositionMode([jointsToMove])

    # put finger in startingPosition1
    setJointPosition(jointToMove,startingPosition1)

    # wait for the user
    raw_input("- press enter to move the finger -")

    # initialize open loop mode
    iCubI.setOpenLoopMode([jointToMove])

    # move finger from startingPosition1 to targetPosition
    reachPosition(jointToMove,pwm,targetPosition,maxIterations,False)

    # move finger from targetPosition to startingPosition2
    reachPosition(jointToMove,pwm,startingPosition2,maxIterations,True)

    # set position mode
    iCubI.setPositionMode([jointsToMove])

    # put finger in startingPosition2
    setJointPosition(jointToMove,startingPosition2)

    # wait for the user
    raw_input("- press enter to move the finger -")

    # initialize open loop mode
    iCubI.setOpenLoopMode([jointToMove])

    # move finger from startingPosition2 to targetPosition
    reachPosition(jointToMove,-pwm,targetPosition,maxIterations,False)

    # move finger from targetPosition to startingPosition2
    reachPosition(jointToMove,pwm,startingPosition2,maxIterations,True)

    # restore position mode and close iCubInterface
    iCubI.setPositionMode(jointsToMove)
    iCubI.closeInterface()
 
		
if __name__ == "__main__":
    main()
