import iCubInterface
import numpy as np
import yarp
import time
import sys

def reachPosition(jointToMove,pwm,actionSteps,velocity,kpGain,targetPos,maxIterations,logEnabled,iCubI,fd):
   
    fullEncodersData = iCubI.readEncodersData()
    startingEncoderValue = fullEncodersData[jointToMove]

    if targetPos > startingEncoderValue:
        scale = 1
    else:
        scale = -1

    pwmToUse = kpGain*scale*velocity
    iCubI.openLoopCommand(jointToMove,pwmToUse)

    if logEnabled:
        fd.write(str(0.00))
        fd.write(" ")
        fd.write(str(0))
        fd.write(" ")
        fd.write(str(startingEncoderValue))
        fd.write("\n")

    encoderValue = startingEncoderValue

    iterCounter = 0
    while iterCounter < maxIterations and scale*encoderValue < scale*targetPos:

        currentTarget = encoderValue + velocity;
        intIterCounter = 0
        while intIterCounter < actionSteps:

            time.sleep(0.01)

            fullEncodersData = iCubI.readEncodersData()
            encoderValue = fullEncodersData[jointToMove]

            pwmToUse = kpGain*scale*(currentTarget - encoderValue)
            iCubI.openLoopCommand(jointToMove,pwmToUse)

            if logEnabled:
                fd.write(str(iterCounter*0.10))
                fd.write(" ")
                fd.write(str(encoderValue-startingEncoderValue))
                fd.write(" ")
                fd.write(str(encoderValue))
                fd.write("\n")

            if logEnabled:
                print iterCounter*0.10,encoderValue-startingEncoderValue,encoderValue

            intIterCounter = intIterCounter + 1

        iterCounter = iterCounter + 1

    iCubI.openLoopCommand(jointToMove,0.0)

    if iterCounter == maxIterations:
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
    pwm = 400
    
    fd = open("direction.txt","w")

    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()

    # set position mode
    iCubI.setPositionMode([jointToMove])

    # put finger in startingPosition1
    iCubI.setJointPosition(jointToMove,startingPosition1)

    # wait for the user
    raw_input("- press enter to move the finger -")

    # initialize open loop mode
    iCubI.setOpenLoopMode([jointToMove])

    # move finger from startingPosition1 to targetPosition
    reachPosition(jointToMove,pwm,targetPosition,maxIterations,False,iCubI,fd)

    time.sleep(0.5)

    # move finger from targetPosition to startingPosition2
    reachPosition(jointToMove,pwm,startingPosition2,maxIterations,True,iCubI,fd)

    # set position mode
#    iCubI.setPositionMode([jointToMove])

    # put finger in startingPosition2
#    iCubI.setJointPosition(jointToMove,startingPosition2)

    time.sleep(0.5)

    # initialize open loop mode
#    iCubI.setOpenLoopMode([jointToMove])

    # move finger from startingPosition2 to targetPosition
    reachPosition(jointToMove,pwm,targetPosition,maxIterations,False,iCubI,fd)

    time.sleep(0.5)

    # move finger from targetPosition to startingPosition2
    reachPosition(jointToMove,pwm,startingPosition2,maxIterations,True,iCubI,fd)

    fd.close()

    # restore position mode and close iCubInterface
    iCubI.setPositionMode([jointToMove])
    iCubI.closeInterface()
 
		
if __name__ == "__main__":
    main()
