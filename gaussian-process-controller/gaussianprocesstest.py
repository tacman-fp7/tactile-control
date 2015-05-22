import gp_controller as gpc
import iCubInterface
import numpy as np
import yarp
import time

def exitModule(state):
   
    # TODO
    return False

def main():

    # module parameters
    maxIterations = 10
    #                    0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15 
    startingPosEncs = [-30, 23,  0, 19,-14,  3,-20, 14, 50,  0, 15,  0,  0,  0, 15,  0]   
    
    finger_1 = 1
    proximalJoint_1 = 13
    distalJoint_1 = 14

    finger_2 = 4
    proximalJoint_2 = 9
    distalJoint_2 = 10

    refAcceleration = 100000
    actionDuration = 0.04
    pauseDuration = 0.06
    actionRefreshRate = 0.02
    
    dataDumperPortName = "/gpc/log:i"
    iCubIconfigFileName = "iCubInterface"
    gpConfigFileSuffix = "_example"

    actionApplicationNum = int(actionDuration/actionRefreshRate)    
    jointsToActuate = [proximalJoint_1,proximalJoint_2]
    
    # load gaussian process controller
    gp = gpc.GPController(gpConfigFileSuffix)
    gp.load_controller()

    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()

    # set start position
    iCubI.setArmPosition(startingPosEncs)

    # initialize velocity mode
    iCubI.setVelocityMode(jointsToActuate)
    iCubI.setRefAcceleration(jointsToActuate,refAcceleration)

    iterCounter = 0
    exit = False
    # main loop
    while iterCounter < maxIterations and not exit:

        # read tactile data
        fullTactileData = iCubI.readTactileData()
        tactileData = []              
        for j in range(12):
            tactileData.append(fullTactileData.get(12*finger_1+j).asDouble())
        for j in range(12):
            tactileData.append(fullTactileData.get(12*finger_2+j).asDouble())
        # read encoders data
        fullEncodersData = iCubI.readEncodersData()
        encodersData = []
        encodersData.append(fullEncodersData[proximalJoint_1])
        encodersData.append(fullEncodersData[proximalJoint_2])
        encodersData.append(fullEncodersData[distalJoint_1])
        encodersData.append(fullEncodersData[distalJoint_2])
        state = [tactileData,encodersData]

        # choose action
        action = gp.get_control(state)

        # apply action and keep it applied for 'actionDuration' seconds
        # in case of velocity mode, it should be refreshed at least every 100 ms        
        for k in range(actionApplicationNum):
            iCubI.velocityCommand(proximalJoint_1,action[0])        
            iCubI.velocityCommand(proximalJoint_2,action[1])        
            time.sleep(actionRefreshRate)
        time.sleep(actionDuration - actionRefreshRate*actionApplicationNum)
        
        # pause the system and wait for stabilization
        iCubI.stopMoving(jointsToActuate)
        time.sleep(pauseDuration)
 
        # log data
        iCubI.logData(tactileData + encodersData + [action[0],action[1]])

        iterCounter = iterCounter + 1
        exit = exitModule(state)

    # restore position mode and close iCubInterface
    iCubI.setPositionMode(jointsToActuate)
    iCubI.closeInterface()
 
		
if __name__ == "__main__":
    main()
