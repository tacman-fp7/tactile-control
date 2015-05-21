import gp_controller as gpc
import iCubInterface
import numpy as np
import yarp
import time


def main():

    # module parameters
    numIterations = 10
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
    
    actionApplicationNum = actionDuration/actionRefreshRate    
    jointsToActuate = [proximalJoint_1,proximalJoint_2]
    
    # load gaussian process controller
    gp = gpc.GPController('_example')
    gp.load_controller()

    # load iCub interface
    iCubI = iCubInterface.ICubInterface('iCubInterface')
    iCubI.loadInterfaces()

    # initialize velocity mode
    iCubI.setVelocityMode(jointsToActuate)
    iCubI.setRefAcceleration(jointsToActuate,refAcceleration)

    # main loop
    for i in range(numIterations):

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
        actionValues = gp.get_control(state)

        # TO REMOVE
        actionValues[0] = actionValues[0]*100
        actionValues[1] = actionValues[1]*100

        # apply action and keep it applied for 'actionDuration' seconds        
        for k in range(int(actionDuration/actionRefreshRate)):
            iCubI.velocityCommand(proximalJoint_1,actionValues[0])        
            iCubI.velocityCommand(proximalJoint_2,actionValues[1])        
            time.sleep(actionRefreshRate)
        time.sleep(actionDuration - actionRefreshRate*actionApplicationNum)
        
        # pause the system
        iCubI.stopMoving(jointsToActuate)
        # wait for the system to stabilize
        time.sleep(pauseDuration)
 
        # log data
        iCubI.logData(tactileData + encodersData + [actionValues[0]] + [actionValues[1]])

    iCubI.setPositionMode(jointsToActuate)
    iCubI.closeInterface()
 
		
if __name__ == "__main__":
    main()
