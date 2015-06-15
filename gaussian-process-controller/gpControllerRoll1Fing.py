import gp_controller as gpc
import iCubInterface
import numpy as np
import yarp
import time
import random
import os

def exitModule(resetProbability):
    randomNum = random.random()
    if randomNum < resetProbability:
        return True
    return False

def logArray(array,fd):
    for i in range(len(array)):
        fd.write(str(array[i]))
        fd.write(" ")

def readValueFromFile(fileName):
    fd = open(fileName,"r")
    line = fd.readline().split()
    value = int(line[0])
    fd.close()
    return value

def writeIntoFile(fileName,string):
    fd = open(fileName,"w")
    fd.write(string)
    fd.close()

def addDescriptionData(dataString,parameter,value):
    dataString = dataString + parameter + " " + value + "\n"

def main():

    # module parameters
    maxIterations = [    77,    14,   134,    66,    10,    81,    22,    31,     3,    66]

    proximalJointStartPos = 40
    distalJointStartPos = 0
    #                    0   1   2   3   4   5   6   7   8   9  10  11  12                    13                  14  15
    startingPosEncs = [-44, 18, -4, 39,-14,  2,  2, 18, 12, 20,163,  0,  0,proximalJointStartPos,distalJointStartPos,  0]   
    
    finger = 1
    proximalJoint = 13
    distalJoint = 14
    proximalJointEnc = 6
    distalJointEnc_1 = 7
    distalJointEnc_2 = 8

    resetProbability = 0.02

    actionDuration = 0.1
    pauseDuration = 0.0

    normalizedMaxVoltageY = 1.0
    maxVoltageProxJointY = 250.0
    maxVoltageDistJointY = 500.0
    slopeAtMaxVoltageY = 1.0

    waitTimeForFingersRepositioning = 0.0

    dataDumperPortName = "/gpc/log:i"
    iCubIconfigFileName = "iCubInterface.txt"
    inputFilePath = "./"
    initInputFileName = "controller_init.txt"
    standardInputFileName = "controller_input.txt"
    outputFilePath = "./"
    outputFileName = "controller_output.txt"
    dataPath = "./data/experiments/"
  
    jointsToActuate = [proximalJoint,distalJoint]
    
    fileNameIterID = "iterationID.txt"
    fileNameExpParams = "parameters.txt"

    # create output folder name
    expID = 8
    experimentFolderName = dataPath + "exp_" + str(expID) + "/" # could be changed adding more information about the experiment

    if os.path.exists(experimentFolderName):
        # get iteration ID
        iterID = readValueFromFile(fileNameIterID)        
        writeIntoFile(fileNameIterID,str(iterID+1))    
        inputFileFullName = inputFilePath + standardInputFileName
        rolloutsNum = 10
    else:
        # create directory, create an experiment descrition file and reset iteration ID
        os.mkdir(experimentFolderName)
        descriptionData = ""
        descriptionData = descriptionData + "proximalJointMaxVoltage " + str(maxVoltageProxJointY) + "\n"
        descriptionData = descriptionData + "distalJointMaxVoltage " + str(maxVoltageDistJointY) + "\n"
        descriptionData = descriptionData + "slopeAtMaxVoltage " + str(slopeAtMaxVoltageY) + "\n"
        descriptionData = descriptionData + "actionDuration " + str(actionDuration) + "\n"
        descriptionData = descriptionData + "pauseDuration " + str(pauseDuration) + "\n"
        descriptionData = descriptionData + "finger " + str(finger) + "\n"
        descriptionData = descriptionData + "jointActuated " + str(proximalJoint) + " " + str(distalJoint) + "\n"
        descriptionData = descriptionData + "jointStartingPositions " + str(proximalJointStartPos) + " " + str(distalJointStartPos) + "\n"
        descriptionData = descriptionData + "resetProbabilty " + str(resetProbability) + "\n"
        descriptionData = descriptionData + "additionaNotes " + "" + "\n"
        writeIntoFile(experimentFolderName + fileNameExpParams,descriptionData)
        iterID = 0
        writeIntoFile(fileNameIterID,"1")
        inputFileFullName = inputFilePath + initInputFileName
        rolloutsNum = 30

    outputInputFileSuffix = str(expID) + "_" + str(iterID);
    backupOutputFileFullName = experimentFolderName + "contr_out_" + outputInputFileSuffix + ".txt"
    backupInputFileFullName = experimentFolderName + "contr_in_" + outputInputFileSuffix + ".txt"
    outputFileFullName = outputFilePath + outputFileName
    fd = open(outputFileFullName,"w")

    # calculate voltageX-voltageY mapping parameters (voltageY = k*(voltageX^(1/3)))
    k = pow(3*slopeAtMaxVoltageY*(pow(normalizedMaxVoltageY,2)),(1/3.0))

    maxVoltageX = pow(normalizedMaxVoltageY/k,3)

    # load gaussian process controller
    gp = gpc.GPController(inputFileFullName)
    gp.load_controller()

    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()

    # set start position
#    iCubI.setArmPosition(startingPosEncs)

    # wait for the user
    raw_input("- press enter to start the controller -")

    fd.write("nrollouts: ")
    fd.write(str(rolloutsNum))
    fd.write("\n")
    fd.close()
    
    # initialize velocity mode
#    iCubI.setOpenLoopMode(jointsToActuate)

    rolloutsCounter = 0
    while rolloutsCounter < rolloutsNum:

        print "starting iteration n. ",rolloutsCounter + 1
        fd = open(outputFileFullName,"a")
        fd.write("# HEADER ")
        fd.write(str(rolloutsCounter + 1))
        fd.write("\n")

        iterCounter = 0
        exit = False
        voltage = [0,0]
        realVoltage = [0,0]
        # main loop
        while iterCounter < maxIterations[rolloutsCounter%10] and not exit:

            # read tactile data
            fullTactileData = iCubI.readTactileData()
            tactileData = []              
            for j in range(12):
                tactileData.append(fullTactileData.get(12*finger+j).asDouble())

            # read encoders data from port
            fullEncodersData = iCubI.readEncodersDataFromPort()
            encodersData = []
            encodersData.append(fullEncodersData.get(proximalJointEnc).asDouble())
            encodersData.append(fullEncodersData.get(distalJointEnc_1).asDouble())
            encodersData.append(fullEncodersData.get(distalJointEnc_2).asDouble())

            state = [tactileData,encodersData,voltage]

            # choose action
            action = gp.get_control(state)

            # log data
            iCubI.logData(tactileData + encodersData + voltage + [action[0],action[1]])
            logArray(tactileData,fd)
            logArray(encodersData,fd)
            logArray(voltage,fd)
            logArray(action,fd)
            fd.write("\n")

            # update and cut voltage
            voltage[0] = voltage[0] + action[0];
            voltage[1] = voltage[1] + action[1];
            if abs(voltage[0]) > maxVoltageX:
                voltage[0] = maxVoltageX*np.sign(voltage[0])
            if abs(voltage[1]) > maxVoltageX:
                voltage[1] = maxVoltageX*np.sign(voltage[1])

            # calculate real applied voltage
            realVoltage[0] = maxVoltageProxJointY*k*pow(abs(voltage[0]),1/3.0)*np.sign(voltage[0])
            realVoltage[1] = maxVoltageDistJointY*k*pow(abs(voltage[1]),1/3.0)*np.sign(voltage[1])

            # voltage safety check (it should never happen!)
            if abs(realVoltage[0]) > maxVoltageProxJointY:
                realVoltage[0] = maxVoltageProxJointY*np.sign(realVoltage[0])
                print 'warning, voltage out of bounds!'
            if abs(realVoltage[1]) > maxVoltageDistJointY:
                realVoltage[1] = maxVoltageDistJointY*np.sign(realVoltage[1])
                print 'warning, voltage out of bounds!'


            # apply action
#            iCubI.openLoopCommand(proximalJoint,realVoltage[0])        
#            iCubI.openLoopCommand(distalJoint,realVoltage[1])        
            time.sleep(actionDuration)

            # wait for stabilization
            time.sleep(pauseDuration)
 
            iterCounter = iterCounter + 1
            exit = False #exitModule(resetProbability)

        fd.close()
        print "finger ripositioning..."
        # finger repositioning
#        iCubI.setPositionMode(jointsToActuate)
#        iCubI.setJointPosition(proximalJoint,0.0)
#        iCubI.setJointPosition(distalJoint,0.0)
        time.sleep(waitTimeForFingersRepositioning)
#        iCubI.setJointPosition(proximalJoint,proximalJointStartPos)
#        iCubI.setJointPosition(distalJoint,distalJointStartPos)
        time.sleep(waitTimeForFingersRepositioning)
#        iCubI.setOpenLoopMode(jointsToActuate)
        print "...done"
        rolloutsCounter = rolloutsCounter + 1
            
    os.system("cp " + inputFileFullName + " " + backupInputFileFullName)
    os.system("cp " + outputFileFullName + " " + backupOutputFileFullName)

    # copy input and output file
    # restore position mode and close iCubInterface
#    iCubI.setPositionMode(jointsToActuate)
    iCubI.closeInterface()
 
		
if __name__ == "__main__":
    main()
