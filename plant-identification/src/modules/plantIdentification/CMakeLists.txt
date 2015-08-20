# Copyright: 2015 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Massimo Regoli
# email:   massimo.regoli@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 


set(MODULENAME plantIdentification)

# Search for source code.
set(INC_HEADERS
    include/iCub/plantIdentification/PlantIdentificationModule.h
    include/iCub/plantIdentification/PlantIdentificationEnums.h

    include/iCub/plantIdentification/thread/TaskThread.h

    include/iCub/plantIdentification/task/Task.h
    include/iCub/plantIdentification/task/StepTask.h
    include/iCub/plantIdentification/task/ControlTask.h
    include/iCub/plantIdentification/task/RampTask.h
    include/iCub/plantIdentification/task/ApproachTask.h

    include/iCub/plantIdentification/data/LogData.h
    include/iCub/plantIdentification/data/TaskData.h
    include/iCub/plantIdentification/data/RPCCommandsData.h

    include/iCub/plantIdentification/util/ControllersUtil.h
    include/iCub/plantIdentification/util/PortsUtil.h
    include/iCub/plantIdentification/util/RPCCommandsUtil.h
    include/iCub/plantIdentification/util/ICubUtil.h

)

set(INC_SOURCES

    main.cpp

    PlantIdentificationModule.cpp

    thread/TaskThread.cpp

    task/Task.cpp
    task/StepTask.cpp
    task/ControlTask.cpp
    task/RampTask.cpp
    task/ApproachTask.cpp

    data/LogData.cpp
    data/TaskData.cpp
    data/RPCCommandsData.cpp

    util/ControllersUtil.cpp
    util/PortsUtil.cpp
    util/RPCCommandsUtil.cpp
    util/ICubUtil.cpp

)

source_group("Source Files" FILES ${INC_SOURCES})
source_group("Header Files" FILES ${INC_HEADERS})

# Include directories
include_Directories(${skinDynLib_INCLUDE_DIRS})
include_directories(include ${iKin_INCLUDE_DIRS})

add_executable(${MODULENAME} ${INC_SOURCES} ${INC_HEADERS})
target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} skinDynLib icubmod iKin)

if(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin/${CMAKE_BUILD_TYPE})
else(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif(WIN32)
