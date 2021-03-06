# Copyright: 2015 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Massimo Regoli
# email:   massimo.regoli@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 


set(MODULENAME stableGrasp)

# Search for source code.
set(INC_HEADERS
    include/iCub/plantIdentification/PlantIdentificationModule.h
    include/iCub/plantIdentification/PlantIdentificationEnums.h

    include/iCub/plantIdentification/thread/TaskThread.h
    include/iCub/plantIdentification/thread/EventsThread.h
	
    include/iCub/plantIdentification/task/Task.h
    include/iCub/plantIdentification/task/StepTask.h
    include/iCub/plantIdentification/task/ControlTask.h
    include/iCub/plantIdentification/task/RampTask.h
    include/iCub/plantIdentification/task/ApproachTask.h

    include/iCub/plantIdentification/data/LogData.h
    include/iCub/plantIdentification/data/GMMData.h
    include/iCub/plantIdentification/data/TaskData.h
    include/iCub/plantIdentification/data/RPCCommandsData.h

    include/iCub/plantIdentification/util/ControllersUtil.h
    include/iCub/plantIdentification/util/PortsUtil.h
    include/iCub/plantIdentification/util/RPCCommandsUtil.h
    include/iCub/plantIdentification/util/ICubUtil.h
    include/iCub/plantIdentification/util/MLUtil.h

    include/iCub/plantIdentification/action/WaveAction.h

)

set(INC_SOURCES

    main.cpp

    PlantIdentificationModule.cpp

    thread/TaskThread.cpp
    thread/EventsThread.cpp

    task/Task.cpp
    task/StepTask.cpp
    task/ControlTask.cpp
    task/RampTask.cpp
    task/ApproachTask.cpp

    data/LogData.cpp
    data/GMMData.cpp
    data/TaskData.cpp
    data/RPCCommandsData.cpp

    util/ControllersUtil.cpp
    util/PortsUtil.cpp
    util/RPCCommandsUtil.cpp
    util/ICubUtil.cpp
    util/MLUtil.cpp

    action/WaveAction.cpp

)

source_group("Source Files" FILES ${INC_SOURCES})
source_group("Header Files" FILES ${INC_HEADERS})

# Include directories
include_Directories(${ctrlLib_INCLUDE_DIRS} ${skinDynLib_INCLUDE_DIRS})
include_directories(include ${iKin_INCLUDE_DIRS})

add_executable(${MODULENAME} ${INC_SOURCES} ${INC_HEADERS})
target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ctrlLib skinDynLib iKin ${Gurls_LIBRARIES})

if(WIN32)
    install(TARGETS ${MODULENAME} DESTINATION bin/${CMAKE_BUILD_TYPE})
else(WIN32)
    install(TARGETS ${MODULENAME} DESTINATION bin)
endif(WIN32)
