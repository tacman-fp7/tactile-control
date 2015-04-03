# Copyright: 2015 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Massimo Regoli
# email:   massimo.regoli@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 


set(MODULENAME objectGrasping)

# Search for source code.
set(INC_HEADERS
    include/iCub/objectGrasping/ObjectGraspingModule.h
    include/iCub/objectGrasping/ObjectGraspingEnums.h

    include/iCub/objectGrasping/thread/TaskThread.h

    include/iCub/objectGrasping/task/Task.h
    include/iCub/objectGrasping/task/StepTask.h
    include/iCub/objectGrasping/task/ControlTask.h
    include/iCub/objectGrasping/task/RampTask.h

    include/iCub/objectGrasping/data/LogData.h
    include/iCub/objectGrasping/data/TaskData.h
    include/iCub/objectGrasping/data/RPCCommandsData.h

    include/iCub/objectGrasping/util/ControllersUtil.h
    include/iCub/objectGrasping/util/PortsUtil.h
    include/iCub/objectGrasping/util/RPCCommandsUtil.h

)

set(INC_SOURCES

    main.cpp

    ObjectGraspingModule.cpp

    thread/TaskThread.cpp

    task/Task.cpp
    task/StepTask.cpp
    task/ControlTask.cpp
    task/RampTask.cpp

    data/LogData.cpp
    data/TaskData.cpp
    data/RPCCommandsData.cpp

    util/ControllersUtil.cpp
    util/PortsUtil.cpp
    util/RPCCommandsUtil.cpp
)

source_group("Source Files" FILES ${INC_SOURCES})
source_group("Header Files" FILES ${INC_HEADERS})

# Include directories
include_Directories(${skinDynLib_INCLUDE_DIRS})
include_directories(include)

add_executable(${MODULENAME} ${INC_SOURCES} ${INC_HEADERS})
target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} skinDynLib icubmod)

if(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin/${CMAKE_BUILD_TYPE})
else(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif(WIN32)
