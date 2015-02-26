# Copyright: 2015 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Massimo Regoli
# email:   massimo.regoli@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 


set(MODULENAME plantIdentification)

# Search for source code.
set(INC_HEADERS
    idl/include/plantIdentification_IDLServer.h
    include/iCub/plantIdentification/PlantIdentificationModule.h
    include/iCub/plantIdentification/PlantIdentificationEnums.h

    include/iCub/plantIdentification/thread/TaskThread.h

    include/iCub/plantIdentification/task/Task.h
    include/iCub/plantIdentification/task/StepTask.h
    include/iCub/plantIdentification/task/ControlTask.h
    include/iCub/plantIdentification/task/RampTask.h

    include/iCub/plantIdentification/data/LogData.h
    include/iCub/plantIdentification/data/TaskData.h

    include/iCub/plantIdentification/util/ControllersUtil.h
    include/iCub/plantIdentification/util/PortsUtil.h

)

set(INC_SOURCES

    idl/src/plantIdentification_IDLServer.cpp

    main.cpp

    PlantIdentificationModule.cpp

    thread/TaskThread.cpp

    task/Task.cpp
    task/StepTask.cpp
    task/ControlTask.cpp
    task/RampTask.cpp

    data/LogData.cpp
    data/TaskData.cpp

    util/ControllersUtil.cpp
    util/PortsUtil.cpp
)

# Search for thrift files
set(IDL ${MODULENAME}.thrift)
yarp_idl_to_dir(${IDL} ${CMAKE_CURRENT_SOURCE_DIR}/idl)


source_group("Source Files" FILES ${INC_SOURCES})
source_group("Header Files" FILES ${INC_HEADERS})
source_group("IDL Files"    FILES ${IDL})

# Include directories
include_Directories(${skinDynLib_INCLUDE_DIRS})
include_directories(include)
include_directories(idl/include)

add_executable(${MODULENAME} ${INC_SOURCES} ${INC_HEADERS} ${IDL})
target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} skinDynLib icubmod)

if(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin/${CMAKE_BUILD_TYPE})
else(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif(WIN32)
