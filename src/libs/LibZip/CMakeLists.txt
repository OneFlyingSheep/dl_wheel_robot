IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp" "*.h")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     LibDLWheelRobotConfigData
     LibDLWheelRobotDBOperation
)

ADD_LIBRARY(LibZip ${SOURCES})
TARGET_LINK_LIBRARIES(LibZip  ${EXTRA_LIBS})