IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     LibSocket
     LibDLWheelRobotDBOperation
     LibDLWheelRobotConfigData
     LibDLWheelRobotCreateReport
     LibDLCreateExcel
)

ADD_LIBRARY(LibSocketLogin ${SOURCES})
TARGET_LINK_LIBRARIES(LibSocketLogin ${EXTRA_LIBS})