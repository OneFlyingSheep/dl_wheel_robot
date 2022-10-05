IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 LibDLWheelRobotDBOperation
	 LibDLWheelRobotConfigData
	 LibDLWheelCountDateTime
)

ADD_LIBRARY(LibDLWheelTaskDeviceDBSearch ${SOURCES})
TARGET_LINK_LIBRARIES(LibDLWheelTaskDeviceDBSearch ${EXTRA_LIBS})