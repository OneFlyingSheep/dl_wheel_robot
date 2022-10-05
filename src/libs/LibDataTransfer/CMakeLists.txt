find_package( Qt5Widgets REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     LibRCF
	 LibProtoClient
	 LibDLWheelRobotConfigData
)

ADD_LIBRARY(LibDataTransfer ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDataTransfer ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})