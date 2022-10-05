find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 LibDLHangRailCommonWidget
	 LibDLWheelRobotConfigData
)

ADD_LIBRARY(LibDLWheelPowerStationEdit ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLWheelPowerStationEdit ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})