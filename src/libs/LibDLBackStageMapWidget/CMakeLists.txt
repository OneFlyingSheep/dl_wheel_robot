find_package( Qt5Widgets REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 LibDlToolItems
	 LibMapReader
	 LibQJson
	 LibMapReaderInfoWidget
	 LibDLWheelRobotCoreSocket
	 LibDLWheelRobotDBOperation
	 LibDLDeviceViewWidget
)

ADD_LIBRARY(LibDLBackStageMapWidget ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLBackStageMapWidget ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})