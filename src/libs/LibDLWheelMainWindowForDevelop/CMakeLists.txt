find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	LibDLHangRailCommonWidget
	LibDLWheelCollectControlWidget
	LibDLHangUserControl
	LibHCNetCamera
	LibDLWheelRobotCoreSocket
	LibDLWheelRobotDBOperation
	LibDLWheelCollectMapWidget
	LibDLWheelRobotStateShow
	#LibDLInfraredTemperature
	LibDLWheelPowerStationEdit
	LibDLWheelRobotConfigData
	LibProtoClient
	LibDLLuaScript
	LibDataTransfer
	WheelTaskManageWidget
	LibDLWheelPatrolResultBrowse
)

qt5_add_resources( QRCS resource.qrc )

ADD_LIBRARY(LibDLWheelMainWindowForDevelop ${SOURCES} ${QRCS}) 
TARGET_LINK_LIBRARIES(LibDLWheelMainWindowForDevelop ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})