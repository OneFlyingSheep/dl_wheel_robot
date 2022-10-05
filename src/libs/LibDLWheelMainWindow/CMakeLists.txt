find_package( Qt5Widgets REQUIRED )
find_package( Qt5WebChannel REQUIRED )
find_package( Qt5WebEngineWidgets REQUIRED )
find_package( Qt5Multimedia REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.h" "*.cpp" "*.hpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 LibDLHangRailCommonWidget
	 LibDLHangRailCommonTools
	 LibDLWheelCustomWidget
	 LibHCNetCamera
	 LibDLWheelTaskDeviceDBSearch
	 LibDLWheelRobotStateShow
	 LibDLWheelRobotCoreSocket
	 #LibDLInfraredTemperature
     LibProtoClient
	 LibDLWheelRobotCreateReport
	 LibDLWheelRobotDeviceCure
	 LibDLBackStageMapWidget
	 LibConvertSymbolsToLua
	 LibDLFixedAreaWidget
	 LibDLWheelCollectMapWidget
	 LibDLWheelCompareDetection
	 LibRobotControlWidget
	 LibAudioClient
	 LibDLCreateExcel
	 LibFftreal
	 LibSpectrum
	 LibZip
)

qt5_add_resources( QRCS resource.qrc )

ADD_LIBRARY(LibDLWheelMainWindow ${SOURCES} ${QRCS}) 
TARGET_LINK_LIBRARIES(LibDLWheelMainWindow ${Qt5Widgets_LIBRARIES} ${Qt5Multimedia_LIBRARIES} ${Qt5WebChannel_LIBRARIES} ${Qt5WebEngineWidgets_LIBRARIES} ${EXTRA_LIBS})