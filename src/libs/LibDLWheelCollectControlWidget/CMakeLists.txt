find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.h" "*.cpp" "*.ui")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 LibDLHangRailCommonWidget
	 LibDLHangRailConfigData
	 LibDLHangRailHardwareCtrl
	 LibDLWheelCustomWidget
	 LibDLWheelRobotConfigData
	 LibDLWheelTaskDeviceDBSearch
	 LibDLCollectMap
	 LibProtoClient
	 LibHCNetCamera
)

qt5_wrap_ui(ui_FILES CollectStandardizationWidget.ui)

ADD_LIBRARY(LibDLWheelCollectControlWidget ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLWheelCollectControlWidget CalculatePT ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})