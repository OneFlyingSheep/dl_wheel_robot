find_package( Qt5Widgets REQUIRED )
find_package( Qt5Network REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp" "*.ui")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
	LibProtoClient
	LibDLWheelRobotCoreSocket
	LibDLWheelTaskDeviceDBSearch
	LibDLHangRailCommonWidget
	LibDLWheelRobotCreateReport
	LibDLWheelRobotConfigData
	LibDLWheelCustomWidget
	LibZip
)

qt5_wrap_ui(ui_FILES AddTypeAndEquipmentDlg.ui)
#qt5_add_resources( QRCS resource.qrc )

ADD_LIBRARY(LibDLWheelPatrolResultBrowse ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLWheelPatrolResultBrowse ${Qt5Widgets_LIBRARIES} ${Qt5Network_LIBRARIES} ${EXTRA_LIBS})