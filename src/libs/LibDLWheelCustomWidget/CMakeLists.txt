find_package( Qt5Widgets REQUIRED )
find_package( Qt5Network REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp" "*.ui")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
	LibDLWheelRobotConfigData
	LibDLWheelTaskDeviceDBSearch
)

qt5_wrap_ui(ui_FILES AddTypeAndEquipmentDlg.ui)
#qt5_add_resources( QRCS resource.qrc )

ADD_LIBRARY(LibDLWheelCustomWidget ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLWheelCustomWidget ${Qt5Widgets_LIBRARIES} ${Qt5Network_LIBRARIES} ${EXTRA_LIBS})