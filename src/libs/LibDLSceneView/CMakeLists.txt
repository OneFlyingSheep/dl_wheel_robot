find_package( Qt5Widgets REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp" "*.h" "*.ui")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 LibDlToolItems
	 LibMapReader
	 LibDLRelocateProcesser
	 LibDLWheelRobotCoreSocket
	 LibDLDeviceViewWidget
	 LibDLCollectMap
)

qt5_wrap_ui(ui_FILES DLViewGrabDlg.ui)
qt5_wrap_ui(ui_FILES DLOverfittingDlg.ui)

ADD_LIBRARY(LibDLSceneView ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLSceneView ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})