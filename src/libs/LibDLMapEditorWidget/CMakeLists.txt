find_package( Qt5Widgets REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	LibDataTransfer
	LibSmapBuilder
	LibDLSceneView
	LibLaserParamWidget
	LibDLStatusBar
	LibDLHangRailCommonWidget
)

ADD_LIBRARY(LibDLMapEditorWidget ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLMapEditorWidget ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})