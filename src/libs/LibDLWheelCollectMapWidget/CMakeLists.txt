find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 LibDLHangRailCommonWidget
	LibDLMapEditorWidget
)

ADD_LIBRARY(LibDLWheelCollectMapWidget ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLWheelCollectMapWidget ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})