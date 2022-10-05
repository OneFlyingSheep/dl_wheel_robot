find_package( Qt5Widgets REQUIRED )
find_package( Qt5MultimediaWidgets REQUIRED )
find_package( Qt5Multimedia REQUIRED )
find_package( Qt5PrintSupport )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.h" "*.cpp" "*.ui")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	#LibDLNewInfraredSDK
	LibDLHangRailCommonTools
	LibDLWheelRobotConfigData
	LibDLHangRailRobotConfigData
	LibDLWheelRobotCoreSocket
	LibDLNewInfraredSDK
)
 
ADD_LIBRARY(LibDLHangRailCommonWidget ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLHangRailCommonWidget ${Qt5Widgets_LIBRARIES}  ${Qt5MultimediaWidgets_LIBRARIES} ${Qt5Multimedia_LIBRARIES} ${EXTRA_LIBS} Qt5::PrintSupport  avcodec avformat avutil swresample swscale)