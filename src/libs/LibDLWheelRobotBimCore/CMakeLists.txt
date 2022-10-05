find_package( Qt5Widgets REQUIRED)
find_package( Qt5Network REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
    ##ADD LIB FILE HERE
	libcurl_imp
	LibDLWheelRobotConfigData
	LibDLWheelRobotDBOperation
)

ADD_LIBRARY(LibDLWheelRobotBimCore ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLWheelRobotBimCore ${Qt5Network_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS} )