find_package( Qt5Widgets REQUIRED)
find_package( Qt5Network REQUIRED )
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
    ##ADD LIB FILE HERE
    image_recognition
    LibDLWheelRobotDBOperation
    LibDLWheelRobotCoreSocket
    LibDLLuaScript
    LibDLSMSMessage
    LibDLNewInfraredSDK
    LibZip
	LibHttpProtocol
)

ADD_LIBRARY(LibDLWheelRobotTask ${SOURCES}) 
#TARGET_LINK_LIBRARIES(LibDLWheelRobotTask ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS}  ${TORCH_LIBRARIES} ${EXTRA_LIBS} )
TARGET_LINK_LIBRARIES(LibDLWheelRobotTask ${Qt5Network_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${EXTRA_LIBS} )