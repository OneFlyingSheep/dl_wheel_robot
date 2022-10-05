IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.h" "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     LibSocketLogin
     LibDLWheelRobotDBOperation
     LibDLWheelRobotConfigData
     LibConvertSymbolsToLua
     LibHCNetCamera
     LibVoiceSpeak
#	 LibDLWheelMainWindow
)

ADD_LIBRARY(LibDLWheelRobotCoreSocket ${SOURCES})
TARGET_LINK_LIBRARIES(LibDLWheelRobotCoreSocket ${EXTRA_LIBS})