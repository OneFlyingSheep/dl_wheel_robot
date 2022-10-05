find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp" "*.rc")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
    ##ADD LIB FILE HERE
	#LibDLWheelRobotCoreConsole
	#LibDLIntelligentSocket
    dbghelp
    libglog
    LibDLWheelRobotCoreSocket 
    LibDLWheelRobotTask
    LibProtoServer
    LibDLWheelRobotConfigData
    LibConvertSymbolsToLua
	LibDLCreateExcel
    LibDLWheelIntelligentMainWiindow
	LibDLWheelRobotBimCore
)

set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /SAFESEH:NO")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SAFESEH:NO")

ADD_EXECUTABLE(AppDLWheelRobotCore ${SOURCES}) 

TARGET_LINK_LIBRARIES(AppDLWheelRobotCore ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})