find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.*")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     FTPSClient
	 LibDLWheelRobotCoreSocket
     LibHCNetCamera 
     LibDLNewInfraredSDK
)

ADD_LIBRARY(LibDLIntelligentSocket ${SOURCES}) 

TARGET_LINK_LIBRARIES(LibDLIntelligentSocket ${EXTRA_LIBS})

