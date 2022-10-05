find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 LibProtoClient
)

##默认VS工程是多字节字符集，这里需要使用Unicode字符集(GetDiskFreeSpaceEx)
Add_Definitions(-DUNICODE -D_UNICODE)

ADD_LIBRARY(LibDLHangRailCommonTools ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLHangRailCommonTools ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})