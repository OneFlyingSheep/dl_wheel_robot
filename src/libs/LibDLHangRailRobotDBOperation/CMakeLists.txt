find_package( Qt5Widgets REQUIRED)
find_package( Qt5Sql REQUIRED)
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

ADD_LIBRARY(LibDLHangRailRobotDBOperation ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLHangRailRobotDBOperation ${Qt5Widgets_LIBRARIES} Qt5::Sql)