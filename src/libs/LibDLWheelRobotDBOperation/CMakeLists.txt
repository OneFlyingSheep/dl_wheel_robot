find_package( Qt5Widgets REQUIRED)
find_package( Qt5Sql REQUIRED)
IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

ADD_LIBRARY(LibDLWheelRobotDBOperation ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLWheelRobotDBOperation ${Qt5Widgets_LIBRARIES} Qt5::Sql)