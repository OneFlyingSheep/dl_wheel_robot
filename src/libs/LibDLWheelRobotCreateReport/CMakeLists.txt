find_package( Qt5Widgets REQUIRED)
find_package( Qt5Sql REQUIRED)
find_package( Qt5AxContainer REQUIRED)


IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     LibDLWheelRobotDBOperation
)

ADD_LIBRARY(LibDLWheelRobotCreateReport ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLWheelRobotCreateReport ${Qt5Widgets_LIBRARIES} Qt5::Sql Qt5::AxContainer)