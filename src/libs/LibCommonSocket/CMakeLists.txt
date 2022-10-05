IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     LibCommonMsg
     LibDLHangRailRobotDBOperation
     LibDLHangRailConfigData
)


ADD_LIBRARY(LibCommonSocket ${SOURCES})
TARGET_LINK_LIBRARIES(LibCommonSocket ${EXTRA_LIBS})