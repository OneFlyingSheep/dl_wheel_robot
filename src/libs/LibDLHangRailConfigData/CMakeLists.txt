IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     LibCommonFileOperation
     LibDLHangRailRobotDBOperation 
)

ADD_LIBRARY(LibDLHangRailConfigData ${SOURCES})
TARGET_LINK_LIBRARIES(LibDLHangRailConfigData ${EXTRA_LIBS})