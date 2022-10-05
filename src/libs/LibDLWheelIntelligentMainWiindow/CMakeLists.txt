find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.h"  "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
     LibDLIntelligentSocket
)

ADD_LIBRARY(LibDLWheelIntelligentMainWiindow ${SOURCES} ${QRCS}) 
TARGET_LINK_LIBRARIES(LibDLWheelIntelligentMainWiindow ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})