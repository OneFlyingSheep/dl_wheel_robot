find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS

)
 
ADD_LIBRARY(LibQCustomPlot ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibQCustomPlot ${Qt5Widgets_LIBRARIES} ${EXTRA_LIBS})