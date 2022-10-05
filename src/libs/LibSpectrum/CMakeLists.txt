find_package( Qt5Widgets REQUIRED)
find_package(Qt5MultimediaWidgets REQUIRED)

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.h" "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
    LibFftreal
)

include_directories(../LibFftreal)

ADD_LIBRARY(LibSpectrum ${SOURCES})
TARGET_LINK_LIBRARIES(LibSpectrum ${EXTRA_LIBS} ${Qt5Network_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5Multimedia_LIBRARIES})