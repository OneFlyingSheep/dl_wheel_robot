IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.c" "*.cpp")
ENDIF (NOT SOURCES)
SET(EXTRA_LIBS
)
ADD_LIBRARY(LibHttpProtocol ${SOURCES})
target_link_libraries(LibHttpProtocol)