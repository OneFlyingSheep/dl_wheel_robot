find_package( Qt5Widgets REQUIRED )

IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
)


ADD_LIBRARY(LibDLLuaScript ${SOURCES}) 

TARGET_LINK_LIBRARIES(LibDLLuaScript ${Qt5Widgets_LIBRARIES} Lua ${EXTRA_LIBS})