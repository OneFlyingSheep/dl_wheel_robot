IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 
)

ADD_LIBRARY(LibDLSMSMessage ${SOURCES}) 
TARGET_LINK_LIBRARIES(LibDLSMSMessage ${EXTRA_LIBS})