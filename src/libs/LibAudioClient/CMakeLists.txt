IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)

SET(EXTRA_LIBS
     ##ADD LIB FILE HERE
	 jrtplib 
	 jthread
)

ADD_LIBRARY(LibAudioClient ${SOURCES} )
TARGET_LINK_LIBRARIES(LibAudioClient ${EXTRA_LIBS})