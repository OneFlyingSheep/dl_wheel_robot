SET(BimInterfaceProtocolSrcs
	#ADD SOURCE FILE FOR THIS LIB HERE
	BimInterfaceProtocol.cpp
)

SET(BimInterfaceProtocolLibs
	#ADD LIB FILE FOR THIS LIB HERE
)

ADD_LIBRARY(BimInterfaceProtocol STATIC  ${BimInterfaceProtocolSrcs})
TARGET_LINK_LIBRARIES(BimInterfaceProtocol ${BimInterfaceProtocolLibs})
