IF (NOT SOURCES)
    FILE(GLOB_RECURSE SOURCES "*.cpp")
ENDIF (NOT SOURCES)
SET(EXTRA_LIBS
    #LibDLHangRailConfigData
    LibDLWheelRobotConfigData
)
ADD_LIBRARY(LibHCNetCamera ${SOURCES})

target_link_libraries(LibHCNetCamera HCCore HCNetSDK PlayCtrl GdiPlus winmm)