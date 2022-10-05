FIND_PACKAGE (Qt5 REQUIRED Core LinguistTools Gui Qml Widgets Quick)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
SET(WheelTaskManageWidgetMocSrcs
	##ADD MOC FILE HERE ,REALLY IS *.h that CONTAINS Q_OBJECT
	WheelTaskManageWidget.h
	DLWheelTaskExcuteWindow.h
)

SET(WheelTaskManageWidgetSrcs
	#ADD SOURCE FILE HERE
	WheelTaskManageWidget.cpp
	DLWheelTaskExcuteWindow.cpp
)

SET(WheelTaskManageWidgetUis
	#ADD UI FILE HERE
)

SET(WheelTaskManageWidgetLibs
	#ADD LIB FILE HERE
	LibDLWheelCustomWidget
	LibDLWheelTaskDeviceDBSearch
	LibDLHangRailCommonWidget
	LibDLWheelRobotCoreSocket
)

SET(WheelTaskManageWidgetQrcs
	#ADD LIB FILE HERE
)

SET(WheelTaskManageWidgetTrans
	#ADD LIB FILE HERE
)

QT5_WRAP_CPP(WheelTaskManageWidgetMoc  ${WheelTaskManageWidgetMocSrcs})
QT5_WRAP_UI(WheelTaskManageWidgetUic ${WheelTaskManageWidgetUis})
QT5_ADD_RESOURCES(WheelTaskManageWidgetRcc ${WheelTaskManageWidgetQrcs})
QT5_ADD_TRANSLATION(WheelTaskManageWidgetQm ${WheelTaskManageWidgetTrans})
ADD_DEFINITIONS(-D_REENTRANT -DQT_NO_DEBUG -DQT_SQL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED )
ADD_LIBRARY(WheelTaskManageWidget ${WheelTaskManageWidgetMoc} ${WheelTaskManageWidgetSrcs} ${WheelTaskManageWidgetUic} ${WheelTaskManageWidgetRcc} ${WheelTaskManageWidgetQm})
TARGET_LINK_LIBRARIES(WheelTaskManageWidget ${WheelTaskManageWidgetLibs} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Qml Qt5::Quick)
