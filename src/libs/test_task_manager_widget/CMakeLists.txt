FIND_PACKAGE (Qt5 REQUIRED Core LinguistTools Gui Qml Widgets Quick)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /SAFESEH:NO")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SAFESEH:NO")
set( CMAKE_BUILD_TYPE Release )
SET(test_task_manager_widget_moc_srcs
	##ADD MOC FILE HERE ,REALLY IS *.h that CONTAINS Q_OBJECT
)

SET(test_task_manager_widget_srcs
	#ADD SOURCE FILE HERE
	test_task_manager_widget_main.cpp
)

SET(test_task_manager_widget_uis
	#ADD UI FILE HERE
)

SET(test_task_manager_widget_libs
	#ADD LIB FILE HERE
	WheelTaskManageWidget
	LibDLHangRailRobotDBOperation
    LibDLWheelRobotConfigData
	LibDLWheelCustomWidget
	LibDLWheelTaskDeviceDBSearch
	LibDLHangRailCommonWidget
	LibDLHangRailConfigData
)

SET(test_task_manager_widget_qrcs
	#ADD LIB FILE HERE
)

SET(test_task_manager_widget_trans
	#ADD LIB FILE HERE
)

QT5_WRAP_CPP(test_task_manager_widget_moc  ${test_task_manager_widget_moc_srcs})
QT5_WRAP_UI(test_task_manager_widget_uic ${test_task_manager_widget_uis})
QT5_ADD_RESOURCES(test_task_manager_widget_rcc ${test_task_manager_widget_qrcs})
QT5_ADD_TRANSLATION(test_task_manager_widget_qm ${test_task_manager_widget_trans})
ADD_DEFINITIONS(-D_REENTRANT -DQT_NO_DEBUG -DQT_SQL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED )
ADD_EXECUTABLE(test_task_manager_widget ${test_task_manager_widget_moc} ${test_task_manager_widget_srcs} ${test_task_manager_widget_uic} ${test_task_manager_widget_rcc} ${test_task_manager_widget_qm})
TARGET_LINK_LIBRARIES(test_task_manager_widget ${test_task_manager_widget_libs} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Qml Qt5::Quick)
