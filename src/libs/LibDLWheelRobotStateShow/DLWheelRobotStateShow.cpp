#include "DLWheelRobotStateShow.h"
#include "LibDLWheelCustomWidget/BorderWidget.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLHangRailCommonWidget/SwitchControl.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QDesktopWidget>
#include <QApplication>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include "LibProtoClient/ProtoClient.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"

#pragma execution_character_set("utf-8")

DLWheelRobotStateShow::DLWheelRobotStateShow(QWidget* parent /* = NULL */)
	: QWidget(parent)
	, m_updateRealTimeDataTimer(false)
	, m_isInitWidget(false)
    , m_isCharge(false)
{
    initCommunicationStatusTest();

	this->setStyleSheet("QWidget#BackWidget{background:white;}\
						QLabel#TitleLabel{font-weight:bold;font-size:25px;background:rgb(0, 110, 110)}\
						QPushButton#CommonButton{color:white;font-weight:bold;font-size:14px;background:rgb(0, 110, 110);border-radius:3px;}\
						QLabel#DotSplitter{border-top:2px dashed rgb(0, 110, 110)}\
						QLabel#SubHead{color:red;font-size:18px;font-weight:bold;}");

}

void DLWheelRobotStateShow::setDevelopMode()
{
	initWidget();
	QString strStyleSheet = this->styleSheet();
	strStyleSheet += "QLabel#TitleLabel{font-weight:bold;font-size:25px;background:rgb(116, 188, 248);color:white;}";
	this->setStyleSheet(strStyleSheet);

	QLayout* mainLayout = this->layout();
	mainLayout->setMargin(10);
}

void DLWheelRobotStateShow::onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus)
{
    // 如果没有初始化直接返回;
    if (!m_isInitWidget)
    {
        return;
    }

    m_mutex.lock();

	// 云台参数;
    float fPanAngle = 1.0 * realTimeStatus.firePtzStatus.pan / 100;
	m_firePtzHorizontalPos->setShowValue(QString::number(fPanAngle, 'f', 1));

     float fTiltAngle = -1;
     if (realTimeStatus.firePtzStatus.tilt <= 9000 && realTimeStatus.firePtzStatus.tilt >= 0)
     {
         fTiltAngle = -1.0 * realTimeStatus.firePtzStatus.tilt / 100;
     }
     else if (realTimeStatus.firePtzStatus.tilt <= 36000 & realTimeStatus.firePtzStatus.tilt >= 27000)
     {
         fTiltAngle = 1.0 * (realTimeStatus.firePtzStatus.tilt - 27000) / 100;
     }
     m_firePtzVerticalPos->setShowValue(QString::number(fTiltAngle, 'f', 1));

	 //双光云台
	 fPanAngle = 1.0 * realTimeStatus.doubleLightPtzStatus.pan / 100;
	 m_doublePtzHorizontalPos->setShowValue(QString::number(fPanAngle, 'f', 1));

	 fTiltAngle = -1;
	 if (realTimeStatus.doubleLightPtzStatus.tilt <= 9000 && realTimeStatus.doubleLightPtzStatus.tilt >= 0)
	 {
		 fTiltAngle = -1.0 * realTimeStatus.doubleLightPtzStatus.tilt / 100;
	 }
	 else if (realTimeStatus.doubleLightPtzStatus.tilt <= 36000 & realTimeStatus.doubleLightPtzStatus.tilt >= 27000)
	 {
		 fTiltAngle = 1.0 * (realTimeStatus.doubleLightPtzStatus.tilt - 27000) / 100;
	 }
	 m_doublePtzVerticalPos->setShowValue(QString::number(fTiltAngle, 'f', 1));

	m_cameraFocus->setShowValue(QString("%1").arg(realTimeStatus.visibleCameraStatus.focus));
	m_cameraZoom->setShowValue(QString("%1").arg(realTimeStatus.visibleCameraStatus.zoom));

	// 运行速度;
	m_runSpeed->setShowValue(QString::number(realTimeStatus.moveStatus.vx + realTimeStatus.moveStatus.w));

	// 当前电池电量;
	m_batteryWidget->setCurrentBatteryEnergy(realTimeStatus.batteryStatus.battery_level * 100);

	m_batteryValueValue->setText(QString("%1%").arg(int(realTimeStatus.batteryStatus.battery_level * 100)));

	// 左轮右轮;
	m_leftWheel->setShowValue(QString("%1m/s").arg(realTimeStatus.wheelVel.vLeft));
	m_rightWheel->setShowValue(QString("%1m/s").arg(realTimeStatus.wheelVel.vRight));

	// 外供电源;
	m_externaSupplyBattery->setShowValue(QString("%1A %2V").arg(realTimeStatus.batteryStatus.current).arg(realTimeStatus.batteryStatus.voltage));

	// 充电状态;
	bool isCharge = realTimeStatus.batteryStatus.charging;
    if (m_isCharge != isCharge)
    {
        m_isCharge = isCharge;
        try
        {
            if (isCharge)
            {
                m_chargeStateLabel->setPixmap(QPixmap(":/Resources/Common/image/Charging.png").scaled(m_chargeStateLabel->size()));
                m_chargeValue->setShowValue("29.2V");
            }
            else
            {
                m_chargeStateLabel->setPixmap(QPixmap(":/Resources/Common/image/No_Charge.png").scaled(m_chargeStateLabel->size()));
                m_chargeValue->setShowValue("0.0V");
            }
        }
        catch (...)
        {
            ROS_ERROR("ChargeStateLabel SetPixmap Error");
        }
    }

	//磁盘空间
	float used_disk = realTimeStatus.robotDiskInfo.usage * 100.0;
	m_robotDiskUsage->setShowValue(QString::number(used_disk, 'f', 2) + "%");
	m_robotDiskTotal->setShowValue(QString("%1GB").arg(realTimeStatus.robotDiskInfo.total));

	//灭火罐状态
	if (realTimeStatus.fireExtinguisherStatusLeft.dev_switch)
	{
		m_firePotStatusMajor->setShowValue("开启");
	}
	else
	{
		m_firePotStatusMajor->setShowValue("关闭");
	}
	m_firePotInfoMajor->setShowValue(QString("%1MPa").arg(realTimeStatus.fireExtinguisherStatusLeft.pressure));

	if (realTimeStatus.fireExtinguisherStatusRight.dev_switch)
	{
		m_firePotStatusAlt->setShowValue("开启");
	}
	else
	{
		m_firePotStatusAlt->setShowValue("关闭");
	}
	m_firePotInfoAlt->setShowValue(QString("%1MPa").arg(realTimeStatus.fireExtinguisherStatusRight.pressure));

    m_mutex.unlock();

	m_isUpdateRealTimeData = true;
}

void DLWheelRobotStateShow::onUpdateRobotNoneRealtimeStatus(WheelRobotNoneRealtimeStatus realTimeStatus)
{
    // 如果没有初始化直接返回;
    if (!m_isInitWidget)
    {
        return;
    }
	// 机身温度;
	m_bodyTmpWidget->setShowValue(QString::number(realTimeStatus.controller_temp, 'f', 1));
	m_bodyHuiWidget->setShowValue(QString::number(realTimeStatus.controller_humi, 'f', 1));


    //m_bodyTmpWidget->setShowValue(QString::number(realTimeStatus.controller_temp, 'f', 1));

	// 运行里程、运行时间;
	m_runMileage->setShowValue(QString("%1公里").arg(realTimeStatus.odo));
	int runtime = realTimeStatus.time / 1000;
	m_runTime->setShowValue(QString("%1时%2分").arg(int((runtime / 60) / 60)).arg(int(runtime / 60)));
//	m_runTime->setShowValue(QString("%1小时").arg(realTimeStatus.time));
}

void DLWheelRobotStateShow::onUpdateVisibleVideoParam(int visibleZoomValue)
{
	m_cameraZoom->setShowValue(QString::number(visibleZoomValue));
}

void DLWheelRobotStateShow::initRunStateInfo()
{
	m_runStateInfoWidget = new BorderWidget;
	m_runStateInfoWidget->setTitleText("运行状态信息");
	m_runStateInfoWidget->setFixedWidth(600);

	m_bodyTmpWidget = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_bodyTmpWidget->setTipText("机身温度");
	m_bodyTmpWidget->setShowValue("0℃");

	m_bodyHuiWidget = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_bodyHuiWidget->setTipText("机身湿度");
	m_bodyHuiWidget->setShowValue("0%rh");

	m_firePtzHorizontalPos = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_firePtzHorizontalPos->setTipText("灭火云台水平值");
	m_firePtzHorizontalPos->setShowValue("-1");

	m_firePtzVerticalPos = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_firePtzVerticalPos->setTipText("灭火云台垂直值");
	m_firePtzVerticalPos->setShowValue("-1");

	m_doublePtzHorizontalPos = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_doublePtzHorizontalPos->setTipText("双光云台水平值");
	m_doublePtzHorizontalPos->setShowValue("-1");

	m_doublePtzVerticalPos = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_doublePtzVerticalPos->setTipText("双光云台垂直值");
	m_doublePtzVerticalPos->setShowValue("-1");

	m_cameraZoom = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_cameraZoom->setTipText("相机倍数");
	m_cameraZoom->setShowValue("-1");

	m_cameraFocus = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_cameraFocus->setTipText("相机焦距");
	m_cameraFocus->setShowValue("-1");

	QGridLayout* gRunStateInfoLayout = new QGridLayout(m_runStateInfoWidget->getCenterWidget());
	gRunStateInfoLayout->addWidget(m_bodyTmpWidget, 0, 0);
	gRunStateInfoLayout->addWidget(m_bodyHuiWidget, 0, 1);
	gRunStateInfoLayout->addWidget(m_cameraZoom, 1, 0);
	gRunStateInfoLayout->addWidget(m_cameraFocus, 1, 1);
	gRunStateInfoLayout->addWidget(m_doublePtzHorizontalPos, 2, 0);
	gRunStateInfoLayout->addWidget(m_doublePtzVerticalPos, 2, 1);
	gRunStateInfoLayout->addWidget(m_firePtzHorizontalPos, 3, 0);
	gRunStateInfoLayout->addWidget(m_firePtzVerticalPos, 3, 1);
	gRunStateInfoLayout->setHorizontalSpacing(100);
	gRunStateInfoLayout->setVerticalSpacing(10);
	gRunStateInfoLayout->setMargin(10);
}

void DLWheelRobotStateShow::initCommunicateStateInfo()
{
	m_communitcateStateInfoWidget = new BorderWidget;
	m_communitcateStateInfoWidget->setTitleText("通信状态信息");
	m_communitcateStateInfoWidget->setFixedWidth(600);

	m_wirelessStation = new InputWidget(InputWidgetType::WheelSignalWidget);
	m_wirelessStation->setTipText("无线基站");
	m_wirelessStation->setSignalStatus(false);

	m_controlSystem = new InputWidget(InputWidgetType::WheelSignalWidget);
	m_controlSystem->setTipText("控制系统");
	m_controlSystem->setSignalStatus(false);

	m_visibleCamera = new InputWidget(InputWidgetType::WheelSignalWidget);
	m_visibleCamera->setTipText("可见光摄像");
	m_visibleCamera->setSignalStatus(false);

	m_chargeSystem = new InputWidget(InputWidgetType::WheelSignalWidget);
	m_chargeSystem->setTipText("充电系统");
	m_chargeSystem->setSignalStatus(false);

	m_infreredCamera = new InputWidget(InputWidgetType::WheelSignalWidget);
	m_infreredCamera->setTipText("红外摄像");
	m_infreredCamera->setSignalStatus(false);

	m_robotConnect = new InputWidget(InputWidgetType::WheelSignalWidget);
	m_robotConnect->setTipText("机器人连接");
	m_robotConnect->setSignalStatus(false);

	QGridLayout* gRunStateInfoLayout = new QGridLayout(m_communitcateStateInfoWidget->getCenterWidget());
	gRunStateInfoLayout->addWidget(m_wirelessStation, 0, 0);
	gRunStateInfoLayout->addWidget(m_controlSystem, 0, 1);
	gRunStateInfoLayout->addWidget(m_visibleCamera, 0, 2);
	gRunStateInfoLayout->addWidget(m_chargeSystem, 1, 0);
	gRunStateInfoLayout->addWidget(m_infreredCamera, 1, 1);
	gRunStateInfoLayout->addWidget(m_robotConnect, 1, 2);
	gRunStateInfoLayout->setHorizontalSpacing(80);
	gRunStateInfoLayout->setVerticalSpacing(10);
	gRunStateInfoLayout->setMargin(10);

}

void DLWheelRobotStateShow::initBattertyStateInfo()
{
	m_batteryStateInfoWidget = new BorderWidget;
	m_batteryStateInfoWidget->setTitleText("电池状态信息");
	m_batteryStateInfoWidget->setFixedWidth(600);

	m_currentBatteryValue = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_currentBatteryValue->setTipText("当前电池电量");
	m_batteryWidget = new BatteryWidget;
	m_batteryWidget->setCurrentBatteryEnergy(100);
	m_batteryValueValue = new QLabel;
	m_batteryValueValue->setStyleSheet("color:rgb(51, 163, 195);font-size:15px;font-weight:bold;");


	QHBoxLayout* hBatteryStateLayout = new QHBoxLayout(m_batteryStateInfoWidget->getCenterWidget());
	hBatteryStateLayout->addStretch();
	hBatteryStateLayout->addWidget(m_currentBatteryValue);
	hBatteryStateLayout->addWidget(m_batteryWidget);
	hBatteryStateLayout->addWidget(m_batteryValueValue);
	hBatteryStateLayout->addStretch();
	hBatteryStateLayout->setSpacing(0);
	hBatteryStateLayout->setMargin(10);
}

void DLWheelRobotStateShow::initRobotSelfModeInfo()
{
	m_robotSelfModeInfoWidget = new BorderWidget;
	m_robotSelfModeInfoWidget->setTitleText("机器人自身模块状态信息");
	m_robotSelfModeInfoWidget->setFixedWidth(600);

	QLabel* labelDriverMode = new QLabel;
	labelDriverMode->setText("驱动模块");
	labelDriverMode->setAlignment(Qt::AlignCenter);
	labelDriverMode->setObjectName("SubHead");

	m_runSpeed = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_runSpeed->setTipText("运行速度");
	m_runSpeed->setShowValue("0m/s");

	m_leftWheel = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_leftWheel->setTipText("左轮");
	m_leftWheel->setShowValue("-1.0m/s");

	m_rightWheel = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_rightWheel->setTipText("右轮");
	m_rightWheel->setShowValue("-1.0m/s");

	QLabel* dotSplitterLabelFirst = new QLabel;
	dotSplitterLabelFirst->setFixedHeight(2);
	dotSplitterLabelFirst->setObjectName("DotSplitter");

	QLabel* labelBatteryMode = new QLabel;
	labelBatteryMode->setText("电源模块");
	labelBatteryMode->setAlignment(Qt::AlignCenter);
	labelBatteryMode->setObjectName("SubHead");

	m_externaSupplyBattery = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_externaSupplyBattery->setTipText("外供电源");
	m_externaSupplyBattery->setShowValue("-1A -1V");
	m_externaSupplyBattery->setFixedWidth(200);

	m_chargeState = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_chargeState->setTipText("充电状态:");
	m_chargeState->setShowValue("");
	m_chargeState->setFixedWidth(80);

	m_chargeStateLabel = new QLabel;
	m_chargeStateLabel->setFixedSize(QSize(60, 30));
	m_chargeStateLabel->setPixmap(QPixmap(":/Resources/Common/image/Charging.png").scaled(m_chargeStateLabel->size()));

	QHBoxLayout* hChargeLayout = new QHBoxLayout;
	hChargeLayout->addWidget(m_chargeState);
	hChargeLayout->addWidget(m_chargeStateLabel);
	hChargeLayout->setSpacing(0);
	hChargeLayout->setMargin(0);

	m_chargeValue = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_chargeValue->setTipText("充电");
	m_chargeValue->setShowValue("0.0V");

	QLabel* dotSplitterLabelSecond = new QLabel;
	dotSplitterLabelSecond->setFixedHeight(2);
	dotSplitterLabelSecond->setObjectName("DotSplitter");

	QLabel* labelSystemMode = new QLabel;
	labelSystemMode->setText("系统模块");
	labelSystemMode->setAlignment(Qt::AlignCenter);
	labelSystemMode->setObjectName("SubHead");

	m_robotDiskUsage = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_robotDiskUsage->setTipText("已用空间");
	m_robotDiskUsage->setShowValue("0%");

	m_robotDiskTotal = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_robotDiskTotal->setTipText("总空间");
	m_robotDiskTotal->setShowValue("0GB");

	m_runMileage = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_runMileage->setTipText("运行里程");
	m_runMileage->setShowValue("-1公里");

	m_runTime = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_runTime->setTipText("运行时间");
	m_runTime->setShowValue("-1小时");

	m_patrolDeviceCount = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_patrolDeviceCount->setTipText("巡检设备数目");
	m_patrolDeviceCount->setShowValue("200个");
	m_patrolDeviceCount->setFixedWidth(160);

	m_defectFindedCount = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_defectFindedCount->setTipText("发现缺陷数量");
	m_defectFindedCount->setShowValue("34个");
	m_defectFindedCount->setFixedWidth(160);

	QGridLayout* gRobotSelfModeGridLayout = new QGridLayout(m_robotSelfModeInfoWidget->getCenterWidget());
	gRobotSelfModeGridLayout->addWidget(labelDriverMode, 0, 1);
	
	gRobotSelfModeGridLayout->addWidget(m_runSpeed, 1, 0, 1, 1);
	gRobotSelfModeGridLayout->addWidget(m_leftWheel, 1, 1, 1, 1);
	gRobotSelfModeGridLayout->addWidget(m_rightWheel, 1, 2, 1, 1);

	gRobotSelfModeGridLayout->addWidget(dotSplitterLabelFirst, 2, 0, 1, 3);

	gRobotSelfModeGridLayout->addWidget(labelBatteryMode, 3, 1);
	gRobotSelfModeGridLayout->addWidget(m_externaSupplyBattery, 4, 0, 1, 1);
	gRobotSelfModeGridLayout->addLayout(hChargeLayout, 4, 1, 1, 1);
	gRobotSelfModeGridLayout->addWidget(m_chargeValue, 4, 2, 1, 1);

	gRobotSelfModeGridLayout->addWidget(dotSplitterLabelSecond, 5, 0, 1, 3);

	gRobotSelfModeGridLayout->addWidget(labelSystemMode, 6, 1);

	gRobotSelfModeGridLayout->addWidget(m_robotDiskUsage, 7, 0, 1, 1);
	gRobotSelfModeGridLayout->addWidget(m_runMileage, 7, 1, 1, 1);
	gRobotSelfModeGridLayout->addWidget(m_runTime, 7, 2, 1, 1);

	gRobotSelfModeGridLayout->addWidget(m_robotDiskTotal, 8, 0, 1, 1);
	gRobotSelfModeGridLayout->addWidget(m_patrolDeviceCount, 8, 1, 1, 1);
	gRobotSelfModeGridLayout->addWidget(m_defectFindedCount, 8, 2, 1, 1);
	
	gRobotSelfModeGridLayout->setVerticalSpacing(10);
	gRobotSelfModeGridLayout->setMargin(10);
}

void DLWheelRobotStateShow::initLeftWidget()
{
	initRunStateInfo();
	initBattertyStateInfo();
	initRobotSelfModeInfo();

	QLabel* robotStateTitleLabel = new QLabel("机器人状态");
	robotStateTitleLabel->setObjectName("TitleLabel");
	robotStateTitleLabel->setFixedHeight(60);
	robotStateTitleLabel->setAlignment(Qt::AlignCenter);

	QDesktopWidget * desktopWidget = QApplication::desktop();
	m_leftBackWidget = new QWidget;
	m_leftBackWidget->setObjectName("BackWidget");
//	m_leftBackWidget->setFixedWidth(desktopWidget->screenGeometry().width() / 2);

	QVBoxLayout* vLeftLayout = new QVBoxLayout;
	vLeftLayout->addWidget(m_runStateInfoWidget);
	vLeftLayout->addWidget(m_batteryStateInfoWidget);
	vLeftLayout->addWidget(m_robotSelfModeInfoWidget);
	vLeftLayout->setSpacing(10);
	vLeftLayout->setMargin(0);

	QHBoxLayout* hLeftLayout = new QHBoxLayout;
	hLeftLayout->addStretch();
	hLeftLayout->addLayout(vLeftLayout);
	hLeftLayout->addStretch();
	hLeftLayout->setMargin(0);

	QVBoxLayout* vLayout = new QVBoxLayout(m_leftBackWidget);
	vLayout->addWidget(robotStateTitleLabel);
	vLayout->addLayout(hLeftLayout);
	vLayout->setSpacing(10);
	vLayout->setContentsMargins(0, 0, 0, 10);
}

void DLWheelRobotStateShow::initEnvironmentStateInfo()
{
	m_environmentStateInfoWidget = new BorderWidget;
	m_environmentStateInfoWidget->setTitleText("环境状态信息");
	m_environmentStateInfoWidget->setFixedWidth(600);

	m_environmentTmp = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_environmentTmp->setTipText("温度(℃)");
	m_environmentTmp->setShowValue("15");

	m_environmentHumidity = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_environmentHumidity->setTipText("湿度(%RH)");
	m_environmentHumidity->setShowValue("0.2");

	m_environmentWindSpeed = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_environmentWindSpeed->setTipText("风速(M/S)");
	m_environmentWindSpeed->setShowValue("4");

	QVBoxLayout* vLayout = new QVBoxLayout;
	vLayout->addWidget(m_environmentTmp);
	vLayout->addWidget(m_environmentHumidity);
	vLayout->addWidget(m_environmentWindSpeed);
	vLayout->setSpacing(15);
	vLayout->setMargin(0);

	QHBoxLayout* hLayout = new QHBoxLayout(m_environmentStateInfoWidget->getCenterWidget());
	hLayout->addStretch();
	hLayout->addLayout(vLayout);
	hLayout->addStretch();
	hLayout->setMargin(10);
}

//void DLWheelRobotStateShow::initControlStateInfo()
//{
//    initUploadBinWidget();
//
//	m_controlStateInfoWidget = new BorderWidget;
//	m_controlStateInfoWidget->setTitleText("设备状态信息");
//	m_controlStateInfoWidget->setFixedWidth(600);
//	m_controlStateInfoWidget->getCenterWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//
//	m_infraredFunction = new SwitchWidget(true);
//	m_infraredFunction->setSwitchName("红外功能");
//    m_infraredFunction->setSwitchState(true);
//    connect(m_infraredFunction, &SwitchWidget::toggled, this, [=](bool isCheck) {
//        emit signalInfraredSwitch(isCheck);
//    });
//
//	m_visibleFunction = new SwitchWidget(true);
//	m_visibleFunction->setSwitchName("可见光功能");
//    m_visibleFunction->setSwitchState(true);
//    connect(m_visibleFunction, &SwitchWidget::toggled, this, [=](bool isCheck) {
//        emit signalVisibleSwitch(isCheck);
//    });
//
//	m_rainBrushState = new SwitchWidget(true);
//	m_rainBrushState->setSwitchName("雨刷状态");
//    m_rainBrushState->setSwitchState(false);
//    connect(m_rainBrushState, &SwitchWidget::toggled, this, [=](bool isCheck) {
//        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_wiper_req(isCheck);
//    });
//
//
//	m_avoidObstacleFunction = new SwitchWidget(true);
//	m_avoidObstacleFunction->setSwitchName("避障功能");
//    m_avoidObstacleFunction->setSwitchState(true);
//
//	m_carLightState = new SwitchWidget(true);
//	m_carLightState->setSwitchName("车灯状态");
//    m_carLightState->setSwitchState(false);
//    connect(m_carLightState, &SwitchWidget::toggled, this, [=](bool isCheck) {
//        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_light_req(isCheck);
//    });
//
//	m_chargeRoom = new SwitchWidget(true);
//	m_chargeRoom->setSwitchName("充电房");
//    m_chargeRoom->setSwitchState(false);
//    connect(m_chargeRoom, &SwitchWidget::toggled, this, [=](bool isCheck) {
//        WHEEL_BACK_TO_CORE_SOCKET.robot_temporary_door_req(isCheck);
//    });
//
//	m_robotState = new SwitchWidget(true);
//	m_robotState->setSwitchName("机器人状态");
//    m_robotState->setSwitchState(true);
//
//	QVBoxLayout* vLayout = new QVBoxLayout;
//	vLayout->addWidget(m_infraredFunction);
//	vLayout->addWidget(m_visibleFunction);
//	vLayout->addWidget(m_rainBrushState);
//	vLayout->addWidget(m_avoidObstacleFunction);
//	vLayout->addWidget(m_carLightState);
//	vLayout->addWidget(m_chargeRoom);
//	vLayout->addWidget(m_robotState);
//    vLayout->addSpacing(20);
//    vLayout->addWidget(m_uploadBinBackWidget);
//	vLayout->setSpacing(20);
//	vLayout->setMargin(0);
//
//	QHBoxLayout* hLayout = new QHBoxLayout(m_controlStateInfoWidget->getCenterWidget());
//	hLayout->addStretch();
//	hLayout->addLayout(vLayout);
//	hLayout->addStretch();
//	hLayout->setMargin(30);
//
//    // 默认不显示，采集程序显示;
//    m_uploadBinBackWidget->setVisible(false);
//}

void DLWheelRobotStateShow::initDeviceStateInfo()
{
	initUploadBinWidget();

	m_controlStateInfoWidget = new BorderWidget;
	m_controlStateInfoWidget->setTitleText("设备状态信息");
	m_controlStateInfoWidget->setFixedWidth(600);
	m_controlStateInfoWidget->getCenterWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

	m_infraredFunction = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_infraredFunction->setTipText("红外功能");
	m_infraredFunction->setShowValue("正常");

	m_rainBrushState = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_rainBrushState->setTipText("雨刷状态");
	m_rainBrushState->setShowValue("停止");

	m_avoidObstacleFunction = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_avoidObstacleFunction->setTipText("避障功能");
	m_avoidObstacleFunction->setShowValue("正常");

	m_carLightState = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_carLightState->setTipText("车灯状态");
	m_carLightState->setShowValue("关闭");
	//
	//m_robotState = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	//m_robotState->setTipText("机器人状态");
	//m_robotState->setShowValue("正常");

	m_chargeRoom = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_chargeRoom->setTipText("充电房");
	m_chargeRoom->setShowValue("正常");

	m_visibleFunction = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_visibleFunction->setTipText("可见光功能");
	m_visibleFunction->setShowValue("正常");

	m_firePotStatusMajor = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_firePotStatusMajor->setTipText("左灭火罐状态");
	m_firePotStatusMajor->setShowValue("关闭");

	m_firePotInfoMajor = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_firePotInfoMajor->setTipText("左灭火罐压力值");
	m_firePotInfoMajor->setShowValue(QString("%1MPa").arg(100));


	m_firePotStatusAlt = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_firePotStatusAlt->setTipText("右灭火罐状态");
	m_firePotStatusAlt->setShowValue("关闭");

	m_firePotInfoAlt = new InputWidget(InputWidgetType::WheelValueShowWidget_Green);
	m_firePotInfoAlt->setTipText("右灭火罐压力值");
	m_firePotInfoAlt->setShowValue(QString("%1MPa").arg(100));

	QGridLayout* vLayout = new QGridLayout;
	vLayout->addWidget(m_infraredFunction, 0, 0);
	vLayout->addWidget(m_rainBrushState, 0, 1);

	vLayout->addWidget(m_avoidObstacleFunction, 1, 0);
	vLayout->addWidget(m_carLightState, 1, 1);

	//vLayout->addWidget(m_robotState, 2, 0, 1, 3);
	vLayout->addWidget(m_chargeRoom, 2, 0);
	vLayout->addWidget(m_visibleFunction, 2, 1);

	vLayout->addWidget(m_firePotStatusMajor, 3, 0);
	vLayout->addWidget(m_firePotInfoMajor, 3, 1);

	vLayout->addWidget(m_firePotStatusAlt, 4, 0);
	vLayout->addWidget(m_firePotInfoAlt, 4, 1);

	vLayout->setHorizontalSpacing(100);
	vLayout->setVerticalSpacing(10);
	vLayout->setMargin(10);

	//vLayout->setSpacing(15);
	//vLayout->setMargin(0);

	QHBoxLayout* hLayout = new QHBoxLayout(m_controlStateInfoWidget->getCenterWidget());
	hLayout->addStretch();
	hLayout->addLayout(vLayout);
	hLayout->addStretch();
	hLayout->setMargin(10);


	// 默认不显示，采集程序显示;
	//m_uploadBinBackWidget->setVisible(false);
}

void DLWheelRobotStateShow::initRightWidget()
{
	initEnvironmentStateInfo();
	initCommunicateStateInfo();
	initDeviceStateInfo();

	QLabel* robotControlTitleLabel = new QLabel("传感器状态");
	robotControlTitleLabel->setObjectName("TitleLabel");
	robotControlTitleLabel->setFixedHeight(60);
	robotControlTitleLabel->setAlignment(Qt::AlignCenter);

	QDesktopWidget * desktopWidget = QApplication::desktop();
	m_rightBackWidget = new QWidget;
//	m_rightBackWidget->setFixedWidth(desktopWidget->screenGeometry().width() / 2);
	m_rightBackWidget->setObjectName("BackWidget");

	QVBoxLayout* vRightLayout = new QVBoxLayout;
	vRightLayout->addWidget(m_environmentStateInfoWidget);
	vRightLayout->addWidget(m_communitcateStateInfoWidget);
	//vRightLayout->addWidget(m_controlStateInfoWidget);
	vRightLayout->setSpacing(20);
	vRightLayout->setMargin(0);

	QHBoxLayout* hRightLayout = new QHBoxLayout;
	hRightLayout->addStretch();
	hRightLayout->addLayout(vRightLayout);
	hRightLayout->addStretch();
	hRightLayout->setMargin(0);

	QVBoxLayout* vLayout = new QVBoxLayout(m_rightBackWidget);
	vLayout->addWidget(robotControlTitleLabel);
	vLayout->addLayout(hRightLayout);
	vLayout->setSpacing(10);
	vLayout->setContentsMargins(0, 0, 0, 10);
}

void DLWheelRobotStateShow::initUploadBinWidget()
{
    m_uploadBinBackWidget = new QWidget;
    m_uploadBinBackWidget->setStyleSheet("*{font-family:Microsoft Yahei;font-size:15px;}\
						QPushButton#PatrolButton{font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton#PatrolButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton#PatrolButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");

    // 客户端向core通过rcf发送需烧写的文件;
    QLineEdit* robotBinUpLineEdit = new QLineEdit;
    robotBinUpLineEdit->setFixedWidth(180);
    robotBinUpLineEdit->setStyleSheet("QLineEdit{border:1px solid gray;border-radius:2px;color:black;}\
								QLineEdit:read-only{background: lightgray;}");

    QLabel* pUploadFileName = new QLabel("嵌入式升级");

    QPushButton* pButtonChoosePathUp = new QPushButton("选择路径");
    pButtonChoosePathUp->setObjectName("PatrolButton");
    pButtonChoosePathUp->setFixedSize(QSize(75, 25));
    connect(pButtonChoosePathUp, &QPushButton::clicked, this, [=] {
        QString strBinFilePath = QFileDialog::getOpenFileName(this, "选择文件", "", "bin(Wheeled_Robot.bin)");
        if (strBinFilePath.isEmpty())
        {
            return;
        }
        robotBinUpLineEdit->setText(strBinFilePath);
    });

    QPushButton* pButtonUploadFileUp = new QPushButton("上传");
    pButtonUploadFileUp->setObjectName("PatrolButton");
    pButtonUploadFileUp->setFixedSize(QSize(75, 25));
    connect(pButtonUploadFileUp, &QPushButton::clicked, this, [=] {
        QString strBinFilePath = robotBinUpLineEdit->text();
        if (strBinFilePath.isEmpty() || !QFile::exists(strBinFilePath))
        {
            DLMessageBox::showDLMessageBox(NULL, "错误", "请选择文件", MessageButtonType::BUTTON_OK, true);
            return;
        }

        int bUpload = GFILE_TRANSFER_CLIENT.uploadFile(std::string(strBinFilePath.toLocal8Bit()), std::string("tftp"));
        if (!bUpload)
        {
            emit signalSendOperateMsg("烧写的文件发送成功");
            WHEEL_BACK_TO_CORE_SOCKET.robot_update_embedded_software();
            // todo

        }
        else
        {
            emit signalSendOperateMsg("烧写的文件发送失败");
        }
    });

    QHBoxLayout* hTftpLineEditLayout = new QHBoxLayout;
    hTftpLineEditLayout->addWidget(pUploadFileName);
    hTftpLineEditLayout->addWidget(robotBinUpLineEdit);
    hTftpLineEditLayout->addStretch();
    hTftpLineEditLayout->setSpacing(10);
    hTftpLineEditLayout->setMargin(0);

    QHBoxLayout* hTftpButtonLayout = new QHBoxLayout;
    hTftpButtonLayout->addWidget(pButtonChoosePathUp);
    hTftpButtonLayout->addWidget(pButtonUploadFileUp);
    hTftpButtonLayout->setSpacing(10);
    hTftpButtonLayout->setMargin(0);


    QVBoxLayout* vRobotTftpLayout = new QVBoxLayout(m_uploadBinBackWidget);
    vRobotTftpLayout->addLayout(hTftpLineEditLayout);
    vRobotTftpLayout->addLayout(hTftpButtonLayout);
    vRobotTftpLayout->setSpacing(30);
    vRobotTftpLayout->setMargin(0); 
}

void DLWheelRobotStateShow::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initLeftWidget();
	initRightWidget();

	QLabel* splitterLable = new QLabel;
	splitterLable->setFixedWidth(2);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_leftBackWidget);
	hMainLayout->addWidget(splitterLable);
	hMainLayout->addWidget(m_rightBackWidget);
	hMainLayout->setSpacing(0);
	hMainLayout->setMargin(0);

	initUpdateRealTimeDataTimer();

    // 开启通信状态检测;
    m_wirelessBaseStationStatus->start();
    m_visibleCameraStatus->start();
    m_visibleCameraStatus->start();
    m_chargeSystemStatus->start();
	m_infraredCameraStatus->start(); 
	m_fireRobotStatus->start(); 

    // 刷新页面连接状态;
    m_communicationStatusUpdateTimer.start();
}

void DLWheelRobotStateShow::initUpdateRealTimeDataTimer()
{
	// 云台角度数据从Core处获取，如果1s之内接收不到，则将水平，垂直角度置为-1;
	m_updateRealTimeDataTimer.setInterval(1000);
	connect(&m_updateRealTimeDataTimer, &QTimer::timeout, this, [=] {
		if (!m_isUpdateRealTimeData)
		{
			// 这里把实时数据值置为-1;
			// 云台参数;
			m_firePtzHorizontalPos->setShowValue("-1");

			m_firePtzVerticalPos->setShowValue("-1");

			// 运行速度;
			m_runSpeed->setShowValue("-1");

			// 当前电池电量;
		//	m_batteryWidget->setCurrentBatteryEnergy(realTimeStatus.batteryStatus.battery_level * 100);

			// 左轮右轮;
			m_leftWheel->setShowValue(QString("-1m/s"));
			m_rightWheel->setShowValue(QString("-1m/s"));

			// 外供电源;
			m_externaSupplyBattery->setShowValue(QString("%1A %2V").arg(-1).arg(-1));
			// 是否充电;
			m_chargeStateLabel->setPixmap(QPixmap(":/Resources/Common/image/No_Charge.png").scaled(m_chargeStateLabel->size()));
		}
		else
		{
			m_isUpdateRealTimeData = false;
		}
	});
	m_updateRealTimeDataTimer.start();
}

void DLWheelRobotStateShow::initCommunicationStatusTest()
{
    m_wirelessBaseStationStatus = new CommunicationStatusTest(this);

    m_controlSystemStatus = new CommunicationStatusTest(this);  

    m_visibleCameraStatus = new CommunicationStatusTest(this); 

    m_chargeSystemStatus = new CommunicationStatusTest(this);  

    m_infraredCameraStatus = new CommunicationStatusTest(this);

	m_fireRobotStatus = new CommunicationStatusTest(this);

    m_communicationStatusUpdateTimer.setInterval(1000);
    connect(&m_communicationStatusUpdateTimer, &QTimer::timeout, this, [=] {
        if (m_isInitWidget)
        {
            m_wirelessStation->setSignalStatus(m_wirelessBaseStationStatus->getCurrentConnectStatus());
            m_controlSystem->setSignalStatus(m_controlSystemStatus->getCurrentConnectStatus());
            m_visibleCamera->setSignalStatus(m_visibleCameraStatus->getCurrentConnectStatus());
            m_chargeSystem->setSignalStatus(m_chargeSystemStatus->getCurrentConnectStatus());
            m_infreredCamera->setSignalStatus(m_infraredCameraStatus->getCurrentConnectStatus());
			m_robotConnect->setSignalStatus(m_fireRobotStatus->getCurrentConnectStatus());
		}
    });
}

void DLWheelRobotStateShow::setWirelessBaseStationIP(QString strIp, int iPort)
{
    m_wirelessBaseStationStatus->setTestIpAndPort(strIp, iPort);
}

void DLWheelRobotStateShow::setControlSystemIP(QString strIp, int iPort)
{
    m_controlSystemStatus->setTestIpAndPort(strIp, iPort);
}

void DLWheelRobotStateShow::setVisibleCameraIP(QString strIp, int iPort)
{
    m_visibleCameraStatus->setTestIpAndPort(strIp, iPort);
}

void DLWheelRobotStateShow::setChargeSystemIP(QString strIp, int iPort)
{
    m_chargeSystemStatus->setTestIpAndPort(strIp, iPort);
}

void DLWheelRobotStateShow::setInfraredCameraSIP(QString strIp, int iPort)
{
    m_infraredCameraStatus->setTestIpAndPort(strIp, iPort);
}

void DLWheelRobotStateShow::setFireRobotIP(QString strIp, int iPort)
{
	m_fireRobotStatus->setTestIpAndPort(strIp, iPort);
}

void DLWheelRobotStateShow::setIsShowUploadBinWidget(bool isShow)
{
    m_uploadBinBackWidget->setVisible(isShow);
}

void DLWheelRobotStateShow::SetVisableZoom(int iZoom)
{
	if (nullptr != m_cameraZoom)
	{
		m_cameraZoom->setShowValue(QString::number(iZoom));
	}
}
