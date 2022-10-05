#include "LibDLWheelCollectControlWidget.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QHBoxLayout>
#include <QPainter>
#include <QPixmap>
#include <QApplication>
#include <QDesktopWidget>
#include "LibDLHangRailCommonWidget/SwitchControl.h"
#include "LibDLHangRailCommonWidget/CustomButton.h"
#include "LibDLHangRailCommonWidget/VideoBackWidget.h"
#include "LibDLHangRailCommonWidget/CommonDataDef.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLHangRailCommonWidget/CameraObject.h"
#include <QUuid>
#include <QApplication>
#include <QMenu>
#include "LibDLHangRailCommonWidget/MessageNotifier.h"
#include <QClipboard>
#include <QFileDialog>
#include <QMessageBox>
#include <QTime>
#include <QGroupBox>
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPointTreeData.h"
#include "LibDLCollectMap/DLCollectMap.h"

#include <QDebug>
#include <QMetaType>  
//#include "LibDLInfraredTemperature/InfraredTemperature.h"
#include "LibProtoClient/ProtoClient.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotVirtualDevice.h"
#include "LibDLWheelCustomWidget/AddEquipmentTreeWgt.h"
#include "LibDLWheelCustomWidget/DefData.h"
#include "LibDLSceneView/DLCustomScene.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"

#include "LibDataTransfer/DataTransfer.h"
#include "DLMapListWidget.h"
#include "LibDLSceneView/DLOperator.h"
#include "LibDLStatusBar/DLStatusBar.h"
#include "LibDLSceneView/DLView.h"


#define SPEED_VALUE 100
#define ICON_SIZE (40)

DLHangWheelCollectControlWidget::DLHangWheelCollectControlWidget(QWidget *parent)
	: QWidget(parent)
	, m_updateCount(0)
	, m_addDeviceInfoWidget(NULL)
	, m_addDeviceFirstStageWidget(NULL)
	, m_voltageLevelIndex(0)
	, m_areaNameIndex(0)
	, m_intervalNameIndex(0)
	, m_deviceTypeIndex(0)
	, m_deviceChildTypeIndex(0)
	, m_devicePointPosIndex(0)
	, m_isModifyDevice(false)
	, m_pointId(-1)
	, m_isUpdateRealTimeData(false)
	, m_bIsPressedState(true)
//	, m_collectStWidget(NULL)
	, m_pThreadDataTransfer(NULL)
	, m_pMapListWgt(NULL)
	//, m_bIsFirstIn(true)
	, m_strOpenMapPath("")
	,m_pCollectMapScene(NULL)
{
	WHEEL_ROBOT_DB;
	qRegisterMetaType<QList<WheelRobortAlarmPathStruct> >("QList<WheelRobortAlarmPathStruct>");
	qRegisterMetaType<QList<WheelPointTreeDataStruct> >("QList<WheelPointTreeDataStruct>");
	
	initWidget();
	initUpdatePtzAngleDataTimer();

	connect(&m_updateDataTimer, SIGNAL(timeout()), this, SLOT(onUpdateRobotInfo()));
	m_updateDataTimer.start(1000);

	this->setStyleSheet("QPushButton#CommonButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton#CommonButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton#CommonButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}\
						QPushButton#CommonButton:disabled{background:gray;}\
						.QGroupBox{color:rgb(98, 98, 98);}\
						QWidget#RobotInfoBackWidget,QWidget#ModeChangeBackWidget{background:white;}");

	m_keyboardOperateThread = new KeyboardOperateThread;
	m_keyboardOperateThread->start();

	m_wheelPointTreeData = new DLWheelPointTreeData;
	
	//初始化状态，为键盘模式，选择状态
	InitStatus();

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotControlUpgradeSignal.connect(boost::bind(&DLHangWheelCollectControlWidget::slot_ret_finished_upload, this, _1, _2));
}

DLHangWheelCollectControlWidget::~DLHangWheelCollectControlWidget()
{
	m_keyboardOperateThread->stop();
	delete m_keyboardOperateThread;
	delete m_wheelPointTreeData;
}

void DLHangWheelCollectControlWidget::initMapWidget()
{
	m_mapWidget = new BaseWidget(this);
	m_mapWidget->setTitleContent("地图");
	m_mapWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	
 	QHBoxLayout *layout = new QHBoxLayout(m_mapWidget->getCenterWidget());
 	DLCollectMapView *view = new DLCollectMapView(m_mapWidget->getCenterWidget());
 	//view->setFocusPolicy(Qt::NoFocus);
	m_pCollectMapScene = new DLCustomScene(DL_COLLECT_EDIT_TYPE, this);
	m_pCollectMapScene->setSceneRect(-20000, -20000, 40000, 40000);
 	view->setScene(m_pCollectMapScene);
 	layout->addWidget(view);
	//m_collectMapScene->load_json_map(".\\40215.smap");
    connect(view, SIGNAL(sig_add_patrolPoint()), m_pCollectMapScene, SLOT(slot_on_add_patrol_point()));
    connect(view, SIGNAL(sig_add_patrolPoint()), this, SIGNAL(signalCollectPatrolPoint()));
	connect(m_pCollectMapScene, SIGNAL(sig_flush_viewPort(QPointF)), view, SLOT(slot_on_flush_viewPort(QPointF)));
	connect(view, SIGNAL(sig_viewRect_changed(const QRectF&)), m_pCollectMapScene, SIGNAL(sig_sceneRect_changed(const QRectF&)));

	connect(m_pCollectMapScene, SIGNAL(ChangeTypeSignal(const QString &, int)), this, SLOT(ChangeTypeSlot(const QString &, int)));
	connect(m_pCollectMapScene, SIGNAL(SMAPChangedSignal()), this, SLOT(SMAPChangedSlot()));
	
	//connect(m_pCollectMapScene, SIGNAL(ReadFinishedSignal(bool)), this, SLOT(ReadFinishedSlot(bool)));


	QWidget* mapTitleWidget = m_mapWidget->getTitleBackWidget();
	if (mapTitleWidget != NULL)
	{
		QPushButton* pButtonRobotInfo = new QPushButton;
		pButtonRobotInfo->setObjectName("CommonButton");
		pButtonRobotInfo->setFixedSize(QSize(90, 25));
		pButtonRobotInfo->setText("添加设备");
		connect(pButtonRobotInfo, &QPushButton::clicked, this, [=] {
			if (m_addDeviceInfoWidget != NULL)
			{
				return;
			}
			initAddDeviceWidget();
			m_isModifyDevice = false;
			m_addDeviceInfoWidget->show();
			m_currentAddDeviceId = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
		});

// 		QPushButton* pButtonLoadMap = new QPushButton;
// 		pButtonLoadMap->setObjectName("CommonButton");
// 		pButtonLoadMap->setFixedSize(QSize(90, 25));
// 		pButtonLoadMap->setText("加载地图");
// 		connect(pButtonLoadMap, &QPushButton::clicked, this, [=] {
// 			//QString strMapFilePath = QFileDialog::getOpenFileName(this, "选择地图", "", "Map(*.smap)");
// 			QString strMapDir = QFileDialog::getExistingDirectory(this, tr("选择地图"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
// 			m_pCollectMapScene->removeall();
// 			m_pCollectMapScene->load_json_map(strMapDir);
// 
// 			QDir dir(strMapDir);
// 			//smap文件
// 			QStringList nameFilters;
// 			nameFilters.clear();
// 			nameFilters << "*.smap";
// 			QStringList files = dir.entryList(nameFilters, QDir::Files | QDir::Readable, QDir::Name);
// 			if (files.size() > 0)
// 			{//默认取第一个
// 				m_strOpenMapPath = QString("%1/%2").arg(strMapDir).arg(files.at(0));
// 			}
// 
//            // emit signalOpenSmap(strMapDir);
// 		});


		
        /*  zoom标定程序，保留当前功能，需重加lib和头文件
		QPushButton* pButtonStartDema = new QPushButton;
		pButtonStartDema->setObjectName("CommonButton");
		pButtonStartDema->setFixedSize(QSize(90, 25));
		pButtonStartDema->setText("zoom标定");
		connect(pButtonStartDema, &QPushButton::clicked, this, [=] {

			if (m_collectStWidget == NULL)
			{
				m_collectStWidget = new CollectStandardizationWidget;
			}
			m_collectStWidget->show();
			connect(m_collectStWidget, &CollectStandardizationWidget::startCollectPhotoSignals, this, [=]{
				boost::thread *runDema = new boost::thread(boost::bind(&DLHangWheelCollectControlWidget::onStartCollectZoomPicture, this));
			});
		//	runDema->join();
		});
        */
// 		QPushButton* pButtonTestButton = new QPushButton;
// 		pButtonTestButton->setObjectName("CommonButton");
// 		pButtonTestButton->setFixedSize(QSize(90, 25));
// 		pButtonTestButton->setText("测试按钮");
// 		connect(pButtonTestButton, &QPushButton::clicked, this, [=] {
// 		//	HikPointData::getInitance()->readZoomPtzValueFromTxt();
// 		});

		//QLayout* titleLayout = mapTitleWidget->layout();
		//titleLayout->addWidget(pButtonRobotInfo);
		//titleLayout->addWidget(pOverFittingBtn);
		//titleLayout->addWidget(pButtonLoadMap);
		//titleLayout->addWidget(pButtonDataSync);
		//titleLayout->addWidget(pButtonStartDema);
//		titleLayout->addWidget(pButtonTestButton);
	}
	
}

void DLHangWheelCollectControlWidget::initRobotInfo()
{
	// 机器人信息;
	m_robotInfoWidget = new QWidget;
	m_robotInfoWidget->setObjectName("RobotInfoBackWidget");
	m_robotInfoWidget->setFixedHeight(60);

	m_visibleLightRate = new InputWidget(InputWidgetType::ValueShowWidget);
	m_visibleLightRate->setTipText(tr("Zoom"));
	m_visibleLightRate->setFixedWidth(90);
	m_visibleLightRate->setShowValue("-1");

	m_visibleLightFocus = new InputWidget(InputWidgetType::ValueShowWidget);
	m_visibleLightFocus->setTipText(tr("Focus"));
	m_visibleLightFocus->setFixedWidth(90);
	m_visibleLightFocus->setShowValue("-1");

	QGroupBox* visibleGroupBox = new QGroupBox;
	visibleGroupBox->setTitle("可见光");
	visibleGroupBox->setFixedHeight(50);
	QHBoxLayout* vVisibleLayout = new QHBoxLayout(visibleGroupBox);
	vVisibleLayout->addWidget(m_visibleLightRate);
	vVisibleLayout->addWidget(m_visibleLightFocus);
	vVisibleLayout->setSpacing(5);
	vVisibleLayout->setContentsMargins(15, 2, 0, 5);
    
	m_infraredFocus = new InputWidget(InputWidgetType::ValueShowWidget);
	m_infraredFocus->setTipText(tr("Focus"));
	m_infraredFocus->setFixedWidth(90);
	m_infraredFocus->setShowValue("0");

	QGroupBox* infraredGroupBox = new QGroupBox;
	infraredGroupBox->setTitle("红外");
	QHBoxLayout* vInfraredLayout = new QHBoxLayout(infraredGroupBox);
	vInfraredLayout->addWidget(m_infraredFocus);
	vInfraredLayout->setContentsMargins(15, 2, 0, 5);

	m_PtzHRotate = new InputWidget(InputWidgetType::ValueShowWidget);
	m_PtzHRotate->setTipText(tr("Pan"));
	m_PtzHRotate->setFixedWidth(90);
	m_PtzHRotate->setShowValue("-1");

	m_PtzVRotate = new InputWidget(InputWidgetType::ValueShowWidget);
	m_PtzVRotate->setTipText(tr("Tilt"));
	m_PtzVRotate->setFixedWidth(90);
	m_PtzVRotate->setShowValue("-1");

	QGroupBox* ptzGroupBox = new QGroupBox;
	ptzGroupBox->setTitle("云台");
	QHBoxLayout* vPtzLayout = new QHBoxLayout(ptzGroupBox);
	vPtzLayout->addWidget(m_PtzHRotate);
	vPtzLayout->addWidget(m_PtzVRotate);
	vPtzLayout->setSpacing(5);
	vPtzLayout->setContentsMargins(15, 2, 0, 5);

	QHBoxLayout* hRobotInfo = new QHBoxLayout(m_robotInfoWidget);
	hRobotInfo->addWidget(visibleGroupBox);
	hRobotInfo->addWidget(infraredGroupBox);
	hRobotInfo->addWidget(ptzGroupBox);
	hRobotInfo->setSpacing(15);
	hRobotInfo->setContentsMargins(10, 5, 10, 5);	

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotInfraredFocusSignal.connect(boost::bind(&DLHangWheelCollectControlWidget::SlotSetInfraredFocus, this, _1));
}

void DLHangWheelCollectControlWidget::initRobotControl()
{
	// 机器人信息;
	//m_robotControlWidget = new BaseWidget(this);
	//m_robotControlWidget->setObjectName("robotControlWidgeet");
	//m_robotControlWidget->setTitleContent(tr("相关控制"));
	//m_robotControlWidget->setFixedHeight(176);

	// 车体控制;
	//QPushButton *CarControlButtonText = new QPushButton("车体控制", m_robotControlWidget->getCenterWidget());
	//CarControlButtonText->setFixedHeight(17);
	//CarControlButtonText->setStyleSheet("font-weight:bold;border:1px solid rgb(121,134,203);background:rgb(175, 191, 255);");

	//CustomButton *BodyControlButton = new CustomButton(Wheel_RobotControl_BodyMove, m_robotControlWidget->getCenterWidget());
	//BodyControlButton->setRadiusValue(65);
	//BodyControlButton->setArcLength(40);
	//BodyControlButton->setArcAngle(45, 89.8);

	//connect(BodyControlButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotBodyControlPressed(int)));
	//connect(BodyControlButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotBodyControlReleased(int)));

	// 查打一体机器人双光云台控制;
	//QPushButton *PlatformButtonText = new QPushButton("云台控制", m_robotControlWidget->getCenterWidget());
	//PlatformButtonText->setFixedHeight(17);
	//PlatformButtonText->setStyleSheet("font-weight:bold;border:1px solid rgb(121,134,203);background:rgb(175, 191, 255);");

	//CustomButton *PtzControlButton = new CustomButton(Wheel_RobotControl_PTZMove, m_mapWidget->getCenterWidget());
	//PtzControlButton->setRadiusValue(65);
	//PtzControlButton->setArcLength(40);
	//PtzControlButton->setArcAngle(45, 89.8);

	//connect(PtzControlButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotPTZControlPressed(int)));
	//connect(PtzControlButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotPTZControlReleased(int)));


	//QGridLayout* robotControlLayout = new QGridLayout(m_robotControlWidget->getCenterWidget());
	//robotControlLayout->addWidget(CarControlButtonText, 0, 0);
	//robotControlLayout->addWidget(BodyControlButton, 1, 0);

	//robotControlLayout->addWidget(PlatformButtonText, 0, 1);
	//robotControlLayout->addWidget(PtzControlButton, 1, 1);
	//robotControlLayout->setSpacing(15);
	//robotControlLayout->setContentsMargins(10, 5, 10, 5);
}

void DLHangWheelCollectControlWidget::initAddDeviceFirstStageWidget()
{
	m_addDeviceFirstStageWidget = new QWidget;
	m_voltageLevel = new InputWidget(InputWidgetType::ComboBox);
	m_voltageLevel->setTipText("电压等级");
	connect(m_voltageLevel, &InputWidget::signalComboBoxIndexChanged, this, [=] {
		QString voltageLevelId = m_voltageLevelMap.key(m_voltageLevel->getComboBoxCurrentContent());
		m_intervalNameMap.clear();
		std::map<QString, QString> intervalNameMap = WHEEL_DEVICE_CONFIG.getWheelRobortEquipmentIntervalFromVoltageLevelId(voltageLevelId);
		QStringList intervalNameList;
		std::map<QString, QString>::iterator intervalNameIter;
		for (intervalNameIter = intervalNameMap.begin(); intervalNameIter != intervalNameMap.end(); intervalNameIter++)
		{
			m_intervalNameMap.insert(intervalNameIter->first, intervalNameIter->second);
			intervalNameList.append(intervalNameIter->second);
		}
		m_intervalName->setComboBoxContent(intervalNameList);
	});
	
	m_areaName = new InputWidget(InputWidgetType::ComboBox);
	m_areaName->setTipText("区域名称");

	m_intervalName = new InputWidget(InputWidgetType::ComboBox);
	m_intervalName->setTipText("间隔名称");

	m_deviceType = new InputWidget(InputWidgetType::ComboBox);
	m_deviceType->setTipText("设备类型");
	connect(m_deviceType, &InputWidget::signalComboBoxIndexChanged, this, [=] {
		QString deviceTypeId = m_deviceTypeMap.key(m_deviceType->getComboBoxCurrentContent());
		// 设备子类;
		m_deviceChildTypeMap.clear();
		std::map<QString, QString> deviceChildTypeMap = WHEEL_DEVICE_CONFIG.getWheelSubDeviceNameFromDeviceType(deviceTypeId);
		QStringList deviceChildTypeList;
		std::map<QString, QString>::iterator deviceChildTypeIter;
		for (deviceChildTypeIter = deviceChildTypeMap.begin(); deviceChildTypeIter != deviceChildTypeMap.end(); deviceChildTypeIter++)
		{
			m_deviceChildTypeMap.insert(deviceChildTypeIter->first, deviceChildTypeIter->second);
			deviceChildTypeList.append(deviceChildTypeIter->second);
		}
		m_deviceChildType->setComboBoxContent(deviceChildTypeList);
		updateAddDeviceRadioState();
	});

	m_deviceChildType = new InputWidget(InputWidgetType::ComboBox);
	m_deviceChildType->setTipText("设备子类");
    // 设置comboBox下拉列表的宽度;
    QComboBox* deviceChildTypeComboBox = m_deviceChildType->getComboBoxWidget();
    deviceChildTypeComboBox->view()->setFixedWidth(210);

	connect(m_deviceChildType, &InputWidget::signalComboBoxIndexChanged, this, [=] {
		QString deviceChildTypeId = m_deviceChildTypeMap.key(m_deviceChildType->getComboBoxCurrentContent());
		m_devicePointPosMap.clear();
		std::map<QString, WheelDevicePointNameStruct> devicePointPosMap = WHEEL_DEVICE_CONFIG.getWheelDevicePointNameFromSubDeviceType(deviceChildTypeId);
		QStringList devicePointPosList;
		std::map<QString, WheelDevicePointNameStruct>::iterator devicePointPosIter;
		for (devicePointPosIter = devicePointPosMap.begin(); devicePointPosIter != devicePointPosMap.end(); devicePointPosIter++)
		{
			m_devicePointPosMap.insert(devicePointPosIter->first, devicePointPosIter->second);
			devicePointPosList.append(devicePointPosIter->second.device_point_type_name);
		}
		m_devicePointPos->setComboBoxContent(devicePointPosList);
		updateAddDeviceRadioState();
	});

	m_devicePointPos = new InputWidget(InputWidgetType::ComboBox);
	m_devicePointPos->setTipText("设备点位");
    // 设置comboBox下拉列表的宽度;
    QComboBox* devicePointPosComboBox = m_devicePointPos->getComboBoxWidget();
    devicePointPosComboBox->view()->setFixedWidth(150);

	connect(m_devicePointPos, &InputWidget::signalComboBoxIndexChanged, this, [=] {
		updateAddDeviceRadioState();
	});

	m_radioButtonA = new QRadioButton("A相");
	m_radioButtonA->setChecked(true);
	m_radioButtonB = new QRadioButton("B相");
	m_radioButtonC = new QRadioButton("C相");

	QWidget* addDeviceButtonBackWidget = new QWidget;
	addDeviceButtonBackWidget->setFixedWidth(200);
	m_pButtonAddDeviceNextStage = new QPushButton("下一步");
	m_pButtonAddDeviceNextStage->setFixedSize(QSize(80, 25));
	m_pButtonAddDeviceNextStage->setObjectName("CommonButton");
	connect(m_pButtonAddDeviceNextStage, &QPushButton::clicked, this, [=] {
		if (m_voltageLevel->getComboBoxCurrentContent().isEmpty() || m_areaName->getComboBoxCurrentContent().isEmpty() ||
			m_intervalName->getComboBoxCurrentContent().isEmpty() || m_deviceType->getComboBoxCurrentContent().isEmpty() ||
			m_deviceChildType->getComboBoxCurrentContent().isEmpty() || m_devicePointPos->getComboBoxCurrentContent().isEmpty())
		{
			DLMessageBox* messageBox = new DLMessageBox(m_addDeviceInfoWidget);
			messageBox->setFixedWidth(250);
			messageBox->setMessageContent("请选择正确选项");
			messageBox->setWindowModality(Qt::ApplicationModal);
			messageBox->show();
			return;
		}
		// 根据点位判断下一步是跳转可见光设备还是红外设备;
		WheelJudgeTakePhoto type = WHEEL_DEVICE_CONFIG.getWheelChooseRecognitionBool(WHEEL_DEVICE_CONFIG.getWheelDevicePointTypeUUidQString(m_devicePointPos->getComboBoxCurrentContent()));
		if (type == WheelJudgeTakePhoto::VisibleLightJudge)
		{
			m_addDeviceStackedWidget->setCurrentIndex(0);
		}
		else if(type == WheelJudgeTakePhoto::InfraredLightJudge)
		{
			m_addDeviceStackedWidget->setCurrentIndex(1);
		}
	});

	QHBoxLayout* hRadioLayout = new QHBoxLayout;
	hRadioLayout->addWidget(m_radioButtonA);
	hRadioLayout->addWidget(m_radioButtonB);
	hRadioLayout->addWidget(m_radioButtonC);
	hRadioLayout->setSpacing(10);
	hRadioLayout->setMargin(0);

	QHBoxLayout* hAddDeviceButtonLayout = new QHBoxLayout(addDeviceButtonBackWidget);
	hAddDeviceButtonLayout->addStretch();
	hAddDeviceButtonLayout->addWidget(m_pButtonAddDeviceNextStage);

	QGridLayout* gDeviceInfoLayout = new QGridLayout(m_addDeviceFirstStageWidget);
	gDeviceInfoLayout->addWidget(m_voltageLevel, 0, 0);
	gDeviceInfoLayout->addWidget(m_areaName, 0, 1);
	gDeviceInfoLayout->addWidget(m_intervalName, 0, 2);
	gDeviceInfoLayout->addWidget(m_deviceType, 1, 0);
	gDeviceInfoLayout->addWidget(m_deviceChildType, 1, 1);
	gDeviceInfoLayout->addWidget(m_devicePointPos, 1, 2);
	gDeviceInfoLayout->addLayout(hRadioLayout, 2, 2);
	gDeviceInfoLayout->addWidget(addDeviceButtonBackWidget, 3, 2);
	gDeviceInfoLayout->setMargin(0);
	gDeviceInfoLayout->setVerticalSpacing(5);

	initAddDeviceComboBox();
}

void DLHangWheelCollectControlWidget::initAddDeviceComboBox()
{
	// 电压等级;
	m_voltageLevelMap.clear();
	std::map<QString, QString> voltageLevelMap = WHEEL_DEVICE_CONFIG.getWheelVoltageLevelDataMap();
	QStringList voltageLevelList;
	std::map<QString, QString>::iterator voltageLevelIter;
	for (voltageLevelIter = voltageLevelMap.begin(); voltageLevelIter != voltageLevelMap.end(); voltageLevelIter++)
	{
		m_voltageLevelMap.insert(voltageLevelIter->first, voltageLevelIter->second);
		voltageLevelList.append(voltageLevelIter->second);
	}
	m_voltageLevel->setComboBoxContent(voltageLevelList);
    if (m_voltageLevelIndex < 0)
    {
        m_voltageLevelIndex = 0;
    }

	if (voltageLevelList.count() > m_voltageLevelIndex)
	{
		m_voltageLevel->setComboBoxCurrentIndex(m_voltageLevelIndex);
	}
	else
	{
		m_voltageLevelIndex = 0;
	}

	// 区域名称;
	m_areaNameMap.clear();
	std::map<QString, QString> areaNameMap = WHEEL_DEVICE_CONFIG.getWheelDeviceAreaDataMap();
	QStringList areaNameList;
	std::map<QString, QString>::iterator areaNameIter;
	for (areaNameIter = areaNameMap.begin(); areaNameIter != areaNameMap.end(); areaNameIter++)
	{
		m_areaNameMap.insert(areaNameIter->first, areaNameIter->second);
		areaNameList.append(areaNameIter->second);
	}
	m_areaName->setComboBoxContent(areaNameList);
	if (areaNameList.count() > m_areaNameIndex)
	{
		m_areaName->setComboBoxCurrentIndex(m_areaNameIndex);
	}
	else
	{
		m_areaNameIndex = 0;
	}

	// 间隔名称;
	m_intervalNameMap.clear();
	if (voltageLevelList.size())
	{
		std::map<QString, QString> intervalNameMap = WHEEL_DEVICE_CONFIG.getWheelRobortEquipmentIntervalFromVoltageLevelId(m_voltageLevelMap.key(voltageLevelList.at(m_voltageLevelIndex)));
		QStringList intervalNameList;
		std::map<QString, QString>::iterator intervalNameIter;
		for (intervalNameIter = intervalNameMap.begin(); intervalNameIter != intervalNameMap.end(); intervalNameIter++)
		{
			m_intervalNameMap.insert(intervalNameIter->first, intervalNameIter->second);
			intervalNameList.append(intervalNameIter->second);
		}
		m_intervalName->setComboBoxContent(intervalNameList);
		if (intervalNameList.count() > m_intervalNameIndex)
		{
			m_intervalName->setComboBoxCurrentIndex(m_intervalNameIndex);
		}
		else
		{
			m_intervalNameIndex = 0;
		}
	}
    else
    {
        m_intervalName->setComboBoxContent(QStringList() << "");
    }
	

	// 设备类型;
	m_deviceTypeMap.clear();
	std::map<QString, QString> deviceTypeMap = WHEEL_DEVICE_CONFIG.getWheelDeviceTypeDataMap();
	QStringList deviceTypeList;
	std::map<QString, QString>::iterator deviceTypeIter;
	for (deviceTypeIter = deviceTypeMap.begin(); deviceTypeIter != deviceTypeMap.end(); deviceTypeIter++)
	{
		m_deviceTypeMap.insert(deviceTypeIter->first, deviceTypeIter->second);
		deviceTypeList.append(deviceTypeIter->second);
	}
	m_deviceType->setComboBoxContent(deviceTypeList);
    if (m_deviceTypeIndex < 0)
    {
        m_deviceTypeIndex = 0;
    }
	if (deviceTypeList.count() > m_deviceTypeIndex)
	{
		m_deviceType->setComboBoxCurrentIndex(m_deviceTypeIndex);
	}
	else
	{
		m_deviceTypeIndex = 0;
	}

	// 设备子类;
	m_deviceChildTypeMap.clear();
	if (deviceTypeList.size())
	{
		std::map<QString, QString> deviceChildTypeMap = WHEEL_DEVICE_CONFIG.getWheelSubDeviceNameFromDeviceType(m_deviceTypeMap.key(deviceTypeList.at(m_deviceTypeIndex)));
		QStringList deviceChildTypeList;
		std::map<QString, QString>::iterator deviceChildTypeIter;
		for (deviceChildTypeIter = deviceChildTypeMap.begin(); deviceChildTypeIter != deviceChildTypeMap.end(); deviceChildTypeIter++)
		{
			m_deviceChildTypeMap.insert(deviceChildTypeIter->first, deviceChildTypeIter->second);
			deviceChildTypeList.append(deviceChildTypeIter->second);
		}
		m_deviceChildType->setComboBoxContent(deviceChildTypeList);
        if (m_deviceChildTypeIndex < 0)
        {
            m_deviceChildTypeIndex = 0;
        }
		if (deviceChildTypeList.count() > m_deviceChildTypeIndex)
		{
			m_deviceChildType->setComboBoxCurrentIndex(m_deviceChildTypeIndex);
		}
		else
		{
			m_deviceChildTypeIndex = 0;
		}


		// 设备点位;
		m_devicePointPosMap.clear();
		if (deviceChildTypeList.size())
		{
			std::map<QString, WheelDevicePointNameStruct> devicePointPosMap = WHEEL_DEVICE_CONFIG.getWheelDevicePointNameFromSubDeviceType(m_deviceChildTypeMap.key(deviceChildTypeList.at(m_deviceChildTypeIndex)));
			QStringList devicePointPosList;
			std::map<QString, WheelDevicePointNameStruct>::iterator devicePointPosIter;
			for (devicePointPosIter = devicePointPosMap.begin(); devicePointPosIter != devicePointPosMap.end(); devicePointPosIter++)
			{
				m_devicePointPosMap.insert(devicePointPosIter->first, devicePointPosIter->second);
				devicePointPosList.append(devicePointPosIter->second.device_point_type_name);
			}
			m_devicePointPos->setComboBoxContent(devicePointPosList);
			if (devicePointPosList.count() > m_devicePointPosIndex)
			{
				m_devicePointPos->setComboBoxCurrentIndex(m_devicePointPosIndex);
			}
			else
			{
				m_devicePointPosIndex = 0;
			}
		}
	}

	updateAddDeviceRadioState();
}

void DLHangWheelCollectControlWidget::initAddDeviceVisibleNearStageWidget()
{
	m_addDeviceVisibleNearWidget = new QWidget;
	// 倍率;
	m_visibleLightScaleNearWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_visibleLightScaleNearWidget->setTipText(tr("可见光倍率"));
	m_visibleLightScaleNearWidget->setButtonText(tr("应用"));

	connect(m_visibleLightScaleNearWidget, &InputWidget::signalButtonClicked, this, [=]() {
		m_visibleLightVideoBackWidget->setVideoZoomAbs(m_visibleLightScaleNearWidget->getLineEditText().toInt());
	});

	// 焦距;
	m_visibleLightFocusNearWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_visibleLightFocusNearWidget->setTipText(tr("可见光焦距"));
	m_visibleLightFocusNearWidget->setButtonText(tr("应用"));
	connect(m_visibleLightFocusNearWidget, &InputWidget::signalButtonClicked, this, [=]() {
		m_visibleLightVideoBackWidget->setVideoFocusAbs(m_visibleLightFocusNearWidget->getLineEditText().toInt());
	});

	// 云台水平方向旋转;
	m_ptzHRotateWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_ptzHRotateWidget->setTipText("云台水平旋转");
	m_ptzHRotateWidget->setButtonText("应用");
	m_ptzHRotateWidget->setFixedWidth(290);
    
	m_ptzHRotateWidget->setLineEditValidator(new QIntValidator(0, 36000, this));
	connect(m_ptzHRotateWidget, &InputWidget::signalButtonClicked, this, [=]() {
		//WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(m_ptzHRotateWidget->getLineEditText().toInt(), -1);
        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_monodrome_req(m_ptzHRotateWidget->getLineEditText().toInt(), 0);
	});

	// 云台垂直方向旋转;
	m_ptzVRotateWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_ptzVRotateWidget->setTipText("云台垂直旋转");
	m_ptzVRotateWidget->setButtonText("应用");
	m_ptzVRotateWidget->setFixedWidth(290);

    QRegExp regx("([0-9]{0,3})|([1-8][0-9]{0,3})|(2[7-9][0-9]{3,3})|(3[0-5][0-9]{3,3})|(9000)|(36000)");
    QValidator *userNameValidator = new QRegExpValidator(regx, m_ptzVRotateWidget);
    m_ptzVRotateWidget->setLineEditValidator(userNameValidator);

	connect(m_ptzVRotateWidget, &InputWidget::signalButtonClicked, this, [=]() {
		//WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(-1, m_ptzVRotateWidget->getLineEditText().toInt());
        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_monodrome_req(m_ptzVRotateWidget->getLineEditText().toInt(), 1);
	});

	m_saveZoomShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_saveZoomShowWidget->setFixedWidth(80);
	m_saveZoomShowWidget->setTipText("Zoom");
	m_saveZoomShowWidget->setShowValue("-1");

	m_saveFocusShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_saveFocusShowWidget->setFixedWidth(80);
	m_saveFocusShowWidget->setTipText("Focus");
	m_saveFocusShowWidget->setShowValue("-1");

	m_savePtzPanShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_savePtzPanShowWidget->setFixedWidth(80);
	m_savePtzPanShowWidget->setTipText("Pan");
	m_savePtzPanShowWidget->setShowValue("-1");

	m_savePtzTiltShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_savePtzTiltShowWidget->setFixedWidth(80);
	m_savePtzTiltShowWidget->setTipText("Tilt");
	m_savePtzTiltShowWidget->setShowValue("-1");

	QPushButton* pButtonSave = new QPushButton("保存");
	pButtonSave->setObjectName("CommonButtonChange");
	pButtonSave->setFixedSize(QSize(80, 22));
	connect(pButtonSave, &QPushButton::clicked, this, [=] {
		// 点击保存,相机，云台信息保存;
		m_zoomNearValue = m_visibleLightRate->getShowValue().toInt();
		m_focusNearValue = m_visibleLightFocus->getShowValue().toInt();
		m_ptzPanValue = m_PtzHRotate->getShowValue().toInt();
		m_ptzTiltValue = m_PtzVRotate->getShowValue().toInt();

		m_saveZoomShowWidget->setShowValue(QString::number(m_zoomNearValue));
		m_saveFocusShowWidget->setShowValue(QString::number(m_focusNearValue));
		m_savePtzPanShowWidget->setShowValue(QString::number(m_ptzPanValue));
		m_savePtzTiltShowWidget->setShowValue(QString::number(m_ptzTiltValue));
	});

	QPushButton* pButtonSet = new QPushButton("设置");
	pButtonSet->setObjectName("CommonButtonChange");
	pButtonSet->setFixedSize(QSize(80, 22));
	connect(pButtonSet, &QPushButton::clicked, this, [=] {
		// 云台设置;
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(m_savePtzPanShowWidget->getLineEditText().toInt(), m_savePtzTiltShowWidget->getLineEditText().toInt());
        // 将当前保存的参数全部应用;
        // 相机设置;
        m_visibleLightVideoBackWidget->setVideoZoomAbs(m_saveZoomShowWidget->getShowValue().toInt());
        m_visibleLightVideoBackWidget->setVideoFocusAbs(m_saveFocusShowWidget->getShowValue().toInt());
	});

    QPushButton* pButtonAllSet = new QPushButton("全部应用");
    pButtonAllSet->setObjectName("CommonButtonChange");
    pButtonAllSet->setFixedSize(QSize(80, 22));
    connect(pButtonAllSet, &QPushButton::clicked, this, [=] {
        // 云台设置;
        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(m_ptzHRotateWidget->getLineEditText().toInt(), m_ptzVRotateWidget->getLineEditText().toInt());
        m_visibleLightVideoBackWidget->setVideoZoomAndFocus(m_visibleLightScaleNearWidget->getLineEditText().toInt(), m_visibleLightFocusNearWidget->getLineEditText().toInt());
    //    m_visibleLightVideoBackWidget->setVideoZoomAbs(m_visibleLightScaleNearWidget->getLineEditText().toInt());
    //    m_visibleLightVideoBackWidget->setVideoFocusAbs(m_visibleLightFocusNearWidget->getLineEditText().toInt());
    });

// 	QGridLayout* gSaveInfoLayout = new QGridLayout;
// 	gSaveInfoLayout->addWidget(m_saveZoomShowWidget, 0, 0);
// 	gSaveInfoLayout->addWidget(m_saveFocusShowWidget, 1, 0);
// 	gSaveInfoLayout->addWidget(m_savePtzPanShowWidget, 0, 1);
// 	gSaveInfoLayout->addWidget(m_savePtzTiltShowWidget, 1, 1);
// 	gSaveInfoLayout->addWidget(pButtonSave, 0, 2);
//     gSaveInfoLayout->addWidget(pButtonSet, 1, 2);
//     gSaveInfoLayout->addWidget(pButtonAllSet, 1, 3);
// 
// 	gSaveInfoLayout->setVerticalSpacing(0);
// 	gSaveInfoLayout->setHorizontalSpacing(22);
// 	gSaveInfoLayout->setMargin(0);

	m_pButtonAddDeviceVisibleNearNextStage = new QPushButton("下一步");
	m_pButtonAddDeviceVisibleNearNextStage->setFixedSize(QSize(80, 25));
	m_pButtonAddDeviceVisibleNearNextStage->setObjectName("CommonButtonChange");
	connect(m_pButtonAddDeviceVisibleNearNextStage, &QPushButton::clicked, this, [=] {
		// 点击当前信息是否正确;
		if (m_zoomNearValue == -1 || m_focusNearValue == -1 || m_ptzPanValue == -1)
		{
			// 说明未保存数据或者数据读取错误;
			DLMessageBox* messageBox = new DLMessageBox(m_addDeviceInfoWidget);
			messageBox->setFixedWidth(280);
			messageBox->setMessageContent("数据读取错误/未保存数据");
			messageBox->setWindowModality(Qt::ApplicationModal);
			messageBox->show();
			return;
		}
		QString deviceId;
		if (m_isModifyDevice)
		{
			deviceId = m_strModifyDeviceId;
		}
		else
		{
			deviceId = m_currentAddDeviceId;
		}
		QString filePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/Devices/" + deviceId + "/";
		QDir dir;
		bool isSuccess = dir.mkpath(filePath);
		//m_visibleLightVideoBackWidget->cameraCapture(filePath + deviceId);
        if (!m_visibleLightVideoBackWidget->cameraCaptureBool(filePath + deviceId))
        {
            DLMessageBox* messageBox = new DLMessageBox(m_addDeviceInfoWidget);
            messageBox->setFixedWidth(280);
            messageBox->setMessageContent("拍照失败！");
            messageBox->setWindowModality(Qt::ApplicationModal);
            messageBox->show();
            return;
        }
		m_addDeviceStackedWidget->setCurrentIndex(1);
	});

	m_pButtonAddDeviceVisibleNearLastStage = new QPushButton("上一步");
	m_pButtonAddDeviceVisibleNearLastStage->setFixedSize(QSize(80, 25));
	m_pButtonAddDeviceVisibleNearLastStage->setObjectName("CommonButtonChange");
	connect(m_pButtonAddDeviceVisibleNearLastStage, &QPushButton::clicked, this, [=] {
		m_addDeviceStackedWidget->setCurrentIndex(0);
	});

// 	QHBoxLayout* hTurnPageButtonLayout = new QHBoxLayout;
// 	//hTurnPageButtonLayout->addWidget(m_pButtonAddDeviceVisibleNearLastStage);
// 	hTurnPageButtonLayout->addWidget(m_pButtonAddDeviceVisibleNearNextStage);
// 	hTurnPageButtonLayout->setSpacing(15);
// 	hTurnPageButtonLayout->setContentsMargins(0, 22, 0, 0);

// 	QWidget* addDeviceButtonBackWidget = new QWidget;
// 	addDeviceButtonBackWidget->setFixedWidth(600);

// 	QHBoxLayout* hAddDeviceButtonLayout = new QHBoxLayout(addDeviceButtonBackWidget);
// 	hAddDeviceButtonLayout->addLayout(gSaveInfoLayout);
// 	hAddDeviceButtonLayout->addStretch();
// 	hAddDeviceButtonLayout->addLayout(hTurnPageButtonLayout);
// 	hAddDeviceButtonLayout->setSpacing(15);
// 	hAddDeviceButtonLayout->setMargin(0);

	QGridLayout* gDeviceInfoLayout = new QGridLayout(m_addDeviceVisibleNearWidget);

    gDeviceInfoLayout->addWidget(m_visibleLightScaleNearWidget, 0, 0);
	gDeviceInfoLayout->addWidget(m_ptzHRotateWidget, 0, 1);
	gDeviceInfoLayout->addWidget(m_visibleLightFocusNearWidget, 1, 0);
    gDeviceInfoLayout->addWidget(m_ptzVRotateWidget, 1, 1);

    gDeviceInfoLayout->addWidget(m_saveZoomShowWidget, 0, 2);
    gDeviceInfoLayout->addWidget(m_saveFocusShowWidget, 1, 2);
    gDeviceInfoLayout->addWidget(m_savePtzPanShowWidget, 0, 3);
    gDeviceInfoLayout->addWidget(m_savePtzTiltShowWidget, 1, 3);

    gDeviceInfoLayout->addWidget(pButtonSave, 0, 4);
    gDeviceInfoLayout->addWidget(pButtonSet, 1, 4);
    gDeviceInfoLayout->addWidget(pButtonAllSet, 1, 5);
    gDeviceInfoLayout->addWidget(m_pButtonAddDeviceVisibleNearNextStage, 1, 6);
//    gDeviceInfoLayout->addWidget(addDeviceButtonBackWidget, 0, 0, 1, 2);
//    gDeviceInfoLayout->addWidget(addDeviceButtonBackWidget, 0, 2, 2, 2);
	gDeviceInfoLayout->setMargin(0);
	gDeviceInfoLayout->setVerticalSpacing(15);

//     QVBoxLayout *tttt = new QVBoxLayout(m_addDeviceVisibleNearWidget);
//     tttt->addWidget(m_deviceNameLabel_1);
//     tttt->addLayout(gDeviceInfoLayout);
//     tttt->setMargin(0);
//    tttt->setVerticalSpacing(15);

}

void DLHangWheelCollectControlWidget::initAddDeviceVisibleFarStageWidget()
{
	m_addDeviceVisibleFarWidget = new QWidget;
	// 倍率;
	m_visibleLightScaleFarWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_visibleLightScaleFarWidget->setTipText(tr("可见光倍率"));
	m_visibleLightScaleFarWidget->setButtonText(tr("应用"));

	connect(m_visibleLightScaleFarWidget, &InputWidget::signalButtonClicked, this, [=]() {
		m_visibleLightVideoBackWidget->setVideoZoomAbs(m_visibleLightScaleFarWidget->getLineEditText().toInt());
	});

	// 焦距;
	m_visibleLightFocusFarWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_visibleLightFocusFarWidget->setTipText(tr("可见光焦距"));
	m_visibleLightFocusFarWidget->setButtonText(tr("应用"));
	connect(m_visibleLightFocusFarWidget, &InputWidget::signalButtonClicked, this, [=]() {
		m_visibleLightVideoBackWidget->setVideoFocusAbs(m_visibleLightFocusFarWidget->getLineEditText().toInt());
	});

	m_saveFarZoomShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_saveFarZoomShowWidget->setFixedWidth(80);
	m_saveFarZoomShowWidget->setTipText("Zoom");
	m_saveFarZoomShowWidget->setShowValue("-1");

	m_saveFarFocusShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_saveFarFocusShowWidget->setFixedWidth(80);
	m_saveFarFocusShowWidget->setTipText("Focus");
	m_saveFarFocusShowWidget->setShowValue("-1");

	QPushButton* pButtonSave = new QPushButton("保存");
	pButtonSave->setObjectName("CommonButtonChange");
	pButtonSave->setFixedSize(QSize(80, 22));
	connect(pButtonSave, &QPushButton::clicked, this, [=] {
		// 点击保存,相机，云台信息保存;
		m_zoomFarValue = m_visibleLightRate->getShowValue().toInt();
		m_focusFarValue = m_visibleLightFocus->getShowValue().toInt();

		m_saveFarZoomShowWidget->setShowValue(QString::number(m_zoomFarValue));
		m_saveFarFocusShowWidget->setShowValue(QString::number(m_focusFarValue));
	});

	QPushButton* pButtonSet = new QPushButton("设置");
	pButtonSet->setObjectName("CommonButtonChange");
	pButtonSet->setFixedSize(QSize(80, 22));
	connect(pButtonSet, &QPushButton::clicked, this, [=] {
		// 将当前保存的参数全部应用;

		m_visibleLightVideoBackWidget->setVideoZoomAbs(m_saveFarZoomShowWidget->getShowValue().toInt());
		m_visibleLightVideoBackWidget->setVideoFocusAbs(m_saveFarFocusShowWidget->getShowValue().toInt());

	});

	QGridLayout* gSaveInfoLayout = new QGridLayout;
	gSaveInfoLayout->addWidget(m_saveFarZoomShowWidget, 0, 0);
	gSaveInfoLayout->addWidget(m_saveFarFocusShowWidget, 1, 0);
	gSaveInfoLayout->addWidget(pButtonSave, 0, 1);
	gSaveInfoLayout->addWidget(pButtonSet, 1, 1);
	gSaveInfoLayout->setVerticalSpacing(0);
	gSaveInfoLayout->setHorizontalSpacing(22);
	gSaveInfoLayout->setMargin(0);

	m_pButtonAddDeviceVisibleFinish = new QPushButton("完成");
	m_pButtonAddDeviceVisibleFinish->setFixedSize(QSize(80, 25));
	m_pButtonAddDeviceVisibleFinish->setObjectName("CommonButtonChange");
	connect(m_pButtonAddDeviceVisibleFinish, &QPushButton::clicked, this, [=] {
		if (m_zoomFarValue == -1 || m_focusFarValue == -1)
		{
			// 说明数据读取错误;
			DLMessageBox* messageBox = new DLMessageBox(m_addDeviceInfoWidget);
			messageBox->setFixedWidth(280);
			messageBox->setMessageContent("数据读取错误");
			messageBox->setWindowModality(Qt::ApplicationModal);
			messageBox->show();
			return;
		}
		else if (m_pointId == -1)
		{
			// 说明数据读取错误;
			DLMessageBox* messageBox = new DLMessageBox(m_addDeviceInfoWidget);
			messageBox->setFixedWidth(280);
			messageBox->setMessageContent("点位信息错误");
			messageBox->setWindowModality(Qt::ApplicationModal);
			messageBox->show();
			return;
		}

		
		// 设备信息;
		// 如果当前是修改设备，只修改相机，云台参数;
// 		if (m_isModifyDevice)
// 		{
			WheelRobortDeviceParameterStruct paratmeter;
			paratmeter.device_uuid = m_strModifyDeviceId;
			paratmeter.point_id = m_pointId;
			paratmeter.hc_focus_near = m_focusNearValue;
			paratmeter.hc_focus_far = m_focusFarValue;
			paratmeter.hc_zoom_near = m_zoomNearValue;
			paratmeter.hc_zoom_far = m_zoomFarValue;
			paratmeter.ptz_pan = m_ptzPanValue;
			paratmeter.ptz_tilt = m_ptzTiltValue;
			paratmeter.mag_focus = -1;
			paratmeter.video_length = 10;
			paratmeter.audio_length = 10;
			WHEEL_BACK_TO_CORE_SOCKET.robot_device_update_req(paratmeter);
/*		}
		else
		{
			WheelRobotInsertDeviceStruct insertdeviceData;
			insertdeviceData.device.device_uuid = m_currentAddDeviceId;
			insertdeviceData.device.voltage_level_id = m_voltageLevelMap.key(m_voltageLevel->getComboBoxCurrentContent());
			insertdeviceData.device.equipment_interval_uuid = m_intervalNameMap.key(m_intervalName->getComboBoxCurrentContent());
			insertdeviceData.device.device_area_uuid = m_areaNameMap.key(m_areaName->getComboBoxCurrentContent());
			insertdeviceData.device.device_type_uuid = m_deviceTypeMap.key(m_deviceType->getComboBoxCurrentContent());
			insertdeviceData.device.sub_device_type_uuid = m_deviceChildTypeMap.key(m_deviceChildType->getComboBoxCurrentContent());
			insertdeviceData.device.device_point_type_uuid = WHEEL_DEVICE_CONFIG.getWheelDevicePointTypeUUidQString(m_devicePointPos->getComboBoxCurrentContent());
			WheelDevicePointNameStruct pointPosInfo = m_devicePointPosMap[insertdeviceData.device.device_point_type_uuid];

			insertdeviceData.device.unit_type_uuid = pointPosInfo.unit_type_id;
			insertdeviceData.device.recognition_type_id = WheelRobotRecognizeType(pointPosInfo.recognition_type_id);
			insertdeviceData.device.meter_type_id = WheelRobotMeterType(pointPosInfo.meter_type_id);
			insertdeviceData.device.fever_type_id = WheelRobotFeverType(pointPosInfo.fever_type_id);
			insertdeviceData.device.threshold_filename = WHEEL_DEVICE_CONFIG.getWheelThresholdUUidFromMeterType(pointPosInfo.meter_type_id);
			insertdeviceData.device.save_type_id = WheelRobotSaveType(pointPosInfo.save_type_id);

			if (!m_radioButtonA->isEnabled())
			{
				insertdeviceData.device.device_phase_id = WheelRobotPhaseType::WHEEL_ROBOT_PHASE_TYPE_NONE;
			}
			else
			{
				if (m_radioButtonA->isChecked())
				{
					insertdeviceData.device.device_phase_id = WheelRobotPhaseType::WHEEL_ROBOT_PHASE_TYPE_A_PHASE;
				}
				else if (m_radioButtonB->isChecked())
				{
					insertdeviceData.device.device_phase_id = WheelRobotPhaseType::WHEEL_ROBOT_PHASE_TYPE_B_PHASE;
				}
				else if (m_radioButtonC->isChecked())
				{
					insertdeviceData.device.device_phase_id = WheelRobotPhaseType::WHEEL_ROBOT_PHASE_TYPE_C_PHASE;
				}
			}

			insertdeviceData.paratmeter.device_uuid = insertdeviceData.device.device_uuid;
			insertdeviceData.paratmeter.point_id = m_pointId;
			insertdeviceData.paratmeter.hc_focus_near = m_focusNearValue;
			insertdeviceData.paratmeter.hc_focus_far = m_focusFarValue;
			insertdeviceData.paratmeter.hc_zoom_near = m_zoomNearValue;
			insertdeviceData.paratmeter.hc_zoom_far = m_zoomFarValue;
			insertdeviceData.paratmeter.ptz_pan = m_ptzPanValue;
			insertdeviceData.paratmeter.ptz_tilt = m_ptzTiltValue;
			insertdeviceData.paratmeter.mag_focus = m_infraredFocus->getShowValue().toInt();
			insertdeviceData.paratmeter.video_length = 10;
			insertdeviceData.paratmeter.audio_length = 10;

			//insertdeviceData
			WHEEL_BACK_TO_CORE_SOCKET.robot_device_insert_req(insertdeviceData);

			// 保存此次的设备ComboBox选择类型;
			m_voltageLevelIndex = m_voltageLevel->getComboBoxCurrentIndex();
			m_areaNameIndex = m_areaName->getComboBoxCurrentIndex();
			m_intervalNameIndex = m_intervalName->getComboBoxCurrentIndex();
			m_deviceTypeIndex = m_deviceType->getComboBoxCurrentIndex();
			m_deviceChildTypeIndex = m_deviceChildType->getComboBoxCurrentIndex();
			m_devicePointPosIndex = m_devicePointPos->getComboBoxCurrentIndex();
		}
*/
		QString deviceId;
		if (m_isModifyDevice)
		{
			deviceId = m_strModifyDeviceId;
		}
		else
		{
			deviceId = m_currentAddDeviceId;
		}
		QString filePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/Devices/" + deviceId + "/" + deviceId + "_twostage";
		m_visibleLightVideoBackWidget->cameraCapture(filePath);
        
        m_addDeviceInfoWidget->setEnabled(false);
        changeWidgetButtonStyle(BUTTON_STYLE_GRAY);
//        DLMessageBox::showDLMessageBox(this, "提示", QString("采集完成！"), MessageButtonType::BUTTON_OK, true);
// 		m_addDeviceInfoWidget->close();
// 		m_addDeviceInfoWidget = NULL;
	});

	m_pButtonAddDeviceVisibleFarLastStage = new QPushButton("上一步");
	m_pButtonAddDeviceVisibleFarLastStage->setFixedSize(QSize(80, 25));
	m_pButtonAddDeviceVisibleFarLastStage->setObjectName("CommonButtonChange");
	connect(m_pButtonAddDeviceVisibleFarLastStage, &QPushButton::clicked, this, [=] {
		m_addDeviceStackedWidget->setCurrentIndex(0);
	});

	QWidget* addDeviceButtonBackWidget = new QWidget;
	addDeviceButtonBackWidget->setFixedWidth(600);

	QHBoxLayout* hButtonLayout = new QHBoxLayout();
	hButtonLayout->addWidget(m_pButtonAddDeviceVisibleFarLastStage);
	hButtonLayout->addWidget(m_pButtonAddDeviceVisibleFinish);
	hButtonLayout->setContentsMargins(0, 22, 0, 0);

	QHBoxLayout* hAddDeviceButtonLayout = new QHBoxLayout(addDeviceButtonBackWidget);
	hAddDeviceButtonLayout->addLayout(gSaveInfoLayout);
	hAddDeviceButtonLayout->addStretch();
	hAddDeviceButtonLayout->addLayout(hButtonLayout);
	
	hAddDeviceButtonLayout->setSpacing(15);
	hAddDeviceButtonLayout->setMargin(0);

	QGridLayout* gDeviceInfoLayout = new QGridLayout(m_addDeviceVisibleFarWidget);
	gDeviceInfoLayout->addWidget(m_visibleLightScaleFarWidget, 0, 0);
    gDeviceInfoLayout->addWidget(m_visibleLightFocusFarWidget, 1, 0);

    gDeviceInfoLayout->addWidget(m_saveFarZoomShowWidget, 0, 1);
    gDeviceInfoLayout->addWidget(m_saveFarFocusShowWidget, 1, 1);
    gDeviceInfoLayout->addWidget(pButtonSave, 0, 2);
    gDeviceInfoLayout->addWidget(pButtonSet, 1, 2);

    gDeviceInfoLayout->addWidget(m_pButtonAddDeviceVisibleFarLastStage, 1, 3);
    gDeviceInfoLayout->addWidget(m_pButtonAddDeviceVisibleFinish, 1, 4);

//	gDeviceInfoLayout->addWidget(addDeviceButtonBackWidget, 2, 0, 1, 2);
	gDeviceInfoLayout->setMargin(0);
	gDeviceInfoLayout->setVerticalSpacing(15);
}

void DLHangWheelCollectControlWidget::initAddDeviceInfraredStageWidget()
{
	m_addDeviceInfraredWidget = new QWidget;

	// 焦距;
	m_infraredFocusWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_infraredFocusWidget->setTipText(tr("红外焦距"));
	m_infraredFocusWidget->setButtonText(tr("应用"));

	//m_infraredFocusWidget->setLineEditText("0");

	connect(m_infraredFocusWidget, &InputWidget::signalButtonClicked, this, [=]() {
		//m_infraredVideoBackWidget->setInfraredVideoFocusAbs(m_infraredFocusWidget->getLineEditText().toInt());
        WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_set_focus_req(m_infraredFocusWidget->getLineEditText().toInt());
	});

	// 云台水平方向旋转;
	m_ptzHRotateInfraredWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_ptzHRotateInfraredWidget->setTipText("云台水平旋转");
	m_ptzHRotateInfraredWidget->setButtonText("应用");
	m_ptzHRotateInfraredWidget->setFixedWidth(290);
	m_ptzHRotateInfraredWidget->setLineEditValidator(new QIntValidator(0, 36000, this));
	connect(m_ptzHRotateInfraredWidget, &InputWidget::signalButtonClicked, this, [=]() {
		//WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(m_ptzHRotateInfraredWidget->getLineEditText().toInt(), -1);
        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_monodrome_req(m_ptzHRotateInfraredWidget->getLineEditText().toInt(), 0);
	});

	// 云台垂直方向旋转;
	m_ptzVRotateInfraredWidget = new InputWidget(InputWidgetType::LineEditWithButton);
	m_ptzVRotateInfraredWidget->setTipText("云台垂直旋转");
	m_ptzVRotateInfraredWidget->setButtonText("应用");
	m_ptzVRotateInfraredWidget->setFixedWidth(290);
	m_ptzVRotateInfraredWidget->setLineEditValidator(new QIntValidator(0, 9000, this));
	connect(m_ptzVRotateInfraredWidget, &InputWidget::signalButtonClicked, this, [=]() {
		//WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(-1, m_ptzVRotateInfraredWidget->getLineEditText().toInt());
        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_monodrome_req(m_ptzVRotateInfraredWidget->getLineEditText().toInt(), 1);
	});

	m_saveInfraredFocusShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_saveInfraredFocusShowWidget->setFixedWidth(80);
	m_saveInfraredFocusShowWidget->setTipText("Focus");
	m_saveInfraredFocusShowWidget->setShowValue("0");

	m_saveInfraredPtzPanShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_saveInfraredPtzPanShowWidget->setFixedWidth(80);
	m_saveInfraredPtzPanShowWidget->setTipText("Pan");
	m_saveInfraredPtzPanShowWidget->setShowValue("-1");

	m_saveInfraredPtzTiltShowWidget = new InputWidget(InputWidgetType::ValueShowWidget);
	m_saveInfraredPtzTiltShowWidget->setFixedWidth(80);
	m_saveInfraredPtzTiltShowWidget->setTipText("Tilt");
	m_saveInfraredPtzTiltShowWidget->setShowValue("-1");

	QPushButton* pButtonSave = new QPushButton("保存");
	pButtonSave->setObjectName("CommonButtonChange");
	pButtonSave->setFixedSize(QSize(80, 22));
	connect(pButtonSave, &QPushButton::clicked, this, [=] {
		// 点击保存,相机，云台信息保存;
		m_saveInfraredFocusShowWidget->setShowValue("0");

		m_infraredFocusValue = m_infraredFocus->getShowValue().toInt();
		m_infraredPtzPanValue = m_PtzHRotate->getShowValue().toInt();
		m_infraredPtzTileValue = m_PtzVRotate->getShowValue().toInt();

		m_saveInfraredFocusShowWidget->setShowValue(QString::number(m_infraredFocusValue));
		m_saveInfraredPtzPanShowWidget->setShowValue(QString::number(m_infraredPtzPanValue));
		m_saveInfraredPtzTiltShowWidget->setShowValue(QString::number(m_infraredPtzTileValue));

	});

	QPushButton* pButtonSet = new QPushButton("设置");
	pButtonSet->setObjectName("CommonButtonChange");
	pButtonSet->setFixedSize(QSize(80, 22));
	connect(pButtonSet, &QPushButton::clicked, this, [=] {
		// 将当前保存的参数全部应用;
		// 红外;
		//m_infraredVideoBackWidget->setInfraredVideoFocusAbs(m_saveInfraredFocusShowWidget->getShowValue().toInt());
		// 云台;
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(m_saveInfraredPtzPanShowWidget->getLineEditText().toInt(), m_saveInfraredPtzTiltShowWidget->getLineEditText().toInt());
        WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_set_focus_req(m_saveInfraredFocusShowWidget->getLineEditText().toInt());
	});

    QPushButton* pButtonAllSet = new QPushButton("全部应用");
    pButtonAllSet->setObjectName("CommonButtonChange");
    pButtonAllSet->setFixedSize(QSize(80, 22));
    connect(pButtonAllSet, &QPushButton::clicked, this, [=] {
        // 云台设置;
        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(m_ptzHRotateInfraredWidget->getLineEditText().toInt(), m_ptzVRotateInfraredWidget->getLineEditText().toInt());
        WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_set_focus_req(m_infraredFocusWidget->getLineEditText().toInt());
        //         m_visibleLightVideoBackWidget->setVideoZoomAbs(m_visibleLightScaleNearWidget->getLineEditText().toInt());
        //         m_visibleLightVideoBackWidget->setVideoFocusAbs(m_visibleLightFocusNearWidget->getLineEditText().toInt());
    });

	QGridLayout* gSaveInfoLayout = new QGridLayout;
	gSaveInfoLayout->addWidget(m_saveInfraredFocusShowWidget, 0, 0);
	gSaveInfoLayout->addWidget(m_saveInfraredPtzPanShowWidget, 0, 1);
	gSaveInfoLayout->addWidget(m_saveInfraredPtzTiltShowWidget, 1, 1);
	gSaveInfoLayout->addWidget(pButtonSave, 0, 2);
    gSaveInfoLayout->addWidget(pButtonSet, 1, 2);
    gSaveInfoLayout->addWidget(pButtonAllSet, 1, 3);
	gSaveInfoLayout->setVerticalSpacing(0);
	gSaveInfoLayout->setHorizontalSpacing(22);
	gSaveInfoLayout->setMargin(0);

	m_pButtonAddDeviceInfraredFinish = new QPushButton("完成");
	m_pButtonAddDeviceInfraredFinish->setFixedSize(QSize(80, 25));
	m_pButtonAddDeviceInfraredFinish->setObjectName("CommonButtonChange");
	connect(m_pButtonAddDeviceInfraredFinish, &QPushButton::clicked, this, [=] {
		if (m_infraredFocusValue == -1 || m_infraredPtzPanValue == -1)
		{
			// 说明相机，云台数据读取错误;
			DLMessageBox* messageBox = new DLMessageBox(m_addDeviceInfoWidget);
			messageBox->setFixedWidth(280);
			messageBox->setMessageContent("相机/云台数据读取错误");
			messageBox->setWindowModality(Qt::ApplicationModal);
			messageBox->show();
			return;
		}
		else if (m_pointId == -1)
		{
			// 说明点位信息读取错误;
			DLMessageBox* messageBox = new DLMessageBox(m_addDeviceInfoWidget);
			messageBox->setFixedWidth(280);
			messageBox->setMessageContent("点位信息错误");
			messageBox->setWindowModality(Qt::ApplicationModal);
			messageBox->show();
			return;
		}

		if (m_isModifyDevice)
		{
			WheelRobortDeviceParameterStruct paratmeter;
			paratmeter.device_uuid = m_strModifyDeviceId;
			paratmeter.point_id = m_pointId;
			paratmeter.mag_focus = m_infraredFocusValue;
			paratmeter.ptz_pan = m_infraredPtzPanValue;
			paratmeter.ptz_tilt = m_infraredPtzTileValue;
			paratmeter.hc_focus_near = m_visibleLightFocus->getShowValue().toInt();
			paratmeter.hc_zoom_near = m_visibleLightRate->getShowValue().toInt();
			paratmeter.hc_focus_far = m_visibleLightFocus->getShowValue().toInt();
			paratmeter.hc_zoom_far = m_visibleLightRate->getShowValue().toInt();
			paratmeter.video_length = 10;
			paratmeter.audio_length = 10;
			WHEEL_BACK_TO_CORE_SOCKET.robot_device_update_req(paratmeter);
		}
		else
		{
			WheelRobotInsertDeviceStruct insertdeviceData;
			// 设备信息;
			insertdeviceData.device.device_uuid = m_currentAddDeviceId;
			insertdeviceData.device.voltage_level_id = m_voltageLevelMap.key(m_voltageLevel->getComboBoxCurrentContent());
			insertdeviceData.device.equipment_interval_uuid = m_intervalNameMap.key(m_intervalName->getComboBoxCurrentContent());
			insertdeviceData.device.device_area_uuid = m_areaNameMap.key(m_areaName->getComboBoxCurrentContent());
			insertdeviceData.device.device_type_uuid = m_deviceTypeMap.key(m_deviceType->getComboBoxCurrentContent());
			insertdeviceData.device.sub_device_type_uuid = m_deviceChildTypeMap.key(m_deviceChildType->getComboBoxCurrentContent());
			insertdeviceData.device.device_point_type_uuid = WHEEL_DEVICE_CONFIG.getWheelDevicePointTypeUUidQString(m_devicePointPos->getComboBoxCurrentContent());
			WheelDevicePointNameStruct pointPosInfo = m_devicePointPosMap[insertdeviceData.device.device_point_type_uuid];

			insertdeviceData.device.unit_type_uuid = pointPosInfo.unit_type_id;
			insertdeviceData.device.recognition_type_id = WheelRobotRecognizeType(pointPosInfo.recognition_type_id);
			insertdeviceData.device.meter_type_id = WheelRobotMeterType(pointPosInfo.meter_type_id);
			insertdeviceData.device.fever_type_id = WheelRobotFeverType(pointPosInfo.fever_type_id);
			insertdeviceData.device.threshold_filename = WHEEL_DEVICE_CONFIG.getWheelThresholdUUidFromMeterType(pointPosInfo.meter_type_id);
			insertdeviceData.device.save_type_id = WheelRobotSaveType(pointPosInfo.save_type_id);

			if (!m_radioButtonA->isEnabled())
			{
				insertdeviceData.device.device_phase_id = WheelRobotPhaseType::WHEEL_ROBOT_PHASE_TYPE_NONE;
			}
			else
			{
				if (m_radioButtonA->isChecked())
				{
					insertdeviceData.device.device_phase_id = WheelRobotPhaseType::WHEEL_ROBOT_PHASE_TYPE_A_PHASE;
				}
				else if (m_radioButtonB->isChecked())
				{
					insertdeviceData.device.device_phase_id = WheelRobotPhaseType::WHEEL_ROBOT_PHASE_TYPE_B_PHASE;
				}
				else if (m_radioButtonC->isChecked())
				{
					insertdeviceData.device.device_phase_id = WheelRobotPhaseType::WHEEL_ROBOT_PHASE_TYPE_C_PHASE;
				}
			}

			insertdeviceData.paratmeter.device_uuid = insertdeviceData.device.device_uuid;
			insertdeviceData.paratmeter.point_id = m_pointId;
			insertdeviceData.paratmeter.mag_focus = m_infraredFocusValue;
			insertdeviceData.paratmeter.ptz_pan = m_infraredPtzPanValue;
			insertdeviceData.paratmeter.ptz_tilt = m_infraredPtzTileValue;
            insertdeviceData.paratmeter.hc_focus_near = m_visibleLightFocus->getShowValue().toInt();
            insertdeviceData.paratmeter.hc_zoom_near = m_visibleLightRate->getShowValue().toInt();
            insertdeviceData.paratmeter.hc_focus_far = m_visibleLightFocus->getShowValue().toInt();
            insertdeviceData.paratmeter.hc_zoom_far = m_visibleLightRate->getShowValue().toInt();
			insertdeviceData.paratmeter.video_length = 10;
			insertdeviceData.paratmeter.audio_length = 10;
			WHEEL_BACK_TO_CORE_SOCKET.robot_device_insert_req(insertdeviceData);
			// 保存此次的设备ComboBox选择类型;
			m_voltageLevelIndex = m_voltageLevel->getComboBoxCurrentIndex();
			m_areaNameIndex = m_areaName->getComboBoxCurrentIndex();
			m_intervalNameIndex = m_intervalName->getComboBoxCurrentIndex();
			m_deviceTypeIndex = m_deviceType->getComboBoxCurrentIndex();
			m_deviceChildTypeIndex = m_deviceChildType->getComboBoxCurrentIndex();
			m_devicePointPosIndex = m_devicePointPos->getComboBoxCurrentIndex();
		}
		QString deviceId;
		if (m_isModifyDevice)
		{
			deviceId = m_strModifyDeviceId;
		}
		else
		{
			deviceId = m_currentAddDeviceId;
		}
		QString filePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/Devices/" + deviceId;
		QDir dir;
		bool isSuccess = dir.mkpath(filePath);
		// 红外拍照;
		//m_infraredVideoBackWidget->infraredCapture(filePath, deviceId);
		//WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_take_photo_req(filePath, deviceId);
//         if (WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().infraredManufacturer == 0)
//         {
//             WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_take_photo_req("infraredCapture", deviceId, 0);
//         }
//         else
//         {
            WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_take_photo_req("infraredCapture", deviceId, 1);
//        }
        
		
		ROS_ERROR("infrared error:(filePath:%1,deviceUUis:%2)", "infraredCapture",deviceId.toStdString().c_str());

        m_addDeviceInfoWidget->setEnabled(false);
        changeWidgetButtonStyle(BUTTON_STYLE_GRAY);
    //    DLMessageBox::showDLMessageBox(this, "提示", QString("采集完成！"), MessageButtonType::BUTTON_OK, true);
// 		m_addDeviceInfoWidget->close();
// 		m_addDeviceInfoWidget = NULL;
	});

	m_pButtonAddDeviceInfraredLastStage = new QPushButton("上一步");
	m_pButtonAddDeviceInfraredLastStage->setFixedSize(QSize(80, 25));
	m_pButtonAddDeviceInfraredLastStage->setObjectName("CommonButtonChange");
	connect(m_pButtonAddDeviceInfraredLastStage, &QPushButton::clicked, this, [=] {
		m_addDeviceStackedWidget->setCurrentIndex(0);
	});

	QHBoxLayout* hTurnPageButtonLayout = new QHBoxLayout;
	hTurnPageButtonLayout->addWidget(m_pButtonAddDeviceInfraredLastStage);
	hTurnPageButtonLayout->addWidget(m_pButtonAddDeviceInfraredFinish);
	hTurnPageButtonLayout->setSpacing(15);
	hTurnPageButtonLayout->setContentsMargins(0, 22, 0, 0);

	QWidget* addDeviceButtonBackWidget = new QWidget;
	addDeviceButtonBackWidget->setFixedWidth(600);

	QHBoxLayout* hAddDeviceButtonLayout = new QHBoxLayout(addDeviceButtonBackWidget);
	hAddDeviceButtonLayout->addLayout(gSaveInfoLayout);
	hAddDeviceButtonLayout->addStretch();
	hAddDeviceButtonLayout->addLayout(hTurnPageButtonLayout);
	hAddDeviceButtonLayout->setSpacing(15);
	hAddDeviceButtonLayout->setMargin(0);

	QGridLayout* gDeviceInfoLayout = new QGridLayout(m_addDeviceInfraredWidget);
	gDeviceInfoLayout->addWidget(m_infraredFocusWidget, 0, 0);
	gDeviceInfoLayout->addWidget(m_ptzHRotateInfraredWidget, 0, 1);
    gDeviceInfoLayout->addWidget(m_ptzVRotateInfraredWidget, 1, 1);

    gDeviceInfoLayout->addWidget(m_saveInfraredFocusShowWidget, 0, 2);
    gDeviceInfoLayout->addWidget(m_saveInfraredPtzPanShowWidget, 0, 3);
    gDeviceInfoLayout->addWidget(m_saveInfraredPtzTiltShowWidget, 1, 3);
    gDeviceInfoLayout->addWidget(pButtonSave, 0, 4);
    gDeviceInfoLayout->addWidget(pButtonSet, 1, 4);
    gDeviceInfoLayout->addWidget(pButtonAllSet, 1, 5);
    gDeviceInfoLayout->addWidget(m_pButtonAddDeviceInfraredFinish, 1, 6);

//	gDeviceInfoLayout->addWidget(addDeviceButtonBackWidget, 2, 0, 1, 2);
	gDeviceInfoLayout->setMargin(0);
	gDeviceInfoLayout->setVerticalSpacing(15);
}

void DLHangWheelCollectControlWidget::initAddDeviceWidget()
{
	m_focusNearValue = -1;
	m_zoomNearValue = -1;
	m_focusFarValue = -1;
	m_zoomFarValue = -1;
	m_ptzPanValue = -1;
	m_ptzTiltValue = -1;

	m_infraredFocusValue = -1;
	m_infraredPtzPanValue = -1;
	m_infraredPtzTileValue = -1;

//	initAddDeviceFirstStageWidget();
	initAddDeviceVisibleFarStageWidget();
	initAddDeviceVisibleNearStageWidget();
	initAddDeviceInfraredStageWidget();

//    m_addDeviceInfoWidget = new BaseWidget(this, BaseWidgetType::PopupWindow);
    m_addDeviceInfoWidget = new BaseWidget(this, BaseWidgetType::CommonWidget);
    m_addDeviceInfoWidget->setTitleHide();
	m_addDeviceInfoWidget->setAttribute(Qt::WA_DeleteOnClose);
	m_addDeviceInfoWidget->setTitleContent("设备名");
//	m_addDeviceInfoWidget->setFixedSize(QSize(800, 250));
//	m_addDeviceInfoWidget->setShowCloseButton();
	m_addDeviceInfoWidget->setStyleSheet("QPushButton#CommonButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
								QPushButton#CommonButton:hover{background-color:rgb(44 , 137 , 255);}\
								QPushButton#CommonButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");
    m_addDeviceInfoWidget->setStyleSheet("QPushButton#CommonButtonChange{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
								QPushButton#CommonButtonChange:hover{background-color:rgb(44 , 137 , 255);}\
								QPushButton#CommonButtonChange:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");

	connect(m_addDeviceInfoWidget, &BaseWidget::signalCloseButtonClicked, this, [=] {
		// 保存此次的设备ComboBox选择类型;
		m_voltageLevelIndex = m_voltageLevel->getComboBoxCurrentIndex();
		m_areaNameIndex = m_areaName->getComboBoxCurrentIndex();
		m_intervalNameIndex = m_intervalName->getComboBoxCurrentIndex();
		m_deviceTypeIndex = m_deviceType->getComboBoxCurrentIndex();
		m_deviceChildTypeIndex = m_deviceChildType->getComboBoxCurrentIndex();
		m_devicePointPosIndex = m_devicePointPos->getComboBoxCurrentIndex();

		m_addDeviceInfoWidget->close();
		m_addDeviceInfoWidget = NULL;
	});

	m_addDeviceStackedWidget = new QStackedWidget;
	//m_addDeviceStackedWidget->insertWidget(0, m_addDeviceFirstStageWidget);
	m_addDeviceStackedWidget->insertWidget(0, m_addDeviceVisibleNearWidget);
	m_addDeviceStackedWidget->insertWidget(1, m_addDeviceVisibleFarWidget);
	m_addDeviceStackedWidget->insertWidget(2, m_addDeviceInfraredWidget);
	
	// 相机移动圆盘按钮;
	m_robotVideoControlCustomButton = new CustomButton(Wheel_RobotControl_SmallBodyMove);
    m_robotVideoControlCustomButton->setRadiusValue(45);
    m_robotVideoControlCustomButton->setArcLength(30);
	m_robotVideoControlCustomButton->setArcAngle(45, 89.8);

	connect(m_robotVideoControlCustomButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotVideoControlCustomButtonPressed(int)));
	connect(m_robotVideoControlCustomButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotVideoControlCustomButtonReleased(int)));

	QRadioButton *pPressedRadioBtn = new QRadioButton();
	pPressedRadioBtn->setChecked(true);
	pPressedRadioBtn->setText("连点模式");

	QRadioButton *pClickedRadioBtn = new QRadioButton();
	pClickedRadioBtn->setText("单点模式");

	connect(pPressedRadioBtn, SIGNAL(clicked(bool)), this, SLOT(PressedRadioBtnSlot(bool)));
	connect(pClickedRadioBtn, SIGNAL(clicked(bool)), this, SLOT(ClickedRadioBtnSlot(bool)));

	QHBoxLayout *pRadioBtnLayout = new QHBoxLayout();
//	pRadioBtnLayout->addStretch();
	pRadioBtnLayout->addWidget(pPressedRadioBtn);
	pRadioBtnLayout->addWidget(pClickedRadioBtn);

	QVBoxLayout *pVideoBtnLayout = new QVBoxLayout();
	pVideoBtnLayout->addWidget(m_robotVideoControlCustomButton);
	pVideoBtnLayout->addLayout(pRadioBtnLayout);

	QHBoxLayout* hAddDeviceLayout = new QHBoxLayout(m_addDeviceInfoWidget->getCenterWidget());
	//hAddDeviceLayout->addWidget(m_robotVideoControlCustomButton);
	hAddDeviceLayout->addLayout(pVideoBtnLayout);
	hAddDeviceLayout->addWidget(m_addDeviceStackedWidget);
	hAddDeviceLayout->setSpacing(0);
	hAddDeviceLayout->setMargin(0);
}

void DLHangWheelCollectControlWidget::initDeviceList()
{
	m_pDeviceTreeWidget = new AddEquipmentTreeWgt;
	//m_pDeviceTreeWidget->setDevelopTreeWidget();
//	m_pDeviceTreeWidget->setTreeWidgetType(TreeItemWidgetType::ColorRect_Menu_With);
	//m_pDeviceTreeWidget->setSearchLineEditVisible(false);

	//connect(m_pDeviceTreeWidget, &AddEquipmentTreeWgt::signalRefreshTreeWidget, this, &DLHangWheelCollectControlWidget::onRefreshTreeWodget);
	//connect(m_pDeviceTreeWidget, &AddEquipmentTreeWgt::signalDeleteTreeItemNode, this, &DLHangWheelCollectControlWidget::onDeleteTreeItemNodeReq);
	connect(m_pDeviceTreeWidget, &AddEquipmentTreeWgt::signalModifyDeviceInfo, this, &DLHangWheelCollectControlWidget::onModifyDeviceInfo);
	//connect(m_pDeviceTreeWidget, &AddEquipmentTreeWgt::signalMoveToDevicePoint, this, &DLHangWheelCollectControlWidget::onMoveToDevicePoint);
	connect(m_pDeviceTreeWidget, &AddEquipmentTreeWgt::signalTreeWidgetShrink, this, [=](bool isShrink) {
		// 树控件是否收缩;
        m_bIsShrink = isShrink;
		m_operationButtonBackWidget->setVisible(!isShrink);
        if (m_bIsVisibleFull && isShrink)
        {
            m_samplingVisibleLightVideoBack->setFixedSize(1700, 956);
        }
        if (m_bIsVisibleFull && !isShrink)
        {
            m_samplingVisibleLightVideoBack->setFixedSize(1440, 810);
        }
	});

	//connect(m_pDeviceTreeWidget, SIGNAL(CollectEquipmentSignal(QString)), this, SLOT(CollectEquipmentSlot(QString)));
	
//	connect(this, &DLHangWheelCollectControlWidget::signalAddNewItemToTree, m_pDeviceTreeWidget, &AddEquipmentTreeWgt::addItemToTree);
//	connect(this, &DLHangWheelCollectControlWidget::signalTreeWidgetRefresh, m_deviceTreeWidget, &AddEquipmentTreeWgt::refreshTreeItemList);
	
// 	connect(m_deviceTreeWidget, &AddEquipmentTreeWgt::signalDeleteTreeItemNode, this, [=](QString strDeviceId) {
// 		WHEEL_BACK_TO_CORE_SOCKET.robot_device_delete_single_dev_req(strDeviceId);
// 
// 	});
	//
	// 添加数据;
	onRefreshTreeWodget();
}

void DLHangWheelCollectControlWidget::initVideoWidget()
{

	m_pMapToolWgt = new BaseWidget(this);
	m_pMapToolWgt->setObjectName("toolBtnWgt");
	m_pMapToolWgt->setTitleContent(tr("地图工具"));
	m_pMapToolWgt->setFixedHeight(176);

	QHBoxLayout *pMapToolLayout = new QHBoxLayout(m_pMapToolWgt->getCenterWidget());
	pMapToolLayout->setMargin(0);
	pMapToolLayout->setSpacing(0);

	QVBoxLayout *pVLayout = new QVBoxLayout;
	pVLayout->setMargin(0);
	pVLayout->setSpacing(0);
	pMapToolLayout->addLayout(pVLayout);

	QWidget *pToolBtnWgt = new QWidget(m_pMapToolWgt->getCenterWidget());
	m_pFileBtnGroup = new QButtonGroup(this);
	connect(m_pFileBtnGroup, SIGNAL(buttonClicked(int)), this, SLOT(BtnClickeSlot(int)));

	m_pOperatorBtnGroup = new QButtonGroup(this);
	connect(m_pOperatorBtnGroup, SIGNAL(buttonClicked(int)), this, SLOT(BtnClickeSlot(int)));


	m_pModelBtnGroup = new QButtonGroup(this);
	connect(m_pModelBtnGroup, SIGNAL(buttonClicked(int)), this, SLOT(BtnClickeSlot(int)));


	int iButtonID = 0;
	QHBoxLayout *pFileLayout = new QHBoxLayout;
	pVLayout->addLayout(pFileLayout);
	pFileLayout->setSpacing(20);
	pFileLayout->setMargin(10);

	//打开点边信息文件openMapBtn
	QPushButton *pBtn = CreateToolBtn(pToolBtnWgt, "打开地图文件", "operatorBtn", ":/Resources/Common/image/open.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn);

	//保存点边信息文件saveMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "保存地图文件", "operatorBtn", ":/Resources/Common/image/save.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn);

	//另存点边信息文件saveAsMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "另存地图文件", "operatorBtn", ":/Resources/Common/image/saveAs.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn);

	//下载2D地图download2DBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "下载2D地图", "operatorBtn", ":/Resources/Common/image/Download-2d.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn);
	//上传SmapuploadMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "上传Smap", "operatorBtn", ":/Resources/Common/image/upload.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn);

	//下载Smap downloadMapBtn
	//pBtn = CreateToolBtn(pToolBtnWgt, "下载Smap", "operatorBtn", ":/Resources/Common/image/Download-smap.png");
	//上传mapinfo
	pBtn = CreateToolBtn(pToolBtnWgt, "上传地图文件", "operatorBtn", ":/Resources/Common/image/upload.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn);

	QHBoxLayout *pOperatorLayout = new QHBoxLayout;
	pVLayout->addLayout(pOperatorLayout);
	pOperatorLayout->setSpacing(20);
	pOperatorLayout->setMargin(10);

	//选择工具selectBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "选择工具", "operatorBtn", ":/Resources/Common/image/select.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn);
	pBtn->setChecked(true);
	
	//手动加点addPointManulBtn
	//pBtn = CreateToolBtn(pToolBtnWgt, "手动加点", "operatorBtn", ":/Resources/Common/image/addPoint-manul.png");
// 	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
// 	pOperatorLayout->addWidget(pBtn);

// 	QPushButton *pOverFittingBtn = new QPushButton("拟合");
// 	pOverFittingBtn->setObjectName("overfittingBtn");
// 	connect(pOverFittingBtn, SIGNAL(clicked()), this, SLOT(OverFittingBtnSlot()));
 	pBtn = CreateToolBtn(pToolBtnWgt, "拟合", "overfittingBtn", ":/Resources/Common/image/Fitting.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn);

	//添加路径slideBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "添加路径", "operatorBtn", ":/Resources/Common/image/Slide.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn);
	//添加高级区域advanceAreaBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "添加检修区域", "operatorBtn", ":/Resources/Common/image/AdvancedArea.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn);
	//重定位relocationBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "重定位", "operatorBtn", ":/Resources/Common/image/reLocation.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn);

	//扫描地图scanBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "扫描地图", "operatorBtn", ":/Resources/Common/image/Scann.png");
	//m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn);
	connect(pBtn, SIGNAL(clicked()), this, SLOT(ScannMapBtnSlot()));


	QWidget *pBottomWgt = new QWidget(this);
	pMapToolLayout->addWidget(pBottomWgt);

	QGridLayout *pControlLayout = new QGridLayout(pBottomWgt);
	pControlLayout->setHorizontalSpacing(20);
	pBtn = new QPushButton(pBottomWgt);
	pBtn->setObjectName("redBtn");
	pBtn->setText("键盘");
	pBtn->setFixedSize(60, 50);
	pBtn->setCheckable(true);
	pBtn->setChecked(true);
	pControlLayout->addWidget(pBtn, 0, 0);
	m_pModelBtnGroup->addButton(pBtn, iButtonID++);

	pBtn = new QPushButton(pBottomWgt);
	pBtn->setObjectName("redBtn");
	pBtn->setText("任务");
	pBtn->setFixedSize(60, 50);
	pBtn->setCheckable(true);
	pControlLayout->addWidget(pBtn, 0, 1);
	m_pModelBtnGroup->addButton(pBtn, iButtonID++);

	pBtn = new QPushButton(pBottomWgt);
	pBtn->setObjectName("redBtn");
	pBtn->setText("手柄");
	pBtn->setFixedSize(60, 50);
	pBtn->setCheckable(true);
	pControlLayout->addWidget(pBtn, 1, 0);
	m_pModelBtnGroup->addButton(pBtn, iButtonID++);

	pBtn = new QPushButton(pBottomWgt);
	pBtn->setObjectName("redBtn");
	pBtn->setText("紧急\n定位");
	pBtn->setFixedSize(60, 50);
	pBtn->setCheckable(true);
	pControlLayout->addWidget(pBtn, 1, 1);
	m_pModelBtnGroup->addButton(pBtn, iButtonID++);

	// 可见光视频显示;
	m_samplingVisibleLightVideoBack = new QWidget(this);
	m_samplingVisibleLightVideoBack->setObjectName("lighVideoWgt");
	m_samplingVisibleLightVideoBack->setFixedSize(560, 315);

	m_visibleLightVideoBackWidget = new VideoBackWidget(VideoType::Wheel_Collect_VisibleLightControlVideo, false, m_samplingVisibleLightVideoBack);
    m_visibleLightVideoBackWidget->setAppTypeClient(APP_TYPE_COLLECT_CLIENT);

    connect(m_visibleLightVideoBackWidget, &VideoBackWidget::signalTreeText, [=](bool isFull, VideoType type)
    {
        if (isFull)
        {
            m_bIsVisibleFull = true;
            m_robotInfoWidget->hide();
            m_mapWidget->hide();
            m_pMapToolWgt->hide();
        //    m_pStatusBar->hide();
            
            if (type == VideoType::Wheel_Collect_VisibleLightControlVideo)
            {
                m_samplinginfraredVideoBack->hide();
                if (m_bIsShrink)
                {
                    m_samplingVisibleLightVideoBack->setFixedSize(1700, 956);
                }
                else
                {
                    m_samplingVisibleLightVideoBack->setFixedSize(1440, 810);
                }
                m_visibleLightVideoBackWidget->setVideoHeightFull(true);
            }
        //    m_addDeviceInfoWidget->getCenterWidget()->show();
            m_addDeviceInfoWidget->show();
            m_addDeviceStackedWidget->setCurrentIndex(0);
            m_addDeviceInfoWidget->setEnabled(false);
            changeWidgetButtonStyle(BUTTON_STYLE_GRAY);
        }
        else
        {
            m_bIsVisibleFull = false;
            m_robotInfoWidget->show();
            m_mapWidget->show();
            m_pMapToolWgt->show();
        //    m_pStatusBar->show();
            if (type == VideoType::Wheel_Collect_VisibleLightControlVideo)
            {
                m_samplinginfraredVideoBack->show();
                m_samplingVisibleLightVideoBack->setFixedSize(560, 315);
                m_visibleLightVideoBackWidget->setVideoHeightFull(false);
            }

        //    m_addDeviceInfoWidget->getCenterWidget()->hide();
            m_addDeviceInfoWidget->hide();
        }
    });

//	connect(m_visibleLightVideoBackWidget, SIGNAL(MoveCollectEquipmentPosSignal(bool)), this, SLOT(MoveCollectEquipmentPosSlot(bool)));
	//m_visibleLightVideoBackWidget->setStyleSheet("}");	
//	m_visibleLightVideoBackWidget->setVideoSizeScale4_3();
	m_visibleLightVideoBackWidget->setPtzResetButtonVisible(true);
	//m_samplingVisibleLightVideoBack->setFixedWidth(m_visibleLightVideoBackWidget->width() + 20);

	connect(m_visibleLightVideoBackWidget, &VideoBackWidget::signalOperatePtz, this, &DLHangWheelCollectControlWidget::setCurrentOperationType);
	connect(m_visibleLightVideoBackWidget, &VideoBackWidget::signalPtzResetButtonClicked, this, [=] {
		// 云台复位;
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(0, 0);
	});
	// 可见光视频区域点击;
	connect(m_visibleLightVideoBackWidget, &VideoBackWidget::signalSendMousePressPoint, this, [=](QPoint mousePressPoint, QSize windowSize, int zoomScale) {
		//  WHEEL_BACK_TO_CORE_SOCKET.robot_ctrl_on_mouse_click_on_hc_window(1920, 1080,mousePressPoint.x() * 1920 / windowSize.width(), mousePressPoint.y() * 1080 / windowSize.height(), zoomScale);
		WHEEL_BACK_TO_CORE_SOCKET.robot_ctrl_on_mouse_click_on_hc_window(windowSize.width(), windowSize.height(), mousePressPoint.x(), mousePressPoint.y(), zoomScale, false);
	});

	QHBoxLayout* hVisibleLightVideoLayout = new QHBoxLayout(m_samplingVisibleLightVideoBack);
	hVisibleLightVideoLayout->addWidget(m_visibleLightVideoBackWidget);
	hVisibleLightVideoLayout->setMargin(0);

	///==================== 红外视频;
	m_samplinginfraredVideoBack = new QWidget(this);
	m_samplinginfraredVideoBack->setObjectName("infraredVideoWgt");
    m_samplinginfraredVideoBack->setFixedSize(560, 420);

	m_infraredVideoBackWidget = new VideoBackWidget(VideoType::Wheel_Collect_InfraredControlVideo, false, m_samplinginfraredVideoBack);
    m_infraredVideoBackWidget->setAppTypeClient(APP_TYPE_COLLECT_CLIENT);
	
    //connect(m_infraredVideoBackWidget, &VideoBackWidget::signalSendMousePressPoint, this, [=](QPoint mousePressPoint, QSize windowSize, int zoomScale) {
    //    WHEEL_BACK_TO_CORE_SOCKET.robot_ctrl_on_mouse_click_on_hc_window(windowSize.width(), windowSize.height(), mousePressPoint.x(), mousePressPoint.y(), m_visibleLightRate->getShowValue().toInt(), false);
    //});
	connect(m_infraredVideoBackWidget, &VideoBackWidget::signalSendMousePressPoint, this, [=](QPoint mousePressPoint, QSize windowSize, int zoomScale) {
		WHEEL_BACK_TO_CORE_SOCKET.robot_ctrl_on_mouse_click_on_In_window(windowSize.width(), windowSize.height(), mousePressPoint.x(), mousePressPoint.y(), m_visibleLightRate->getShowValue().toInt(), true);
	});

    connect(m_infraredVideoBackWidget, &VideoBackWidget::signalTreeText, [=](bool isFull, VideoType type)
    {
        if (isFull)
        {
            m_bIsVisibleFull = true;
            m_robotInfoWidget->hide();
            m_mapWidget->hide();
            m_pMapToolWgt->hide();
        //    m_pStatusBar->hide();
            if (type == VideoType::Wheel_Collect_InfraredControlVideo)
            {
                m_samplingVisibleLightVideoBack->hide();
                m_samplinginfraredVideoBack->setFixedSize(1440, 810);
                m_infraredVideoBackWidget->setVideoHeightFull(true);
            }
        //    m_addDeviceInfoWidget->getCenterWidget()->show();
            m_addDeviceInfoWidget->show();
            m_addDeviceInfoWidget->setEnabled(false);
            changeWidgetButtonStyle(BUTTON_STYLE_GRAY);
            m_addDeviceStackedWidget->setCurrentIndex(0);
        }
        else
        {
            m_bIsVisibleFull = false;
            m_robotInfoWidget->show();
            m_mapWidget->show();
            m_pMapToolWgt->show();
        //    m_pStatusBar->show();
            if (type == VideoType::Wheel_Collect_InfraredControlVideo)
            {
                m_samplingVisibleLightVideoBack->show();
                m_samplinginfraredVideoBack->setFixedSize(560, 420);
                m_infraredVideoBackWidget->setVideoHeightFull(false);
            }
        //    m_addDeviceInfoWidget->getCenterWidget()->hide();
            m_addDeviceInfoWidget->hide();
        }
    });

	QHBoxLayout* hInfraredVideoLayout = new QHBoxLayout(m_samplinginfraredVideoBack);
	hInfraredVideoLayout->addWidget(m_infraredVideoBackWidget);
	hInfraredVideoLayout->setMargin(0);

    if (m_addDeviceInfoWidget == NULL)
    {
        initAddDeviceWidget();
    }

}

void DLHangWheelCollectControlWidget::initErrorMessageWidget()
{
	//初始化 操作信息弹出窗口;
	m_errorMsgListBackWidget = new BaseWidget(this, BaseWidgetType::PopupWindow);
	m_errorMsgListBackWidget->setTitleContent("操作信息");
	m_errorMsgListWidget = new QListWidget();
	QHBoxLayout* hErrorMsgListLayout = new QHBoxLayout(m_errorMsgListBackWidget->getCenterWidget());
	hErrorMsgListLayout->addWidget(m_errorMsgListWidget);
	hErrorMsgListLayout->setMargin(10);

	m_errorMsgListBackWidget->setShowCloseButton();
	connect(m_errorMsgListBackWidget, &BaseWidget::signalCloseButtonClicked, this, [=] {
		m_errorMsgListBackWidget->setVisible(false);
	});

	m_errorMsgListBackWidget->setVisible(false);

	m_errorMessageBackWidget = new QWidget;
	m_errorMessageBackWidget->setObjectName("ModeChangeBackWidget");
	m_errorMessageBackWidget->setFixedHeight(40);

	m_errorMessageLabel = new QLabel;
	m_errorMessageLabel->setScaledContents(true);
	m_errorMessageLabel->setStyleSheet("color:red;");

	QHBoxLayout* hLayout = new QHBoxLayout(m_errorMessageBackWidget);
	hLayout->addWidget(m_errorMessageLabel);
	QToolButton* pButtonPop = new QToolButton;
	pButtonPop->setFixedSize(QSize(30, 30));
	pButtonPop->setIcon(QIcon(":/Resources/Common/image/PopButton.png"));
	pButtonPop->setIconSize(QSize(20, 20));
	pButtonPop->setStyleSheet("border:none");
	connect(pButtonPop, &QToolButton::clicked, this, [=] {
		if (m_errorMsgListBackWidget->isVisible())
		{
			m_errorMsgListBackWidget->setVisible(false);
		}
		else
		{
			QPoint pos = m_errorMessageBackWidget->mapToGlobal(QPoint(0, 0));
			m_errorMsgListBackWidget->setFixedSize(QSize(m_errorMessageBackWidget->width(), 250));
			m_errorMsgListBackWidget->setVisible(true);
			m_errorMsgListBackWidget->move(pos.x(), pos.y() - 250);
			m_errorMsgListBackWidget->raise();
		}	
	});

	hLayout->addStretch();
	hLayout->addWidget(pButtonPop);
	hLayout->setContentsMargins(5, 0, 5, 0);	
}

void DLHangWheelCollectControlWidget::initUpdatePtzAngleDataTimer()
{
	// 云台角度数据从Core处获取，如果1s之内接收不到，则将水平，垂直角度置为-1;
	m_updatePtzAngleDataTimer.setInterval(1000);
	connect(&m_updatePtzAngleDataTimer, &QTimer::timeout, this, [=] {
		if (!m_isUpdateRealTimeData)
		{
			m_PtzHRotate->setShowValue("-1");
			m_PtzVRotate->setShowValue("-1");
		}
		else
		{
			m_isUpdateRealTimeData = false;
		}
	});
	m_updatePtzAngleDataTimer.start();
}

void DLHangWheelCollectControlWidget::initOperationButton()
{
	m_operationButtonBackWidget = new QWidget;
	m_operationButtonBackWidget->setFixedHeight(40);
	m_operationButtonBackWidget->setStyleSheet(".QWidget{background:white;}");

	QPushButton* pButtonExportDevice = new QPushButton("导出设备");
	pButtonExportDevice->setFixedSize(QSize(80, 25));
	pButtonExportDevice->setObjectName("CommonButton");
	connect(pButtonExportDevice, &QPushButton::clicked, this, [=] {
		WHEEL_DEVICE_CONFIG.getWheelDeviceParameterDataToText();
	});

	QPushButton* pButtonExporImage = new QPushButton("导出图片");
	pButtonExporImage->setFixedSize(QSize(80, 25));
	pButtonExporImage->setObjectName("CommonButton");
	connect(pButtonExporImage, &QPushButton::clicked, this, &DLHangWheelCollectControlWidget::onExportCameraCaptureImage);

	QPushButton* pButtonDataSync = new QPushButton;
	pButtonDataSync->setObjectName("CommonButton");
	pButtonDataSync->setFixedSize(QSize(90, 25));
	pButtonDataSync->setText("数据同步");
	connect(pButtonDataSync, &QPushButton::clicked, this, [=] {
		QString strFilePath = QFileDialog::getOpenFileName(this, "选择文件", "", "text(*.txt)");
		if (strFilePath.isEmpty())
		{
			return;
		}
		int bUpload = GFILE_TRANSFER_CLIENT.uploadFile(std::string(strFilePath.toLocal8Bit()), std::string("sql"));

		if (!bUpload)
		{
			WHEEL_BACK_TO_CORE_SOCKET.robot_control_update_device_req(strFilePath);
		}
	});

	QHBoxLayout* hButtonLayout = new QHBoxLayout(m_operationButtonBackWidget);
	hButtonLayout->addWidget(pButtonExportDevice);
	hButtonLayout->addWidget(pButtonExporImage);
	hButtonLayout->addWidget(pButtonDataSync);
	hButtonLayout->addStretch();
	hButtonLayout->setSpacing(10);
	hButtonLayout->setMargin(10);
}

void DLHangWheelCollectControlWidget::initWidget()
{
	//左侧设备树
	initDeviceList();
	//底层操作按钮
	initOperationButton();

	//中间地图编辑
	initRobotInfo();
	initMapWidget();
	//initRobotControl();

	//右边地图工具&视频工具
	initVideoWidget();

	initErrorMessageWidget();
	

	QVBoxLayout* vCenterLayout = new QVBoxLayout;
	vCenterLayout->addWidget(m_robotInfoWidget);
	vCenterLayout->addWidget(m_mapWidget);
//	vCenterLayout->addWidget(m_errorMessageBackWidget);
	//vCenterLayout->addWidget(m_robotControlWidget);
	vCenterLayout->setSpacing(10);
	vCenterLayout->setMargin(0);


	QVBoxLayout* vVideoLayout = new QVBoxLayout;
	vVideoLayout->addWidget(m_pMapToolWgt);
	vVideoLayout->addWidget(m_samplingVisibleLightVideoBack);
    vVideoLayout->addWidget(m_samplinginfraredVideoBack);
//    vVideoLayout->addWidget(m_addDeviceInfoWidget->getCenterWidget());
    vVideoLayout->addWidget(m_addDeviceInfoWidget);// ->getCenterWidget());
	vVideoLayout->addStretch();
	vVideoLayout->setSpacing(10);
	vVideoLayout->setMargin(0);
//    m_addDeviceInfoWidget->getCenterWidget()->hide();
    m_addDeviceInfoWidget->hide();
//    m_samplinginfraredVideoBack->hide();

	QVBoxLayout* vLeftLayout = new QVBoxLayout;
	vLeftLayout->addWidget(m_pDeviceTreeWidget);
	vLeftLayout->addWidget(m_operationButtonBackWidget);
	//vLeftLayout->addWidget(m_robotControlWidget);
	vLeftLayout->setSpacing(10);
	vLeftLayout->setMargin(0);

	QHBoxLayout* hMainLayout = new QHBoxLayout;
	hMainLayout->addLayout(vLeftLayout);
	hMainLayout->addLayout(vCenterLayout);
	hMainLayout->addLayout(vVideoLayout);
	hMainLayout->setSpacing(10);
	hMainLayout->setMargin(0);

	m_pStatusBar = new DLStatusBar(this);  //状态栏

	QVBoxLayout *pVMainLayout = new QVBoxLayout(this);
	pVMainLayout->setMargin(10);
	pVMainLayout->addLayout(hMainLayout);
	pVMainLayout->addWidget(m_pStatusBar);


	m_pMapListWgt = new DLMapListWidget(this);
	connect(m_pMapListWgt, SIGNAL(sig_select_file(QString, QString, int)), this, SLOT(slot_on_choose_map(QString, QString, int)));

	m_pThreadDataTransfer = new DataTransfer(this);
    connect(m_pThreadDataTransfer, SIGNAL(sig_finished(int, int, QString)), this, SLOT(slot_on_transfer_finished(int, int, QString)));
    connect(m_pThreadDataTransfer, SIGNAL(sig_finished_infrared(QString, QString)), this, SLOT(slot_on_transfer_finished_infrared(QString, QString)));
}

void DLHangWheelCollectControlWidget::setCameraObject(CameraObject* cameraObject)
{
	m_cameraObject = cameraObject;
	m_visibleLightVideoBackWidget->setCameraObject(cameraObject);
}

void DLHangWheelCollectControlWidget::setInfraredObject(void*, CameraObject* camerObject)
{
    m_infraredVideoBackWidget->setCameraObject(camerObject);
//    m_infraredVideoBackWidget->setInfraredObject(infraredTemperature);
}

void DLHangWheelCollectControlWidget::onUpdateRobotInfo()
{
	// 红外和可见1s更新一次;
// 	m_updateCount++;
// 	if (m_updateCount >= 10)
	{
		m_updateCount = 0;
		// 可见光倍率;
		m_visibleLightRate->setShowValue(QString::number(m_visibleLightVideoBackWidget->getCurrentZoomValue()));
		// 可见光焦距;
		m_visibleLightFocus->setShowValue(QString::number(m_visibleLightVideoBackWidget->getCurrentVideoFocusValue()));
		// 红外焦距;
		//int infraredFocusValue = m_infraredVideoBackWidget->getInfraredVideoFocus();
		if (m_infraredFocusValueCore >= 0)
		{
			m_infraredFocus->setShowValue(QString::number(m_infraredFocusValueCore));
		}
		else
		{
            m_infraredFocus->setShowValue(QString::number(-1));
		}

		// 发送可见光当前参数;
		emit signalUpdateVisibleVideoParam(m_visibleLightRate->getShowValue().toInt());
	}
}

void DLHangWheelCollectControlWidget::onRobotVideoControlCustomButtonPressed(int buttonId)
{
	if (m_bIsPressedState)
	{
		int operationType;
		switch (buttonId)
		{
		case 0:
			operationType = Qt::Key_W;
			break;
		case 1:
			operationType = Qt::Key_A;
			break;
		case 2:
			operationType = Qt::Key_S;
			break;
		case 3:
			operationType = Qt::Key_D;
			break;
		case 4:
			// 云台复位;
			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(0, 0);
			return;
		default:
			return;
		}

		m_keyboardOperateThread->setCurrentOperationType(operationType, false);
	}
}

void DLHangWheelCollectControlWidget::onRobotBodyControlPressed(int buttonId)
{
	switch (buttonId)
	{
	case 0:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0.3 * 1, 0, 0);
		break;
	case 1:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, 0.5 * 1);
		break;
	case 2:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, -0.5 * 1);
		break;
	case 3:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(-0.3 * 1, 0, 0);
		break;
	case 4:
		break;
	default:
		break;
	}
}

void DLHangWheelCollectControlWidget::onRobotBodyControlReleased(int buttonId)
{
	switch (buttonId)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, 0);
		break;
	case 4:
		break;
	default:
		break;
	}
}

void DLHangWheelCollectControlWidget::onRobotPTZControlPressed(int buttonId)
{
	switch (buttonId)
	{
	case 0:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_UP, 0, 0.6 * 1);
		break;
	case 1:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_LEFT, 0, 0.6 * 1);
		break;
	case 2:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_DOWN, 0, 0.6 * 1);
		break;
	case 3:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_RIGHT, 0, 0.6 * 1);
		break;
	case 4:
		//m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomOut);
		break;
	case 5:
		//m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomIn);
		break;
	default:
		break;
	}
}

void DLHangWheelCollectControlWidget::onRobotPTZControlReleased(int buttonId)
{
	switch (buttonId)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_STOP, 0, 0);
		break;

	case 4:
		//m_visibleVideoWidget->onButtonReleased(VideoButtonType::ZoomOut);
		break;
	case 5:
		//m_visibleVideoWidget->onButtonReleased(VideoButtonType::ZoomIn);
		break;
	default:
		break;
	}
}

void DLHangWheelCollectControlWidget::onRobotVideoControlCustomButtonReleased(int buttonId)
{
	int operationType;
	switch (buttonId)
	{
	case 0:
		operationType = Qt::Key_W;
		break;
	case 1:
		operationType = Qt::Key_A;
		break;
	case 2:
		operationType = Qt::Key_S;
		break;
	case 3:
		operationType = Qt::Key_D;
		break;
	default:
		return;
	}
	m_keyboardOperateThread->setCurrentOperationType(operationType, true);
}

void DLHangWheelCollectControlWidget::clearAllInputContent()
{
	for each (InputWidget* widget in m_clearLineEditList)
	{
		widget->clearContent();
	}
}

void DLHangWheelCollectControlWidget::onDeviceItemDoubleClicked(QTreeWidgetItem* item, int column)
{
	if (qApp->mouseButtons() == Qt::LeftButton)
	{
		QString deviceSsiD = item->data(0, Qt::UserRole).toString();
		// 说明选中了根节点;
		if (deviceSsiD.isEmpty())
		{
			return;
		}
	}	
}

void DLHangWheelCollectControlWidget::onDeviceItemClicked(QTreeWidgetItem* item, int column)
{
	if (qApp->mouseButtons() == Qt::RightButton)
	{
	}
}

void DLHangWheelCollectControlWidget::onExportCameraCaptureImage()
{
	QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
		"/home",
		QFileDialog::ShowDirsOnly
		| QFileDialog::DontResolveSymlinks);

    if (dir.isEmpty())
    {
        return;
    }

	QStringList strUuidList = WHEEL_DEVICE_CONFIG.getWheelAllDeviceUUid();
	//QString filePath = QApplication::applicationDirPath() + "/Devices/";
	QString filePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/Devices/";

    QString targetFilePath = dir + "/";
    QString templateFilePath = dir + "/template/";
    QDir targetTargetDir(templateFilePath);
    if (!targetTargetDir.exists(templateFilePath))
    {
        if (!targetTargetDir.mkpath(templateFilePath))
        {
            DLMessageBox::showDLMessageBox(this, "错误", "文件夹创建失败", MessageButtonType::BUTTON_OK, true);
            return;
        }
    }

    QString twostageFilePath = dir + "/twostage/";
    QDir twostageTargetDir(twostageFilePath);
    if (!twostageTargetDir.exists(twostageFilePath))
    {
        if (!twostageTargetDir.mkpath(twostageFilePath))
        {
            DLMessageBox::showDLMessageBox(this, "错误", "文件夹创建失败", MessageButtonType::BUTTON_OK, true);
            return;
        }
    }
	// 如果文件已经存在，则覆盖;
	for (int i = 0; i < strUuidList.count(); i++)
	{
		QString srcFilePath = filePath + strUuidList.at(i);
		copyRecursively(srcFilePath, targetFilePath);
	}
}

void DLHangWheelCollectControlWidget::UpdateEquipmentTreeSlot(bool bIsSuccess, QStringList lstUpdateTreeNode, RootNodeType iType)
{
	//qDebug() << "UpdateEquipmentTreeSlot";
	if (!bIsSuccess) return;
	if (m_pDeviceTreeWidget)
	{
		m_pDeviceTreeWidget->UpdateTreeNode(lstUpdateTreeNode, iType);
	}
}

void DLHangWheelCollectControlWidget::PressedRadioBtnSlot(bool bIsChecked)
{
	if (bIsChecked)
	{
		m_bIsPressedState = true;
		m_keyboardOperateThread->SetPressedState(true);

	}
}

void DLHangWheelCollectControlWidget::ClickedRadioBtnSlot(bool bIsChecked)
{
	if (bIsChecked)
	{
		m_bIsPressedState = false;
		m_keyboardOperateThread->SetPressedState(false);
	}
}

void DLHangWheelCollectControlWidget::BtnClickeSlot(int iID)
{
	switch (iID)
	{
	case OPEN_SMAP_TYPE:					//打开点边信息文件
	{
		QString strMapDir = QFileDialog::getExistingDirectory(this, tr("选择地图"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
		m_pCollectMapScene->removeall();
		m_pCollectMapScene->load_json_map(strMapDir);

		QDir dir(strMapDir);
		//smap文件
		QStringList nameFilters;
		nameFilters.clear();
		nameFilters << "*.mapinfo";
		QStringList files = dir.entryList(nameFilters, QDir::Files | QDir::Readable, QDir::Name);
		if (files.size() > 0)
		{//默认取第一个
			m_strOpenMapPath = QString("%1/%2").arg(strMapDir).arg(files.at(0));
		}
        else
        {
            m_strOpenMapPath = strMapDir + "/default.mapinfo";
        }
		m_bSMAPIsChanged = false;
		m_pFileBtnGroup->button(SAVE_SMAP_TYPE)->setEnabled(false);

	}break;
	case SAVE_SMAP_TYPE:					//保存点边信息文件
	{
		m_pStatusBar->setOperate("保存地图");
		QString file = m_strOpenMapPath.split("/").last();
		if (m_strOpenMapPath.isEmpty() || m_strOpenMapPath.isNull())
		{//保存文件不存在，则另存为
			//m_pStatusBar->setOperate(file + "保存失败。");
			BtnClickeSlot(SAVEAS_SMAP_TYPE);
			return;
		}

		m_pCollectMapScene->save_json_map(m_strOpenMapPath);
		m_pStatusBar->setOperate(file + "保存成功。");

		m_bSMAPIsChanged = false;
		m_pFileBtnGroup->button(SAVE_SMAP_TYPE)->setEnabled(false);

	}break;
	case SAVEAS_SMAP_TYPE:					//另存点边信息文件		
	{
		m_pStatusBar->setOperate("地图另存为");
		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.mapinfo)"));
		QString file = fileName.split("/").last();
		if (fileName.isEmpty() || fileName.isNull()) {
			m_pStatusBar->setOperate(file + "保存失败。");
			return;
		}

		m_pCollectMapScene->save_json_map(fileName);
		m_pStatusBar->setOperate(file + "保存成功。");

		//m_bSMAPIsChanged = false;

	}break;
	case DOWNLOAD_2D_MAP_TYPE:				//下载2D地图
	{
		//获取2dmap列表 
		m_pMapListWgt->set_type(DataTransfer::FILE_2D);
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_2d_map_query_req();
		m_pMapListWgt->show();

		m_pStatusBar->setOperate(("正在下载2dmap地图"));
	}break;
	case UPLOAD_SMAP_TYPE:					//上传Smap
	{
		//向core上传smap地图到巡检车
		if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
			return;
		}

		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getOpenFileName(this, tr("选择文件"), strDir, tr("Maps (*.smap)"));

		TransferPro file_pro;
		file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
		//file_pro.file_name_ = boost::filesystem::path(m_strFileName.toLocal8Bit().data()).filename().string();
		file_pro.file_name_ = boost::filesystem::path(fileName.toLocal8Bit().data()).filename().string();
		file_pro.dst_relative_path_ = "smap";
		//file_pro.src_file_path_ = m_strFileName.toLocal8Bit();
		file_pro.src_file_path_ = fileName.toLocal8Bit();

		//if (QFile::exists(m_strFileName))
		if (QFile::exists(fileName))
		{
			m_pThreadDataTransfer->set_transfer_info(file_pro);
			m_pThreadDataTransfer->start();
			m_pStatusBar->setOperate(tr("正在上传smap地图"));
		}
	}break;
// 	case DOWNLOAD_SMAP_TYPE:				//下载Smap
// 	{
// 		// 获取smap列表
// 		m_pMapListWgt->set_type(DataTransfer::FILE_SMAP);
// 		WHEEL_BACK_TO_CORE_SOCKET.robot_config_smap_query_req();
// 		m_pMapListWgt->show();
// 
// 
// 		m_pStatusBar->setOperate(("正在下载smap地图"));
// 	}break;
	case UPLOAD_MAPINFO_TYPE:				//上传点边信息文件
	{
		//向core上传smap地图到巡检车
		if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
			return;
		}

		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getOpenFileName(this, tr("选择文件"), strDir, tr("Maps (*.mapinfo)"));

		TransferPro file_pro;
		file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
		file_pro.file_name_ = boost::filesystem::path(fileName.toLocal8Bit().data()).filename().string();
		file_pro.dst_relative_path_ = "smap";
		file_pro.src_file_path_ = fileName.toLocal8Bit();

		if (QFile::exists(fileName))
		{
			m_pThreadDataTransfer->set_transfer_info(file_pro);
			m_pThreadDataTransfer->start();
			m_pStatusBar->setOperate(tr("正在上传点边信息文件"));
		}
	}break;
	case SELECT_TYPE:						//选择
	{
		m_pCollectMapScene->SetOperateType(DLOperator::TYPE_SELECT);
		m_pStatusBar->setOperate(tr("选择"));
	}break;
	case OVERFITTING_TYPE:					//拟合
	{
		if (NULL != m_pCollectMapScene)
		{
			m_pCollectMapScene->ShowOverfittingDlg();
		}
	}break;
	case ADD_PATH_TYPE:						//添加路径
	{
		m_pCollectMapScene->SetOperateType(DLOperator::TYPE_ADD_EDGE);
		m_pStatusBar->setOperate(tr("添加巡检路径"));
	}break;
	case ADD_ADVANCED_AREA_TYPE:			//高级区域
	{
		m_pCollectMapScene->SetOperateType(DLOperator::TYPE_ADD_AREA);
		m_pStatusBar->setOperate(tr("添加高级区域"));
	}break;
	case RELOCATION_TYPE:					//重定位
	{
		m_pCollectMapScene->SetOperateType(DLOperator::TYPE_RELOCATION);
		m_pStatusBar->setOperate(tr("重定位"));
	}break;
	case KEY_MODE_TYPE:						//键盘模式
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_REMOTE_CTRL)); 
	}break;
	case TASK_MODE_TYPE:					//任务模式
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_AUTORUNNING));
	}break;
	case HAND_MODE_TYPE:					//手柄模式
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_JOY_CTRL));
	}break;
	case URGENCY_RELOCATION_TYPE:			//紧急定位模式
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_EMERGENCY_LOC)); 
	}break;
	default:break;
	}
}

void DLHangWheelCollectControlWidget::slot_on_choose_map(QString download_path, QString filename, int file_type)
{
	//mapListWidget_->hide();
	if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
		return;
	}

	TransferPro file_pro;
	file_pro.cmd_type_ = DataTransfer::DOWNLOAD_FILE;
	file_pro.file_name_ = filename.toLocal8Bit();
	file_pro.download_path_ = download_path.toLocal8Bit();

	if (file_type == DataTransfer::FILE_SMAP) {
		file_pro.dst_relative_path_ = "smap";
	}
	else if (file_type == DataTransfer::FILE_2D) {
		file_pro.dst_relative_path_ = "2dmap";
	}

	m_pThreadDataTransfer->set_transfer_info(file_pro);
	m_pThreadDataTransfer->start();
	m_pStatusBar->setOperate(tr("正在下载地图"));
}

void DLHangWheelCollectControlWidget::slot_on_transfer_finished(int command_type, int execCode, QString fileName)
{
	switch (command_type)
	{
	case DataTransfer::DOWNLOAD_FILE:
	{
		if (!execCode) {
			m_pStatusBar->setOperate(fileName + " 下载完毕");
		}
		else {
			m_pStatusBar->setOperate(fileName + "下载出错");
		}
		break;
	}
	case DataTransfer::UPLOAD_FILE:
	{
		if (!execCode) {
			//下载完毕给core回一条包含下载的好文件名的消息
			WHEEL_BACK_TO_CORE_SOCKET.robot_config_uploadmap_req(fileName);
			m_pStatusBar->setOperate(fileName + "上传成功");
		}
		else {
			m_pStatusBar->setOperate(fileName + "上传出错");
		}
		break;
	}
	default:
		break;
	}

}

void DLHangWheelCollectControlWidget::slot_on_transfer_finished_infrared(QString path, QString name)
{
    QImage image;
    image.load(path + "/" + name + "/" + name + ".jpg");
    image.save(path + "/" + name + "/" + name + ".jpg");
}

void DLHangWheelCollectControlWidget::ChangeTypeSlot(const QString &strMessage, int iSelectType)
{
	m_pStatusBar->setOperate(strMessage);
	if (iSelectType == DLOperator::TYPE_SELECT)
	{
		m_pOperatorBtnGroup->button(SELECT_TYPE)->setChecked(true);
	}
}

void DLHangWheelCollectControlWidget::SMAPChangedSlot()
{
	if (!m_bSMAPIsChanged)
	{
		m_bSMAPIsChanged = true;
		m_pFileBtnGroup->button(SAVE_SMAP_TYPE)->setEnabled(true);
	}
}

// void DLHangWheelCollectControlWidget::ReadFinishedSlot(bool bIsRunning)
// {//保存结束
// // 	if (!bIsRunning)
// // 	{
// // 		//m_bIsUploadSmap = false;
// // 		//UploadSMAPFile();
// // 		//emit OpenSmapFileSignal(m_strOpenMapPath);
// // 		BtnClickeSlot(UPLOAD_MAPINFO_TYPE);
// // 	}
// 	//m_pCollectMapScene->removeall();
// 	m_bSMAPIsChanged = false;
// 	//m_strOpenMapPath = "";
// }

void DLHangWheelCollectControlWidget::ScannMapBtnSlot()
{//扫描地图的槽函数
	QPushButton *pScannBtn = dynamic_cast<QPushButton *>(QObject::sender());
	if (pScannBtn->isChecked())
	{
		pScannBtn->setIcon(QIcon(":/Resources/Common/image/Scann-red.png"));
		m_pCollectMapScene->ShowMessage("正在扫描地图...");

		WHEEL_BACK_TO_CORE_SOCKET.robot_control_slam_req();
		m_pStatusBar->setOperate(tr("开始扫描地图"));
	}
	else
	{
		pScannBtn->setIcon(QIcon(":/Resources/Common/image/Scann.png"));
		m_pCollectMapScene->HideMessage();

		WHEEL_BACK_TO_CORE_SOCKET.robot_control_endslam_req();
		m_pStatusBar->setOperate(tr("结束扫描地图"));
	}
	pScannBtn->repaint();
}


void DLHangWheelCollectControlWidget::DownloadImageSlot(QString strPath, QString strFileName, int type)
{   
    ROS_INFO("DownloadImageSlot : strPath:%s  strFileName:%s", strPath.toStdString().data(), strFileName.toStdString().data());
    if (nullptr == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) return;
    TransferPro file_pro;
    file_pro.cmd_type_ = DataTransfer::DOWNLOAD_FILE;
    
    QString strTempFileName = QString("%1").arg(strFileName);
    QString strDownloadPath = "";
    if (type == 0)
    {
        strDownloadPath = QString("%1/%2").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath).arg("InfraredCapture");
        if (WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().infraredManufacturer == 0)
        {
            strTempFileName = strTempFileName + ".jpg";
        }
        file_pro.dst_relative_path_ = "infraredCapture";
        file_pro.file_name_ = strTempFileName.toLocal8Bit();
        file_pro.download_path_ = strDownloadPath.toLocal8Bit();
    }
    else
    {
        strDownloadPath = QString("%1/%2").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath).arg("Devices");
        if (WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().infraredManufacturer == 0)
        {
            file_pro.dst_relative_path_ = "infraredCapture";
            file_pro.file_name_ = strTempFileName.toLocal8Bit() + ".jpg";
            file_pro.download_path_ = strDownloadPath.toLocal8Bit() + "/" + strTempFileName.toLocal8Bit();
        }
        else
        {
            file_pro.dst_relative_path_ = "infraredCapture";
            file_pro.file_name_ = strTempFileName.toLocal8Bit();
            file_pro.download_path_ = strDownloadPath.toLocal8Bit();
        }
    }

    ROS_INFO("DownloadImageSlot : strPath:%s  strFileName:%s", strPath.toStdString().data(), strFileName.toStdString().data());
    ROS_INFO("DownloadImageSlot : strPath:%s  strTempFileName:%s   strDownloadPath:%s", strPath.toStdString().data(), strTempFileName.toStdString().data(), strDownloadPath.toStdString().data());
	
    m_pThreadDataTransfer->set_infrared_type(type);
	m_pThreadDataTransfer->set_transfer_info(file_pro);
	m_pThreadDataTransfer->start();
}

void DLHangWheelCollectControlWidget::MoveCollectEquipmentPosSlot(bool bIsCenter)
{
	if (NULL != m_addDeviceInfoWidget && !m_addDeviceInfoWidget->isHidden())
	{
		QPoint pos = m_addDeviceInfoWidget->pos();
		if (bIsCenter)
		{//显示在中间

			int iDeskTopHeight = QApplication::desktop()->height();
			int iHeight = m_addDeviceInfoWidget->height();

			int iY = (iDeskTopHeight - iHeight) / 2;
			m_addDeviceInfoWidget->move(pos.x(), iY);

		}
		else
		{//显示在顶端
			m_addDeviceInfoWidget->move(pos.x(), 0);
		}
	}
}

// void DLHangWheelCollectControlWidget::CollectEquipmentSlot(QString strDeviceID)
// {
// 	if (m_addDeviceInfoWidget != NULL)
// 	{
// 		return;
// 	}
// 	initAddDeviceWidget();
// 	m_isModifyDevice = false;
// 	m_addDeviceInfoWidget->show();
// 	m_currentAddDeviceId = strDeviceID;
// 	//m_currentAddDeviceId = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
// }

bool DLHangWheelCollectControlWidget::copyRecursively(const QString &srcFilePath, const QString &tgtFilePath)
{
	QFileInfo srcFileInfo(srcFilePath);
	if (srcFileInfo.isDir()) {
		QDir sourceDir(srcFilePath);
		QStringList fileNames = sourceDir.entryList(QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot | QDir::Hidden | QDir::System);
		foreach(const QString &fileName, fileNames) {
            QString newTgtFilePath;
            if (fileName.contains("_twostage"))
            {
                newTgtFilePath = tgtFilePath + "twostage/" + fileName;
            }
            else
            {
                newTgtFilePath = tgtFilePath + "template/" + fileName;
            }
			QString newSrcFilePath = srcFilePath + QLatin1Char('/') + fileName;
			
            if (!copyRecursively(newSrcFilePath, newTgtFilePath))
            {
                qDebug() << "文件拷贝失败";
            }
		}
	}
	else {
		// 如果文件已经存在，则进行覆盖(先删除，再保存);
		if (QFile::exists(tgtFilePath))
		{
			QFile::remove(tgtFilePath);
		}
		if (!QFile::copy(srcFilePath, tgtFilePath))
		{
			return false;
		}
	}
	return true;
}

void DLHangWheelCollectControlWidget::updateAddDeviceRadioState()
{
	if (m_addDeviceFirstStageWidget != NULL)
	{
		bool isExist = false;
		if (m_deviceChildType->getComboBoxCurrentContent().contains("X"))
		{
			isExist = true;
		}
		if (m_devicePointPos->getComboBoxCurrentContent().contains("X"))
		{
			isExist = true;
		}

		m_radioButtonA->setEnabled(isExist);
		m_radioButtonB->setEnabled(isExist);
		m_radioButtonC->setEnabled(isExist);
	}
}

QString DLHangWheelCollectControlWidget::getCurrentTime()
{
	return QDateTime::currentDateTime().toString("yyyy/MM/dd hh:mm:ss ");
}

void DLHangWheelCollectControlWidget::UploadSMAPFile()
{
	QString filepath = QFileDialog::getOpenFileName(this, tr("选择上传的smap"), tr("D:/RCF_Server/smap"), tr("Maps (*.smap)"));
	if (filepath.isEmpty() || filepath.isNull()) {
		return;
	}

	//向core上传smap地图到巡检车
	if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
		return;
	}

	TransferPro file_pro;
	file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
	file_pro.file_name_ = boost::filesystem::path(filepath.toLocal8Bit().data()).filename().string();
	file_pro.dst_relative_path_ = "smap";
	file_pro.src_file_path_ = filepath.toLocal8Bit();

	m_pThreadDataTransfer->set_transfer_info(file_pro);
	m_pThreadDataTransfer->start();
	m_pStatusBar->setOperate(tr("正在上传smap地图"));
}

QPushButton * DLHangWheelCollectControlWidget::CreateToolBtn(QWidget *parentWgt, const QString strName, const QString &strObjectName, const QString &strIconPath)
{
	QPushButton *pBtn = new QPushButton(QIcon(strIconPath), "", parentWgt);
	pBtn->setToolTip(strName);
	pBtn->setFixedSize(50, 50);
	pBtn->setCheckable(true);
	pBtn->setIconSize(QSize(40, 40));
	pBtn->setObjectName(strObjectName);
	return pBtn;
}

void DLHangWheelCollectControlWidget::InitStatus()
{
	//默认为键盘模式
	//WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_REMOTE_CTRL));
	//默认为选择状态
	m_pCollectMapScene->SetOperateType(DLOperator::TYPE_SELECT);
	m_pStatusBar->setOperate(tr("选择"));
}

void DLHangWheelCollectControlWidget::changeWidgetButtonStyle(WheelButtonStyleSheet type)
{
    switch (type)
    {
    case BUTTON_STYLE_BLUE:
        m_visibleLightScaleNearWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_visibleLightFocusNearWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_visibleLightScaleFarWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_visibleLightFocusFarWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_ptzHRotateWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_ptzVRotateWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_infraredFocusWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_ptzHRotateInfraredWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_ptzVRotateInfraredWidget->setButtonStyleSheet(BUTTON_STYLE_BLUE);
        m_addDeviceInfoWidget->setStyleSheet("QPushButton#CommonButtonChange{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
								QPushButton#CommonButtonChange:hover{background-color:rgb(44 , 137 , 255);}\
								QPushButton#CommonButtonChange:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");
        break;
    case BUTTON_STYLE_GRAY:
        m_visibleLightScaleNearWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_visibleLightFocusNearWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_visibleLightScaleFarWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_visibleLightFocusFarWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_ptzHRotateWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_ptzVRotateWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_infraredFocusWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_ptzHRotateInfraredWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_ptzVRotateInfraredWidget->setButtonStyleSheet(BUTTON_STYLE_GRAY);
        m_addDeviceInfoWidget->setStyleSheet("QPushButton#CommonButtonChange{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(105 , 105 , 105);border-radius:3px;}\
								QPushButton#CommonButtonChange:hover{background-color:rgb(105 , 105 , 105);}\
								QPushButton#CommonButtonChange:pressed{background-color:rgb(105 , 105 , 105);padding-left:2px;padding-top:2px;}");
        break;
    default:
        break;
    }
}

void DLHangWheelCollectControlWidget::onRefreshTreeWodget()
{
	//m_deviceTreeWidget->refreshTreeItemList();
	//m_pDeviceTreeWidget->ClearAllItemState();
}

// void DLHangWheelCollectControlWidget::onDeleteTreeItemNodeReq(RootNodeType nodeType, QString deviceId, QString parentId)
// {
// 	DLMessageBox* messageBox = new DLMessageBox(m_addDeviceInfoWidget);
// 	messageBox->setFixedWidth(250);
// 	messageBox->setButtonCancelVisible(true);
// 	messageBox->setMessageContent("确认删除该节点?");
// 	bool bResult = messageBox->exec();
// 	if (bResult)
// 	{
// 		switch (nodeType)
// 		{
// 		case RootNode_VoltageLevel:
// 			WHEEL_BACK_TO_CORE_SOCKET.robot_device_delete_voltage_level_req(deviceId);
// 			break;
// 		case RootNode_Interval:
// 			WHEEL_BACK_TO_CORE_SOCKET.robot_device_delete_interval_req(deviceId);
// 			break;
// 		case RootNode_DeviceType:
// 			WHEEL_BACK_TO_CORE_SOCKET.robot_device_delete_device_type_req(parentId, deviceId);
// 			break;
// 		case RootNode_Device:
// 			WHEEL_BACK_TO_CORE_SOCKET.robot_device_delete_single_dev_req(deviceId);
// 			break;
// 		default:
// 			break;
// 		}
// 	}	
// }

void DLHangWheelCollectControlWidget::onModifyDeviceInfo(QString strDeviceId, QString strPointId)
{
    WheelRobortDeviceParameterStruct paramData = WHEEL_DEVICE_CONFIG.getWheelDeviceParameterData(strDeviceId);

    // 修改的设备不在当前点位则不能修改;
    if (paramData.point_id != -1 && paramData.point_id != m_pointId)
    {
        m_addDeviceInfoWidget->setEnabled(false);
        m_addDeviceInfoWidget->setTitleContent("设备名");
        changeWidgetButtonStyle(BUTTON_STYLE_GRAY);
		QString strPoint = "";
		bool bIsSuccess = WHEEL_ROBOT_DB.getPointId(strDeviceId, strPoint);
        DLMessageBox::showDLMessageBox(this, "错误", QString("当前机器人未在该设备点(%1)位置").arg(strPoint), MessageButtonType::BUTTON_OK, true);
        return;
    }

	m_isModifyDevice = true;
	m_strModifyDeviceId = strDeviceId;
	if (m_addDeviceInfoWidget == NULL)
	{
		initAddDeviceWidget();
	}

    if (m_bIsVisibleFull)
    {
        m_addDeviceInfoWidget->show();
        m_addDeviceInfoWidget->setEnabled(true);
        changeWidgetButtonStyle(BUTTON_STYLE_BLUE);
    }
    else
    {
        DLMessageBox::showDLMessageBox(this, "错误", QString("请先放大图像！"), MessageButtonType::BUTTON_OK, true);
        return;
    }
    
    QString strDeviceName = "";
    bool bIsSuccess = WHEEL_ROBOT_DB.getDeviceNameWithDeviceUUid(strDeviceId, strDeviceName);
	m_addDeviceInfoWidget->setTitleContent(strDeviceName);
    WheelJudgeTakePhoto type = WHEEL_DEVICE_CONFIG.getWheelChooseRecognitionBool(strPointId);
    if (type == WheelJudgeTakePhoto::VisibleLightJudge)
	{
		m_addDeviceStackedWidget->setCurrentIndex(0);
		m_pButtonAddDeviceVisibleNearLastStage->setVisible(false);

		m_focusNearValue = paramData.hc_focus_near;
		m_focusFarValue = paramData.hc_focus_far;
		m_zoomNearValue = paramData.hc_zoom_near;
		m_zoomFarValue = paramData.hc_zoom_far;
		m_ptzPanValue = paramData.ptz_pan;
		m_ptzTiltValue = paramData.ptz_tilt;

		m_saveZoomShowWidget->setShowValue(QString::number(paramData.hc_zoom_near));
		m_saveFocusShowWidget->setShowValue(QString::number(paramData.hc_focus_near));
		m_savePtzPanShowWidget->setShowValue(QString::number(paramData.ptz_pan));
		m_savePtzTiltShowWidget->setShowValue(QString::number(paramData.ptz_tilt));
		m_saveFarZoomShowWidget->setShowValue(QString::number(paramData.hc_zoom_far));
		m_saveFarFocusShowWidget->setShowValue(QString::number(paramData.hc_focus_far));
        
        // 更新输入框中的值;
        m_visibleLightScaleNearWidget->setLineEditText(QString::number(m_zoomNearValue));
        m_visibleLightFocusNearWidget->setLineEditText(QString::number(m_focusNearValue));
        m_ptzHRotateWidget->setLineEditText(QString::number(m_ptzPanValue));
        m_ptzVRotateWidget->setLineEditText(QString::number(m_ptzTiltValue));

        m_visibleLightScaleFarWidget->setLineEditText(QString::number(m_zoomFarValue));
        m_visibleLightFocusFarWidget->setLineEditText(QString::number(m_focusFarValue));
	}
    else if (type == WheelJudgeTakePhoto::InfraredLightJudge)
	{
		m_addDeviceStackedWidget->setCurrentIndex(2);
		m_pButtonAddDeviceInfraredLastStage->setVisible(false);

		m_infraredFocusValue = paramData.mag_focus;
		m_infraredPtzPanValue = paramData.ptz_pan;
		m_infraredPtzTileValue = paramData.ptz_tilt;

		m_saveInfraredFocusShowWidget->setShowValue(QString::number(paramData.mag_focus));
		m_saveInfraredPtzPanShowWidget->setShowValue(QString::number(paramData.ptz_pan));
		m_saveInfraredPtzTiltShowWidget->setShowValue(QString::number(paramData.ptz_tilt));

        // 更新输入框中的值;
        m_infraredFocusWidget->setLineEditText(QString::number(m_infraredFocusValue));
        
        m_ptzHRotateInfraredWidget->setLineEditText(QString::number(m_infraredPtzPanValue));
        m_ptzVRotateInfraredWidget->setLineEditText(QString::number(m_infraredPtzTileValue));
	}
//	m_addDeviceInfoWidget->show();
}

void DLHangWheelCollectControlWidget::onMoveToDevicePoint(QString strDeviceId)
{
	// 移动至该点;
	WheelRobortDeviceParameterStruct paramData = WHEEL_DEVICE_CONFIG.getWheelDeviceParameterData(strDeviceId);
	QString strPointId = QString::number(paramData.point_id);
	WHEEL_BACK_TO_CORE_SOCKET.robot_control_gotarget_req(strPointId.toStdString(), 0);
}

void DLHangWheelCollectControlWidget::onAddDeviceResult(QString deviceId, bool isSuccess, QString msg)
{
	if (isSuccess)
	{
		// 如果成功，把当前数据插入到树控件中;
		QList<WheelRobortAlarmPathStruct> treeItemData = m_wheelPointTreeData->getNodeAlarmStatusFromDevices(deviceId);
		if (treeItemData.count() != 6)
		{
			// 这里说明数据查询错误;
            emit signalSendOperateMsg("插入失败，数据库查询错误");
			return;
		}
		QStringList itemPathList = m_wheelPointTreeData->getDeviceNodePathFromDevices(deviceId);
		QString strItemPath = itemPathList.join("-");
		QString strMsg = getCurrentTime() + "插入成功  " + strItemPath;
        emit signalSendOperateMsg(strMsg);
		emit signalAddNewItemToTree(deviceId, treeItemData);
	}
	else
	{
		// 把msg显示到界面下方;
		QString strMsg = getCurrentTime() + msg;
        emit signalSendOperateMsg(strMsg);
	}
}

void DLHangWheelCollectControlWidget::onUpdateDeviceResult(QString deviceId, bool isSuccess, QString msg)
{
    if (isSuccess)
    {
        QString strMsg = getCurrentTime() + "数据修改成功";
        emit signalSendOperateMsg(strMsg);
    }
    else
    {
        QString strMsg = getCurrentTime() + "数据修改失败" ;
        emit signalSendOperateMsg(strMsg);
    }
	
}

// void DLHangWheelCollectControlWidget::onDeleteDeviceResult(bool isSuccess, QString msg)
// {
//     // 删除完设备之后需要刷新树控件;
//     m_pDeviceTreeWidget->refreshTreeItemList();
//     // 把msg显示到界面下方;
//     if (isSuccess)
//     {
//         QString strMsg = getCurrentTime() + "数据删除成功";
//         emit signalSendOperateMsg(strMsg);
//     }
//     else
//     {
//         QString strMsg = getCurrentTime() + "数据删除失败";
//         emit signalSendOperateMsg(strMsg);
//     }
// }

void DLHangWheelCollectControlWidget::onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus)
{
	// 更新云台角度数据;
	m_keyboardOperateThread->setRealtimeStatus(realTimeStatus.ptzStatus);
	m_pointId = realTimeStatus.point_id;
	m_PtzHRotate->setShowValue(QString::number(realTimeStatus.doubleLightPtzStatus.pan));
	m_PtzVRotate->setShowValue(QString::number(realTimeStatus.doubleLightPtzStatus.tilt));
	m_isUpdateRealTimeData = true;
}

void DLHangWheelCollectControlWidget::setCurrentOperationType(int operationType, bool isRelease)
{
	if (!isRelease)
	{
		switch (operationType) {
			// 前进;
		case Qt::Key_Up:
		case Qt::Key_Down:
		case Qt::Key_Left:
		case Qt::Key_Right:
		case Qt::Key_A:
		case Qt::Key_S:
		case Qt::Key_D:
		case Qt::Key_W:
			m_keyboardOperateThread->setCurrentOperationType(operationType, false);
			break;
		case Qt::Key_Q:
			m_visibleLightVideoBackWidget->onButtonPressed(VideoButtonType::ZoomOut);
			break;
		case Qt::Key_E:
			m_visibleLightVideoBackWidget->onButtonPressed(VideoButtonType::ZoomIn);
			break;
		case Qt::Key_Z:
			m_visibleLightVideoBackWidget->onButtonPressed(VideoButtonType::FocusStrong);
			break;
		case Qt::Key_C:
			m_visibleLightVideoBackWidget->onButtonPressed(VideoButtonType::focusWeak);
			break;
		case Qt::Key_Shift:
			m_keyboardOperateThread->setRobotSpeedUp(true);
			break;
		default:
			break;
		}
	}
	else
	{
		switch (operationType) {
			// 前进;
		case Qt::Key_Up:
		case Qt::Key_Down:
		case Qt::Key_Left:
		case Qt::Key_Right:
		case Qt::Key_A:
		case Qt::Key_S:
		case Qt::Key_D:
		case Qt::Key_W:
			m_keyboardOperateThread->setCurrentOperationType(operationType, true);
			break;
		case Qt::Key_Q:
			m_visibleLightVideoBackWidget->onButtonReleased(VideoButtonType::ZoomOut);
			break;
		case Qt::Key_E:
			m_visibleLightVideoBackWidget->onButtonReleased(VideoButtonType::ZoomIn);
			break;
		case Qt::Key_Z:
			m_visibleLightVideoBackWidget->onButtonReleased(VideoButtonType::FocusStrong);
			break;
		case Qt::Key_C:
			m_visibleLightVideoBackWidget->onButtonReleased(VideoButtonType::focusWeak);
			break;
		case Qt::Key_Shift:
			m_keyboardOperateThread->setRobotSpeedUp(false);
			break;
		default:
			break;
		}
	}
}

void DLHangWheelCollectControlWidget::setRobotSpeedUp(bool isSoeedUp)
{

}

void DLHangWheelCollectControlWidget::onStartCollectZoomPicture()
{
	sliZoomPtz.clear();
	int _panValue = 0;
	int _tiltValue = 0;
	QList<int> zoomValueList = HikPointData::getInitance()->getZoomValueList();
	int iPtzPan = m_PtzHRotate->getShowValue().toInt();
	int iPtzTilt = m_PtzVRotate->getShowValue().toInt();
//	m_cameraObject->OnVisibleZoomAbs(0);
	Sleep(200);

	QString fileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/ZoomCaptureImage/image/";
	QDir dir;
	if (!dir.exists(fileName))
	{
		dir.mkpath(fileName);
	}
	Sleep(800);
	
	QString filePath = "";
	for (int _i = 0; _i < zoomValueList.size(); _i++)
	{
		QStringList _data;
		m_cameraObject->OnVisibleZoomAbs(zoomValueList[_i]);
		_data.append(QString("%1").arg(zoomValueList[_i]));
		Sleep(10000);
		//中心点
		filePath = fileName + QString("%1_1").arg(zoomValueList[_i]);
		m_cameraObject->signalVisibleCaptureClicked(filePath);
		Sleep(2000);
		_data.append(QString("%1").arg(m_PtzHRotate->getShowValue().toInt()));
		_data.append(QString("%1").arg(m_PtzVRotate->getShowValue().toInt()));

		//去1/4点处
		zoomInfo info_ = HikPointData::getInitance()->getZoomInfoData(zoomValueList[_i]);
		getPtzPanAndTilt(info_, _panValue, _tiltValue);
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(_panValue, _tiltValue);
		Sleep(10000);
		filePath = fileName + QString("%1_2").arg(zoomValueList[_i]);
		m_cameraObject->signalVisibleCaptureClicked(filePath);
		Sleep(2000);
		_data.append(QString("%1").arg(m_PtzHRotate->getShowValue().toInt()));
		_data.append(QString("%1").arg(m_PtzVRotate->getShowValue().toInt()));
		//回原位
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(iPtzPan, iPtzTilt);
		sliZoomPtz.append(_data);
	}
	createTxt(sliZoomPtz);
}
void DLHangWheelCollectControlWidget::getPtzPanAndTilt(zoomInfo info_, int &_panValue, int &_tiltValue)
{
	float width = 560.0;
	float height = 420.0;
	float x = width / 4.0;
	float y = height / 4.0;
	//当前云台值
	int iPtzPanValue = m_PtzHRotate->getShowValue().toInt();
	int iPtzTiltValue = m_PtzVRotate->getShowValue().toInt();

	float horizontalAngle = (x - width / 2.0) / width * info_.levelAangle;
	float verticalAngle = 0 - (y - height / 2.0) / height * info_.verticalAngle;

	//计算后的偏差
	int rotateAngleValue = horizontalAngle * 100;
	int pitchAngleValue = verticalAngle * 100;

	//
	int panValue = iPtzPanValue + rotateAngleValue;
	int tiltValue = iPtzTiltValue - pitchAngleValue;

	if (panValue < 0)
	{
		panValue = panValue + 36000;
	}

	if (tiltValue < 27090 && iPtzTiltValue > 27090)
	{
		tiltValue = 27090;
	}
	if (tiltValue > 3065 && iPtzTiltValue < 3100)
	{
		tiltValue = 3065;
	}

	if (tiltValue > 36000)
	{
		tiltValue = tiltValue - 36000;
	}

	if (tiltValue < 0)
	{
		tiltValue = tiltValue + 36000;
	}
	_panValue = panValue;
	_tiltValue = tiltValue;
}

void DLHangWheelCollectControlWidget::createTxt(QList<QStringList> _data)
{
	QDir *TEST = new QDir;
	bool exist = TEST->exists("TEST");
	if (!exist)
		bool ok = TEST->mkdir("TEST");

	QString fileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/ZoomCaptureImage/zoom.txt";

	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
	{
		return;
	}
	file.resize(0);
	QTextStream in(&file);
	in.setCodec("utf-8");
	for (int i = 0; i < _data.size(); i++)
	{
		for (int j = 0; j < _data[i].size(); j++)
		{
			in << _data[i][j];
			if (j == _data[i].size() - 1)
			{
				in << "\n";
			}
			else
			{
				in << " ";
			}
		}
	}
	file.close();
}


// void DLHangWheelCollectControlWidget::showEvent(QShowEvent *event)
// {
// 	if (m_bIsFirstIn)
// 	{
// 		m_bIsFirstIn = false;
// 		QWidget *pParentWgt = m_pModelWgt->parentWidget();
// 		if (NULL != pParentWgt)
// 		{
// 			QRect rect = pParentWgt->rect();
// 			QPoint ptTopLeft = rect.center() - QPoint(ICON_SIZE/2, ICON_SIZE/2);
// 			m_pModelWgt->move(ptTopLeft);
// 		}
// 	}
// }

void DLHangWheelCollectControlWidget::slot_ret_finished_upload(int ret_code, QString err_msg)
{
    if (ret_code == 0)
    {
        emit signalSendOperateMsg("远程升级成功！");
    }
    else
    {
        emit signalSendOperateMsg("远程升级失败！");
    }
}
