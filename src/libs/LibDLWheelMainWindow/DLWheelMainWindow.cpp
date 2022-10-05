#include "DLWheelMainWindow.h"
#include <QHBoxLayout>
#include <QFile>
#include "LibDLHangRailCommonWidget/WheelRobotLoginWindow.h"
#include "LibDLHangRailCommonTools/DLHangRailCommonTools.h"
#include "DLWheelRobotManager.h"
#include <QApplication>
#include <QDesktopWidget>
#include "BottomWidget.h"
#include "SystemGuideWidget.h"
#include "DLWheelTaskManage.h"
#include "DLWheelTaskShow.h"
#include "DLWheelSoftSet.h"
#include "DLWheelRobotSet.h"
#include "DLWheelRobotAlarmSearch.h"
#include "DLWheelIdentifyAbnormalPointSearch.h"
#include "DLWheelPatrolPointPosSet.h"
#include "DLWheelDeviceAlarmSearchAffirm.h"
#include "DLWheelMainLineShow.h"
#include "DLWheelPatrolResultBrowse.h"
#include "DLWheelIntervalShow.h"
#include "DLWheelCompareAnalysis.h"
#include "DLWheelGenerateExcel.h"
#include "DLWheelAlarmMessageSubscription.h"
#include "DLWheelStandardPointPosMaintain.h"
#include "LibDLWheelRobotStateShow/DLWheelRobotStateShow.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "LibDLWheelCustomWidget/ParseUrl.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLHangRailCommonWidget/CameraObject.h"
#include "LibProtoClient/ProtoClient.h"
#include "DLWheelAlarmThresholdSet.h"
#include "DLWheelPermissionManager.h"
//#include "LibDLInfraredTemperature/InfraredTemperature.h"
#include "DLWheelPatrolReportGenerate.h"
//#include "LibDLInfraredTemperature/TempChoose.h"
#include "LibDLFixedAreaWidget/DLFixedAreaWidget.h"
#include "LibDLWheelCollectMapWidget/LibDLWheelCollectMapWidget.h"
#include <QProcess>
#include <QGuiApplication>
#include <QWindow>
#include "LibDLHangRailCommonTools/ClearOutDateFileObject.h"
#include "LibDLHangRailCommonWidget/ConfidenceValueShowWidget.h"
#include "LibDLHangRailCommonWidget/BatteryWgt.h"
#include "LibDLWheelCompareDetection/DLWheelCompareDetection.h"
#include "LibDataTransfer/DataTransfer.h"

#pragma execution_character_set("utf-8")

#define APP_NAME "WheelRobot"
#define TTUI 0
DLWheelMainWindow::DLWheelMainWindow(QWidget *parent)
	: QWidget(parent)
    , m_imageSaveMaxDays(365)
    , m_videoSaveMaxDays(365)
	, m_ptStartPos(0, 0)
	, m_bIsPressed(false)
	, m_rBatteryValue(0.0)
	, m_rConfidenValue(0.0)
	, m_pUpdateRobotStatusTimer(NULL)
	, m_bIsChargeState(true)
	, m_pChargeStateLbl(NULL)
	, m_pThreadDataTransfer(NULL)
{
    qRegisterMetaType<WheelRobotCoreRobotConfig>("WheelRobotCoreRobotConfig");

	Q_INIT_RESOURCE(resource);
    qRegisterMetaType<WheelRobotCurrentTaskInfoShow>("WheelRobotCurrentTaskInfoShow");
    qRegisterMetaType<WheelRobotTaskCurrentPointStatus>("WheelRobotTaskCurrentPointStatus");
    qRegisterMetaType<WheelInspectResultStruct>("WheelInspectResultStruct");
    qRegisterMetaType<WheelRobotRealtimeStatus>("WheelRobotRealtimeStatus");
	qRegisterMetaType<WheelRobotNoneRealtimeStatus>("WheelRobotNoneRealtimeStatus");
	qRegisterMetaType<DeviceAlarmLevel>("DeviceAlarmLevel");
    
	m_confidenceImages[0].load(":/Resources/Common/image/Location-low.png");
	m_confidenceImages[1].load(":/Resources/Common/image/Location-ok.png");
	initWidget();
	initCoreSignals();
    initTransferFunction();

	// 设置样式;
	QString strStyleSheet = this->styleSheet();
	QString strFilePath = QString("%1/%2").arg(qApp->applicationDirPath()).arg("style/AppDLWheelRobot/common/main.css");
	QFile file(strFilePath);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		strStyleSheet += file.readAll();
		file.close();
	}
	this->setStyleSheet(strStyleSheet);
	this->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowMinimizeButtonHint);
	DLHangRailCommonTools::loadStyleSheet(this, ":/Resources/DLWheelMainWindow/DLWheelMainWindow.css");


#if TTUI
    onLogin("root", QString("q1w2e3r4").toLatin1().toBase64());
#else
 	m_loginWindow = new WheelRobotLoginWindow;
	m_loginWindow->show();

	connect(m_loginWindow, &WheelRobotLoginWindow::signalLogin, this, &DLWheelMainWindow::onLogin);
#endif
    

}

DLWheelMainWindow::~DLWheelMainWindow()
{
}

bool DLWheelMainWindow::eventFilter(QObject *watched, QEvent *event)
{
	if (watched == m_pMouseWgt)
	{
		switch (event->type())
		{
		case QEvent::MouseButtonPress:
		{
			m_bIsPressed = true;
			QMouseEvent *pMouseEvent = dynamic_cast<QMouseEvent *>(event);
			if (NULL != pMouseEvent)
			{
				m_ptStartPos = QCursor::pos();
			}
		}break;
		case QEvent::MouseMove:
		{
			if (m_bIsPressed)
			{
				QMouseEvent *pMouseEvent = dynamic_cast<QMouseEvent *>(event);
				if (NULL != pMouseEvent)
				{
					QPoint ptCurPos = QCursor::pos();
					//qDebug() << "current pos:" << ptCurPos << ";move pos:" << ptCurPos - m_ptStartPos << __LINE__;

					if (ptCurPos.y() <= 0)
					{
						QDesktopWidget *pDesktopWgt = QApplication::desktop();
						if (pDesktopWgt)
						{
							int iCount = pDesktopWgt->screenCount();
							int iX = ptCurPos.x();
							for (int index = 0; index < iCount; ++index)
							{
								QRect rect = pDesktopWgt->screenGeometry(index);
								if (iX > rect.left() && iX < rect.right())
								{
									this->move(rect.center() - this->rect().center());
								}
							}

							m_bIsPressed = false;
							m_ptStartPos = QPoint(0, 0);
						}
					}
					else
					{
						this->move(this->pos() + (ptCurPos - m_ptStartPos));
						m_ptStartPos = ptCurPos;
					}

				}
			}
		}break;
		case QEvent::MouseButtonRelease:
		{
			m_bIsPressed = false;
			m_ptStartPos = QPoint(0,0);
		}break;
		defalut:break;
		}
	}
	return __super::eventFilter(watched, event);
}

void DLWheelMainWindow::initWidget()
{

	m_pThreadDataTransfer = new DataTransfer(this);
	connect(m_pThreadDataTransfer, SIGNAL(sig_finished(int, int, QString)), this, SLOT(slot_on_transfer_finished(int, int, QString)));
    connect(m_pThreadDataTransfer, SIGNAL(sig_finished_infrared(QString, QString)), this, SLOT(slot_on_transfer_finished_infrared(QString, QString)));

    initBlockWidget();
	initTitleWidget();
	initStackedWidget();
    initImageAndVideoSaveSet();

	m_pBottomWgt = new BottomWidget;
	m_pBottomWgt->setObjectName("BottomWidget");
	connect(m_pBottomWgt, &BottomWidget::signalItemClicked, this, &DLWheelMainWindow::onSystemMenuItemClicked);
	connect(m_pBottomWgt, SIGNAL(CloseItemSignal(SystemGuideMenuItemType)), this, SLOT(CloseSystemMenuItemSlot(SystemGuideMenuItemType)));

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_pTitleWgt);
	vMainLayout->addWidget(m_centerWidget);
	vMainLayout->addWidget(m_pBottomWgt);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(0);

	m_pUpdateRobotStatusTimer = new QTimer(this);
	m_pUpdateRobotStatusTimer->setInterval(600);
	connect(m_pUpdateRobotStatusTimer, SIGNAL(timeout()), this, SLOT(UpdateRobotStatusTimerSlot()));
}

void DLWheelMainWindow::initTitleWidget()
{
	m_pTitleWgt = new QWidget(this);
	m_pTitleWgt->setObjectName("TitleWidget");
	m_pTitleWgt->setFixedHeight(100);

	//拖拽窗口
	m_pMouseWgt = new QWidget(m_pTitleWgt);
	m_pMouseWgt->installEventFilter(this);

	m_pButtonHomePage = new QToolButton(m_pMouseWgt);
	m_pButtonHomePage->setText("首页");
	m_pButtonHomePage->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonHomePage->setObjectName("TitleToolButton");
	m_pButtonHomePage->setIconSize(QSize(25, 25));
	m_pButtonHomePage->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/HomePageButton.png"));
	m_pButtonHomePage->setFixedSize(QSize(70, 30));
	connect(m_pButtonHomePage, &QPushButton::clicked, this, [=] {
		// 首页跳转;
		onSystemMenuItemClicked(Menu_RobotManage);
	});

	//m_pButtonHelp = new QToolButton(m_pMouseWgt);
	//m_pButtonHelp->setText("帮助");
	//m_pButtonHelp->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	//m_pButtonHelp->setObjectName("TitleToolButton");
	//m_pButtonHelp->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/HelpButton.png"));
	//m_pButtonHelp->setIconSize(QSize(25, 25));
	//m_pButtonHelp->setFixedSize(QSize(70, 30));

	m_pButtonMin = new QToolButton(m_pMouseWgt);
	m_pButtonMin->setText("最小化");
	m_pButtonMin->setToolTip("最小化");
	m_pButtonMin->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonMin->setObjectName("TitleToolButton");
	m_pButtonMin->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/min.png"));
	m_pButtonMin->setIconSize(QSize(25, 25));
	m_pButtonMin->setFixedSize(QSize(70, 30));

	connect(m_pButtonMin, &QPushButton::clicked, this, [=] {
		this->showMinimized();
	});

	m_pButtonQuit = new QToolButton(m_pMouseWgt);
	m_pButtonQuit->setText("退出");
	m_pButtonQuit->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonQuit->setObjectName("TitleToolButton");
	m_pButtonQuit->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/QuitButton.png"));
	m_pButtonQuit->setIconSize(QSize(25, 25));
	m_pButtonQuit->setFixedSize(QSize(70, 30));
	
	connect(m_pButtonQuit, &QPushButton::clicked, this, [=] {
        disconnect(this, &DLWheelMainWindow::signalUpdateRobotRealtimeStatus, m_pRobotStateShow, &DLWheelRobotStateShow::onUpdateRobotRealtimeStatus);
        disconnect(this, &DLWheelMainWindow::signalUpdateRobotNoneRealtimeStatus, m_pRobotStateShow, &DLWheelRobotStateShow::onUpdateRobotNoneRealtimeStatus);
		this->close();
	});

	m_labelWelcomeText = new QLabel(m_pMouseWgt);
	m_labelWelcomeText->setText("欢迎您，【运维一班】ztjcys");

	QHBoxLayout *pStopLayout = new QHBoxLayout;
	pStopLayout->setSpacing(10);
	pStopLayout->setMargin(0);

	m_pStutterStopBtn = new QPushButton(m_pMouseWgt);
	pStopLayout->addWidget(m_pStutterStopBtn);
	//connect(m_pStutterStopBtn, SIGNAL(clicked()), this, SLOT(StutterStopBtnSlot()));
	m_pStutterStopBtn->setToolTip("急停");
	m_pStutterStopBtn->setFixedSize(30, 30);
	m_pStutterStopBtn->setObjectName("stutterStopBtn");
	m_pStutterStopBtn->setCheckable(true);
	m_pStutterStopBtn->setEnabled(false);
    m_pStutterStopBtn->setShortcut(QKeySequence("Ctrl+Q"));
	
	QHBoxLayout *pConfidenValueLayout = new QHBoxLayout;
	pConfidenValueLayout->setMargin(0);
	pConfidenValueLayout->setSpacing(0);	
	pStopLayout->addLayout(pConfidenValueLayout);


	m_pConfidenValueLbl = new QLabel(m_pMouseWgt);			//置信度图标
	pConfidenValueLayout->addWidget(m_pConfidenValueLbl);
	m_pConfidenValueLbl->setObjectName("confidenValueLbl");
	m_pConfidenValueLbl->setFixedSize(18, 30);
	QSize size(18, 18);
	QPixmap pixmap;
	pixmap.load(":/Resources/Common/image/Location-low.png");
	pixmap = pixmap.scaled(size, Qt::KeepAspectRatio);
	m_pConfidenValueLbl->setPixmap(pixmap);
	//pIconLayout->addWidget(m_pConfidenValueLbl);

    // 置信度:
    m_pConfidenceValueWgt = new ConfidenceValueShowWidget(m_pMouseWgt);
	pConfidenValueLayout->addWidget(m_pConfidenceValueWgt);
	//	m_electricQuantityLabel = new QLabel;
//	m_electricQuantityLabel->resize(40, 40);
// 	m_electricQuantityLabel->setScaledContents(true);
// 	m_electricQuantityLabel->setAlignment(Qt::AlignCenter);
// 	m_electricQuantityLabel->setStyleSheet(QString("QLabel{border-image:url(E:/twinkstar/11.png);\
// 							font-size:16px;}"));
// 	m_electricQuantityLabel->setText(tr("54%"));

	QHBoxLayout *pBatterLevelLayout = new QHBoxLayout;
	pBatterLevelLayout->setMargin(0);
	pBatterLevelLayout->setSpacing(0);
	pStopLayout->addLayout(pBatterLevelLayout);

	//充电状态的窗口
	QWidget *pChargeWgt = new QWidget(m_pMouseWgt);
	pBatterLevelLayout->addWidget(pChargeWgt);
	pChargeWgt->setFixedHeight(18);
	QHBoxLayout *pChargeLayout = new QHBoxLayout(pChargeWgt);
	pChargeLayout->setSpacing(0);
	pChargeLayout->setMargin(0);

	QVBoxLayout *pLayout = new QVBoxLayout;
	pChargeLayout->addLayout(pLayout);
	pLayout->setSpacing(0);
	pLayout->setMargin(0);
	pLayout->addStretch();
	m_pChargeStateLbl = new QLabel(pChargeWgt);			//充电图标
	m_pChargeStateLbl->setFixedHeight(9);
	pLayout->addWidget(m_pChargeStateLbl);

	m_pBatteryLevelLbl = new QLabel(pChargeWgt);			//电池电量图标
	pChargeLayout->addWidget(m_pBatteryLevelLbl);
	m_pBatteryLevelLbl->setObjectName("batteryLevelLbl");
	m_pBatteryLevelLbl->setFixedSize(18, 18);

// 	m_pBatteryWgt = new BatteryWgt(m_pMouseWgt);
// 	//pBatterLevelLayout->addWidget(m_pBatteryWgt);
// 	m_pBatteryWgt->setFixedSize(14, 20);
// 	m_pBatteryWgt->setCurrentValue(0.4);

	m_pBatteryLevelWgt = new ConfidenceValueShowWidget(m_pMouseWgt);
	pBatterLevelLayout->addWidget(m_pBatteryLevelWgt);
	m_pBatteryLevelWgt->setCurrentValue(0.4);

	//添加系统信息按钮
	m_sys_info_btn = new QPushButton(m_pMouseWgt);
	pStopLayout->addWidget(m_sys_info_btn);
	connect(m_sys_info_btn, SIGNAL(clicked()), this, SLOT(slot_m_sys_info_btn_clicked()));
	m_sys_info_btn->setToolTip("系统信息");
	m_sys_info_btn->setFixedSize(18, 18);
	m_sys_info_btn->setCheckable(true);
	m_sys_info_btn->setChecked(false);
	m_sys_info_btn->setObjectName("sysInfoBtn");
	m_sys_info_btn->setShortcut(QKeySequence("Ctrl+I"));

	QSize sys_info_size(18, 18);
	QPixmap sys_info_pixmap;
	sys_info_pixmap.load(":/Resources/Common/image/sys_info.png");
	sys_info_pixmap = sys_info_pixmap.scaled(sys_info_size, Qt::KeepAspectRatio);
	m_sys_info_btn->setIcon(QIcon(sys_info_pixmap));
	

	// 系统导航;
	m_pSystemGuideWgt = new SystemGuideWidget(this);
	connect(m_pSystemGuideWgt, &SystemGuideWidget::signalMenuItemClicked, this, &DLWheelMainWindow::onSystemMenuItemClicked);
    connect(m_pSystemGuideWgt, &SystemGuideWidget::signalWindowMinsize, this, &DLWheelMainWindow::showMinimized);
    connect(m_pSystemGuideWgt, &SystemGuideWidget::signalCompareDetection, this, &DLWheelMainWindow::onCompareDetection);

	m_pLblGuideText = new QLabel;
	m_pLblGuideText->setObjectName("GuideTextLabel");
	m_pLblGuideText->setText("机器人管理>机器人管理");
	m_pLblGuideText->setFixedSize(QSize(750, 35));

	m_pGuideBackWgt = new QWidget;
	m_pGuideBackWgt->setObjectName("GuideBackWidget");
	m_pGuideBackWgt->setFixedHeight(50);
	/////////
	m_pButtonTitleLogo = new QPushButton;
	m_pButtonTitleLogo->setStyleSheet("border:none;");
	m_pButtonTitleLogo->setFixedSize(QSize(90, 90));
	m_pButtonTitleLogo->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/TitleIcon.png"));
	m_pButtonTitleLogo->setIconSize(m_pButtonTitleLogo->size());

	m_labelTitleSplite = new QLabel;
	m_labelTitleSplite->setObjectName("SpliteLabel");
	m_labelTitleSplite->setFixedWidth(2);
	m_labelTitleSplite->setWordWrap(true);

	m_labelTitleText = new QLabel;
	m_labelTitleText->setObjectName("TitleTextLabel");
	m_labelTitleText->setWordWrap(true);
	m_labelTitleText->setText("集气站机器人\n监控系统"); 
	m_labelTitleText->setAlignment(Qt::AlignCenter);
	m_labelTitleText->setFixedWidth(150);

	m_labelProvinceText = new QLabel;
	m_labelProvinceText->setObjectName("ProvinceTextLabel");
	m_labelProvinceText->setText("中海油");
	m_labelProvinceText->setAlignment(Qt::AlignCenter);
	m_labelProvinceText->setFixedSize(QSize(150, 28));

	QHBoxLayout* hTitleButtonLayout = new QHBoxLayout(m_pMouseWgt);
	hTitleButtonLayout->addWidget(m_pButtonHomePage);
	//hTitleButtonLayout->addWidget(m_pButtonHelp);
	hTitleButtonLayout->addWidget(m_pButtonMin);
	hTitleButtonLayout->addWidget(m_pButtonQuit);
	hTitleButtonLayout->addSpacing(40);
	hTitleButtonLayout->addWidget(m_labelWelcomeText);
	hTitleButtonLayout->addStretch();
	hTitleButtonLayout->addLayout(pStopLayout);
	//	hTitleButtonLayout->addWidget(m_electricQuantityLabel);
//	hTitleButtonLayout->addSpacing(20);
	//hTitleButtonLayout->addWidget(m_pStutterStopBtn);
	//hTitleButtonLayout->addWidget(m_pConfidenValueLbl);		//置信度图标
    //hTitleButtonLayout->addWidget(m_pConfidenceValueWgt);//置信度
	//hTitleButtonLayout->addWidget(m_pBatteryLevelLbl);		//电池电量图标
	//hTitleButtonLayout->addWidget(m_pBatteryLevelWgt);		//电池电量
	//hTitleButtonLayout->addLayout(pBatterLevelLayout);
	hTitleButtonLayout->setSpacing(0);
	hTitleButtonLayout->setContentsMargins(30, 0, 0, 0);

	QHBoxLayout* hGuideLayout = new QHBoxLayout(m_pGuideBackWgt);
	hGuideLayout->addWidget(m_pSystemGuideWgt);
	hGuideLayout->addWidget(m_pLblGuideText);
	hGuideLayout->addStretch();
	hGuideLayout->setSpacing(10);
	hGuideLayout->setContentsMargins(10, 5, 0, 5);

	QVBoxLayout* vLeftTitleLayout = new QVBoxLayout;
	//vLeftTitleLayout->addLayout(hTitleButtonLayout);
	vLeftTitleLayout->addWidget(m_pMouseWgt);
	vLeftTitleLayout->addWidget(m_pGuideBackWgt);
	vLeftTitleLayout->setSpacing(5);
	vLeftTitleLayout->setMargin(0);

	QVBoxLayout* vTitleTextLayout = new QVBoxLayout;
	vTitleTextLayout->addWidget(m_labelTitleText);
	vTitleTextLayout->addWidget(m_labelProvinceText);
	vTitleTextLayout->setSpacing(5);
	vTitleTextLayout->setMargin(5);

	QHBoxLayout* hTitleLayout = new QHBoxLayout(m_pTitleWgt);
	hTitleLayout->addLayout(vLeftTitleLayout);
	hTitleLayout->addWidget(m_pButtonTitleLogo);
	hTitleLayout->addWidget(m_labelTitleSplite);
	hTitleLayout->addLayout(vTitleTextLayout);
	hTitleLayout->setSpacing(10);
	hTitleLayout->setContentsMargins(0, 5, 5, 0);
}

void DLWheelMainWindow::StutterStopBtnSlot()
{
    if (m_pStutterStopBtn->isChecked())
    {
        WHEEL_BACK_TO_CORE_SOCKET.robot_config_urgency_stop_req(1);
    }
    else
    {
        WHEEL_BACK_TO_CORE_SOCKET.robot_config_urgency_stop_req(0);

    }
}

void DLWheelMainWindow::slot_m_sys_info_btn_clicked()
{
	onSystemMenuItemClicked(Menu_RobotStateShow);
}

void DLWheelMainWindow::initStackedWidget()
{
	m_centerWidget = new QWidget;

	m_pRobotRealTimeManager = new DLWheelRobotManager;
    connect(m_pRobotRealTimeManager, &DLWheelRobotManager::signalStartChangeRobotConnect, this, [=] {
        m_blockMessageBox->show();
    });

	connect(m_pRobotRealTimeManager, &DLWheelRobotManager::signalChangeCameraShowIndex, this, [=](int camera_index) {
		ROS_WARN("DLWheelMainWindow::initStackedWidget==camera_index:%d", camera_index);
		if (0 == camera_index)
		{
			m_visibleLightCameraObject->resetCamera(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcIP, WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcCtrlPort, 
								WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName, WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword, 
								WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcRtspPort);
		}
		else if (1 == camera_index)
		{
			m_visibleLightCameraObject->resetCamera(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcIPFront, WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcFrontCtrlPort,
				WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName, WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword,
				WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcFrontRtspPort);
		}
		else if (2 == camera_index)
		{
			m_visibleLightCameraObject->resetCamera(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcIPBack, WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcBackCtrlPort,
				WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName, WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword,
				WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcBackRtspPort);
		}
	});

	// 任务管理;
    // 注意，这里调用顺序不能更改;
	m_robotTaskManageAll_Patrol = new DLWheelTaskManage(Menu_All_Patrol);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageAll_Patrol);

	m_robotTaskManageRoutine_Patrol = new DLWheelTaskManage(Menu_Routine_Patrol);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageRoutine_Patrol);

	m_robotTaskManageInfraredCalculateTmp = new DLWheelTaskManage(Menu_InfraredCalculateTmp);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageInfraredCalculateTmp);

	m_robotTaskManageOilTableRecord = new DLWheelTaskManage(Menu_OilTableRecord);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageOilTableRecord);

	m_robotTaskManageArresterTableRead = new DLWheelTaskManage(Menu_ArresterTableRead);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageArresterTableRead);

	m_robotTaskManageSF6PressureRecord = new DLWheelTaskManage(Menu_SF6PressureRecord);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageSF6PressureRecord);

	m_robotTaskManageHydraumaticTableRecord = new DLWheelTaskManage(Menu_HydraumaticTableRecord);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageHydraumaticTableRecord);

	m_robotTaskManagePosStateRecognition = new DLWheelTaskManage(Menu_PosStateRecognition);
	m_rootTaskManagerWidgetList.append(m_robotTaskManagePosStateRecognition);

	m_robotTaskManageBadWeatherPatrol = new DLWheelTaskManage(Menu_BadWeatherPatrol);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageBadWeatherPatrol);

	m_robotTaskManageDefectTrack = new DLWheelTaskManage(Menu_DefectTrack);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageDefectTrack);

    m_robotTaskManageDistanceAbnormalAlarmAffirm = new DLWheelTaskManage(Menu_DistanceAbnormalAlarmAffirm);
    m_rootTaskManagerWidgetList.append(m_robotTaskManageDistanceAbnormalAlarmAffirm);

	m_robotTaskManageDistanceStateAffirm = new DLWheelTaskManage(Menu_DistanceStateAffirm);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageDistanceStateAffirm);

	m_robotTaskManageSecurityLinkage = new DLWheelTaskManage(Menu_SecurityLinkage);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageSecurityLinkage);

	m_robotTaskManageAssistAccidentDeal = new DLWheelTaskManage(Menu_AssistAccidentDeal);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageAssistAccidentDeal);

	m_robotTaskManageCustomTask = new DLWheelTaskManage(Menu_CustomTask);
	m_rootTaskManagerWidgetList.append(m_robotTaskManageCustomTask);

    // 任务展示点击修改按钮时，跳转到对应任务管理页面，任务管理页面点击返回按钮，再返回至任务展示页面;
    for (int i = 0; i < m_rootTaskManagerWidgetList.count(); i++)
    {
        connect(m_rootTaskManagerWidgetList[i], &DLWheelTaskManage::signalGoBackToTaskShow, this, [=] {
            m_robotTaskShow->onRefreshCalendar();
            m_pStackedWidget->setCurrentIndex(Robot_TaskShow);
        });
    }

	// 任务显示;
	m_robotTaskShow = new DLWheelTaskShow;
    // 任务展示页面点击添加按钮，跳转至全面巡检页面;
	connect(m_robotTaskShow, &DLWheelTaskShow::signalJumpToAllPatrolPage, this, [=] {
		onSystemMenuItemClicked(Menu_All_Patrol);
	});
	connect(m_robotTaskShow, &DLWheelTaskShow::signalModifyButtonClicked, this, &DLWheelMainWindow::onTaskShowTableModifyButtonClicked);
	

	// 软件设置;
	m_robotSoftSet = new DLWheelSoftSet;
	// 机器人设置;
	m_robotSet = new DLWheelRobotSet;
	m_robotControl = new LibRobotControlWidget;
	// 机器人告警查询;
	m_robotAlarmSearch = new DLWheelRobotAlarmSearch;
	// 机器人状态显示;
	m_pRobotStateShow = new DLWheelRobotStateShow;
    m_pRobotStateShow->initWidget();

	// 识别异常点位查询;
	m_identifyAbnormalPointSearch = new DLWheelIdentifyAbnormalPointSearch;
	// 巡检点位设置;
	m_patrolPointPosSet = new DLWheelPatrolPointPosSet;
	// 设备告警查询确认;
	m_deviceAlarmSearchAffirm = new DLWheelDeviceAlarmSearchAffirm;

	// 主接线展示;
	m_mainLineShow = new DLWheelMainLineShow;
	// 巡检结果浏览;
	m_patrolResultBrowse = new DLWheelPatrolResultBrowse;
    // 巡检报告生成;
    m_patrolReportGenerate = new DLWheelPatrolReportGenerate;
	// 间隔展示;
	m_intervalShow = new DLWheelIntervalShow;
	// 对比分析;
	m_compareAnalysis = new DLWheelCompareAnalysis;
	// 生成报表;
	m_generateExcel = new DLWheelGenerateExcel;
    // 告警阈值设置;
    m_alarmThresholdSet = new DLWheelAlarmThresholdSet;

	// 告警消息订阅设置;
	m_alarmMessageSubscription = new DLWheelAlarmMessageSubscription;
    // 权限管理;
    m_permissionManager = new DLWheelPermissionManager;

	// 标准点位库维护;
	m_standardPointPosMaintain = new DLWheelStandardPointPosMaintain;

    // 检修区域设置;
    m_fixedAreaWidget = new DLFixedAreaWidget;

    // 巡检地图维护;
    m_pPatrolMapMaintainWidget = new DLWheelCollectMapWidget(false);
	connect(m_pPatrolMapMaintainWidget, SIGNAL(OpenSmapFileSignal(QString)), this, SLOT(OpenSmapFileSlot(QString)));


    // 异物检测;
    //m_compareDetection = new DLWheelCompareDetection;

	m_pStackedWidget = new QStackedWidget;

	QHBoxLayout* centerLayout = new QHBoxLayout(m_centerWidget);
	centerLayout->addWidget(m_pStackedWidget);
	centerLayout->setMargin(0);
	centerLayout->setSpacing(0);

	m_pStackedWidget->insertWidget(Robot_RealTimeManager, m_pRobotRealTimeManager);
	// 任务管理;
	m_pStackedWidget->insertWidget(Robot_TaskManage_All_Patrol, m_robotTaskManageAll_Patrol);
	m_pStackedWidget->insertWidget(Robot_TaskManage_Routine_Patrol, m_robotTaskManageRoutine_Patrol);
	m_pStackedWidget->insertWidget(Robot_TaskManage_InfraredCalculateTmp, m_robotTaskManageInfraredCalculateTmp);
	m_pStackedWidget->insertWidget(Robot_TaskManage_OilTableRecord, m_robotTaskManageOilTableRecord);
	m_pStackedWidget->insertWidget(Robot_TaskManage_ArresterTableRead, m_robotTaskManageArresterTableRead);
	m_pStackedWidget->insertWidget(Robot_TaskManage_SF6PressureRecord, m_robotTaskManageSF6PressureRecord);
	m_pStackedWidget->insertWidget(Robot_TaskManage_HydraumaticTableRecord, m_robotTaskManageHydraumaticTableRecord);
	m_pStackedWidget->insertWidget(Robot_TaskManage_PosStateRecognition, m_robotTaskManagePosStateRecognition);
	m_pStackedWidget->insertWidget(Robot_TaskManage_BadWeatherPatrol, m_robotTaskManageBadWeatherPatrol);
	m_pStackedWidget->insertWidget(Robot_TaskManage_DefectTrack, m_robotTaskManageDefectTrack);
	m_pStackedWidget->insertWidget(Robot_TaskManage_DistanceStateAffirm, m_robotTaskManageDistanceStateAffirm);
	m_pStackedWidget->insertWidget(Robot_TaskManage_DistanceAbnormalAlarmAffirm, m_robotTaskManageDistanceAbnormalAlarmAffirm);
	m_pStackedWidget->insertWidget(Robot_TaskManage_SecurityLinkage, m_robotTaskManageSecurityLinkage);
	m_pStackedWidget->insertWidget(Robot_TaskManage_AssistAccidentDeal, m_robotTaskManageAssistAccidentDeal);
	m_pStackedWidget->insertWidget(Robot_TaskManage_CustomTask, m_robotTaskManageCustomTask);

	m_pStackedWidget->insertWidget(Robot_TaskShow, m_robotTaskShow);
	m_pStackedWidget->insertWidget(Robot_SoftSet, m_robotSoftSet);
	
	/// 机器人设置
	//m_pStackedWidget->insertWidget(Robot_Set, m_robotSet);
	m_pStackedWidget->insertWidget(Robot_Set, m_robotControl);

	m_pStackedWidget->insertWidget(Robot_AlarmSearch, m_robotAlarmSearch);
	m_pStackedWidget->insertWidget(Robot_StateShow, m_pRobotStateShow);
	m_pStackedWidget->insertWidget(Robot_IdentifyAbnormalPointSearch, m_identifyAbnormalPointSearch);
	m_pStackedWidget->insertWidget(Robot_PatrolPointPosSet, m_patrolPointPosSet);
	m_pStackedWidget->insertWidget(Robot_DeviceAlarmSearchAffirm, m_deviceAlarmSearchAffirm);
	//m_pStackedWidget->insertWidget(Robot_MainLineShow, m_mainLineShow);
	m_pStackedWidget->insertWidget(Robot_PatrolResultBrowse, m_patrolResultBrowse);
    m_pStackedWidget->insertWidget(Robot_PatrolReportGenerate, m_patrolReportGenerate);
	//m_pStackedWidget->insertWidget(Robot_IntervalShow, m_intervalShow);
	m_pStackedWidget->insertWidget(Robot_CompareAnalysis, m_compareAnalysis);
	m_pStackedWidget->insertWidget(Robot_GenerateExcel, m_generateExcel);
    m_pStackedWidget->insertWidget(Robot_AlarmThresholdSet, m_alarmThresholdSet);
	m_pStackedWidget->insertWidget(Robot_AlarmMessageSubscription, m_alarmMessageSubscription);	
    //m_pStackedWidget->insertWidget(Robot_PermissionManager, m_permissionManager);
	//m_pStackedWidget->insertWidget(Robot_StandardPointPosMaintain, m_standardPointPosMaintain);
    //m_pStackedWidget->insertWidget(Robot_FixedAreaSet, m_fixedAreaWidget);
    //m_pStackedWidget->insertWidget(Robot_PatrolMapMaintain, m_pPatrolMapMaintainWidget);
    //m_pStackedWidget->insertWidget(Robot_CompareDetection, m_compareDetection);
	m_pStackedWidget->setCurrentIndex(Robot_RealTimeManager);
	showRealTimeMonitorWidget(Menu_RobotManage);
}

void DLWheelMainWindow::initBlockWidget()
{
    m_blockMessageBox = new DLMessageBox();
    m_blockMessageBox->setFixedSize(QSize(300, 180));
    m_blockMessageBox->setMessageContent("机器人切换中，请耐心等待。。。");
    m_blockMessageBox->setButtonOKVisible(false);
    m_blockMessageBox->setWindowModality(Qt::ApplicationModal);
    m_blockMessageBox->setVisible(false);
}

void DLWheelMainWindow::initCoreSignals()
{
	WHEEL_BACK_TO_CORE_SOCKET.InitVoiceSpeak();			//初始化语音播报
    // 更新任务展示中table数据删除结果;
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotTaskChooseDeleteStatus.connect(boost::bind(&DLWheelMainWindow::signalUpdateDeleteResult, this, _1, _2, _3));
    // 任务管理中保存任务编制返回结果;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotTaskEditInsertStatus.connect(boost::bind(&DLWheelMainWindow::signalTaskManageSaveRsp, this, _1, _2, _3));
    // 任务管理中更新任务编制返回结果;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotTaskEditUpdataStatus.connect(boost::bind(&DLWheelMainWindow::signalTaskManageUpdateRsp, this, _1, _2, _3));
    //任务管理中是删除任务编制返回结果;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotTaskEditDeleteStatus.connect(boost::bind(&DLWheelMainWindow::signalDeleteTaskCallBack, this, _1, _2, _3));

    // 实时更新状态;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotRealtimeStatus.connect(boost::bind(&DLWheelMainWindow::signalUpdateRobotRealtimeStatus, this, _1));
    // 更新非实时状态;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotNoneRealtimeStatus.connect(boost::bind(&DLWheelMainWindow::signalUpdateRobotNoneRealtimeStatus, this, _1));

    
    // core通知当前任务开始回调;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotCurrentTaskInfoCallback.connect(boost::bind(&DLWheelMainWindow::signalTaskCallBack, this, _1));
    // core通知当前点状态开始回调;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotCurrentPointStatusCallback.connect(boost::bind(&DLWheelMainWindow::signalPointStatusCallBack, this, _1));
    // core通知巡检结果开始回调;
    WHEEL_BACK_TO_CORE_SOCKET.WheelRobotInspectResultCallback.connect(boost::bind(&DLWheelMainWindow::signalWheelRobotInspectResultCallback, this, _1));
    // 虚拟设备;
    WHEEL_BACK_TO_CORE_SOCKET.WheelRobotCompareInspectResultCallback.connect(boost::bind(&DLWheelMainWindow::signalWheelRobotInspectResultCallback, this, _1));
    // 巡检结果审核人提交回调;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotPatrolResultAuditStatus.connect(boost::bind(&DLWheelMainWindow::signalPatrolResultBrowseCheckPeopleCallBack, this, _1, _2, _3));
    
    // core通知添加系统告警信息;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotSystemWarningCallback.connect(boost::bind(&DLWheelMainWindow::signalSystemWarningCallback, this, _1));
    // core通知机器人硬件异常代码;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotHardwareAlarmCode.connect(boost::bind(&DLWheelMainWindow::signalHardwareWarningCallback, this, _1));

    // core通知切换到新机器人;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotConnect2NewRobot.connect(boost::bind(&DLWheelMainWindow::signalConnectToNewRobotResult, this, _1, _2));

	//core通知更新节点的level
	WHEEL_BACK_TO_CORE_SOCKET.WheelRobotAlarmLevelStatus.connect(boost::bind(&DLWheelMainWindow::signalUpdateTreeItemLevel, this, _1, _2));

	//boost::signals2::signal<void(bool, QString)> singnal_database_fast_audit_task;
	//一键审核
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotFastAuditTaskSignal.connect(boost::bind(&DLWheelMainWindow::signalKeyAduit, this, _1, _2));

	//获取当前机器人配置
	//WHEEL_BACK_TO_CORE_SOCKET.wheelRobotConnect2NewRobot.connect(boost::bind(&DLWheelMainWindow::signalSaveCurrentConfig, this, _1, _2));

	//下载图片信号
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotInfraredTakePhotoSignal.connect(boost::bind(&DLWheelMainWindow::DownloadImageSignal, this, _1, _2, _3));
}

void DLWheelMainWindow::initTransferFunction()
{
    // 进行信号槽中转;

    // 任务管理保存按钮;
    connect(this, &DLWheelMainWindow::signalTaskManageSaveRsp, this, [=](int typeId, bool isSuccess, QString strMsg) {
        WheelTaskAdminType task_edit_type_id = WheelTaskAdminType(typeId);
        m_rootTaskManagerWidgetList.at(task_edit_type_id - 1)->onGetSaveOperationResult(isSuccess, strMsg);
    }, Qt::QueuedConnection);

    // 任务管理保存按钮操作返回，对任务进行修改;
    connect(this, &DLWheelMainWindow::signalTaskManageUpdateRsp, this, [=] (int typeId, bool isSuccess, QString strMsg){
        WheelTaskAdminType task_edit_type_id = WheelTaskAdminType(typeId);
        m_rootTaskManagerWidgetList.at(task_edit_type_id - 1)->onGetSaveOperationResult(isSuccess, strMsg);
    }, Qt::QueuedConnection);

    connect(this, &DLWheelMainWindow::signalDeleteTaskCallBack, this, [=](int typeId, bool isSuccess, QString strMsg) {
        WheelTaskAdminType task_edit_type_id = WheelTaskAdminType(typeId);
        m_rootTaskManagerWidgetList.at(task_edit_type_id - 1)->onDeleteTaskCallBack(isSuccess, strMsg);
    }, Qt::QueuedConnection);

    // core通知当前任务开始回调;
    connect(this, &DLWheelMainWindow::signalTaskCallBack, this, [=](WheelRobotCurrentTaskInfoShow taskData){
        m_pRobotRealTimeManager->onUpdatePatrolState(taskData);
    }, Qt::QueuedConnection);

    // core通知当前点状态开始回调;
    connect(this, &DLWheelMainWindow::signalPointStatusCallBack, this, [=](WheelRobotTaskCurrentPointStatus pointStatus) {

    }, Qt::QueuedConnection);

    // core通知巡检结果开始回调;
    connect(this, &DLWheelMainWindow::signalWheelRobotInspectResultCallback, this, [=](WheelInspectResultStruct inspectResult) {
        m_pRobotRealTimeManager->onReceiveRealTimeInfo(inspectResult);
    }, Qt::QueuedConnection);
    
    // core通知实时状态刷新;
	connect(this, SIGNAL(signalUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus)), this, SLOT(UpdateRobotRealtimeStatusSlot(WheelRobotRealtimeStatus)), Qt::QueuedConnection);
//     connect(this, &DLWheelMainWindow::signalUpdateRobotRealtimeStatus, this, [=](WheelRobotRealtimeStatus realTimeStatus) {
//         
//     }, Qt::QueuedConnection);

    
   
    // core通知添加系统告警信息;
    connect(this, &DLWheelMainWindow::signalSystemWarningCallback, m_pRobotRealTimeManager, &DLWheelRobotManager::onSystemWarningCallback, Qt::QueuedConnection);

    // core通知机器人硬件异常代码;
    connect(this, &DLWheelMainWindow::signalHardwareWarningCallback, this, [=](QVector<int> codeVector) {
        QStringList codeList;
        for (int i = 0; i < codeVector.size(); i++)
        {
            codeList.append(QString::number(codeVector[i]));
        }
        QString strCode = codeList.join(',');
        m_pRobotRealTimeManager->onSystemWarningCallback(strCode);
    });

    // core通知切换到新机器人;
    connect(this, &DLWheelMainWindow::signalConnectToNewRobotResult, this, [=](bool isSuccess, WheelRobotCoreRobotConfig robotConfig) {
        if (isSuccess)
        {
            WHEEL_ROBOT_BACKGROUND_CONFIG.initCoreRobotCfg(robotConfig);
            // 通知重启可见光和红外;
			m_visibleLightCameraObject->resetCamera(robotConfig.hcIP, robotConfig.hcCtrlPort, robotConfig.hcUserName, robotConfig.hcPassword, robotConfig.hcRtspPort);
			//m_visibleLightCameraObject->resetCamera("124.152.79.70", 8000, "admin", "hilongxx123", 554);
            // 缺少红外重启;

			//ROS_INFO("!!!!!!!need = %d\n", robotConfig.infraredNeed);

			/*if (robotConfig.infraredNeed)
			{
				m_pRobotRealTimeManager->setInfraredVisible(true);
				m_infraredCameraObject->resetCamera(robotConfig.infraredCameraIp, robotConfig.infraredCtrlPort, robotConfig.hcUserName, robotConfig.hcPassword, robotConfig.infraredRtspPort);
			}
			else
			{
				m_pRobotRealTimeManager->setInfraredVisible(false);
			}*/
			m_infraredCameraObject->resetCamera(robotConfig.infraredCameraIp, robotConfig.infraredCtrlPort, robotConfig.hcUserName, robotConfig.hcPassword, robotConfig.infraredRtspPort);
			m_blockMessageBox->hide();
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "错误", "机器人创建失败", MessageButtonType::BUTTON_OK, true);
        }
    });

	//core 通知下载图片
    connect(this, SIGNAL(DownloadImageSignal(QString, QString, int)), this, SLOT(DownloadImageSlot(QString, QString, int)));


    connect(m_robotSet, &DLWheelRobotSet::signalVisibleSwitch, this, [=](bool isOpen) {
        m_visibleLightCameraObject->setIsPausePlay(!isOpen);
    });
    connect(m_robotSet, &DLWheelRobotSet::signalInfraredSwitch, this, [=](bool isOpen) {
        m_infraredCameraObject->setIsPausePlay(!isOpen);
    });

    connect(m_pRobotStateShow, &DLWheelRobotStateShow::signalVisibleSwitch, this, [=](bool isOpen) {
        m_visibleLightCameraObject->setIsPausePlay(!isOpen);
    });

    connect(m_pRobotStateShow, &DLWheelRobotStateShow::signalInfraredSwitch, this, [=](bool isOpen) {
        m_infraredCameraObject->setIsPausePlay(!isOpen);
    });
}

void DLWheelMainWindow::initCameraObject()
{
    m_visibleLightCameraObject = new CameraObject(this, true);
    m_visibleLightCameraObject->setNeedRestartCamera();

#if 0
	FILE* fp = fopen("E:\\WorkSpace\\project\\bin\\test_dt", "wb");
	fprintf(fp, "%s\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().visibleCameraIp.toStdString().c_str());
	fprintf(fp, "%d\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcCtrlPort);
	fprintf(fp, "%s\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName.toStdString().c_str());
	fprintf(fp, "%s\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword.toStdString().c_str());
	fprintf(fp, "%d\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcRtspPort);
	fprintf(fp, "\n");
	fprintf(fp, "%s\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().infraredCameraIp.toStdString().c_str());
	fprintf(fp, "%d\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcCtrlPort);
	fprintf(fp, "%s\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName.toStdString().c_str());
	fprintf(fp, "%s\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword.toStdString().c_str());
	fprintf(fp, "%d\n", WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcRtspPort);
	fclose(fp);
#endif

    // 这里要设置相机的连接配置;
#if 0
    m_visibleLightCameraObject->setCameraConnectInfo(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().visibleCameraIp, 
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcCtrlPort,
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName,
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword,
        QString::number(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcRtspPort));
#endif

	m_visibleLightCameraObject->setCameraConnectInfo(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().visibleCameraIp,
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcCtrlPort,
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName,
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword,
		QString::number(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcRtspPort));
    m_visibleLightCameraObject->startPlay();
	m_pRobotRealTimeManager->setCameraObject(m_visibleLightCameraObject); 
	m_pRobotRealTimeManager->initVoiceTalk();

    // 红外视频流;
    m_infraredCameraObject = new CameraObject(this, true);
    //m_infraredCameraObject->setInfraredVideo();
    m_infraredCameraObject->setNeedRestartCamera();
    // 这里要设置相机的连接配置;
#if 0
    m_infraredCameraObject->setCameraConnectInfo(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().infraredCameraIp,
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcCtrlPort,
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName,
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword,
        QString::number(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcRtspPort));
#endif
	m_infraredCameraObject->setCameraConnectInfo(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().infraredCameraIp,
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcCtrlPort,
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcUserName,
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcPassword,
		QString::number(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreRobotCfg().hcRtspPort));
    m_infraredCameraObject->startPlay();
	m_pRobotRealTimeManager->setInfraredObject(NULL, m_infraredCameraObject);

	m_pRobotRealTimeManager->init();
}

void DLWheelMainWindow::initImageAndVideoSaveSet()
{
    m_imageVideoSaveDealLineSetTimer.setInterval(24 * 60 * 60 * 1000);
    connect(&m_imageVideoSaveDealLineSetTimer, &QTimer::timeout, this, [=] {
        clearOutDateImageAndVideoFiles();
    });
}

void DLWheelMainWindow::initClearOutDateFile()
{
    m_clearOutDateFileObject = new ClearOutDateFileObject(this);
    // 清除log文件夹下文件;
    // 注意这里的 APP_NAME 与main里面的 APP_NAME 保持一致;
    QString strFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/log/" + APP_NAME;
    m_clearOutDateFileObject->setClearFileList(QStringList() << strFilePath);
    m_clearOutDateFileObject->startClear();
}

void DLWheelMainWindow::onLogin(QString userName, QString password)
{
    ParseUrl objParseUrl;
    std::string strIp = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().coreServerIp.toStdString();
	///zhw test
	//const char* szDestIP = "127.0.0.1";

    const char *szDestIP = strIp.c_str();
    PingReply reply;
    bool isPingSuccess = objParseUrl.Ping(szDestIP, &reply);
    if (isPingSuccess)
    {
        userLoginRetVal ret = WHEEL_BACK_TO_CORE_SOCKET.doLogin(strIp, WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().coreServerPort, userName.toStdString(), password.toStdString());
        
		//ret.role = WHEEL_USER_MANAGER;
		//ret.retCode = WHEEL_LOGIN_SUCCESS;
		if (ret.retCode == WHEEL_LOGIN_SUCCESS)
        {
            m_currentLoginRole = ret.role;
            WHEEL_ROBOT_BACKGROUND_CONFIG.initCoreCfg(ret.coreCfg);
            WHEEL_ROBOT_BACKGROUND_CONFIG.initCoreRobotCfg(ret.robotCfg);
            GFILE_TRANSFER_CLIENT.init(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().coreServerIp.toStdString(), WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().rcfServerPort);
            m_pRobotRealTimeManager->setRobotChooseInfo(ret.coreCfg.currentRobotName, ret.coreCfg.robotList);
            loginSuccess();
			m_pUpdateRobotStatusTimer->start();			//启动定时器，更新置信度和电池数据
        }
        else
        {
#if TTUI
            this->close();
#else
            m_loginWindow->loginFailed();
#endif
            
        }
    }
    else
    {
        DLMessageBox* messageBox = new DLMessageBox();
        messageBox->setFixedWidth(200);
        messageBox->setMessageContent("服务器连接失败");
        messageBox->setWindowModality(Qt::ApplicationModal);
        messageBox->show();
    }
}

void DLWheelMainWindow::onSystemMenuItemClicked(SystemGuideMenuItemType itemType)
{	
	switch (itemType)
	{
    case Menu_MainPage:
	case Menu_RobotManage:
	case Menu_RobotControl:
	case Menu_PatrolManage:
		showRealTimeMonitorWidget(itemType);
		break;
	case Menu_All_Patrol:	
	case Menu_Routine_Patrol:		
	case Menu_InfraredCalculateTmp:		
	case Menu_OilTableRecord:
	case Menu_ArresterTableRead:		
	case Menu_SF6PressureRecord:		
	case Menu_HydraumaticTableRecord:		
	case Menu_PosStateRecognition:		
	case Menu_BadWeatherPatrol:		
	case Menu_DefectTrack:
	case Menu_DistanceStateAffirm:
	case Menu_DistanceAbnormalAlarmAffirm:
	case Menu_SecurityLinkage:
	case Menu_AssistAccidentDeal:
	case Menu_CustomTask:
		showTaskManageWidget(itemType);
		break;
	case Menu_MapChoosePoint:
		break;
	case Menu_TaskShow:
	{
		m_pLblGuideText->setText("任务管理>任务展示");
		m_robotTaskShow->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_TaskShow);
		connect(this, &DLWheelMainWindow::signalUpdateDeleteResult, m_robotTaskShow, &DLWheelTaskShow::onUpdateDeleteResult);
	}
		break;
	case Menu_DeviceAlarmSearchAffirm:
	{
		m_pLblGuideText->setText("设备告警>设备告警查询");
		m_deviceAlarmSearchAffirm->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_DeviceAlarmSearchAffirm);
	}
		break;
	case Menu_MainLineShow:
	{
		m_pLblGuideText->setText("设备告警>主接线展示");
		m_mainLineShow->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_MainLineShow);
	}
		break;
	case Menu_IntervalShow:
	{
		m_pLblGuideText->setText("设备告警>间隔展示");
		m_intervalShow->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_IntervalShow);
	}
		break;
	case Menu_PatrolResultBrowse:
	{
		m_pLblGuideText->setText("设备告警>巡检结果浏览");
		m_patrolResultBrowse->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_PatrolResultBrowse);
		//一键审核结束
        connect(this, SIGNAL(signalKeyAduit(bool, QString)), m_patrolResultBrowse, SLOT(AKeyAuditFinishSlot(bool, QString)), Qt::UniqueConnection);
		// 巡检结果审核人提交回调;
		connect(this, &DLWheelMainWindow::signalPatrolResultBrowseCheckPeopleCallBack, m_patrolResultBrowse, &DLWheelPatrolResultBrowse::onPatrolResultCheckPeopleCallBack, Qt::QueuedConnection);
	}
		break;
	case Menu_PatrolReportCreate:
    {
        m_pLblGuideText->setText("设备告警>巡检报告生成");
        m_patrolReportGenerate->initWidget();
        m_pStackedWidget->setCurrentIndex(Robot_PatrolReportGenerate);
    }
		break;
	case Menu_CompareAnalysis:
	{
		m_pLblGuideText->setText("巡检结果分析>对比分析");
		m_compareAnalysis->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_CompareAnalysis);	
	}
		break;
	case Menu_CreateReportForm:
	{
		m_pLblGuideText->setText("巡检结果分析>生成报表");
		m_generateExcel->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_GenerateExcel);
	}
		break;
	case Menu_AlarmThresholdSet:
    {
        m_pLblGuideText->setText("用户设置>告警阈值设置");
        m_alarmThresholdSet->initWidget();
        m_pStackedWidget->setCurrentIndex(Robot_AlarmThresholdSet);
    }
		break;
	case Menu_AlarmInfoSubscribeSet:
	{
		m_pLblGuideText->setText("用户设置>告警消息订阅设置");
		m_alarmMessageSubscription->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_AlarmMessageSubscription);
	}
		break;
	case Menu_RightManage:
    {
        //m_pLblGuideText->setText("用户设置>权限管理");
        //m_permissionManager->initWidget(m_currentLoginRole);
        //m_pStackedWidget->setCurrentIndex(Robot_PermissionManager);
    }
		break;
	case Menu_TypicalPatrolPointMaintain:
	{
		//m_pLblGuideText->setText("用户设置>标准点位库维护");
		//m_standardPointPosMaintain->initWidget();
		//m_pStackedWidget->setCurrentIndex(Robot_StandardPointPosMaintain);
	}
		break;
	case Menu_PatrolPointPosSet:
	{
		//m_pLblGuideText->setText("用户设置>巡检点位设置");
		//m_patrolPointPosSet->initWidget();
		//m_pStackedWidget->setCurrentIndex(Robot_PatrolPointPosSet);
	}
		break;
	case Menu_ServiceAreaSet:
    {
        //m_pLblGuideText->setText("用户设置>检修区域设置");
        //m_pStackedWidget->setCurrentIndex(Robot_FixedAreaSet);
    }
		break;
	case Menu_PatrolMapMaintain:
    {
        m_pLblGuideText->setText("机器人系统调试维护>巡检地图维护");
        m_pStackedWidget->setCurrentIndex(Robot_PatrolMapMaintain);
    }
		break;
	case Menu_SoftSet:
	{
		m_pLblGuideText->setText("机器人系统调试维护>软件设置");
		m_robotSoftSet->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_SoftSet);
	}
		break;
	case Menu_RobotSet:
	{
		m_pLblGuideText->setText("机器人系统调试维护>机器人设置");
		//m_robotSet->initWidget();
		m_robotControl->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_Set);
	}
		break;
	case Menu_RobotStateShow: 
	{
		m_pLblGuideText->setText("机器人系统调试维护>机器人状态显示");
		m_pRobotStateShow->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_StateShow);

		// core通知非实时状态刷新;
		connect(this, &DLWheelMainWindow::signalUpdateRobotNoneRealtimeStatus, m_pRobotStateShow, &DLWheelRobotStateShow::onUpdateRobotNoneRealtimeStatus, Qt::QueuedConnection);

	}
		break;
	case Menu_RobotAlarmSearch:
	{
		m_pLblGuideText->setText("机器人系统调试维护>机器人告警查询");
		m_robotAlarmSearch->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_AlarmSearch);
	}
		break;
	case Menu_RecognizeAbnormalPointSearch:
	{
		m_pLblGuideText->setText("机器人系统调试维护>识别异常点位查询");
		m_identifyAbnormalPointSearch->initWidget();
		m_pStackedWidget->setCurrentIndex(Robot_IdentifyAbnormalPointSearch);
	}
		break;
    case Menu_CompareDetection:
    {
        //m_pLblGuideText->setText("异物检测");
        //m_pStackedWidget->setCurrentIndex(Robot_CompareDetection);
        break;
    }
	case Menu_None:
		break;
	default:
		break;
	}

	m_pSystemGuideWgt->hideMenuWidget();
	m_pBottomWgt->addBottomItem(itemType);
}

void DLWheelMainWindow::CloseSystemMenuItemSlot(SystemGuideMenuItemType iType)
{
	if (iType == Menu_PatrolMapMaintain)
	{//巡检地图维护
		m_pPatrolMapMaintainWidget->ClearMap();
		m_pStackedWidget->setCurrentIndex(Robot_RealTimeManager);
		showRealTimeMonitorWidget(Menu_RobotManage);
		//打开当前地图
		//m_pRobotRealTimeManager->OpenMap(strMapFile.replace("smap", "bsmap"));
	}
}

void DLWheelMainWindow::onTaskShowTableModifyButtonClicked(WheelTaskAdminType task_edit_type_id, QString task_edit_uuid)
{
    QString strTaskEditType = QString("task_edit_type_id:%1").arg(task_edit_uuid);
    ROS_INFO(strTaskEditType.toLocal8Bit());
	SystemGuideMenuItemType itemType = SystemGuideMenuItemType(task_edit_type_id + 2);
    onSystemMenuItemClicked(itemType);
    // 这里防止取值越界;
    int pageIndex = itemType - 3;
    if (m_rootTaskManagerWidgetList.count() <= pageIndex || pageIndex < 0)
    {
        return;
    }
    m_rootTaskManagerWidgetList.at(itemType - 3)->onModifyTaskTable(task_edit_uuid);
}

void DLWheelMainWindow::onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus)
{
    emit signalUpdateRobotRealtimeStatus(realTimeStatus);
}

void DLWheelMainWindow::onCompareDetection()
{
    onSystemMenuItemClicked(Menu_CompareDetection);
}

void DLWheelMainWindow::OpenSmapFileSlot(QString strSmapFile)
{
	m_pRobotRealTimeManager->OpenMap(strSmapFile.replace(".smap", ".bsmap"));
}

void DLWheelMainWindow::UpdateRobotRealtimeStatusSlot(WheelRobotRealtimeStatus realTimeStatus)
{
	m_pRobotStateShow->onUpdateRobotRealtimeStatus(realTimeStatus);
	//m_rBatteryValue = realTimeStatus.batteryStatus.voltage;		//电量
	m_rBatteryValue = realTimeStatus.batteryStatus.battery_level * 100.0;
	m_rConfidenValue = realTimeStatus.locStatus.confidence;		//置信度
	m_bIsChargeState = realTimeStatus.batteryStatus.charging;		//是否充电

	if (0 == realTimeStatus.emergency)
	{//运行正常
		m_pStutterStopBtn->setChecked(false);
	}
	else
	{//停止
		m_pStutterStopBtn->setChecked(true);
	}
}

void DLWheelMainWindow::UpdateRobotStatusTimerSlot()
{

	if (nullptr != m_pRobotRealTimeManager && nullptr != m_pRobotStateShow)
	{
		m_pRobotStateShow->SetVisableZoom(m_pRobotRealTimeManager->GetVisableZoom());
	}

	//更新充电图标
	if (m_bIsChargeState)
	{//充电状态
		QSize size(9, 9);
		QPixmap pixmap;
		pixmap.load(":/Resources/Common/image/Charge-green.png");
		pixmap.scaled(size, Qt::KeepAspectRatio);
		m_pChargeStateLbl->setPixmap(pixmap);
	}
	else
	{
		m_pChargeStateLbl->setPixmap(QPixmap());
	}
	m_pChargeStateLbl->repaint();

	//更新电池电量
	m_pBatteryLevelWgt->setCurrentValue(m_rBatteryValue / 100.0);

	//m_pBatteryLevelLbl
	QSize batterySize(18, 18);
	QPixmap batteryPixmap;
	if (m_rBatteryValue < 5.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-empty.png");
	}
	else if (m_rBatteryValue >= 5.0 && m_rBatteryValue < 20.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-low.png");
	}
	else if (m_rBatteryValue >= 20.0 && m_rBatteryValue < 60.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-ok-orange.png");
	}
	else if (m_rBatteryValue >= 60.0 && m_rBatteryValue < 90.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-ok-green.png");
	}
	else if (m_rBatteryValue >= 90.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-full.png");
	}
	batteryPixmap.scaled(batterySize, Qt::KeepAspectRatio);
	m_pBatteryLevelLbl->setPixmap(batteryPixmap);
	m_pBatteryLevelLbl->repaint();

	// 更新置信度;
	int iValue = m_rConfidenValue * 100;
	QSize size(18, 18);
	QPixmap pixmap;
	if (iValue < 60)
	{
		//pixmap.load(":/Resources/Common/image/Location-low.png");
		pixmap = m_confidenceImages[0];
	}
	else
	{
		pixmap = m_confidenceImages[1];
		//pixmap.load(":/Resources/Common/image/Location-ok.png");
	}


	pixmap = pixmap.scaled(size, Qt::KeepAspectRatio);
	m_pConfidenValueLbl->setPixmap(pixmap);
	m_pConfidenValueLbl->repaint();
	// 更新置信度;
	m_pConfidenceValueWgt->setCurrentValue(m_rConfidenValue);
	m_pConfidenceValueWgt->repaint();
}

void DLWheelMainWindow::DownloadImageSlot(QString strPath, QString strFileName, int type)
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

void DLWheelMainWindow::slot_on_transfer_finished(int command_type, int execCode, QString fileName)
{
	switch (command_type)
	{
	case DataTransfer::DOWNLOAD_FILE:
	{
		if (!execCode) {
			//m_pStatusBar->setOperate(fileName + " 下载完毕");
		}
		else {
			//m_pStatusBar->setOperate(fileName + "下载出错");
		}
		break;
	}
	case DataTransfer::UPLOAD_FILE:
	{
		if (!execCode) {
			//下载完毕给core回一条包含下载的好文件名的消息
			WHEEL_BACK_TO_CORE_SOCKET.robot_config_uploadmap_req(fileName);
			//m_pStatusBar->setOperate(fileName + "上传成功");
		}
		else {
			//m_pStatusBar->setOperate(fileName + "上传出错");
		}
		break;
	}
	default:
		break;
	}
}

void DLWheelMainWindow::slot_on_transfer_finished_infrared(QString path, QString name)
{
    qDebug() << path;
    QImage image;
    image.load(path + "/" + name + "/" + name + ".jpg");
    image.save(path + "/" + name + "/" + name + ".jpg");
}

void DLWheelMainWindow::showRealTimeMonitorWidget(SystemGuideMenuItemType itemType)
{
	QString titleText;
	switch (itemType)
	{
    case Menu_MainPage:
	case Menu_RobotManage:
	{
		titleText = "机器人管理>机器人管理";
		m_pRobotRealTimeManager->setRobotMangerComboBoxVisible(true);
		m_pRobotRealTimeManager->setRobotControlWidgetVisible(false);
	}
		break;
	case Menu_RobotControl:
	{
		titleText = "实时监控>机器人遥控";
		m_pRobotRealTimeManager->setRobotMangerComboBoxVisible(false);
		m_pRobotRealTimeManager->setRobotControlWidgetVisible(true);
	}
		break;
	case Menu_PatrolManage:
	{
		titleText = "实时监控>巡检监测";
		m_pRobotRealTimeManager->setRobotMangerComboBoxVisible(false);
		m_pRobotRealTimeManager->setRobotControlWidgetVisible(false);
	}
		break;
	default:
		break;
	}

	m_pLblGuideText->setText(titleText);
	m_pStackedWidget->setCurrentIndex(Robot_RealTimeManager);
}

void DLWheelMainWindow::showTaskManageWidget(SystemGuideMenuItemType itemType)
{
	QString strGuideText = "";
	switch (itemType)
	{
	case Menu_All_Patrol:
		strGuideText = "全面巡检";
		break;
	case Menu_Routine_Patrol:
		strGuideText = "例行巡检";
		break;
	case Menu_InfraredCalculateTmp:
		strGuideText = "红外测温";
		break;
	case Menu_OilTableRecord:
		strGuideText = "油位、油温表抄录";
		break;
	case Menu_ArresterTableRead:
		strGuideText = "避雷器表计读取";
		break;
	case Menu_SF6PressureRecord:
		strGuideText = "SF6压力抄录";
		break;
	case Menu_HydraumaticTableRecord:
		strGuideText = "液压表抄录";
		break;
	case Menu_PosStateRecognition:
		strGuideText = "位置状态识别";
		break;
	case Menu_BadWeatherPatrol:
		strGuideText = "恶劣天气特巡";
		break;
	case Menu_DefectTrack:
		strGuideText = "缺陷跟踪";
		break;
	case Menu_DistanceStateAffirm:
		strGuideText = "远方状态确认";
		break;
	case Menu_DistanceAbnormalAlarmAffirm:
		strGuideText = "远方异常告警确认";
		break;
	case Menu_SecurityLinkage:
		strGuideText = "安防联动";
		break;
	case Menu_AssistAccidentDeal:
		strGuideText = "协助应急事故处理";
		break;
	case Menu_CustomTask:
		strGuideText = "自定义任务";
		break;
	default:
		break;
	}
	m_pLblGuideText->setText(QString("任务管理>%1").arg(strGuideText));
	m_pStackedWidget->setCurrentIndex(StackedWidgetType(itemType - 2));

	DLWheelTaskManage *pTaskManage = m_rootTaskManagerWidgetList.at(itemType - 3);
	if (NULL != pTaskManage)
	{
		pTaskManage->initWidget();
		//任务管理的槽函数
		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), pTaskManage, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
	}
// 	{
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageAll_Patrol, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageRoutine_Patrol, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageInfraredCalculateTmp, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageOilTableRecord, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageArresterTableRead, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageSF6PressureRecord, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageHydraumaticTableRecord, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManagePosStateRecognition, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageBadWeatherPatrol, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageDefectTrack, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageDistanceStateAffirm, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageDistanceAbnormalAlarmAffirm, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageSecurityLinkage, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageAssistAccidentDeal, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 		connect(this, SIGNAL(signalUpdateTreeItemLevel(QString, DeviceAlarmLevel)), m_robotTaskManageCustomTask, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)));
// 	}



}

void DLWheelMainWindow::loginSuccess()
{
    // 图像和视频保存期限设置时钟开启;
    clearOutDateImageAndVideoFiles();
    m_imageVideoSaveDealLineSetTimer.start();

	// 判断数据库连接IP是否能够Ping通;
    QString strDatabaseIp = WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().databaseRemoteIp;
	ParseUrl objParseUrl;
	std::string strIp = strDatabaseIp.toStdString();
	const char *szDestIP = strIp.c_str();
	PingReply reply;
	bool isPingSuccess = objParseUrl.Ping(szDestIP, &reply);
	bool isConnectSuccess = false;
	if (isPingSuccess)
	{
		// ping通之后再连接数据库;
        isConnectSuccess = WHEEL_ROBOT_DB.openDb(WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().databaseRemoteIp,
						WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().databaseName,
						WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().databaseUsername,
						WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().databasePassword,
						WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().databasePort);
	}
	if (!isPingSuccess || !isConnectSuccess)
	{
		// ping不通或者数据库连接失败;
        DLMessageBox* messageBox = new DLMessageBox();
        messageBox->setFixedWidth(200);
        messageBox->setMessageContent("数据库连接失败");
        messageBox->setWindowModality(Qt::ApplicationModal);
        messageBox->show();
        return;
	}
	// 根据当前登录用户权限进行设置;
    userAuthoritySet();

    initCameraObject();
    initClearOutDateFile();
    // 设置通信状态Ip;
    m_pRobotStateShow->setWirelessBaseStationIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().wirelessStationIp, 443);
    m_pRobotStateShow->setControlSystemIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().controlSystemIp, 443);
    m_pRobotStateShow->setVisibleCameraIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().visibleCameraIp, 443);
    m_pRobotStateShow->setChargeSystemIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().chargeSystemIp, 443);
	m_pRobotStateShow->setInfraredCameraSIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().infraredCameraIp, 443);
	m_pRobotStateShow->setFireRobotIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().firRobotIp, 443);

#if TTUI
    int currentScreenIndex = 0;
#else
    int currentScreenIndex = m_loginWindow->getCurrentScreenChoosedIndex();
    m_loginWindow->close();
#endif
    //this->showMaximized();
	if (qApp->screens().count() > 1 && currentScreenIndex <= qApp->screens().count())
	{
		//this->windowHandle()->setScreen(qApp->screens().at(currentScreenIndex));
	}

	this->showFullScreen();

	m_pBottomWgt->setScreenWidth(this->width());
}

void DLWheelMainWindow::closeEvent(QCloseEvent *event)
{
    WHEEL_BACK_TO_CORE_SOCKET.closeConnect();
//    TempChoose::releaseInfrared();

    TerminateProcess(GetCurrentProcess(), NULL);
    return __super::closeEvent(event);
}


void DLWheelMainWindow::clearOutDateImageAndVideoFiles()
{
    QProcess cmd;
    // 红外抓拍图片路径;
    QString strInfraredImageFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/InfraredCapture";
    strInfraredImageFilePath.replace("/", "\\");

    QString strVisibleLightImageFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/VisibleLightCapture";
    strVisibleLightImageFilePath.replace("/", "\\");

    QString imageCmdstr = QString("forfiles /p \"%1\" /s /m *.* /d -%2 /c \"cmd /c del @path\"")
        .arg(strInfraredImageFilePath).arg(m_imageSaveMaxDays);
    
    cmd.start(imageCmdstr);
    cmd.waitForFinished(1000);

    imageCmdstr = QString("forfiles /p \"%1\" /s /m *.* /d -%2 /c \"cmd /c del @path\"")
        .arg(strVisibleLightImageFilePath).arg(m_imageSaveMaxDays);
    cmd.start(imageCmdstr);
    cmd.waitForFinished(1000);

    QString strInfraredfileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/InfraredVideo";
    strInfraredfileName.replace("/", "\\");

    QString strRecordFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/VisibleLightVideo";
    strRecordFilePath.replace("/", "\\");

    QString videoCmdstr = QString("forfiles /p \"%1\" /s /m *.* /d -%2 /c \"cmd /c del @path\"")
        .arg(strInfraredfileName).arg(m_videoSaveMaxDays);

    cmd.start(videoCmdstr);
    cmd.waitForFinished(1000);

    videoCmdstr = QString("forfiles /p \"%1\" /s /m *.* /d -%2 /c \"cmd /c del @path\"")
        .arg(strRecordFilePath).arg(m_videoSaveMaxDays);

    cmd.start(videoCmdstr);
    cmd.waitForFinished(1000);
}

void DLWheelMainWindow::userAuthoritySet()
{
    if (m_currentLoginRole == WHEEL_USER_MANAGER || m_currentLoginRole == WHEEL_USER_SUPER_MANAGER)
    {
        m_deviceAlarmSearchAffirm->isAdminLogin(true);
    }
    
    m_pSystemGuideWgt->updateSystemMenu(m_currentLoginRole);
}

void DLWheelMainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->modifiers() & Qt::AltModifier)
    {
        if (event->key() == Qt::Key_1)//自定义任务
        {
            onSystemMenuItemClicked(Menu_CustomTask);
        }
        if (event->key() == Qt::Key_2)//实时监控
        {
            onSystemMenuItemClicked(Menu_RobotControl);
        }
        if (event->key() == Qt::Key_3)//巡检结果浏览
        {
            onSystemMenuItemClicked(Menu_PatrolResultBrowse);
        }
        if (event->key() == Qt::Key_Z)//标准点位库导入
        {
            onSystemMenuItemClicked(Menu_PatrolPointPosSet);
        }
        if (event->key() == Qt::Key_Space)//首页
        {
            onSystemMenuItemClicked(Menu_PatrolManage);
        }
    }
}