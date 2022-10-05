#include <QDesktopWidget>
#include <QApplication>
#include <QToolButton>
#include <QDebug>
#include "DLWheelMainWindowForDevelop.h"
#include "LibDLHangRailCommonWidget/CameraObject.h"
#include "LibDLHangRailCommonWidget/ConfigWindow.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLHangRailCommonWidget/CustomButton.h"
#include "LibDLHangRailCommonWidget/VideoBackWidget.h"
#include "LibDLHangRailCommonWidget/LoginWindow.h"
#include "LibDLWheelCollectControlWidget/LibDLWheelCollectControlWidget.h"
#include "LibDLWheelCollectMapWidget/LibDLWheelCollectMapWidget.h"
#include "LibDLHangUserControl/DLHangUserControl.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"
#include "LibDLWheelRobotStateShow/DLWheelRobotStateShow.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include "LibDLWheelPowerStationEdit/DLWheelPowerStationEdit.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "LibDLWheelCustomWidget/ParseUrl.h"
#include "LibProtoClient/ProtoClient.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
//#include "LibDLInfraredTemperature/TempChoose.h"
#include <QGuiApplication>
#include "DLWheelVirtualDeviceWidget.h"
#include <QWindow>
#include "LibDLHangRailCommonTools/ClearOutDateFileObject.h"
#include "LibDLHangRailCommonWidget/ConfidenceValueShowWidget.h"
#include "LibHCNetCamera/HikCameraPointData.h"
#include "WheelTaskManageWidget/WheelTaskManageWidget.h"
#include "LibDLWheelPatrolResultBrowse/DLWheelPatrolResultBrowse.h"
#include "LibDLWheelPatrolResultBrowse/DLWheelStandardPointPosMaintain.h"
#include "LibDLWheelPatrolResultBrowse/DLWheelPatrolPointPosSet.h"



#define TITLE_HEIGHT 70				// 标题栏高度;
#define LEFTWIDGET_WIDTH 150		// 左侧widget宽度;
#define APP_NAME "WheelRobotForDevelop"

#pragma execution_character_set("utf-8") 

#define UPDATE_TIME (600)

DLHangWheelMainWindowFroDevelop::DLHangWheelMainWindowFroDevelop(QWidget *parent)
	: QWidget(parent)
	, m_welcomeWindow(NULL)
    , m_operateMessageFlickerCount(0)
	,m_bIsPressed(false)
	,m_ptStartPos(0,0)
	, m_iConfidenceValue(0)
	, m_bIsChargeState(true)
	, m_fCurvoltage(0.0)
	,m_pBatteryLevelLbl(NULL)
{
	qRegisterMetaType<RootNodeType>("RootNodeType");
	Q_INIT_RESOURCE(resource);



	m_loginWindow = new LoginWindow;
	m_loginWindow->setTitleText("南京南智能巡检\n机器人信息采集软件");
	connect(m_loginWindow, &LoginWindow::signalLogin, this, &DLHangWheelMainWindowFroDevelop::onLogin);
	connect(m_loginWindow, &LoginWindow::signalQuit, this, &DLHangWheelMainWindowFroDevelop::onQuit);

	m_pUpdateConfidenceValueTimer = new QTimer(this);
	m_pUpdateConfidenceValueTimer->setInterval(UPDATE_TIME);
	connect(m_pUpdateConfidenceValueTimer, SIGNAL(timeout()), this, SLOT(UpdateConfidenceValueSlot()));
	m_loginWindow->show();

	// 设置样式;
	QString strStyleSheet = this->styleSheet();
	QString strFilePath = QString("%1/%2").arg(qApp->applicationDirPath()).arg("style/AppDLWheelRobotForDeveloper/common/main.css");
	QFile file(strFilePath);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		strStyleSheet += file.readAll();
		file.close();
	}

	this->setStyleSheet(strStyleSheet);
}

DLHangWheelMainWindowFroDevelop::~DLHangWheelMainWindowFroDevelop()
{
    
}

void DLHangWheelMainWindowFroDevelop::onLogin(QByteArray userName, QByteArray password, QString strIp, QString strPort)
{
	QString UserName = QByteArray::fromBase64(userName);
	QString Password = QString(password);
	userLoginRetVal ret = WHEEL_BACK_TO_CORE_SOCKET.doLogin(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().coreServerIp.toStdString(), WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().coreServerPort,
		UserName.toStdString(), Password.toStdString());
	// 登录成功;
	if (ret.retCode == WheelLoginRetCode::WHEEL_LOGIN_SUCCESS)
	{
        m_currentScreenChoosedIndex = m_loginWindow->getCurrentScreenChoosedIndex();
		m_loginWindow->close();
		WHEEL_ROBOT_BACKGROUND_CONFIG.initCoreCfg(ret.coreCfg);
        WHEEL_ROBOT_BACKGROUND_CONFIG.initCoreRobotCfg(ret.robotCfg);
        GFILE_TRANSFER_CLIENT.init(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().coreServerIp.toStdString(), WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().rcfServerPort);
		loginSuccess();
		m_pUpdateConfidenceValueTimer->start();
	}
	else
	{
		m_loginWindow->loginFailed();
	}
}

void DLHangWheelMainWindowFroDevelop::onQuit()
{
	m_loginWindow->close();
	this->close();
}

void DLHangWheelMainWindowFroDevelop::loginSuccess()
{
	m_welcomeWindow = new WelcomeWindow();
	m_welcomeWindow->show();

	QTimer::singleShot(100, this, [=] {
		// 判断数据库连接IP是否能够Ping通;
		//QString strDatabaseIp = WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().databaseIp;
		ParseUrl objParseUrl;
		std::string strIp = WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().databaseRemoteIp.toStdString();
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
			DLMessageBox::showDLMessageBox(m_welcomeWindow, "错误", "数据库连接失败", MessageButtonType::BUTTON_OK, true);
			m_welcomeWindow->close();
			this->close();
		}
		// 初始化程序界面;
		initWindow();
        initOperateMsgWidget();
		initTitleWidget();
		initLeftButton();
		initStackedWidget();
		initConnections();
		initCameraObject();
		initCoreSignals();
        initTransferFunction();
        initOperateMessageFlickerTimer();
        initClearOutDateFile();
		WHEEL_DEVICE_CONFIG;
		WHEEL_ROBOT_DB;
		HikPointData::getInitance()->init();
		this->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowMinimizeButtonHint);

		// 关闭启动界面，打开主界面;
		m_welcomeWindow->close();

        this->show();
        if (qApp->screens().count() > 1 && m_currentScreenChoosedIndex <= qApp->screens().count())
        {
            this->windowHandle()->setScreen(qApp->screens().at(m_currentScreenChoosedIndex));
        }

        this->showFullScreen();
	});
}

void DLHangWheelMainWindowFroDevelop::initWindow()
{
	m_iconBackWidget = new QWidget(this);
	m_iconBackWidget->setObjectName("iconBackWidget");
	m_iconBackWidget->setFixedSize(QSize(LEFTWIDGET_WIDTH, TITLE_HEIGHT));

	m_iconBackMaskWidget = new QWidget(m_iconBackWidget);
	m_iconBackMaskWidget->setObjectName("iconBackMaskWidget");
	m_iconLabel = new QLabel;
	m_iconLabel->setObjectName("iconLabel");
	m_iconLabel->setFixedSize(QSize(64, 64));

	// 左侧按钮背景widget;
	m_leftBackWidget = new QWidget(this);
	m_leftBackWidget->setObjectName("leftBackWidget");
	m_leftBackWidget->setFixedWidth(LEFTWIDGET_WIDTH);

	// 标题栏背景widget;
	m_titleBackWidget = new QWidget(this);
	m_titleBackWidget->setObjectName("titleBackWidget");
	m_titleBackWidget->setFixedHeight(TITLE_HEIGHT);

	m_titleBackWidget->installEventFilter(this);

	// 中心背景widget;
	m_centerBackWidget = new QWidget;
	m_centerBackWidget->setObjectName("centerBackWidget");

	// 图标布局;
	QHBoxLayout* hIconMainLayout = new QHBoxLayout(m_iconBackWidget);
	hIconMainLayout->addWidget(m_iconBackMaskWidget);
	hIconMainLayout->setMargin(0);
	hIconMainLayout->setSpacing(0);

	QHBoxLayout* hIconMaskLayout = new QHBoxLayout(m_iconBackMaskWidget);
	hIconMaskLayout->addWidget(m_iconLabel);
	hIconMaskLayout->setMargin(0);
	hIconMaskLayout->setSpacing(0);

	QGridLayout* mainGridLayout = new QGridLayout(this);
	mainGridLayout->addWidget(m_iconBackWidget, 0, 0);
	mainGridLayout->addWidget(m_titleBackWidget, 0, 1);
	mainGridLayout->addWidget(m_leftBackWidget, 1, 0);
	mainGridLayout->addWidget(m_centerBackWidget, 1, 1);
	mainGridLayout->setSpacing(0);
	mainGridLayout->setMargin(0);
}

void DLHangWheelMainWindowFroDevelop::initTitleWidget()
{
	QSize size(18, 18);
	m_confidenceImages[0].load(":/Resources/Common/image/Location-ok.png");
	m_confidenceImages[0] = m_confidenceImages[0].scaled(size, Qt::KeepAspectRatio);

	m_confidenceImages[1].load(":/Resources/Common/image/Location-low.png");
	m_confidenceImages[1] = m_confidenceImages[1].scaled(size, Qt::KeepAspectRatio);

	/*QPixmap image((":/Resources/logo.png"));
	m_iconLabel->setPixmap(image.scaled(m_iconLabel->size()));*/

	// 标题;
	QLabel* titleLabel = new QLabel(m_titleBackWidget);
	titleLabel->setObjectName("titleLabel");
	titleLabel->setText(tr("南京南智能巡检\n机器人信息采集软件"));

    QPushButton* pButtonMinWindow = new QPushButton(m_titleBackWidget);
	pButtonMinWindow->setObjectName("minWindowBtn");
    pButtonMinWindow->setIcon(QIcon(":/Resources/minWindow.png"));
    pButtonMinWindow->setIconSize(QSize(20, 20));
   // pButtonMinWindow->setStyleSheet("");
    connect(pButtonMinWindow, &QPushButton::clicked, this, &DLHangWheelMainWindowFroDevelop::showMinimized);

	// 关闭按钮;
	m_pButtonClose = new QPushButton(m_titleBackWidget);
	m_pButtonClose->setObjectName("closeButton");
	m_pButtonClose->setIcon(QIcon(":/Resources/close.png"));
	m_pButtonClose->setIconSize(QSize(20, 20));

	m_back_charge_button = new QPushButton;
	m_stop_task_button = new QPushButton;

	m_back_charge_button->setIcon(QIcon(":/Resources/PlayBack.png"));
	m_back_charge_button->setToolTip("终止任务");
	m_stop_task_button->setIcon(QIcon(":/Resources/RobotGoBack.png"));
	m_stop_task_button->setToolTip("一键返航");
	connect(m_back_charge_button, &QPushButton::clicked, this, [=]() {
		int result = DLMessageBox::showDLMessageBox(this, "提示", "是否进行一键返航", MessageButtonType::BUTTON_OK_AND_CANCEL, true, QSize(250, 150));
		if (result == QDialog::Accepted)
		{
			WHEEL_BACK_TO_CORE_SOCKET.robot_task_cancel_req();
			QTimer::singleShot(1000, this, [=] {
				WHEEL_BACK_TO_CORE_SOCKET.robot_control_back_to_charge();
			});
		}
	});
	connect(m_stop_task_button, &QPushButton::clicked, this, [=]() {
		WHEEL_BACK_TO_CORE_SOCKET.robot_task_cancel_req();
	});

	m_currentRobotWidget = new InputWidget(WheelComboBox);
	m_currentRobotWidget->setTipText(QString("当前机器人"));
	QStringList content;
	for (int i = 0; i < WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg().size(); ++i)
	{
		QString robot_name = WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].robotName + QString("号机器人");
		content << robot_name;
	}

	if (!WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg().empty()) {
		WHEEL_BACK_TO_CORE_SOCKET.switch_robot_req(WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg().front().robotName.toInt());
	}

	m_currentRobotWidget->setComboBoxContent(content);
	connect(m_currentRobotWidget, &InputWidget::signalComboBoxIndexChanged, this, [=](int id) {
		//	切换机器人
		WHEEL_BACK_TO_CORE_SOCKET.switch_robot_req(WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].robotName.toInt());
		//	切换机器人摄像头
		m_visibleLightCameraObject->resetCamera(WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].hcIP,
			WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].hcCtrlPort,
			WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].hcUserName,
			WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].hcPassword,
			WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].hcRtspPort);
		m_infraredCameraObject->resetCamera(WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].infraredCameraIp,
			WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].infraredCtrlPort,
			WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].hcUserName,
			WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].hcPassword,
			WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[id].infraredRtspPort);
	});

	m_pStutterStopBtn = new QPushButton(m_titleBackWidget);
	connect(m_pStutterStopBtn, SIGNAL(clicked()), this, SLOT(StutterStopBtnSlot()));
	m_pStutterStopBtn->setToolTip("急停");
	m_pStutterStopBtn->setFixedSize(40, 40);
	m_pStutterStopBtn->setObjectName("stutterStopBtn");
	m_pStutterStopBtn->setCheckable(true);

	QVBoxLayout *pIconLayout = new QVBoxLayout();
	pIconLayout->setSpacing(1);
	pIconLayout->setMargin(0);

	m_pConfidenValueLbl = new QLabel(m_titleBackWidget);			//置信度图标
	m_pConfidenValueLbl->setObjectName("confidenValueLbl");
	m_pConfidenValueLbl->setFixedSize(18, 18);
	m_pConfidenValueLbl->setPixmap(m_confidenceImages[0]);

	QLabel *pLbl = new QLabel(m_titleBackWidget);
	pLbl->setFixedWidth(9);
	QHBoxLayout *pConfidenValueLblLayot = new QHBoxLayout;
	pConfidenValueLblLayot->setMargin(0);
	pConfidenValueLblLayot->setSpacing(1);

	pConfidenValueLblLayot->addWidget(pLbl);
	pConfidenValueLblLayot->addWidget(m_pConfidenValueLbl);
	//pIconLayout->addWidget(m_pConfidenValueLbl);
	pIconLayout->addLayout(pConfidenValueLblLayot);

	m_pBatteryLevelLbl = new QLabel(m_titleBackWidget);			//电池电量
	m_pBatteryLevelLbl->setObjectName("batteryLevelLbl");
	m_pBatteryLevelLbl->setFixedSize(18, 18);
	//pIconLayout->addWidget(pBatteryLevelLbl);


	QVBoxLayout *pChargeLayout = new QVBoxLayout;
	pChargeLayout->setSpacing(0);
	pChargeLayout->setMargin(0);
	pChargeLayout->addStretch();

	m_pChargeState = new QLabel(m_titleBackWidget);
	m_pChargeState->setFixedSize(9,9);
	pChargeLayout->addWidget(m_pChargeState);

	QWidget *pBatteryWgt = new QWidget(m_titleBackWidget);
	pBatteryWgt->setFixedHeight(18);

	QHBoxLayout *pBatteryLayout = new QHBoxLayout(pBatteryWgt);
	pBatteryLayout->setMargin(0);
	pBatteryLayout->setSpacing(1);

	pBatteryLayout->addLayout(pChargeLayout);
	pBatteryLayout->addWidget(m_pBatteryLevelLbl);

	//pIconLayout->addLayout(pBatteryLayout);

	pIconLayout->addWidget(pBatteryWgt);

    m_pConfidenceValueShowWgt = new ConfidenceValueShowWidget;	//置信度显示
	m_pBatteryLevelWgt = new ConfidenceValueShowWidget(this);		//电池电量

	m_pBatteryLevelWgt->setCurrentValue(0.0);

	QVBoxLayout *pShowValueLayout = new QVBoxLayout();  //显示值
	pShowValueLayout->setSpacing(1);
	pShowValueLayout->setMargin(0);
	pShowValueLayout->addWidget(m_pConfidenceValueShowWgt);
	pShowValueLayout->addWidget(m_pBatteryLevelWgt);

	QHBoxLayout *pIcon2ValueLayout = new QHBoxLayout();
	pIcon2ValueLayout->setSpacing(3);
	pIcon2ValueLayout->addLayout(pIconLayout);
	pIcon2ValueLayout->addLayout(pShowValueLayout);

	QHBoxLayout* hLayout = new QHBoxLayout(m_titleBackWidget);
	hLayout->addWidget(titleLabel);
	hLayout->addSpacerItem(new QSpacerItem(65, 20, QSizePolicy::Fixed, QSizePolicy::Fixed));
    hLayout->addWidget(m_operateMessageBackWidget);
    hLayout->addStretch();
	hLayout->addWidget(m_currentRobotWidget);
	hLayout->addWidget(m_back_charge_button);
	hLayout->addWidget(m_stop_task_button);
	hLayout->addWidget(m_pStutterStopBtn);
	hLayout->addLayout(pIcon2ValueLayout);
    //hLayout->addLayout(pShowValueLayout);
    hLayout->addSpacing(20);
    hLayout->addWidget(pButtonMinWindow);
	hLayout->addWidget(m_pButtonClose);
	hLayout->setSpacing(30);
	hLayout->setMargin(0);
	hLayout->setContentsMargins(25, 0, 20, 0);

	connect(m_pButtonClose, SIGNAL(clicked()), this, SLOT(close()));
}

void DLHangWheelMainWindowFroDevelop::initCameraObject()
{
	m_visibleLightCameraObject = new CameraObject(this, true);
    m_visibleLightCameraObject->setNeedRestartCamera();
	// 这里要设置相机的连接配置;
    m_visibleLightCameraObject->setCameraConnectInfo(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraIp,
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraPort.toInt(),
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraUser,
        WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraPassword);
	m_visibleLightCameraObject->startPlay();
 	m_collectControlWidget->setCameraObject(m_visibleLightCameraObject);

	// 红外视频流;
    m_infraredCameraObject = new CameraObject(this, true);
    m_infraredCameraObject->setInfraredVideo();
    m_infraredCameraObject->setNeedRestartCamera();
	// 这里要设置相机的连接配置;
    m_infraredCameraObject->setCameraConnectInfo(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().infraredCameraIp,
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraPort.toInt(),
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraUser,
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraPassword);

    m_infraredCameraObject->startPlay();    

 	m_collectControlWidget->setInfraredObject(NULL, m_infraredCameraObject);
}

void DLHangWheelMainWindowFroDevelop::initConnections()
{

}

void DLHangWheelMainWindowFroDevelop::initOperateMessageFlickerTimer()
{
    m_operateMessageFlickerTimer.setInterval(500);

    connect(&m_operateMessageFlickerTimer, &QTimer::timeout, this, [=] {
        m_operateMessageFlickerCount++;
        if (m_operateMessageFlickerCount > 8)
        {
            m_operateMessageFlickerCount = 0;
            m_operateMessageFlickerTimer.stop();
            //m_operateMessageLabel->setStyleSheet("color:red;");
        }
        else
        {
            if (m_operateMessageFlickerCount % 2 == 0)
            {
               // m_operateMessageLabel->setStyleSheet("color:red;");
            }
            else
            {
              //  m_operateMessageLabel->setStyleSheet("color:black;");
            }
        }
    });


}

void DLHangWheelMainWindowFroDevelop::initClearOutDateFile()
{
    m_clearOutDateFileObject = new ClearOutDateFileObject(this);
    // 清除log文件夹下文件;
    // 注意这里的 APP_NAME 与main里面的 APP_NAME 保持一致;
    QString strFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/log/" + APP_NAME;
    m_clearOutDateFileObject->setClearFileList(QStringList() << strFilePath);
    m_clearOutDateFileObject->startClear();
}

void DLHangWheelMainWindowFroDevelop::initLeftButton()
{
	QButtonGroup* leftButtonGroup = new QButtonGroup(this);

	QPushButton* pButtonCollectControl = new QPushButton(m_leftBackWidget);
	pButtonCollectControl->setText(tr("采集控制"));
	pButtonCollectControl->setFixedHeight(80);
	pButtonCollectControl->setObjectName("LeftButton");
	pButtonCollectControl->setCheckable(true);
	leftButtonGroup->addButton(pButtonCollectControl, CollectControlWidget);

	QPushButton* pButtonUserControl = new QPushButton(m_leftBackWidget);
	pButtonUserControl->setText(tr("用户管理"));
	pButtonUserControl->setFixedHeight(80);
	pButtonUserControl->setObjectName("LeftButton");
	pButtonUserControl->setCheckable(true);
	leftButtonGroup->addButton(pButtonUserControl, UserControlWidget);

	QPushButton* pButtonTaskManage = new QPushButton(m_leftBackWidget);
	pButtonTaskManage->setText(tr("任务管理"));
	pButtonTaskManage->setFixedHeight(80);
	pButtonTaskManage->setObjectName("LeftButton");
	pButtonTaskManage->setCheckable(true);
	leftButtonGroup->addButton(pButtonTaskManage, CustomTaskManageWidget);

	// 巡检结果浏览;
	QPushButton* pButtonTaskViewManage = new QPushButton(m_leftBackWidget);
	pButtonTaskViewManage->setText(tr("结果浏览"));
	pButtonTaskViewManage->setFixedHeight(80);
	pButtonTaskViewManage->setObjectName("LeftButton");
	pButtonTaskViewManage->setCheckable(true);
	leftButtonGroup->addButton(pButtonTaskViewManage, Robot_PatrolResultBrowse);

// 	QPushButton* pButtonCollectMap = new QPushButton(m_leftBackWidget);
// 	pButtonCollectMap->setText(tr("采集地图"));
// 	pButtonCollectMap->setFixedHeight(80);
// 	pButtonCollectMap->setObjectName("LeftButton");
// 	pButtonCollectMap->setCheckable(true);
	//leftButtonGroup->addButton(pButtonCollectMap, CollectMapWidget);

	QPushButton* pButtonRobotStatus = new QPushButton(m_leftBackWidget);
	pButtonRobotStatus->setText(tr("机器人状态"));
	pButtonRobotStatus->setFixedHeight(80);
	pButtonRobotStatus->setObjectName("LeftButton");
	pButtonRobotStatus->setCheckable(true);
	leftButtonGroup->addButton(pButtonRobotStatus, RobotStatus);

	QPushButton* pButtonStationEdit = new QPushButton(m_leftBackWidget);
	pButtonStationEdit->setText(tr("电站编辑"));
	pButtonStationEdit->setFixedHeight(80);
	pButtonStationEdit->setObjectName("LeftButton");
	pButtonStationEdit->setCheckable(true);
	leftButtonGroup->addButton(pButtonStationEdit, PowerStationEdit);

	QPushButton* pButtonPatrolProEdit = new QPushButton(m_leftBackWidget);
	pButtonPatrolProEdit->setText(tr("巡检点位维护"));
	pButtonPatrolProEdit->setFixedHeight(80);
	pButtonPatrolProEdit->setObjectName("LeftButton");
	pButtonPatrolProEdit->setCheckable(true);
	leftButtonGroup->addButton(pButtonPatrolProEdit, PatrolProEdit);

	//QPushButton* pButtonPatrolPosEdit = new QPushButton(m_leftBackWidget);
	//pButtonPatrolPosEdit->setText(tr("巡检点位设置"));
	//pButtonPatrolPosEdit->setFixedHeight(80);
	//pButtonPatrolPosEdit->setObjectName("LeftButton");
	//pButtonPatrolPosEdit->setCheckable(true);
	//leftButtonGroup->addButton(pButtonPatrolPosEdit, PatrolPosSet);

    QPushButton* pButtonVirtualDevice = new QPushButton(m_leftBackWidget);
    pButtonVirtualDevice->setText(tr("其他设置"));
    pButtonVirtualDevice->setFixedHeight(80);
    pButtonVirtualDevice->setObjectName("LeftButton");
    pButtonVirtualDevice->setCheckable(true);
    leftButtonGroup->addButton(pButtonVirtualDevice, VirtualDeviceWidget);

	connect(leftButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(onLeftButtonClicked(int)));

	QVBoxLayout* leftButtonLayout = new QVBoxLayout(m_leftBackWidget);
	leftButtonLayout->addWidget(pButtonCollectControl);
	leftButtonLayout->addWidget(pButtonUserControl);
	leftButtonLayout->addWidget(pButtonTaskManage);
	leftButtonLayout->addWidget(pButtonTaskViewManage);
	//leftButtonLayout->addWidget(pButtonCollectMap);
	leftButtonLayout->addWidget(pButtonRobotStatus);
	leftButtonLayout->addWidget(pButtonStationEdit);
	leftButtonLayout->addWidget(pButtonPatrolProEdit);
	//leftButtonLayout->addWidget(pButtonPatrolPosEdit);
    leftButtonLayout->addWidget(pButtonVirtualDevice);
	leftButtonLayout->addStretch();
	leftButtonLayout->setSpacing(30);
	leftButtonLayout->setMargin(0);
	leftButtonLayout->setContentsMargins(0, 30, 0, 0);

	pButtonCollectControl->setChecked(true);
	//QString strStyleSheet = this->styleSheet();
	//strStyleSheet.append("");

	//this->setStyleSheet(strStyleSheet);

	QDesktopWidget* desktopWidget = QApplication::desktop();
	QRect screenRect = desktopWidget->screenGeometry();
	this->setFixedSize(screenRect.size());
}

void DLHangWheelMainWindowFroDevelop::initStackedWidget()
{
	m_stackedWidget = new QStackedWidget(m_centerBackWidget);
	m_collectControlWidget = new DLHangWheelCollectControlWidget(m_stackedWidget);
	m_collectControlWidget->installEventFilter(this);
	this->installEventFilter(this);
	m_userControlWidget = new DLHangUserControl;

	m_robotTaskManageCustomTaskWidget = new DLWheelTaskManage(Menu_CustomTask);
	m_robotTaskManageCustomTaskWidget->initWidget();

	m_patrolResultBrowse = new DLWheelPatrolResultBrowse;
	m_patrolResultBrowse->initWidget();

	m_standardPointPosMaintain = new DLWheelStandardPointPosMaintain;
	m_standardPointPosMaintain->initWidget();

	//m_patrolPointPosSet = new DLWheelPatrolPointPosSet;
	//m_patrolPointPosSet->initWidget();
	
	//m_dLWheelCollectMapWidget = new DLWheelCollectMapWidget;

	m_robotStatusWidget = new DLWheelRobotStateShow;
	m_robotStatusWidget->setDevelopMode();
    m_robotStatusWidget->setIsShowUploadBinWidget(false);
    m_virtualDeviceWidget = new DLWheelVirtualDeviceWidget;
	m_powerStationEdit = new DLWheelPowerStationEdit;
    // 设置通信状态Ip;
    m_robotStatusWidget->setWirelessBaseStationIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().wirelessStationIp, 443);
    m_robotStatusWidget->setControlSystemIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().controlSystemIp, 443);
    m_robotStatusWidget->setVisibleCameraIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().visibleCameraIp, 443);
    m_robotStatusWidget->setChargeSystemIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().chargeSystemIp, 443);
    m_robotStatusWidget->setInfraredCameraSIP(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().infraredCameraIp, 443);

	// 通知机器人状态更新可见光相机参数;
	connect(m_collectControlWidget, &DLHangWheelCollectControlWidget::signalUpdateVisibleVideoParam, m_robotStatusWidget, &DLWheelRobotStateShow::onUpdateVisibleVideoParam);
    //connect(m_collectControlWidget, &DLHangWheelCollectControlWidget::signalCollectPatrolPoint, m_dLWheelCollectMapWidget, &DLWheelCollectMapWidget::signalSendCollectPatrolPoint);
	//connect(this, &DLHangWheelMainWindowFroDevelop::signalDeleteDeviceResult, m_collectControlWidget, &DLHangWheelCollectControlWidget::onDeleteDeviceResult);
	//connect(m_collectControlWidget, &DLHangWheelCollectControlWidget::signalOpenSmap, m_dLWheelCollectMapWidget, &DLWheelCollectMapWidget::signalSendOpenSmap);
    connect(m_collectControlWidget, &DLHangWheelCollectControlWidget::signalSendOperateMsg, this, &DLHangWheelMainWindowFroDevelop::onGetOperateMsg);
    connect(this, &DLHangWheelMainWindowFroDevelop::signalRefreshTreeWidget, m_collectControlWidget, &DLHangWheelCollectControlWidget::onRefreshTreeWodget);
    connect(m_virtualDeviceWidget, &DLWheelVirtualDeviceWidget::signalSendOperateMsg, this, &DLHangWheelMainWindowFroDevelop::onGetOperateMsg);
	
    // 控制可见光 / 红外显示开关;
    connect(m_robotStatusWidget, &DLWheelRobotStateShow::signalVisibleSwitch, this, [=](bool isOpen) {
        m_visibleLightCameraObject->setIsPausePlay(!isOpen);
    });

    connect(m_robotStatusWidget, &DLWheelRobotStateShow::signalInfraredSwitch, this, [=](bool isOpen) {
        m_infraredCameraObject->setIsPausePlay(!isOpen);
    });

	//m_stackedWidget = new QStackedWidget(m_centerBackWidget);

	QHBoxLayout* centerLayout = new QHBoxLayout(m_centerBackWidget);
	centerLayout->addWidget(m_stackedWidget);
	centerLayout->setMargin(0);
	centerLayout->setSpacing(0);

	m_stackedWidget->insertWidget(CollectControlWidget, m_collectControlWidget);
	m_stackedWidget->insertWidget(UserControlWidget, m_userControlWidget); 
	m_stackedWidget->insertWidget(CustomTaskManageWidget, m_robotTaskManageCustomTaskWidget);
	//m_stackedWidget->insertWidget(CollectMapWidget, m_dLWheelCollectMapWidget);
	m_stackedWidget->insertWidget(Robot_PatrolResultBrowse, m_patrolResultBrowse);
	m_stackedWidget->insertWidget(RobotStatus, m_robotStatusWidget);
	m_stackedWidget->insertWidget(PatrolProEdit, m_standardPointPosMaintain); 
	//m_stackedWidget->insertWidget(PatrolPosSet, m_patrolPointPosSet);
	m_stackedWidget->insertWidget(PowerStationEdit, m_powerStationEdit);
    m_stackedWidget->insertWidget(VirtualDeviceWidget, m_virtualDeviceWidget);
	m_stackedWidget->setCurrentIndex(0);
}

void DLHangWheelMainWindowFroDevelop::initOperateMsgWidget()
{
    m_operateMsgListBackWidget = new BaseWidget(this, BaseWidgetType::PopupWindow);
    m_operateMsgListBackWidget->setTitleContent("操作信息");
    m_operateMsgListBackWidget->setFixedSize(QSize(850, 250));

    m_operateMsgListWidget = new QListWidget(m_operateMsgListBackWidget->getCenterWidget());
    QHBoxLayout* hErrorMsgListLayout = new QHBoxLayout(m_operateMsgListBackWidget->getCenterWidget());
    hErrorMsgListLayout->addWidget(m_operateMsgListWidget);
    hErrorMsgListLayout->setMargin(10);

    m_operateMsgListBackWidget->setShowCloseButton();
    connect(m_operateMsgListBackWidget, &BaseWidget::signalCloseButtonClicked, this, [=] {
        m_operateMsgListBackWidget->setVisible(false);
    });

    m_operateMsgListBackWidget->setVisible(false);

    m_operateMessageBackWidget = new QWidget(m_titleBackWidget);
    m_operateMessageBackWidget->setObjectName("ModeChangeBackWidget");
    m_operateMessageBackWidget->setFixedSize(QSize(850, 40));
  //  m_operateMessageBackWidget->setStyleSheet(".");

    m_operateMessageLabel = new QLabel(m_operateMessageBackWidget);
	m_operateMessageLabel->setObjectName("messageLbl");
    m_operateMessageLabel->setScaledContents(true);
	m_operateMessageLabel->setFixedWidth(843);
	//m_operateMessageLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
   // m_operateMessageLabel->setStyleSheet("");

    QHBoxLayout* hLayout = new QHBoxLayout(m_operateMessageBackWidget);
    hLayout->addWidget(m_operateMessageLabel);
    QToolButton* pButtonPop = new QToolButton(m_operateMessageBackWidget);
	pButtonPop->setObjectName("popBtn");
    pButtonPop->setFixedSize(QSize(30, 30));
    pButtonPop->setIcon(QIcon(":/Resources/Common/image/PopButton.png"));
    pButtonPop->setIconSize(QSize(20, 20));
    //pButtonPop->setStyleSheet("");
    connect(pButtonPop, &QToolButton::clicked, this, [=] {
        if (m_operateMsgListBackWidget->isVisible())
        {
            m_operateMsgListBackWidget->setVisible(false);
        }
        else
        {
            QPoint pos = m_operateMessageBackWidget->mapToGlobal(QPoint(0, 0));
            m_operateMsgListBackWidget->setVisible(true);
            m_operateMsgListBackWidget->move(pos.x(), pos.y() + 62);
            m_operateMsgListBackWidget->raise();
        }
    });

    hLayout->addStretch();
    hLayout->addWidget(pButtonPop);
    hLayout->setContentsMargins(5, 0, 5, 0);
}

void DLHangWheelMainWindowFroDevelop::initCoreSignals()
{
	// 更新任务展示中table数据删除结果;
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotTaskEditDeleteStatus.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::signalUpdateDeleteResult, this, _1, _2, _3));
	// 任务管理中保存任务编制返回结果;
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotTaskEditInsertStatus.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::signalTaskManageSaveRsp, this, _1, _2, _3));
	//// 任务管理中更新任务编制返回结果;
	//WHEEL_BACK_TO_CORE_SOCKET.wheelRobotTaskEditUpdataStatus.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::signalTaskManageUpdateRsp, this, _1, _2, _3));


	// 删除设备结果返回函数绑定;
	//WHEEL_BACK_TO_CORE_SOCKET.wheelRobotDeviceDeleteStatus.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::signalDeleteDeviceResult, this, _1, _2));
    // 插入设备结果返回函数绑定;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotDeviceInsertStatus.connect(boost::bind(&DLHangWheelCollectControlWidget::onAddDeviceResult, m_collectControlWidget, _1, _2, _3));
    // 更新设备结果返回函数绑定;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotDeviceUpdateStatus.connect(boost::bind(&DLHangWheelCollectControlWidget::onUpdateDeviceResult, m_collectControlWidget, _1, _2, _3));

    // 实时更新状态;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotRealtimeStatus.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::signalUpdateRobotRealtimeStatus, this, _1));

    // 更新非实时状态;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotNoneRealtimeStatus.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::signalUpdateRobotNoneRealtimeStatus, this, _1));

    // 电站编辑插入结果;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotInsertAreaStatus.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::signalStationEditInsertResult, this, _1, _2));

    // 电站编辑删除结果;
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotDeleteAreaStatus.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::signalStationEditDeleteResult, this, _1, _2));
	
	//更新设备树的信息
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotCollectDeviceTreeSignal.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::UpdateEquipmentTreeSignal, this, _1, _2, _3));

	//下载图片信号
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotInfraredTakePhotoSignal.connect(boost::bind(&DLHangWheelMainWindowFroDevelop::DownloadImageSignal, this, _1, _2, _3));
}

void DLHangWheelMainWindowFroDevelop::initTransferFunction()
{
    // Core通知电站编辑插入结果;
    connect(this, &DLHangWheelMainWindowFroDevelop::signalStationEditInsertResult, [=](bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            m_powerStationEdit->onRefreshListWidget();
            onGetOperateMsg("电站编辑添加成功");
        }
        else
        {
            onGetOperateMsg("电站编辑添加失败,错误码: " + strMsg);
        }
    });

    // Core通知电站编辑删除结果;
    connect(this, &DLHangWheelMainWindowFroDevelop::signalStationEditDeleteResult, [=](bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            m_powerStationEdit->onRefreshListWidget();
            onGetOperateMsg("电站编辑删除成功");
            emit signalRefreshTreeWidget();
        }
        else
        {
            onGetOperateMsg("电站编辑删除失败，错误码: " + strMsg);
        }
    });

    // Core 通知更新实时信息;
    connect(this, &DLHangWheelMainWindowFroDevelop::signalUpdateRobotRealtimeStatus, [=](WheelRobotRealtimeStatus realTimeStatus) {
		//电量信息
	//	m_fCurvoltage = realTimeStatus.batteryStatus.voltage;		//电量
		m_fCurvoltage = realTimeStatus.batteryStatus.battery_level * 100.0;
		m_bIsChargeState = realTimeStatus.batteryStatus.charging;		//是否充电
		//qDebug() << "==============电池电量:" << m_fCurvoltage;


        // 更新置信度;
		m_iConfidenceValue = realTimeStatus.locStatus.confidence * 100;

		if (0 == realTimeStatus.urgency_type)
		{//运行正常
			m_pStutterStopBtn->setChecked(false);
		}
		else
		{//停止
			m_pStutterStopBtn->setChecked(true);
		}


        m_collectControlWidget->onUpdateRobotRealtimeStatus(realTimeStatus);

        m_robotStatusWidget->onUpdateRobotRealtimeStatus(realTimeStatus);
    });

    // Core 通知更新非实时信息;
    connect(this, &DLHangWheelMainWindowFroDevelop::signalUpdateRobotNoneRealtimeStatus, [=](WheelRobotNoneRealtimeStatus noneRealTimeStatus) {
        m_robotStatusWidget->onUpdateRobotNoneRealtimeStatus(noneRealTimeStatus);
    });

	//Core 通知设备树更新节点信息
	connect(this, SIGNAL(UpdateEquipmentTreeSignal(bool, QStringList, RootNodeType)), m_collectControlWidget, SLOT(UpdateEquipmentTreeSlot(bool, QStringList, RootNodeType)));


	//core 通知下载图片信号
    connect(this, SIGNAL(DownloadImageSignal(QString, QString, int)), m_collectControlWidget, SLOT(DownloadImageSlot(QString, QString, int)));

	// 任务管理保存按钮;
	connect(this, SIGNAL(signalUpdateDeleteResult(int, bool, QString)), m_robotTaskManageCustomTaskWidget, SLOT(onUpdateDeleteResult(int, bool, QString)));

	// 任务管理保存按钮;
	connect(this, SIGNAL(signalTaskManageSaveRsp(int, bool, QString)), m_robotTaskManageCustomTaskWidget, SLOT(onUpdateSaveTaskResult(int, bool, QString)));

//	// 任务管理保存按钮操作返回，对任务进行修改;
//	connect(this, &DLHangWheelMainWindowFroDevelop::signalTaskManageUpdateRsp, this, [=](int typeId, bool isSuccess, QString strMsg) {
//		WheelTaskAdminType task_edit_type_id = WheelTaskAdminType(typeId);
//		m_rootTaskManagerWidgetList.at(task_edit_type_id - 1)->onGetSaveOperationResult(isSuccess, strMsg);
//	}, Qt::QueuedConnection);
//
//	// 任务管理保存按钮操作返回，对任务进行修改;
//	connect(this, &DLHangWheelMainWindowFroDevelop::signalDeleteTaskCallBack, this, [=](int typeId, bool isSuccess, QString strMsg) {
//		WheelTaskAdminType task_edit_type_id = WheelTaskAdminType(typeId);
//		m_rootTaskManagerWidgetList.at(task_edit_type_id - 1)->onDeleteTaskCallBack(isSuccess, strMsg);
//	}, Qt::QueuedConnection);
}

void DLHangWheelMainWindowFroDevelop::onLeftButtonClicked(int buttonId)
{
	QWidget* widget = m_stackedWidget->widget(buttonId);
	if (widget == NULL)
	{
		return;
	}
	m_stackedWidget->setCurrentIndex(buttonId);
}

void DLHangWheelMainWindowFroDevelop::onGetOperateMsg(QString strMsg)
{
    m_operateMessageLabel->setText(strMsg);
    m_operateMsgListWidget->addItem(strMsg);
    // 开始闪动文字;
    m_operateMessageFlickerTimer.start();
}

void DLHangWheelMainWindowFroDevelop::UpdateConfidenceValueSlot()
{
	//更新置信度信息
	m_pConfidenceValueShowWgt->setCurrentValue(m_iConfidenceValue / 100.0);
	if (m_iConfidenceValue < 60)
	{
		m_pConfidenValueLbl->setPixmap(m_confidenceImages[1]);
	}
	else
	{
		m_pConfidenValueLbl->setPixmap(m_confidenceImages[0]);
	}
	m_pConfidenValueLbl->update();

	if (NULL != m_pConfidenceValueShowWgt)
	{
 		m_pConfidenceValueShowWgt->repaint();
	}

	//更新电量信息
	m_pBatteryLevelWgt->setCurrentValue(m_fCurvoltage / 100.0);

	if (m_bIsChargeState)
	{//充电状态
		QSize size(9, 9);
		QPixmap pixmap;
		pixmap.load(":/Resources/Common/image/Charge-green.png");
		pixmap.scaled(size, Qt::KeepAspectRatio);
		m_pChargeState->setPixmap(pixmap);
	}
	else
	{
		m_pChargeState->setPixmap(QPixmap());
	}
	m_pChargeState->repaint();

	QSize batterySize(18, 18);
	QPixmap batteryPixmap;
	if (m_fCurvoltage < 5.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-empty.png");
	}
	else if (m_fCurvoltage >= 5.0 && m_fCurvoltage < 20.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-low.png");
	}
	else if (m_fCurvoltage >= 20.0 && m_fCurvoltage < 60.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-ok-orange.png");
	}
	else if (m_fCurvoltage >= 60.0 && m_fCurvoltage < 90.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-ok-green.png");
	}
	else if (m_fCurvoltage >= 90.0)
	{
		batteryPixmap.load(":/Resources/Common/image/Battery-full.png");
	}
	batteryPixmap.scaled(batterySize, Qt::KeepAspectRatio);
	m_pBatteryLevelLbl->setPixmap(batteryPixmap);
	m_pBatteryLevelLbl->repaint();

	if (NULL != m_pBatteryLevelWgt)
	{
		m_pBatteryLevelWgt->repaint();
	}

}

void DLHangWheelMainWindowFroDevelop::StutterStopBtnSlot()
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

bool DLHangWheelMainWindowFroDevelop::eventFilter(QObject *watched, QEvent *event)
{
	if (watched == m_titleBackWidget)
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
			m_ptStartPos = QPoint(0, 0);
		}break;
	defalut:break;
		}
		//return __super::eventFilter(watched, event);
	}
	else if (m_stackedWidget->currentIndex() == CollectControlWidget)
	{
		if (event->type() == QEvent::KeyPress)
		{
			QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
			if (!keyEvent->isAutoRepeat())
			{
				qDebug() << "11111111keyPressEvent " << Qt::Key(keyEvent->key());
				m_collectControlWidget->setCurrentOperationType(keyEvent->key(), false);
			}
			
			return true;
		}
		else if (event->type() == QEvent::KeyRelease)
		{
			QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
			if (!keyEvent->isAutoRepeat())
			{
				qDebug() << "11111111keyReleaseEvent " << Qt::Key(keyEvent->key());
				m_collectControlWidget->setCurrentOperationType(keyEvent->key(), true);
			}
						
			return true;
		}
		else
		{
			return false;
		}
	}
	return __super::eventFilter(watched, event);
}

void DLHangWheelMainWindowFroDevelop::closeEvent(QCloseEvent *event)
{
	WHEEL_BACK_TO_CORE_SOCKET.closeConnect();
//    TempChoose::releaseInfrared();

    TerminateProcess(GetCurrentProcess(), NULL);
	return __super::closeEvent(event);
}
