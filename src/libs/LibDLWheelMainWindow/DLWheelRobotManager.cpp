#include "DLWheelRobotManager.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLHangRailCommonWidget/VideoBackWidget.h"
#include "LibDLHangRailCommonWidget/CameraObject.h"
#include "LibDLHangRailCommonTools/DLHangRailCommonTools.h"
#include "LibDLHangRailCommonWidget/CustomButton.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
//#include "LibDLInfraredTemperature/InfraredTemperature.h"
#include <QDesktopWidget>
#include <QApplication>
#include <QButtonGroup>
#include <QDir>
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "LibDLBackStageMapWidget/DLBackStageMapWidget.h"
#include "DLWheelTaskCheck.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include "LibDLHangRailCommonWidget/SwitchControl.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QFileDialog>
#include "LibHCNetCamera/HCNetCameraInterface.h"
#include <QAudioProbe>
#include <QGroupBox>

#include "LibDLSceneView/DLCustomScene.h"
#include "LibSpectrum/mainwidget.h"

#define PATROLSTATE_WIDGET_HEIGHT 80				// 巡检状态widget高度;
#define BOTTOM_WIDGET_HEIGHT 300					// 巡检状态widget高度;

#pragma execution_character_set("utf-8") 

#define TABLE_PER_PAGE_COUNT 7
#define List_PER_PAGE_COUNT 9

#define SYSTEM_ALARM_MAX_COUNT 200                  // 系统告警信息最大条数;

DLWheelRobotManager::DLWheelRobotManager(QWidget *parent)
	: QWidget(parent)
    , m_realTimeInfoPageIndex(1)
    , m_deviceAlarmInfoPageIndex(1)
    , m_systemAlarmInfoPageIndex(1)
    , m_isSystemAlarmtTextRed(false)
    , m_isSystemAlarmtButtonTextRed(false)
    , m_currentDeviceAlarmCheckIndex(0)
    , m_unCheckedDeviceAlarmCount(0)
	, m_robotControlThread(NULL)
	, m_hCNetCameraInterfaceForVoiceTalk(NULL)
	, bDoFaceRecognition(false)
{
    qRegisterMetaType<WheelRobotSwitchRunningStatus>("WheelRobotSwitchRunningStatus");
    
	QDesktopWidget * desktopWidget = QApplication::desktop();
	m_sreenWidth = desktopWidget->screenGeometry().width();

	//initVoiceTalk();
	initWidget();
	DLHangRailCommonTools::loadStyleSheet(this, ":/Resources/DLWheelRobotManager/DLWheelRobotManager.css");
	this->setStyleSheet("background:rgb(208,217,255);");
}

DLWheelRobotManager::~DLWheelRobotManager()
{
	if (NULL != m_robotControlThread)
	{
		m_robotControlThread->stop();
		delete m_robotControlThread;
		m_robotControlThread = NULL;
	}

	if (NULL != m_hCNetCameraInterfaceForVoiceTalk)
	{
		delete m_hCNetCameraInterfaceForVoiceTalk;
		m_hCNetCameraInterfaceForVoiceTalk = NULL;
	}
}

int DLWheelRobotManager::GetVisableZoom()
{
	if (nullptr != m_visibleVideoWidget)
	{
		return m_visibleVideoWidget->getCurrentZoomValue();
	}
	return 0;
}

void DLWheelRobotManager::initVoiceTalk()
{
//     m_hCNetCameraInterfaceForVoiceTalk = new HCNetCameraInterface(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraIp,
//                                                                     WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraUser,
//                                                                     WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraPassword, 
//                                                                     WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().voiceTalkCameraPort.toInt());
// 
//     m_hCNetCameraInterfaceForVoiceTalk->connect();
//     m_hCNetCameraInterfaceForVoiceTalk->initCamera(m_hCNetCameraInterfaceForVoiceTalk->getCurrentChannel() + 1);
	m_talkVoiceCom = new TalkVoiceCom;
	m_talkVoiceCom->setHCNetCameraInterface(m_hCNetCameraInterface);
	m_hCNetCameraInterface->setHCNetDvrConfig();
}
void DLWheelRobotManager::setRobotChooseInfo(QString strCurrentChooseRobotName, QStringList strRobotNameList)
{
    m_inputWidgetRobotType->setComboBoxContent(strRobotNameList);
    m_inputWidgetRobotType->setComboBoxCurrentContent(strCurrentChooseRobotName);

    connect(m_inputWidgetRobotType, &InputWidget::signalComboBoxIndexChanged, this, [=](int index) {
        WHEEL_BACK_TO_CORE_SOCKET.robot_connect_2_new_robot_req(m_inputWidgetRobotType->getComboBoxCurrentContent());
		nTabWidgetControl->setCurrentIndex(m_inputWidgetRobotType->getComboBoxCurrentIndex());
        emit signalStartChangeRobotConnect(index);

		if (index == 0)
		{
			m_currentModeLabel->setTipText("巡检机器人");
		}
		else {
			m_currentModeLabel->setTipText("消防机器人");
		}
    });
}

void DLWheelRobotManager::initWidget()
{
	initPatrolState();
	initMapWidget();
	initVideoWidget();
	initDataShowWidget();
	initRobotControlWidget();
    initRealTimeTableData();
    initDeviceAlarmInfoTableData();
    initSystemAlarmTextRefreshTimer();
    initAudioPlayWindow();
    initCoreFunction();

	QVBoxLayout* vTopLeftLayout = new QVBoxLayout;
	vTopLeftLayout->addWidget(m_patrolStateWidget);
	vTopLeftLayout->addWidget(m_pMapWidget);
	vTopLeftLayout->setSpacing(5);
	vTopLeftLayout->setMargin(0);

	QHBoxLayout* hTopLayout = new QHBoxLayout;
	hTopLayout->addLayout(vTopLeftLayout);
	hTopLayout->addWidget(m_videoWidget);
	hTopLayout->setSpacing(5);
	hTopLayout->setMargin(0);

	QWidget* bottomWidget = new QWidget;
	bottomWidget->setFixedHeight(BOTTOM_WIDGET_HEIGHT);

	QHBoxLayout* hBottomLayout = new QHBoxLayout(bottomWidget);
	hBottomLayout->addWidget(m_dataShowWidget);
	hBottomLayout->addWidget(m_robotControlWidget);
	hBottomLayout->setSpacing(5);
	hBottomLayout->setMargin(0);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addLayout(hTopLayout);
	vMainLayout->addWidget(bottomWidget);
	vMainLayout->setSpacing(5);
	vMainLayout->setMargin(5);

    m_robotControlThread = new RobotControlThread;
    m_robotControlThread->start();

	m_faceRecognitionTimer = new QTimer(this);
	m_faceRecognitionTimer->start(3000);
	connect(m_faceRecognitionTimer, SIGNAL(timeout()), this, SLOT(slotDoFaceRecognition()));
}

void DLWheelRobotManager::initPatrolState()
{
	m_patrolStateWidget = new QWidget;
	m_patrolStateWidget->setObjectName("PatrolStateWidget");
	m_patrolStateWidget->setFixedHeight(PATROLSTATE_WIDGET_HEIGHT);	
	m_patrolStateWidget->setStyleSheet("QWidget{background:rgb(175,191,255);}");

	m_inputWidgetRobotType = new InputWidget(InputWidgetType::WheelComboBox);
	m_inputWidgetRobotType->setTipText("机器人");
	m_inputWidgetRobotType->setTipTextBold(true);

	m_inputWidgetPatrolTaskName = new InputWidget(InputWidgetType::WheelValueShowWidget);
	m_inputWidgetPatrolTaskName->setTipText("巡检任务名称");
	m_inputWidgetPatrolTaskName->setShowValue("(无)");

    m_inputWidgetTaskProperty = new InputWidget(InputWidgetType::WheelValueShowWidget);
    m_inputWidgetTaskProperty->setTipText("任务性质    ");
    m_inputWidgetTaskProperty->setShowValue("(无)");

	m_inputWidgetPatrolPointCount = new InputWidget(InputWidgetType::WheelValueShowWidget);
	m_inputWidgetPatrolPointCount->setTipText("巡检点总数  ");
	m_inputWidgetPatrolPointCount->setShowValue("-1");

	m_inputWidgetAbnormalPatrolPointCount = new InputWidget(InputWidgetType::WheelValueShowWidget);
	m_inputWidgetAbnormalPatrolPointCount->setTipText("异常巡检点数");
	m_inputWidgetAbnormalPatrolPointCount->setShowValue("-1");

	m_inputWidgetCurrentPatrolPoint = new InputWidget(InputWidgetType::WheelValueShowWidget);
	m_inputWidgetCurrentPatrolPoint->setTipText("当前巡检点  ");
	m_inputWidgetCurrentPatrolPoint->setShowValue("(无)");

	m_inputWidgetPredictPatrolTime = new InputWidget(InputWidgetType::WheelValueShowWidget);
	m_inputWidgetPredictPatrolTime->setTipText("预计巡检时间");
	m_inputWidgetPredictPatrolTime->setShowValue("-1min");

	m_inputWidgetPatrolProcess = new InputWidget(InputWidgetType::WheelValueShowWidget);
	m_inputWidgetPatrolProcess->setTipText("巡检进度  ");
	m_inputWidgetPatrolProcess->setShowValue("-1%");

	m_inputWidgetPatroledPointCount = new InputWidget(InputWidgetType::WheelValueShowWidget);
	m_inputWidgetPatroledPointCount->setTipText("已巡检点数");
	m_inputWidgetPatroledPointCount->setShowValue("-1");

	QGridLayout* gPatrolStateGridLayout = new QGridLayout(m_patrolStateWidget);
	gPatrolStateGridLayout->addWidget(m_inputWidgetRobotType, 0, 0);
	gPatrolStateGridLayout->addItem(new QSpacerItem(10, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 0, 4);

	gPatrolStateGridLayout->addWidget(m_inputWidgetPatrolTaskName, 1, 0, 1, 1);
    gPatrolStateGridLayout->addWidget(m_inputWidgetTaskProperty, 1, 1, 1, 1);
	gPatrolStateGridLayout->addWidget(m_inputWidgetPredictPatrolTime, 1, 2, 1, 1);
	gPatrolStateGridLayout->addWidget(m_inputWidgetPatrolProcess, 1, 3, 1, 1);

	gPatrolStateGridLayout->addWidget(m_inputWidgetCurrentPatrolPoint, 2, 0, 1, 1);					//当前巡检点
	gPatrolStateGridLayout->addWidget(m_inputWidgetAbnormalPatrolPointCount, 2, 1,1,1);
	gPatrolStateGridLayout->addWidget(m_inputWidgetPatrolPointCount, 2, 2, 1, 1);					//巡检点总数
	gPatrolStateGridLayout->addWidget(m_inputWidgetPatroledPointCount, 2, 3,1,1);

	gPatrolStateGridLayout->setMargin(3);
	gPatrolStateGridLayout->setHorizontalSpacing(20);
	gPatrolStateGridLayout->setVerticalSpacing(5);
}

void DLWheelRobotManager::slotOpenOrCloseFaceRecognition(bool flag)
{
#if 0
	if ("打开人脸识别" == m_buttonOpenOrCloseFaceRecognition->text())
	{
		m_buttonOpenOrCloseFaceRecognition->setText("关闭人脸识别");
		bDoFaceRecognition = true;
	}
	else if ("关闭人脸识别" == m_buttonOpenOrCloseFaceRecognition->text())
	{
		m_buttonOpenOrCloseFaceRecognition->setText("打开人脸识别");
		bDoFaceRecognition = false;
	}
#endif
	if (flag)
	{
		bDoFaceRecognition = true;
	}
	else
	{
		bDoFaceRecognition = false;
	}
}

void DLWheelRobotManager::slotDoFaceRecognition()
{
	if (bDoFaceRecognition) 
	{
		QString input_path = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/FaceRecognition/";

		QDir dir(input_path);
		if (!dir.exists()) {
			dir.mkpath(input_path);
		}
		input_path += QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
		if (m_visibleVideoWidget->cameraCaptureBool(input_path)) {
			WHEEL_BACK_TO_CORE_SOCKET.robot_face_recognition_req(input_path.toStdString());
		}

	}
}

void DLWheelRobotManager::initMapWidget()
{
	//int width = (m_sreenWidth - 20) / 2;

	m_pMapWidget = new DLBackStageMapView;
	m_pMapScene = new DLCustomScene(DL_BROWSE_TYPE, this);
	m_pMapWidget->setSceneRect(-20000, -20000, 40000, 40000);
	m_pMapWidget->setScene(m_pMapScene);
	connect(m_pMapWidget, SIGNAL(DrawStateChangedSignal(bool)), m_pMapScene, SLOT(DrawStateChangedSlot(bool)));
	connect(m_pMapScene, SIGNAL(ExitDrawStateSignal()), m_pMapWidget, SLOT(ExitDrawStateSlot()));
	//test,load_backstage_map是对外添加地图的接口
	//m_mapScene->load_backstage_map("C:\\Users\\alex_wei\\Desktop\\11111d.bsmap");

	m_pMapWidget->setStyleSheet("background:rgb(208,217,255);");

	m_pMapWidget->OpenDefaultMap();
	//m_mapWidget->setFixedWidth(width);

	QLabel* mapLabel = new QLabel;
	mapLabel->setScaledContents(false);
	mapLabel->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	mapLabel->setPixmap(QPixmap(":/Resources/DLWheelRobotManager/Image/MapImage.png"));

	QHBoxLayout* hMapLayout = new QHBoxLayout(m_pMapWidget);
	hMapLayout->addWidget(mapLabel);
	hMapLayout->setMargin(0);
}

void DLWheelRobotManager::initVideoLayout(bool show_infrared)
{
	if (!show_infrared)
	{
		m_vVideoLayout->addStretch();
	}
}

void DLWheelRobotManager::initVideoWidget()
{
	int width = (m_sreenWidth - 20) / 2;

	m_videoWidget = new QWidget;
	m_videoWidget->setFixedWidth(width);

	m_visibleVideoWidget = new VideoBackWidget(VideoType::Wheel_VisibleLightControlVideo);
    m_visibleVideoWidget->setAppTypeClient(APP_TYPE_MAIN_CLIENT);
	m_visibleVideoWidget->setStyleSheet("QWidget{background:rgb(69,94,222);}");
	m_visibleVideoWidget->setFixedHeight(270);
	
	//m_infraredVideoWidget = new VideoBackWidget(VideoType::Wheel_InfraredControlVideo);
	m_infraredVideoWidget = new VideoBackWidget(VideoType::Wheel_VisibleLightControlVideo);
    m_infraredVideoWidget->setAppTypeClient(APP_TYPE_MAIN_CLIENT);
	//m_infraredVideoWidget->setStyleSheet("QWidget{background:rgb(230,74,25);}");
	m_visibleVideoWidget->setStyleSheet("QWidget{background:rgb(69,94,222);}");
	m_infraredVideoWidget->setFixedHeight(360);

	m_vVideoLayout = new QVBoxLayout(m_videoWidget);
	m_vVideoLayout->addWidget(m_visibleVideoWidget);
	m_vVideoLayout->addWidget(m_infraredVideoWidget);
	m_vVideoLayout->setSpacing(5);
	m_vVideoLayout->setMargin(0);
	initVideoLayout(true);
}

void DLWheelRobotManager::initDataShowWidget()
{
	m_dataShowWidget = new QWidget;
	m_dataShowWidget->setObjectName("DataShowWidget");
	m_dataShowTabWidget = new QTabWidget;

	m_timelyInfoTable = new CustomTableWidget(6, false);
	
    m_deviceAlarmInfoTable = new CustomTableWidget(6, false);
    m_deviceAlarmInfoTable->setUnCheckedLabelVisibel(true);
    m_deviceAlarmInfoTable->setItemBackWhite();
    QTableWidget* tableWidget = m_deviceAlarmInfoTable->getTableWidget();
    connect(tableWidget, &QTableWidget::itemDoubleClicked, this, &DLWheelRobotManager::onDeviceAlarmTableDoubleClicked);

    QWidget* systemAlarmInfoBackWidget = new QWidget;
    QVBoxLayout* vSystemAlarmInfoLayout = new QVBoxLayout(systemAlarmInfoBackWidget);
    m_systemAlarmInfoTable = new QListWidget;
    m_systemAlarmInfoTable->setStyleSheet("QListWidget::item{height:25, font-size:14px;}");
    vSystemAlarmInfoLayout->addWidget(m_systemAlarmInfoTable);
    vSystemAlarmInfoLayout->setSpacing(0);
    vSystemAlarmInfoLayout->setMargin(0);

	m_dataShowTabWidget->addTab(m_timelyInfoTable, "实时信息");
	m_dataShowTabWidget->addTab(m_deviceAlarmInfoTable, "设备告警信息");
	m_dataShowTabWidget->addTab(systemAlarmInfoBackWidget, "系统告警信息");

	QVBoxLayout* vDataShowLayout = new QVBoxLayout(m_dataShowWidget);
	vDataShowLayout->addWidget(m_dataShowTabWidget);
	vDataShowLayout->setMargin(0);
	vDataShowLayout->setSpacing(0);

    connect(m_dataShowTabWidget, &QTabWidget::currentChanged, this, [=](int index) {
        if (index == 2)
        {
            m_isSystemAlarmtButtonTextRed = false;
            m_dataShowTabWidget->tabBar()->setTabTextColor(2, Qt::black);
        }
    });

    initTableWidget();
}

void DLWheelRobotManager::initRobotControlWidget()
{
	connect(this, SIGNAL(signalMapReset(bool)), m_pMapScene, SLOT(slot_on_map_reset(bool)));

	int width = (m_sreenWidth - 20) / 2;

	m_robotControlWidget = new QWidget;
	m_robotControlWidget->setFixedWidth(width);

	QWidget* buttonBackWidget = new QWidget;
    buttonBackWidget->setFixedHeight(115);
	buttonBackWidget->setObjectName("ButtonBackWidget");
    QButtonGroup* operateButtonGroup = new QButtonGroup(this);
    operateButtonGroup->setExclusive(false);
	for (int i = 0; i < 10; i++)
	{
		QPushButton* pButton = new QPushButton;
		pButton->setObjectName("ControlButton");
		pButton->setFixedSize(QSize(70, 30));
        pButton->setIconSize(QSize(25, 25));
		m_controlButtonList.append(pButton);
        operateButtonGroup->addButton(pButton, i);
	}

    connect(operateButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, [=](int buttonId) {
        switch (buttonId)
        {
        // 一键返航;
        case 0:
        {
            int result = DLMessageBox::showDLMessageBox(this, "提示", "是否进行一键返航", MessageButtonType::BUTTON_OK_AND_CANCEL, true, QSize(250, 150));
            if (result == QDialog::Accepted)
            {
                WHEEL_BACK_TO_CORE_SOCKET.robot_task_cancel_req();
                QTimer::singleShot(1000, this, [=] {
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_back_to_charge();
                });
                emit signalMapReset(true);
            }
        }
            break;
        // 抓图;
        case 1:
        {
            QString fileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/VisibleLightCapture/";
            QDir dir;
            if (!dir.exists(fileName))
            {
                dir.mkpath(fileName);
            }
            QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
            fileName.append(strTime);
            m_visibleVideoWidget->cameraCapture(fileName);
        }
            break;
        // 停止任务;
        case 2:
			emit signalMapReset(true);
            WHEEL_BACK_TO_CORE_SOCKET.robot_task_cancel_req();
            break;
        // 视频录制;
        case 3:
        {
            if (m_controlButtonList[3]->isChecked())
            {
                QString strCurrentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
                QString strRecordFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/VisibleLightVideo/";
                QDir dir;
                if (!dir.exists(strRecordFilePath))
                {
                    dir.mkpath(strRecordFilePath);
                }

                QString strRecordFileName = strRecordFilePath + strCurrentTime + ".mp4";

                m_hCNetCameraInterface->startRecord(strRecordFileName.toStdString());

                m_controlButtonList[3]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoRecordStop.png"));
            }
            else
            {
                m_hCNetCameraInterface->stopRecord();

                m_controlButtonList[3]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoRecord.png"));
            }
        }
            break;
        // 语音对讲;
        case 4:
        {
            if (m_controlButtonList[4]->isChecked())
            {
			//	m_hCNetCameraInterfaceForVoiceTalk->startVoiceTalk();
			//	m_hCNetCameraInterface->startVoiceTalk();
				m_talkVoiceCom->startVoiceTalkCom();
                m_controlButtonList[8]->setDisabled(false);
                m_controlButtonList[4]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/AudioTalk_Stop.png"));
            }
            else
            {
//                 m_hCNetCameraInterfaceForVoiceTalk->onRecordCameraAudio(false);
//                 m_controlButtonList[8]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/AudioRecord.png"));
//                 m_controlButtonList[8]->setChecked(false);

			//	m_hCNetCameraInterfaceForVoiceTalk->stopVoiceTalk();
			//	m_hCNetCameraInterface->stopVoiceTalk();
				m_talkVoiceCom->stopVoiceTalkCom();
                m_controlButtonList[8]->setDisabled(true);
                m_controlButtonList[4]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/AudioTalk.png"));
            }
        }
            break;
        // 
        case 5:

            break;
        // 任务开始;
        case 6:
            WHEEL_BACK_TO_CORE_SOCKET.robot_task_resume_req();
            break;
        // 任务暂停;
        case 7:
            WHEEL_BACK_TO_CORE_SOCKET.robot_task_pause_req();
            break;
        // 音频录制;
        case 8:
        {
            if (m_controlButtonList[8]->isChecked())
            {
                QString strCurrentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
                QString strRecordFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/AudioRecord/";
                QDir dir;
                if (!dir.exists(strRecordFilePath))
                {
                    dir.mkpath(strRecordFilePath);
                }

                QString strRecordFileName = strRecordFilePath + strCurrentTime;

                m_hCNetCameraInterface->onRecordCameraAudio(true, strRecordFileName);
                //m_hCNetCameraInterfaceForVoiceTalk->onRecordCameraAudio(true, strRecordFileName);

                m_controlButtonList[8]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/AudioRecord_Stop.png"));
            }
            else
            {
                m_hCNetCameraInterface->onRecordCameraAudio(false);
                //m_hCNetCameraInterfaceForVoiceTalk->onRecordCameraAudio(false);
                m_controlButtonList[8]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/AudioRecord.png"));
            }     
        }
            break;
        // 音频回放;
        case 9:
        {
            MainWidget *voiceWidget = new MainWidget;
            voiceWidget->show();
//             QString fileName = QFileDialog::getOpenFileName(this, ("Open File"), "", ("Audio Files ()"));
//             if (fileName.isEmpty())
//             {
//                 return;
//             }
//             m_videoPlayWidget->setVideoPlayPath(fileName);
//             m_videoBackWindow->activateWindow();
//             m_videoBackWindow->show();
        }
            break;
        default:
            break;
        }
    });

    m_controlButtonList[0]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/RobotGoBack.png"));
    m_controlButtonList[0]->setToolTip("一键返航");

    m_controlButtonList[1]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Capture.png"));
    m_controlButtonList[1]->setToolTip("抓图");

    m_controlButtonList[2]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/PlayBack.png"));
    m_controlButtonList[2]->setToolTip("终止任务");

    m_controlButtonList[3]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoRecord.png"));
    m_controlButtonList[3]->setToolTip("录像");
    m_controlButtonList[3]->setCheckable(true);

    m_controlButtonList[4]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/AudioTalk.png"));
    m_controlButtonList[4]->setToolTip("语音对讲");
    m_controlButtonList[4]->setCheckable(true);

    m_controlButtonList[5]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/MagnifyingGlass.png"));
    m_controlButtonList[5]->setToolTip("");

    m_controlButtonList[6]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Play.png"));
    m_controlButtonList[6]->setToolTip("任务开始");

    m_controlButtonList[7]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Pause.png"));
    m_controlButtonList[7]->setToolTip("任务暂停");

    m_controlButtonList[8]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/AudioRecord.png"));
    m_controlButtonList[8]->setToolTip("音频录制");
    m_controlButtonList[8]->setCheckable(true);
    m_controlButtonList[8]->setDisabled(true);

    m_controlButtonList[9]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/AudioPlayBack.png"));
    m_controlButtonList[9]->setToolTip("音频回放");

    m_currentModeLabel = new InputWidget(InputWidgetType::WheelValueShowWidget);
    m_currentModeLabel->setTipText("巡检机器人");
    m_currentModeLabel->setFixedSize(QSize(160, 20));

	QButtonGroup* modeButtonGroup = new QButtonGroup(this);
	for (int i = 0; i < 4; i++)
	{
		QPushButton* pButton = new QPushButton;
		pButton->setCheckable(true);
		pButton->setObjectName("ModeButton");
		pButton->setFixedSize(QSize(100, 30));
		m_ModeButtonList.append(pButton);
		modeButtonGroup->addButton(pButton, i);
	}

	m_ModeButtonList[0]->setText("任务模式");
	m_ModeButtonList[0]->setCheckable(true);
	m_ModeButtonList[0]->setChecked(true);
	m_ModeButtonList[1]->setText("紧急定位模式");
	m_ModeButtonList[1]->setCheckable(true);
	m_ModeButtonList[1]->setChecked(false);
	m_ModeButtonList[2]->setText("后台遥控模式");
	m_ModeButtonList[2]->setCheckable(true);
	m_ModeButtonList[2]->setChecked(false);
	m_ModeButtonList[3]->setText("手持遥控模式");
	m_ModeButtonList[3]->setCheckable(true);
	m_ModeButtonList[3]->setChecked(false);

    // 模式选择;
    connect(modeButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, [=](int buttonId) {
        WheelRobotSwitchRunningStatus status;
        switch (buttonId)
        {
        case 0:
            status = WHEEL_ROBOT_SWITCH_AUTORUNNING;
            break;
        case 1:
            status = WHEEL_ROBOT_SWITCH_EMERGENCY_LOC;
            break;
        case 2:
            status = WHEEL_ROBOT_SWITCH_REMOTE_CTRL;
            break;
        case 3:
            status = WHEEL_ROBOT_SWITCH_JOY_CTRL;
            break;
        default:
            break;
        }
        WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(status);
    });

	QGridLayout* gControlButtonLayout = new QGridLayout;
    gControlButtonLayout->addWidget(m_controlButtonList[8], 0, 0);
	gControlButtonLayout->addWidget(m_controlButtonList[0], 0, 1);
	gControlButtonLayout->addWidget(m_controlButtonList[1], 0, 2);
	gControlButtonLayout->addWidget(m_controlButtonList[2], 0, 3);
	gControlButtonLayout->addWidget(m_controlButtonList[3], 0, 4);
    gControlButtonLayout->addWidget(m_controlButtonList[9], 1, 0);
	gControlButtonLayout->addWidget(m_controlButtonList[4], 1, 1);
	gControlButtonLayout->addWidget(m_controlButtonList[5], 1, 2);
	gControlButtonLayout->addWidget(m_controlButtonList[6], 1, 3);
	gControlButtonLayout->addWidget(m_controlButtonList[7], 1, 4);
    
	gControlButtonLayout->setSpacing(5);
	gControlButtonLayout->setContentsMargins(0, 18, 0, 15);

	QGridLayout* gModeButtonLayout = new QGridLayout;
    gModeButtonLayout->addWidget(m_currentModeLabel, 0, 0, 1, 2);
	gModeButtonLayout->addWidget(m_ModeButtonList[0], 1, 0);
	gModeButtonLayout->addWidget(m_ModeButtonList[1], 1, 1);
    gModeButtonLayout->addItem(new QSpacerItem(20, 5, QSizePolicy::Fixed, QSizePolicy::Fixed), 2, 0);
	gModeButtonLayout->addWidget(m_ModeButtonList[2], 3, 0);
	gModeButtonLayout->addWidget(m_ModeButtonList[3], 3, 1);
	gModeButtonLayout->setHorizontalSpacing(20);
	gModeButtonLayout->setVerticalSpacing(5);
	gModeButtonLayout->setContentsMargins(0, 0, 0, 25);


	QHBoxLayout* hButtonLayout = new QHBoxLayout(buttonBackWidget);
	hButtonLayout->addStretch();
	hButtonLayout->addLayout(gControlButtonLayout);
	hButtonLayout->addLayout(gModeButtonLayout);
	hButtonLayout->addStretch();
	hButtonLayout->setSpacing(20);
	hButtonLayout->setContentsMargins(5, 0, 5, 0);

	initControlBackWidget();
	QHBoxLayout* hControlWidgetLayout = new QHBoxLayout;
	//hControlWidgetLayout->addWidget(m_radarControlWidget);
	hControlWidgetLayout->addWidget(m_bodyControlWidget);
	hControlWidgetLayout->addWidget(m_ptzControlWidget);
	//hControlWidgetLayout->addWidget(m_FireFightingControlWidget);
	hControlWidgetLayout->setSpacing(0);
	hControlWidgetLayout->setMargin(0);
	QWidget* nContrilWidget = new QWidget;
	nContrilWidget->setLayout(hControlWidgetLayout);

	initControlBackWidgetPutOutTheFire();
	QHBoxLayout* hControlWidgetLayoutPutFire = new QHBoxLayout;
	hControlWidgetLayoutPutFire->addWidget(m_bodyControlWidgetPutFire); 
	//hControlWidgetLayoutPutFire->addWidget(m_ptzControlWidgetPutFire);
	hControlWidgetLayoutPutFire->setSpacing(0);
	hControlWidgetLayoutPutFire->setMargin(0);
	QWidget* nContrilWidgetPutFire = new QWidget;
	nContrilWidgetPutFire->setLayout(hControlWidgetLayoutPutFire);

	initSwitchWidget();

	nTabWidgetControl = new QStackedWidget;
	//m_inputWidgetRobotType->getComboBoxWidget();
	nTabWidgetControl->insertWidget(0, nContrilWidget);
	nTabWidgetControl->insertWidget(1, nContrilWidgetPutFire);
	nTabWidgetControl->setCurrentIndex(0);
	connect(this, SIGNAL(signalStartChangeRobotConnect(int)), nTabWidgetControl, SLOT(setCurrentIndex(int)));

	m_switch_back_stacked_widget = new QStackedWidget;
	//m_inputWidgetRobotType->getComboBoxWidget();
	m_switch_back_stacked_widget->insertWidget(0, m_switchBackWidget);
	m_switch_back_stacked_widget->insertWidget(1, m_fire_robot_control_widget);
	m_switch_back_stacked_widget->setCurrentIndex(0);
	connect(this, SIGNAL(signalStartChangeRobotConnect(int)), m_switch_back_stacked_widget, SLOT(setCurrentIndex(int)));

	QGridLayout* vRobotControlLayout = new QGridLayout(m_robotControlWidget);
	vRobotControlLayout->addWidget(buttonBackWidget, 0, 0);
	vRobotControlLayout->addWidget(nTabWidgetControl, 1, 0);
	vRobotControlLayout->addWidget(m_switch_back_stacked_widget, 0, 1, 2, 1);
	vRobotControlLayout->setSpacing(5);
	vRobotControlLayout->setMargin(0);
}

void DLWheelRobotManager::initControlBackWidget()
{
	// 车体控制;
	m_bodyControlWidget = new ControlBackWidget;
	m_bodyControlWidget->setTitleText("车体控制");

	m_bodyControlButton = new CustomButton(Wheel_RobotControl_BodyMove);
	m_bodyControlButton->setRadiusValue(65);
	m_bodyControlButton->setArcLength(40);
	m_bodyControlButton->setArcAngle(45, 89.8);

    connect(m_bodyControlButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotBodyControlPressed(int)));
    connect(m_bodyControlButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotBodyControlReleased(int)));

	m_bodyControlWidget->setCenterWidget(m_bodyControlButton);

	// 查打一体机器人双光云台控制;
	m_ptzControlWidget = new ControlBackWidget;
	m_ptzControlWidget->setTitleText("双光云台控制");
	
	m_ptzControlButton = new CustomButton(Wheel_RobotControl_PTZMove);
	m_ptzControlButton->setRadiusValue(65);
	m_ptzControlButton->setArcLength(40);
	m_ptzControlButton->setArcAngle(45, 89.8);

    connect(m_ptzControlButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotPTZControlPressed(int)));
    connect(m_ptzControlButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotPTZControlReleased(int)));

	m_ptzControlWidget->setCenterWidget(m_ptzControlButton);

	/// 查打一体机器人灭火云台控制
	//m_FireFightingControlWidget = new ControlBackWidget;
	//m_FireFightingControlWidget->setTitleText("灭火云台控制");
	
	//m_fireFightingContiotButton = new CustomButton(Wheel_RobotControl_BodyMove);
	//m_fireFightingContiotButton->setRadiusValue(65);
	//m_fireFightingContiotButton->setArcLength(40);
	//m_fireFightingContiotButton->setArcAngle(45, 89.8);

	//connect(m_fireFightingContiotButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onFireRobotPTZControlPressed(int)));
	//connect(m_fireFightingContiotButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onFireRobotPTZControlReleased(int)));

	//m_FireFightingControlWidget->setCenterWidget(m_fireFightingContiotButton);
}

void DLWheelRobotManager::initControlBackWidgetPutOutTheFire()
{
	// 查打一体机器人双光云台控制;
	m_bodyControlWidgetPutFire = new ControlBackWidget;
	m_bodyControlWidgetPutFire->setTitleText("车体控制");


	m_bodyControlButtonPutFire = new CustomButton(Wheel_RobotControl_BodyMove);
	m_bodyControlButtonPutFire->setRadiusValue(65);
	m_bodyControlButtonPutFire->setArcLength(40);
	m_bodyControlButtonPutFire->setArcAngle(45, 89.8);

	/*connect(m_bodyControlButtonPutFire, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotBodyControlPressed(int)));
	connect(m_bodyControlButtonPutFire, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotBodyControlReleased(int)));*/

	//connect(m_ptzControlButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotPTZControlPressed(int)));
	//connect(m_ptzControlButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotPTZControlReleased(int)));

	m_bodyControlWidgetPutFire->setCenterWidget(m_bodyControlButtonPutFire);

	//添加消防机器人控制按钮
	//initFireRobotControlWidget();

	/// 查打一体机器人灭火云台控制
	m_ptzControlWidgetPutFire = new ControlBackWidget;
	//m_ptzControlWidgetPutFire->setTitleText("灭火云台控制");

	//m_ptzControlButtonPutFir = new CustomButton(Wheel_RobotControl_BodyMove);
	//m_ptzControlButtonPutFir->setRadiusValue(65);
	//m_ptzControlButtonPutFir->setArcLength(40);
	//m_ptzControlButtonPutFir->setArcAngle(45, 89.8);

	//connect(m_ptzControlButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotPTZControlPressed(int)));
	//connect(m_ptzControlButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotPTZControlReleased(int)));

	//m_ptzControlWidgetPutFire->setCenterWidget(m_ptzControlButtonPutFir);
}

void DLWheelRobotManager::initSwitchWidget()
{
	//1.初始化查打机器人控制界面
	m_switchBackWidget = new QWidget;

	for (int i = 0; i < 6; i++)
    {
        m_SwitchControl[i] = new SwitchWidget;
        m_SwitchControl[i]->setSwitchSize(QSize(70, 25));
        m_SwitchControl[i]->setFixedSize(QSize(150, 25));
        m_SwitchControl[i]->setSwitchCheckColor(QColor(80, 215, 105));
		m_SwitchControl[i]->setTextSize(13);
    }
	nButtonWellRobotCheckCamare = new QPushButton("前置摄像头");
	nButtonWellRobotCheckCamare->setFixedHeight(30);
	connect(nButtonWellRobotCheckCamare, &QPushButton::clicked, this, &DLWheelRobotManager::slotCheckWheelRobotCamare);
	
    m_SwitchControl[0]->setSwitchName("超    声");
    //m_SwitchControl[1]->setSwitchName("雨    刷");
    //m_SwitchControl[2]->setSwitchName("自 动 门");
    //m_SwitchControl[3]->setSwitchName("充电机构");
	m_SwitchControl[1]->setSwitchName("雨    刷");
	m_SwitchControl[2]->setSwitchName("车    灯");
	m_SwitchControl[3]->setSwitchName("警 示 灯");
	m_SwitchControl[4]->setSwitchName("人脸识别");
	m_SwitchControl[5]->setSwitchName("灭 火 罐");
	m_SwitchControl[5]->setSwitchSize(QSize(140, 25));
	m_SwitchControl[5]->setFixedSize(QSize(300, 25));

	
	QGridLayout* gSwitchLayout = new QGridLayout(m_switchBackWidget);
	//for (int i = 0; i < 5; i++)
 //   {
	//	gSwitchLayout->addWidget(m_SwitchControl[i], i / 2, i % 2);
	//	if (4 != i)
	//	{
	//		connect(m_SwitchControl[i], &SwitchWidget::toggled, this, &DLWheelRobotManager::onSwitchStateChanged);
	//	}
 //   }
	connect(m_SwitchControl[4], &SwitchWidget::toggled, this, &DLWheelRobotManager::slotOpenOrCloseFaceRecognition);

	/// 添加摄像头控制按钮
	//gSwitchLayout->addWidget(nButtonWellRobotCheckCamare, 4, 0, 1, 4);

	QGroupBox		*cdk_robot_emergency_task_groupbox = new QGroupBox;//查打卡机器人
	cdk_robot_emergency_task_groupbox->setTitle("紧急任务区");
	cdk_robot_emergency_task_groupbox->setStyleSheet("QGroupBox{border:2px solid rgb(0,0,0);}");
	QVBoxLayout *cdk_robot_emergency_task_layout = new QVBoxLayout;
	cdk_robot_emergency_task_layout->addWidget(m_SwitchControl[5]);
	cdk_robot_emergency_task_groupbox->setLayout(cdk_robot_emergency_task_layout);
	//gSwitchLayout->addWidget(cdk_robot_emergency_task_groupbox, 5, 0, 4, 2);
	//cdk_robot_emergency_task_groupbox->setFixedHeight(80);
    gSwitchLayout->setContentsMargins(0, 15, 0, 15);
	gSwitchLayout->setVerticalSpacing(15);

	// 模式选择;
	connect(m_SwitchControl[5], &SwitchWidget::toggled, this, [=](bool checked) {
		int ret = DLMessageBox::showDLMessageBox(this, "警告", QString("是否确认进行灭火罐喷淋!"), MessageButtonType::BUTTON_OK_AND_CANCEL, true);
		if (ret) {
			m_SwitchControl[5]->setSwitchState(true);
		}
		else {
			m_SwitchControl[5]->setSwitchState(false);
		}
		onSwitchStateChanged();
	});

	//2.初始化灭火机器人控制界面
	m_fire_robot_control_widget = new QWidget;
	nButtonFireRobotCheck = new QPushButton("前置摄像头");
	//nButtonFireRobotCheck->setStyleSheet("QPushButton{background:rgb(121,134,203);}QFont{color:rgb(51, 163, 195);}");
	nButtonFireRobotCheck->setFixedHeight(30);
	connect(nButtonFireRobotCheck, &QPushButton::clicked, this, &DLWheelRobotManager::slotCheckFireRobotCamare);

	//m_fire_robot_image_check_widget = new InputWidget(InputWidgetType::WheelComboBox);
	//m_fire_robot_image_check_widget->setTipText("图像切换");
	//m_fire_robot_image_check_widget->setTipTextBold(false);

	//QStringList image_check_list;
	//image_check_list << "前置" << "后置" << "图像";
	//m_fire_robot_image_check_widget->setComboBoxContent(image_check_list); 
	//m_fire_robot_image_check_widget->setComboBoxCurrentIndex(0);

	for (int i = 0; i < 5; i++)
	{
		m_fire_robot_switch_control[i] = new SwitchWidget;
		m_fire_robot_switch_control[i]->setSwitchSize(QSize(60, 25));
		m_fire_robot_switch_control[i]->setFixedSize(QSize(130, 25));
		m_fire_robot_switch_control[i]->setSwitchCheckColor(QColor(80, 215, 105));
	}
	m_fire_robot_switch_control[0]->setSwitchName("自喷淋");
	m_fire_robot_switch_control[1]->setSwitchName("水  柱");
	m_fire_robot_switch_control[2]->setSwitchName("水  雾");
	m_fire_robot_switch_control[3]->setSwitchName("扫  射");
	m_fire_robot_switch_control[4]->setSwitchName("灭火罐");

	QGridLayout* m_fire_robot_control_layout = new QGridLayout(m_fire_robot_control_widget);
	//m_fire_robot_control_layout->addWidget(m_fire_robot_image_check_widget, 0, 0, 1, 2);

	for (int i = 0; i < 4; i++)
	{
		m_fire_robot_control_layout->addWidget(m_fire_robot_switch_control[i], (i + 2) / 2 - 1, (i + 2) % 2);
		connect(m_SwitchControl[i], &SwitchWidget::toggled, this, &DLWheelRobotManager::onSwitchStateChanged);
	}
	//m_fire_robot_control_layout->addWidget(nButtonFireRobotCheck, 2, 0, 1, 2);

	QGroupBox		*fire_robot_emergency_task_groupbox = new QGroupBox;
	fire_robot_emergency_task_groupbox->setTitle("紧急任务区");
	fire_robot_emergency_task_groupbox->setStyleSheet("QGroupBox{border:2px solid rgb(0,0,0);}");
	QHBoxLayout *fire_robot_emergency_task_layout = new QHBoxLayout;
	fire_robot_emergency_task_layout->addWidget(m_fire_robot_switch_control[4]);
	fire_robot_emergency_task_groupbox->setLayout(fire_robot_emergency_task_layout);
	m_fire_robot_control_layout->addWidget(fire_robot_emergency_task_groupbox, 3, 0, 1, 2);

	m_fire_robot_control_layout->setContentsMargins(0, 15, 0, 15);
}

void DLWheelRobotManager::slotCheckWheelRobotCamare(bool) 
{
	if ("图像摄像头" == nButtonWellRobotCheckCamare->text())
	{
		emit signalChangeCameraShowIndex(0);
		nButtonWellRobotCheckCamare->setText("前置摄像头");
	}
	else if ("前置摄像头" == nButtonWellRobotCheckCamare->text())
	{
		emit signalChangeCameraShowIndex(1);
		nButtonWellRobotCheckCamare->setText("后置摄像头");
	}
	else if ("后置摄像头" == nButtonWellRobotCheckCamare->text())
	{
		emit signalChangeCameraShowIndex(2);
		nButtonWellRobotCheckCamare->setText("图像摄像头");
	}
}


void DLWheelRobotManager::slotCheckFireRobotCamare(bool)
{
	if ("前置摄像头" == nButtonFireRobotCheck->text()) 
	{
		nButtonFireRobotCheck->setText("后置摄像头");
	}
	else if ("后置摄像头" == nButtonFireRobotCheck->text())
	{
		nButtonFireRobotCheck->setText("前置摄像头");
	}
}

void DLWheelRobotManager::initCoreFunction()
{
    // 更新当前机器人模式状态;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotSwitchRunningStatusSignal.connect(boost::bind(&DLWheelRobotManager::signalRobotModeStateCallBack, this, _1));

    connect(this, &DLWheelRobotManager::signalRobotModeStateCallBack, this, [=](WheelRobotSwitchRunningStatus status) {
        switch (status)
        {
        case WHEEL_ROBOT_SWITCH_AUTORUNNING:
        {
            m_currentModeLabel->setShowValue("任务模式");
			m_ModeButtonList[0]->setChecked(true);
			m_ModeButtonList[1]->setChecked(false);
			m_ModeButtonList[2]->setChecked(false);
			m_ModeButtonList[3]->setChecked(false);
            break;
        }
        case WHEEL_ROBOT_SWITCH_REMOTE_CTRL:
        {
			m_currentModeLabel->setShowValue("后台遥控模式");
            m_ModeButtonList[2]->setChecked(true);
			m_ModeButtonList[0]->setChecked(false);
			m_ModeButtonList[1]->setChecked(false);
			m_ModeButtonList[3]->setChecked(false);
			break;
        }
        case WHEEL_ROBOT_SWITCH_JOY_CTRL:
        {
            m_currentModeLabel->setShowValue("手持遥控模式");
            m_ModeButtonList[3]->setChecked(true);
			m_ModeButtonList[0]->setChecked(false);
			m_ModeButtonList[1]->setChecked(false);
			m_ModeButtonList[2]->setChecked(false);
			break;
        }
        case WHEEL_ROBOT_SWITCH_EMERGENCY_LOC:
        {
            m_currentModeLabel->setShowValue("紧急定位模式");
            m_ModeButtonList[1]->setChecked(true);
			m_ModeButtonList[0]->setChecked(false);
			m_ModeButtonList[2]->setChecked(false);
			m_ModeButtonList[3]->setChecked(false);
			break;
        }
        default:
            break;
        }
    });
}

void DLWheelRobotManager::initTableWidget()
{
    QTableWidget* tableWidget = m_timelyInfoTable->getTableWidget();
    tableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "识别时间" << "机器人名称" << "点位名称" << "识别结果" << "告警等级");
    connect(m_timelyInfoTable, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_realTimeInfoPageIndex = currentPageIndex;
        initRealTimeTableData();
    });
    connect(m_timelyInfoTable, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initRealTimeTableData();
    });
   
    tableWidget = m_deviceAlarmInfoTable->getTableWidget();
    tableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "识别时间" << "机器人名称" << "点位名称" << "识别结果" << "告警等级");
    connect(m_deviceAlarmInfoTable, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_deviceAlarmInfoPageIndex = currentPageIndex;
        initDeviceAlarmInfoTableData();
    });
    connect(m_deviceAlarmInfoTable, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initDeviceAlarmInfoTableData();
    });
}

void DLWheelRobotManager::initRealTimeTableData()
{
    if (m_realTimeInfoList.isEmpty())
    {
        m_timelyInfoTable->setTurnPageInfo(0, 0, 0, TABLE_PER_PAGE_COUNT);
        return;
    }

    QTableWidget* tableWidget = m_timelyInfoTable->getTableWidget();
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    int totalPage, totalCount;
    totalCount = m_realTimeInfoList.count();
    totalPage = totalCount / TABLE_PER_PAGE_COUNT;
    if (totalCount % TABLE_PER_PAGE_COUNT != 0)
    {
        totalPage++;
    }
    m_timelyInfoTable->setTurnPageInfo(m_realTimeInfoPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

    int startIndex = (m_realTimeInfoPageIndex - 1) * TABLE_PER_PAGE_COUNT;
    int endIndex;
    if (m_realTimeInfoPageIndex == totalPage)
    {
        endIndex = m_realTimeInfoList.count();
    }
    else
    {
        endIndex = m_realTimeInfoPageIndex * TABLE_PER_PAGE_COUNT;
    }

    for (int i = startIndex; i < endIndex; i++)
    {
        int rowIndex = i - startIndex;
        tableWidget->insertRow(rowIndex);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 0, idItem);

        QTableWidgetItem* recognitionItem = new QTableWidgetItem(m_realTimeInfoList[i].inspect_time);
        tableWidget->setItem(rowIndex, 1, recognitionItem);
        recognitionItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* robotNameItem = new QTableWidgetItem("xxx");
        robotNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 2, robotNameItem);

        QString pointName;
        if (!m_realTimeInfoList[i].virtual_name.isEmpty())
        {
            pointName = m_realTimeInfoList[i].virtual_name;
        }
        else
        {
            if (m_realTimeInfoList[i].isHangOut)
            {
                pointName = QString("已跳过%1个挂牌设备！").arg(m_realTimeInfoList[i].alarm_count);
            }
            else
            {
                WHEEL_ROBOT_DB.getDeviceFullNameByDeviceUuid(m_realTimeInfoList[i].device_uuid, pointName);
            }
        }
        QTableWidgetItem* pointNameItem = new QTableWidgetItem(pointName);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 3, pointNameItem);

        QTableWidgetItem* recognitionResultItem = new QTableWidgetItem(m_realTimeInfoList[i].inspect_result);
        recognitionResultItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 4, recognitionResultItem);

        QTableWidgetItem* AlarmLevelItem = new QTableWidgetItem(getAlarmText(m_realTimeInfoList[i].alarm_level_id));
        AlarmLevelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 5, AlarmLevelItem);
    }
}

void DLWheelRobotManager::initDeviceAlarmInfoTableData()
{
    if (m_deviceAlarmInfoList.isEmpty())
    {
        m_deviceAlarmInfoTable->setTurnPageInfo(0, 0, 0, TABLE_PER_PAGE_COUNT, m_unCheckedDeviceAlarmCount);
        return;
    }

    QTableWidget* tableWidget = m_deviceAlarmInfoTable->getTableWidget();
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    int totalPage, totalCount;
    totalCount = m_deviceAlarmInfoList.count();
    totalPage = totalCount / TABLE_PER_PAGE_COUNT;
    if (totalCount % TABLE_PER_PAGE_COUNT != 0)
    {
        totalPage++;
    }
    m_deviceAlarmInfoTable->setTurnPageInfo(m_deviceAlarmInfoPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT, m_unCheckedDeviceAlarmCount);

    int startIndex = (m_deviceAlarmInfoPageIndex - 1) * TABLE_PER_PAGE_COUNT;
    int endIndex;
    if (m_deviceAlarmInfoPageIndex == totalPage)
    {
        endIndex = m_deviceAlarmInfoList.count();
    }
    else
    {
        endIndex = m_deviceAlarmInfoPageIndex * TABLE_PER_PAGE_COUNT;
    }

    for (int i = startIndex; i < endIndex; i++)
    {
        int rowIndex = i - startIndex;
        tableWidget->insertRow(rowIndex);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 0, idItem);        

        QTableWidgetItem* recognitionItem = new QTableWidgetItem(m_deviceAlarmInfoList[i].inspect_time);
        tableWidget->setItem(rowIndex, 1, recognitionItem);
        recognitionItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* robotNameItem = new QTableWidgetItem("xxx");
        robotNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 2, robotNameItem);

        QString pointName;
        if (!m_deviceAlarmInfoList[i].virtual_name.isEmpty())
        {
            pointName = m_deviceAlarmInfoList[i].virtual_name;
        }
        else
        {
            if(m_deviceAlarmInfoList[i].isHangOut)
            {
                pointName = QString("已跳过%1个挂牌设备！").arg(m_deviceAlarmInfoList[i].alarm_count);
            }
            else
            {
                WHEEL_ROBOT_DB.getDeviceFullNameByDeviceUuid(m_deviceAlarmInfoList[i].device_uuid, pointName);
            }
        }
        
        QTableWidgetItem* pointNameItem = new QTableWidgetItem(pointName);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 3, pointNameItem);

        QTableWidgetItem* recognitionResultItem = new QTableWidgetItem(m_deviceAlarmInfoList[i].inspect_result);
        recognitionResultItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 4, recognitionResultItem);

        QTableWidgetItem* AlarmLevelItem = new QTableWidgetItem(getAlarmText(m_deviceAlarmInfoList[i].alarm_level_id));
        AlarmLevelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, 5, AlarmLevelItem);

        if (m_deviceAlarmInfoList[i].is_dealed)
        {
            idItem->setBackgroundColor(QColor(175, 191, 255));
            recognitionItem->setBackgroundColor(QColor(175, 191, 255));
            robotNameItem->setBackgroundColor(QColor(175, 191, 255));
            pointNameItem->setBackgroundColor(QColor(175, 191, 255));
            recognitionResultItem->setBackgroundColor(QColor(175, 191, 255));
            AlarmLevelItem->setBackgroundColor(QColor(175, 191, 255));
        }
    }
}

void DLWheelRobotManager::initSystemAlarmTextRefreshTimer()
{
    m_systemAlarmTextRefreshTimer.setInterval(500);
    connect(&m_systemAlarmTextRefreshTimer, &QTimer::timeout, this, [=] {
        if (!m_systemAlarmInfoList.isEmpty())
        {
            if (m_isSystemAlarmtTextRed)
            {
                m_systemAlarmInfoTable->setStyleSheet("QListWidget::item{color:red;height:25px;font-size:14px;}");
            }
            else
            {
                m_systemAlarmInfoTable->setStyleSheet("QListWidget::item{color:black;height:25px;font-size:14px;}");
            }
        }

        if (!m_deviceAlarmInfoList.isEmpty())
        {
            QTableWidget* tableWidget = m_deviceAlarmInfoTable->getTableWidget();
            for (int i = 0; i < tableWidget->rowCount(); i++)
            {
                QColor textColor;
                if (m_isSystemAlarmtTextRed)
                {
                    QString strAlarmText = tableWidget->item(i, 5)->text();
                    textColor = getItemColor(strAlarmText);
                }
                else
                {
                    textColor = Qt::black;
                }

                for (int j = 0; j < tableWidget->columnCount(); j++)
                {
                    tableWidget->item(i, j)->setForeground(textColor);
                }
            }
        }        

        if (m_isSystemAlarmtButtonTextRed)
        {
            if (m_isSystemAlarmtTextRed)
            {
                m_dataShowTabWidget->tabBar()->setTabTextColor(2, Qt::red);
            }
            else
            {
                m_dataShowTabWidget->tabBar()->setTabTextColor(2, Qt::black);
            }
        }

        m_isSystemAlarmtTextRed = !m_isSystemAlarmtTextRed;
    });

    m_systemAlarmTextRefreshTimer.start();
}

void DLWheelRobotManager::initAudioPlayWindow()
{
    // 视频播放窗口;
    m_videoBackWindow = new BaseWidget(NULL, BaseWidgetType::PopupWindow);
    m_videoBackWindow->setShowCloseButton();
    m_videoBackWindow->setFixedSize(QSize(600, 400));
    m_videoBackWindow->setWindowFlags(Qt::FramelessWindowHint);
    m_videoBackWindow->setTitleContent("音频回放");
    connect(m_videoBackWindow, &BaseWidget::signalCloseButtonClicked, this, [=] {
        m_videoPlayWidget->stopVideoPlay();
        m_videoBackWindow->hide();
    });
    m_videoPlayWidget = new VideoPlayer(NULL, AudioPlay);
    QHBoxLayout* hVideoLayout = new QHBoxLayout(m_videoBackWindow->getCenterWidget());
    hVideoLayout->addWidget(m_videoPlayWidget);
    hVideoLayout->setMargin(0);
}

void DLWheelRobotManager::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), Qt::white);
}

QString DLWheelRobotManager::getAlarmText(DeviceAlarmLevel alarm_level_id)
{
    QString strAlarmText;
    switch (alarm_level_id)
    {
    case Alarm_NONE:
        strAlarmText = "";
        break;
    case Alarm_Normal:
        strAlarmText = "正常";
        break;
    case Alarm_Waring:
        strAlarmText = "预警";
        break;
    case Alarm_Common:
        strAlarmText = "一般告警";
        break;
    case Alarm_Serious:
        strAlarmText = "严重告警";
        break;
    case Alarm_Dangerous:
        strAlarmText = "危急告警";
        break;
    case Alarm_NoIdentifyAbnormal:
        strAlarmText = "未识别异常";
        break;
    case Alarm_TakePhotoError:
        strAlarmText = "拍照失败";
        break;
    case Alarm_DeviceRecondition:
        strAlarmText = "设备挂牌";
        break;
    default:
        strAlarmText = "其他";
        break;
    }
    return strAlarmText;
}

DeviceAlarmLevel DLWheelRobotManager::getAlarmId(QString alarm_level_Name)
{
    DeviceAlarmLevel alarmLevel;
    if (alarm_level_Name.isEmpty())
    {
        alarmLevel = Alarm_NONE;
    }
    else if (alarm_level_Name == "正常")
    {
        alarmLevel = Alarm_Normal;
    }
    else if (alarm_level_Name == "预警")
    {
        alarmLevel = Alarm_Waring;
    }
    else if (alarm_level_Name == "一般告警")
    {
        alarmLevel = Alarm_Common;
    }
    else if (alarm_level_Name == "严重告警")
    {
        alarmLevel = Alarm_Serious;
    }
    else if (alarm_level_Name == "危急告警")
    {
        alarmLevel = Alarm_Dangerous;
    }
    else if (alarm_level_Name == "未识别异常")
    {
        alarmLevel = Alarm_NoIdentifyAbnormal;
    }
    else if (alarm_level_Name == "拍照失败")
    {
        alarmLevel = Alarm_TakePhotoError;
    }
    else if (alarm_level_Name == "设备挂牌")
    {
        alarmLevel = Alarm_DeviceRecondition;
    }
    else if (alarm_level_Name == "其他")
    {
        alarmLevel = Alarm_NUM;
    }

    return alarmLevel;
}

QColor DLWheelRobotManager::getItemColor(QString strSaskStatus)
{
    QColor color;
    if (!strSaskStatus.compare("正常"))
    {
        color = QColor(0, 128, 0);
    }
    else if (!strSaskStatus.compare("预警"))
    {
        color = QColor(0, 0, 255);
    }
    else if (!strSaskStatus.compare("一般告警"))
    {
        color = QColor(255, 255, 0);
    }
    else if (!strSaskStatus.compare("严重告警"))
    {
        color = QColor(255, 128, 10);
    }
    else if (!strSaskStatus.compare("危急告警"))
    {
        color = QColor(255, 0, 0);
    }
    else if (!strSaskStatus.compare("未识别异常"))
    {
        color = Qt::gray;
    }

    return color;
}

void DLWheelRobotManager::onRobotBodyControlPressed(int buttonId)
{
    switch (buttonId)
    {
    case 0:
        m_robotControlThread->setCurrentOperationType(Robot_Move_Head, false);
        break;
    case 1:
        m_robotControlThread->setCurrentOperationType(Robot_Move_Left, false);
        break;
    case 2:
        m_robotControlThread->setCurrentOperationType(Robot_Move_Tail, false);
        break;
    case 3:
        m_robotControlThread->setCurrentOperationType(Robot_Move_Right, false);
        break;
    case 4:
        break;
    default:
        break;
    }
}

void DLWheelRobotManager::onRobotBodyControlReleased(int buttonId)
{
    switch (buttonId)
    {
    case 0:
        m_robotControlThread->setCurrentOperationType(Robot_Move_Head, true);
        break;
    case 1:
        m_robotControlThread->setCurrentOperationType(Robot_Move_Left, true);
        break;
    case 2:
        m_robotControlThread->setCurrentOperationType(Robot_Move_Tail, true);
        break;
    case 3:
        m_robotControlThread->setCurrentOperationType(Robot_Move_Right, true);
        break;
    case 4:
        break;
    default:
        break;
    }
}

void DLWheelRobotManager::onRobotPTZControlPressed(int buttonId)
{
    switch (buttonId)
    {
    case 0:
        m_robotControlThread->setCurrentOperationType(PTZ_Move_Up, false);
        break;
    case 1:
        m_robotControlThread->setCurrentOperationType(PTZ_Move_Left, false);
        break;
    case 2:
        m_robotControlThread->setCurrentOperationType(PTZ_Move_Down, false);
        break;
    case 3:
        m_robotControlThread->setCurrentOperationType(PTZ_Move_Right, false);
        break;
    case 4:
        m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomOut);
        break;
    case 5:
        m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomIn);
        break;
    default:
        break;
    }
}

void DLWheelRobotManager::onRobotPTZControlReleased(int buttonId)
{
    switch (buttonId)
    {
    case 0:
        m_robotControlThread->setCurrentOperationType(PTZ_Move_Up, true);
        break;
    case 1:
        m_robotControlThread->setCurrentOperationType(PTZ_Move_Left, true);
        break;
    case 2:
        m_robotControlThread->setCurrentOperationType(PTZ_Move_Down, true);
        break;
    case 3:
        m_robotControlThread->setCurrentOperationType(PTZ_Move_Right, true);
        break;
    case 4:
        m_visibleVideoWidget->onButtonReleased(VideoButtonType::ZoomOut);
        break;
    case 5:
        m_visibleVideoWidget->onButtonReleased(VideoButtonType::ZoomIn);
        break;
    default:
        break;
    }
}

void DLWheelRobotManager::onFireRobotPTZControlPressed(int buttonId)
{
	switch (buttonId)
	{
	case 0:
		m_robotControlThread->setCurrentOperationType(Fire_PTZ_Move_Up, false);
		break;
	case 1:
		m_robotControlThread->setCurrentOperationType(Fire_PTZ_Move_Left, false);
		break;
	case 2:
		m_robotControlThread->setCurrentOperationType(Fire_PTZ_Move_Down, false);
		break;
	case 3:
		m_robotControlThread->setCurrentOperationType(Fire_PTZ_Move_Right, false);
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

void DLWheelRobotManager::onFireRobotPTZControlReleased(int buttonId)
{
	switch (buttonId)
	{
	case 0:
		m_robotControlThread->setCurrentOperationType(Fire_PTZ_Move_Up, true);
		break;
	case 1:
		m_robotControlThread->setCurrentOperationType(Fire_PTZ_Move_Left, true);
		break;
	case 2:
		m_robotControlThread->setCurrentOperationType(Fire_PTZ_Move_Down, true);
		break;
	case 3:
		m_robotControlThread->setCurrentOperationType(Fire_PTZ_Move_Right, true);
		break;
	case 4:
		m_visibleVideoWidget->onButtonReleased(VideoButtonType::ZoomOut);
		break;
	case 5:
		m_visibleVideoWidget->onButtonReleased(VideoButtonType::ZoomIn);
		break;
	default:
		break;
	}
}

void DLWheelRobotManager::onDeviceAlarmTableDoubleClicked(QTableWidgetItem* item)
{
    if (item != NULL)
    {
        int rowCount = item->row();
        m_currentDeviceAlarmCheckIndex = (m_deviceAlarmInfoPageIndex - 1) * TABLE_PER_PAGE_COUNT + rowCount;
        if (m_currentDeviceAlarmCheckIndex < m_deviceAlarmInfoList.count())
        {
            WheelInspectResultStruct itemData = m_deviceAlarmInfoList[m_currentDeviceAlarmCheckIndex];
            // 虚拟设备暂时不显示设备审核页面(非虚拟设备virtual_name字段为空);
            if (itemData.virtual_name.isEmpty())
            {
                DeviceAlarmSearchStruct data = WHEEL_PATROL_RESULT.getWheelAlarmDataForDeviceTaskUUid(itemData.task_uuid, itemData.device_uuid);
                m_taskCheckWidget->setData(data);
                m_taskCheckWidget->setIsSingleTask(true);
                m_taskCheckWidget->show();
            }
        }
    }
}

void DLWheelRobotManager::onSwitchStateChanged()
{
    bool bUtralsonic, bWiper, bAutoDoor, bChargerArm, bLedLamp, bWarnLamp, bFireExtinguisher;
    bUtralsonic = m_SwitchControl[0]->getSwitchState();
	bWiper = m_SwitchControl[1]->getSwitchState();
 //   bAutoDoor = m_SwitchControl[2]->getSwitchState();
	//bChargerArm = m_SwitchControl[3]->getSwitchState();

	bLedLamp = m_SwitchControl[2]->getSwitchState();
	bWarnLamp = m_SwitchControl[3]->getSwitchState();
	bFireExtinguisher = m_SwitchControl[5]->getSwitchState();
    WHEEL_BACK_TO_CORE_SOCKET.robot_control_device_ctrl_req(bUtralsonic, bWiper, bAutoDoor, bChargerArm, bLedLamp, bWarnLamp, bFireExtinguisher);
}

void DLWheelRobotManager::onFireFuncSwitchStateChanged()
{
}

void DLWheelRobotManager::setRobotControlWidgetVisible(bool isVisible)
{
	m_robotControlWidget->setVisible(isVisible);
}

void DLWheelRobotManager::setRobotMangerComboBoxVisible(bool isVisible)
{
	//m_inputWidgetRobotType->setVisible(isVisible);
	//m_buttonOpenOrCloseFaceRecognition->setVisible(isVisible);
}

void DLWheelRobotManager::setCameraObject(CameraObject* cameraObject)
{
    m_visibleVideoWidget->setCameraObject(cameraObject);
    m_hCNetCameraInterface = m_visibleVideoWidget->getHCNetCameraInterface();
}

// void DLWheelRobotManager::setInfraredObject(InfraredTemperature* infraredTemperature, CameraObject* camerObject)
// {
//     m_infraredVideoWidget->setCameraObject(camerObject);
// //    m_infraredVideoWidget->setInfraredObject(infraredTemperature);
// }

void DLWheelRobotManager::setInfraredObject(void*, CameraObject* camerObject)
{
    m_infraredVideoWidget->setCameraObject(camerObject);
}

void DLWheelRobotManager::setInfraredVisible(bool isVisible)
{
	if (isVisible)
	{
		m_visibleVideoWidget->setFixedHeight(270);
		m_infraredVideoWidget->setFixedHeight(360);
		//m_visibleVideoWidget->hide();
		m_infraredVideoWidget->show();
		//initVideoLayout(true);
	}
	else
	{
		m_visibleVideoWidget->setFixedHeight(620);
		m_infraredVideoWidget->hide();
		//initVideoLayout(false);
	}
}

void DLWheelRobotManager::onUpdatePatrolState(WheelRobotCurrentTaskInfoShow taskData)
{
    m_inputWidgetPatrolTaskName->setShowValue(taskData.task_name);

    m_inputWidgetTaskProperty->setShowValue(taskData.task_property);

    m_inputWidgetPatrolPointCount->setShowValue(QString::number(taskData.total_devices));

    m_inputWidgetAbnormalPatrolPointCount->setShowValue(QString::number(taskData.alarmDeviceCount));

    m_inputWidgetCurrentPatrolPoint->setShowValue(taskData.current_device_name);

    int patrolTime = taskData.predict_duration / 60;
    m_inputWidgetPredictPatrolTime->setShowValue(QString::number(patrolTime) + "min");

    int processPercent = taskData.percent * 100;
    m_inputWidgetPatrolProcess->setShowValue(QString::number(processPercent) + "%");

    m_inputWidgetPatroledPointCount->setShowValue(QString::number(taskData.checked_devices));
}

void DLWheelRobotManager::onReceiveRealTimeInfo(WheelInspectResultStruct inspectResult)
{
    m_realTimeInfoList.push_front(inspectResult);
    
    initRealTimeTableData();

    if (inspectResult.alarm_level_id != DeviceAlarmLevel::Alarm_Normal)
    {
        m_deviceAlarmInfoList.push_front(inspectResult);
        m_unCheckedDeviceAlarmCount++;
        initDeviceAlarmInfoTableData();
    }    

	// 添加报警声音;
    QMediaPlayer* mediaPlayer = new QMediaPlayer;
    QAudioProbe* audioProbe = new QAudioProbe(this);
    audioProbe->setSource(mediaPlayer);
    connect(mediaPlayer, &QMediaPlayer::stateChanged, this, [=](QMediaPlayer::State state) {
        if (state == QMediaPlayer::StoppedState)
        {
            mediaPlayer->deleteLater();
            audioProbe->deleteLater();
        }
    });
    switch (inspectResult.alarm_level_id)
    {
    case Alarm_Waring:
    case Alarm_Common:
    {
        QString strFilePath = QApplication::applicationDirPath() + "/WavFile/AlarmWarnning.WAV";
        mediaPlayer->setMedia(QUrl::fromLocalFile(strFilePath));
        mediaPlayer->play();
    }
        break;
    case Alarm_Serious:
    case Alarm_Dangerous:
    {
        QString strFilePath = QApplication::applicationDirPath() + "/WavFile/AlarmSerious.WAV";
        mediaPlayer->setMedia(QUrl::fromLocalFile(strFilePath));
        mediaPlayer->play();
    }
        break;
    default:
        break;
    }
}  

void DLWheelRobotManager::onSystemWarningCallback(QString strMsg)
{
    if (m_dataShowTabWidget->currentIndex() == 2)
    {
        m_isSystemAlarmtButtonTextRed = false;
    }
    else
    {
        m_isSystemAlarmtButtonTextRed = true;
    }

    QString strDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss ");
    strMsg.prepend(strDateTime);

    m_systemAlarmInfoList.prepend(strMsg);
    m_systemAlarmInfoTable->insertItem(0, strMsg);
    // 最大200条;
    if (m_systemAlarmInfoTable->count() > SYSTEM_ALARM_MAX_COUNT)
    {
        m_systemAlarmInfoList.removeLast();
        m_systemAlarmInfoTable->takeItem(m_systemAlarmInfoTable->count() - 1);
    }
}

void DLWheelRobotManager::init()
{
	m_taskCheckWidget = new DLWheelTaskCheck(this);
	m_taskCheckWidget->setTaskCheckType(DeviceAlarmInfo_Check);
	connect(m_taskCheckWidget, &DLWheelTaskCheck::signalRefreshTable, this, [=] {
		if (m_deviceAlarmInfoList.isEmpty() || m_currentDeviceAlarmCheckIndex >= m_deviceAlarmInfoList.count())
		{
			return;
		}
		m_deviceAlarmInfoList[m_currentDeviceAlarmCheckIndex].alarm_level_id = getAlarmId(m_taskCheckWidget->getData().alarm_level_name);
		m_deviceAlarmInfoList[m_currentDeviceAlarmCheckIndex].is_dealed = true;
		m_unCheckedDeviceAlarmCount--;
		initDeviceAlarmInfoTableData();
	});
}

void DLWheelRobotManager::OpenMap(const QString &strMapFile)
{
	if (QFile::exists(strMapFile))
	{
		m_pMapScene->load_json_map(strMapFile);
		QPointF center_point = m_pMapScene->GetCenter();
		m_pMapWidget->centerOn(center_point);
	}
}
