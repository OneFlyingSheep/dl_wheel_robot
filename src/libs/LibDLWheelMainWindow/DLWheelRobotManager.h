#pragma once

#include <QWidget>
#include <QPushButton>
#include <QPainter>
#include <QVBoxLayout>
#include <QLabel>
#include <QTabWidget>
#include <QThread>
#include <QListWidget>
#include <QTimer>
#include <QTableWidget>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "DLWheelTalkVoiceCom.h"

class InputWidget;
class VideoBackWidget;
class CustomButton;
class CustomTableWidget;
class CameraObject;
class InfraredTemperature;
class DLBackStageMapView;
class DLCustomScene;
class DLWheelTaskCheck;
class TurnPageWidget;
class VideoPlayer;
class BaseWidget;
class HCNetCameraInterface;
class SwitchWidget;
class QStackedWidget;

/********机器人控制类型*********/

enum RobotControlType
{
    // 机器人行走;
    Robot_Move_Head,
    Robot_Move_Left,
    Robot_Move_Right,
    Robot_Move_Tail,
    // 云台方向; 双光云台
    PTZ_Move_Up,
    PTZ_Move_Left,
    PTZ_Move_Right,
    PTZ_Move_Down,

	// 云台方向; 灭火云台
	Fire_PTZ_Move_Up,
	Fire_PTZ_Move_Left,
	Fire_PTZ_Move_Right,
	Fire_PTZ_Move_Down,
	Fire_PTZ_Move_Reset
};

/***********机器人按键控制操作线程***********/

class RobotControlThread : public QThread
{
public:
    RobotControlThread()
        : m_isOperationStop(true)
        , m_isRelease(false)
        , m_isThreadStop(false)
        , m_moveSpeedScale(1)
    {

    }

    ~RobotControlThread()
    {
        this->wait();
        this->quit();
    }

    void stop()
    {
        m_isThreadStop = true;
    }

    // 设置当前操作类型;
    void setCurrentOperationType(RobotControlType operationType, bool isRelease)
    {
        m_operationType = operationType;
        if (isRelease)
        {
            m_isRelease = true;
        }
        else
        {
            m_isRelease = false;
            m_isOperationStop = false;
        }
    }

    // 设置当前是否加速;
    void setRobotSpeedUp(bool isSpeedUp)
    {
        if (isSpeedUp)
        {
            m_moveSpeedScale = 2;
        }
        else
        {
            m_moveSpeedScale = 1;
        }
    }

private:
    void run()
    {
        while (!m_isThreadStop)
        {
            if (m_isRelease && !m_isOperationStop)
            {
                m_isOperationStop = true;
                switch (m_operationType)
                {
                case Robot_Move_Head:
                case Robot_Move_Left:
                case Robot_Move_Right:
                case Robot_Move_Tail:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, 0);
                    break;

                case PTZ_Move_Up:
                case PTZ_Move_Left:
                case PTZ_Move_Right:
                case PTZ_Move_Down:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_STOP, 0, 0);
                    break;

				case Fire_PTZ_Move_Up:
				case Fire_PTZ_Move_Left:
				case Fire_PTZ_Move_Right:
				case Fire_PTZ_Move_Down:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_STOP, 1, 0);
					break;
                default:
                    break;
                }
            }
            else if (!m_isRelease && !m_isOperationStop)
            {
                switch (m_operationType)
                {
                case Robot_Move_Head:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0.3 * m_moveSpeedScale, 0, 0);
                    break;
                case Robot_Move_Left:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, 0.5 * m_moveSpeedScale);
                    break;
                case Robot_Move_Right:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, -0.5 * m_moveSpeedScale);
                    break;
                case Robot_Move_Tail:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(-0.3 * m_moveSpeedScale, 0, 0);
                    break;

                case PTZ_Move_Up:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_UP, 0, 0.6 * m_moveSpeedScale);
                    break;
                case PTZ_Move_Left:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_LEFT, 0, 0.6 * m_moveSpeedScale);
                    break;
                case PTZ_Move_Right:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_RIGHT, 0, 0.6 * m_moveSpeedScale);
                    break;
                case PTZ_Move_Down:
                    WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_DOWN, 0, 0.6 * m_moveSpeedScale);
                    break;

				case Fire_PTZ_Move_Up:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_UP, 1, 0.6 * m_moveSpeedScale);
					break;
				case Fire_PTZ_Move_Left:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_LEFT, 1, 0.6 * m_moveSpeedScale);
					break;
				case Fire_PTZ_Move_Right:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_RIGHT, 1, 0.6 * m_moveSpeedScale);
					break;
				case Fire_PTZ_Move_Down:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_DOWN, 1, 0.6 * m_moveSpeedScale);
					break;
				case Fire_PTZ_Move_Reset:
					break;

                default:
                    break;
                }
            }
            QThread::msleep(100);
        }
    }
private:
    RobotControlType m_operationType;
    bool m_isOperationStop;
    bool m_isRelease;
    bool m_isThreadStop;
    // 当前速度倍数;
    int m_moveSpeedScale;
};

/******带标题的控件(界面上雷达，车体控制等控件的背景Widget)********/

class ControlBackWidget : public QWidget
{
public:
	ControlBackWidget(QWidget* parent = NULL)
	{
		m_pButtonText = new QPushButton();
		m_pButtonText->setFixedHeight(17);
		m_pButtonText->setStyleSheet("font-weight:bold;border:1px solid rgb(121,134,203);background:rgb(175, 191, 255);");
		m_centerWidget = new QWidget;

		QVBoxLayout* vLayout = new QVBoxLayout(this);
		vLayout->addWidget(m_pButtonText);
		vLayout->addWidget(m_centerWidget);
		vLayout->setMargin(0);
		vLayout->setSpacing(0);

		this->setFixedHeight(170);
	}
	
    // 设置标题;
	void setTitleText(QString text)
	{
		m_pButtonText->setText(text);
	}

    // 设置中心Widget;
	void setCenterWidget(QWidget* widget)
	{
		QHBoxLayout* hCenterLayout = new QHBoxLayout(m_centerWidget);
		hCenterLayout->addStretch();
		hCenterLayout->addWidget(widget);
		hCenterLayout->addStretch();
		hCenterLayout->setMargin(5);
	}

private:
    // 绘制边框;
	void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);
		painter.fillRect(this->rect(), Qt::white);
		painter.setPen(QPen(Qt::lightGray, 1));
		painter.drawRect(QRect(0, 0, this->width() - 1, this->height() - 1));

		return __super::paintEvent(event);
	}

private:
	QPushButton * m_pButtonText;
	QWidget* m_centerWidget;
};

/******* 机器人管理/巡检监控/机器人控制 页面********/

class DLWheelRobotManager : public QWidget
{
	Q_OBJECT

public:
	DLWheelRobotManager(QWidget *parent = NULL);
	~DLWheelRobotManager();

	int GetVisableZoom();

    // 设置当前机器人选择信息;
    void setRobotChooseInfo(QString strCurrentChooseRobotName, QStringList strRobotNameList);

	// 隐藏机器人控制widget;
	void setRobotControlWidgetVisible(bool isVisible);
	// 隐藏机器人管理comboBox;
	void setRobotMangerComboBoxVisible(bool isVisible);

    // 设置可见光视频操作对象;
    void setCameraObject(CameraObject* cameraObject);

    // 设置红外视频操作对象;
    void setInfraredObject(void*, CameraObject* camerObject);
	void setInfraredVisible(bool isVisible);

    // 更新巡检状态;
    void onUpdatePatrolState(WheelRobotCurrentTaskInfoShow taskData);

    // 收到实时信息;
    void onReceiveRealTimeInfo(WheelInspectResultStruct inspectResult);

    // 收到系统告警信息;
    void onSystemWarningCallback(QString strMsg);

	// 控件初始化;
	void init();

	void OpenMap(const QString &strMapFile);
	// 初始化语音对讲控件;
	void initVoiceTalk();
private:
    // 初始化控件;
	void initWidget();

    // 初始化巡检状态控件;
	void initPatrolState();
    // 初始化地图控件;
	void initMapWidget();
    // 初始化视可见光/红外视频;
	void initVideoWidget();
	void initVideoLayout(bool show_infrared);

    // 初始化数据显示控件;
	void initDataShowWidget();
    // 初始化机器人控制控件;
	void initRobotControlWidget();
    // 初始化带标题栏的控件;
	void initControlBackWidget();
	void initControlBackWidgetPutOutTheFire();

    // 初始化开关控件;
    void initSwitchWidget();

    // 初始化core回调函数;
    void initCoreFunction();

    // 初始化表格;
    void initTableWidget();

    // 初始化表格数据;
    void initRealTimeTableData();
    void initDeviceAlarmInfoTableData();

    // 系统告警文字颜色刷新时钟初始化;
    void initSystemAlarmTextRefreshTimer();

    // 初始化音频播放窗口;
    void initAudioPlayWindow();

    // 绘制事件;s
	void paintEvent(QPaintEvent* event);

    // 根据告警类型获取对应文字;
    QString getAlarmText(DeviceAlarmLevel alarm_level_id);
    // 根据文字获取报警类型;
    DeviceAlarmLevel getAlarmId(QString alarm_level_Name);

    // 根据当前记录告警状态获取颜色;
    QColor getItemColor(QString strSaskStatus);

signals:
    // 开始改变机器人的连接;
    void signalStartChangeRobotConnect(int);

	// 切换摄像头显示;
	void signalChangeCameraShowIndex(int);//1.图像摄像头，2.前置摄像头，3.后置摄像头

    // core回调;
    void signalRobotModeStateCallBack(WheelRobotSwitchRunningStatus);

	// 地图模块显示刷新
	void signalMapReset(bool);

private slots:
    // 机器人身体控制;
    void onRobotBodyControlPressed(int buttonId);
    void onRobotBodyControlReleased(int buttonId);

    // 机器人云台控制;
    void onRobotPTZControlPressed(int buttonId);
    void onRobotPTZControlReleased(int buttonId);

	// 机器人灭火云台控制;
	void onFireRobotPTZControlPressed(int buttonId);
	void onFireRobotPTZControlReleased(int buttonId);

    // 设备告警信息table双击弹出任务审核页面;
    void onDeviceAlarmTableDoubleClicked(QTableWidgetItem* item);
    // 开关按钮操作;
	void onSwitchStateChanged();
	void onFireFuncSwitchStateChanged();

	/// 打开关闭人脸识别按钮
	void slotOpenOrCloseFaceRecognition(bool);
	void slotDoFaceRecognition();

	/// 切换摄像头（消防机器人）
	void slotCheckFireRobotCamare(bool);
	void slotCheckWheelRobotCamare(bool);	///< 切换摄像头,查打一体机器人
private:
	// 巡检状态;
	QWidget* m_patrolStateWidget;
	InputWidget* m_inputWidgetRobotType;
	InputWidget* m_inputWidgetPatrolTaskName;
    InputWidget* m_inputWidgetTaskProperty;
	InputWidget* m_inputWidgetPatrolPointCount;
	InputWidget* m_inputWidgetAbnormalPatrolPointCount;
	InputWidget* m_inputWidgetCurrentPatrolPoint;
	InputWidget* m_inputWidgetPredictPatrolTime;
	InputWidget* m_inputWidgetPatrolProcess;
	InputWidget* m_inputWidgetPatroledPointCount;
	QPushButton* m_buttonOpenOrCloseFaceRecognition;

	// 地图Widget;
	//QWidget* m_mapWidget;
	DLBackStageMapView *m_pMapWidget;
	DLCustomScene *m_pMapScene;

	// 可见光红外widget;
	QVBoxLayout* m_vVideoLayout;
	QWidget* m_videoWidget;
	VideoBackWidget* m_visibleVideoWidget;
	VideoBackWidget* m_infraredVideoWidget;

	// 实时信息显示;
	QWidget* m_dataShowWidget;
	QTabWidget* m_dataShowTabWidget;
	CustomTableWidget* m_timelyInfoTable;
	CustomTableWidget* m_deviceAlarmInfoTable;
    // 系统告警信息;
	QListWidget* m_systemAlarmInfoTable;

	// 机器人控制;
	QWidget* m_robotControlWidget;
	QList<QPushButton*> m_controlButtonList;
	QList<QPushButton*> m_ModeButtonList;
    InputWidget* m_currentModeLabel;

	// 当前屏幕宽度;
	int m_sreenWidth;

	// 控制widget;
	ControlBackWidget* m_radarControlWidget;
	ControlBackWidget* m_bodyControlWidget;
	ControlBackWidget* m_ptzControlWidget;
	ControlBackWidget* m_FireFightingControlWidget;

	/// 灭火机器人控制
	//1.车体控制部分
	ControlBackWidget* m_bodyControlWidgetPutFire;
	CustomButton* m_bodyControlButtonPutFire;

	//2.灭火基本功能
	QWidget			   *m_fire_robot_control_widget;
	InputWidget		   * m_fire_robot_image_check_widget;
	SwitchWidget	   *m_fire_robot_switch_control[5];//分别对应：自喷淋、水柱、水雾、扫射、消防栓6个功能

	QPushButton* nButtonFireRobotCheck;
	QPushButton* nButtonWellRobotCheckCamare;

	//3.云台控制部分
	ControlBackWidget* m_ptzControlWidgetPutFire;
	CustomButton* m_ptzControlButtonPutFir;

    // 机器人行走，云台控制按钮;
	CustomButton* m_bodyControlButton;
	CustomButton* m_ptzControlButton;
	CustomButton* m_fireFightingContiotButton;

    // 机器人控制操作命令线程;
    RobotControlThread* m_robotControlThread;

	QStackedWidget* nTabWidgetControl;

    // 开关BackWidget；
    QWidget*		m_switchBackWidget;
    SwitchWidget*	m_SwitchControl[6];
	QStackedWidget* m_switch_back_stacked_widget;

    // 当前table页数;
    int m_realTimeInfoPageIndex;
    int m_deviceAlarmInfoPageIndex;
    int m_systemAlarmInfoPageIndex;

    // 实时信息table数据;
    QList<WheelInspectResultStruct> m_realTimeInfoList;
    // 设备告警信息table数据;
    QList<WheelInspectResultStruct> m_deviceAlarmInfoList;
    // 系统告警信息table数据;
    QList<QString> m_systemAlarmInfoList;

    // 系统告警文字颜色刷新时钟;
    QTimer m_systemAlarmTextRefreshTimer;
    // 系统告警table文字是否变红色;
    bool m_isSystemAlarmtTextRed;
    // 系统告警按钮文字是否变红色;
    bool m_isSystemAlarmtButtonTextRed;

    // 任务审核;
    DLWheelTaskCheck* m_taskCheckWidget;
    // 当前审核的index;
    int m_currentDeviceAlarmCheckIndex;
    // 当前未审核的设备告警信息条数;
    int m_unCheckedDeviceAlarmCount;

    // 音频回放;
    // 视频播放窗口;
    BaseWidget* m_videoBackWindow;
    // 播放视频控件;
    VideoPlayer* m_videoPlayWidget;
    // 海康控制对象;
    HCNetCameraInterface* m_hCNetCameraInterface;

    // 用于语音对讲功能;
    HCNetCameraInterface* m_hCNetCameraInterfaceForVoiceTalk;
	TalkVoiceCom* m_talkVoiceCom;

	//	人脸识别任务
	QTimer *m_faceRecognitionTimer;
	bool bDoFaceRecognition;

};
