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

/********�����˿�������*********/

enum RobotControlType
{
    // ����������;
    Robot_Move_Head,
    Robot_Move_Left,
    Robot_Move_Right,
    Robot_Move_Tail,
    // ��̨����; ˫����̨
    PTZ_Move_Up,
    PTZ_Move_Left,
    PTZ_Move_Right,
    PTZ_Move_Down,

	// ��̨����; �����̨
	Fire_PTZ_Move_Up,
	Fire_PTZ_Move_Left,
	Fire_PTZ_Move_Right,
	Fire_PTZ_Move_Down,
	Fire_PTZ_Move_Reset
};

/***********�����˰������Ʋ����߳�***********/

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

    // ���õ�ǰ��������;
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

    // ���õ�ǰ�Ƿ����;
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
    // ��ǰ�ٶȱ���;
    int m_moveSpeedScale;
};

/******������Ŀؼ�(�������״������Ƶȿؼ��ı���Widget)********/

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
	
    // ���ñ���;
	void setTitleText(QString text)
	{
		m_pButtonText->setText(text);
	}

    // ��������Widget;
	void setCenterWidget(QWidget* widget)
	{
		QHBoxLayout* hCenterLayout = new QHBoxLayout(m_centerWidget);
		hCenterLayout->addStretch();
		hCenterLayout->addWidget(widget);
		hCenterLayout->addStretch();
		hCenterLayout->setMargin(5);
	}

private:
    // ���Ʊ߿�;
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

/******* �����˹���/Ѳ����/�����˿��� ҳ��********/

class DLWheelRobotManager : public QWidget
{
	Q_OBJECT

public:
	DLWheelRobotManager(QWidget *parent = NULL);
	~DLWheelRobotManager();

	int GetVisableZoom();

    // ���õ�ǰ������ѡ����Ϣ;
    void setRobotChooseInfo(QString strCurrentChooseRobotName, QStringList strRobotNameList);

	// ���ػ����˿���widget;
	void setRobotControlWidgetVisible(bool isVisible);
	// ���ػ����˹���comboBox;
	void setRobotMangerComboBoxVisible(bool isVisible);

    // ���ÿɼ�����Ƶ��������;
    void setCameraObject(CameraObject* cameraObject);

    // ���ú�����Ƶ��������;
    void setInfraredObject(void*, CameraObject* camerObject);
	void setInfraredVisible(bool isVisible);

    // ����Ѳ��״̬;
    void onUpdatePatrolState(WheelRobotCurrentTaskInfoShow taskData);

    // �յ�ʵʱ��Ϣ;
    void onReceiveRealTimeInfo(WheelInspectResultStruct inspectResult);

    // �յ�ϵͳ�澯��Ϣ;
    void onSystemWarningCallback(QString strMsg);

	// �ؼ���ʼ��;
	void init();

	void OpenMap(const QString &strMapFile);
	// ��ʼ�������Խ��ؼ�;
	void initVoiceTalk();
private:
    // ��ʼ���ؼ�;
	void initWidget();

    // ��ʼ��Ѳ��״̬�ؼ�;
	void initPatrolState();
    // ��ʼ����ͼ�ؼ�;
	void initMapWidget();
    // ��ʼ���ӿɼ���/������Ƶ;
	void initVideoWidget();
	void initVideoLayout(bool show_infrared);

    // ��ʼ��������ʾ�ؼ�;
	void initDataShowWidget();
    // ��ʼ�������˿��ƿؼ�;
	void initRobotControlWidget();
    // ��ʼ�����������Ŀؼ�;
	void initControlBackWidget();
	void initControlBackWidgetPutOutTheFire();

    // ��ʼ�����ؿؼ�;
    void initSwitchWidget();

    // ��ʼ��core�ص�����;
    void initCoreFunction();

    // ��ʼ�����;
    void initTableWidget();

    // ��ʼ���������;
    void initRealTimeTableData();
    void initDeviceAlarmInfoTableData();

    // ϵͳ�澯������ɫˢ��ʱ�ӳ�ʼ��;
    void initSystemAlarmTextRefreshTimer();

    // ��ʼ����Ƶ���Ŵ���;
    void initAudioPlayWindow();

    // �����¼�;s
	void paintEvent(QPaintEvent* event);

    // ���ݸ澯���ͻ�ȡ��Ӧ����;
    QString getAlarmText(DeviceAlarmLevel alarm_level_id);
    // �������ֻ�ȡ��������;
    DeviceAlarmLevel getAlarmId(QString alarm_level_Name);

    // ���ݵ�ǰ��¼�澯״̬��ȡ��ɫ;
    QColor getItemColor(QString strSaskStatus);

signals:
    // ��ʼ�ı�����˵�����;
    void signalStartChangeRobotConnect(int);

	// �л�����ͷ��ʾ;
	void signalChangeCameraShowIndex(int);//1.ͼ������ͷ��2.ǰ������ͷ��3.��������ͷ

    // core�ص�;
    void signalRobotModeStateCallBack(WheelRobotSwitchRunningStatus);

	// ��ͼģ����ʾˢ��
	void signalMapReset(bool);

private slots:
    // �������������;
    void onRobotBodyControlPressed(int buttonId);
    void onRobotBodyControlReleased(int buttonId);

    // ��������̨����;
    void onRobotPTZControlPressed(int buttonId);
    void onRobotPTZControlReleased(int buttonId);

	// �����������̨����;
	void onFireRobotPTZControlPressed(int buttonId);
	void onFireRobotPTZControlReleased(int buttonId);

    // �豸�澯��Ϣtable˫�������������ҳ��;
    void onDeviceAlarmTableDoubleClicked(QTableWidgetItem* item);
    // ���ذ�ť����;
	void onSwitchStateChanged();
	void onFireFuncSwitchStateChanged();

	/// �򿪹ر�����ʶ��ť
	void slotOpenOrCloseFaceRecognition(bool);
	void slotDoFaceRecognition();

	/// �л�����ͷ�����������ˣ�
	void slotCheckFireRobotCamare(bool);
	void slotCheckWheelRobotCamare(bool);	///< �л�����ͷ,���һ�������
private:
	// Ѳ��״̬;
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

	// ��ͼWidget;
	//QWidget* m_mapWidget;
	DLBackStageMapView *m_pMapWidget;
	DLCustomScene *m_pMapScene;

	// �ɼ������widget;
	QVBoxLayout* m_vVideoLayout;
	QWidget* m_videoWidget;
	VideoBackWidget* m_visibleVideoWidget;
	VideoBackWidget* m_infraredVideoWidget;

	// ʵʱ��Ϣ��ʾ;
	QWidget* m_dataShowWidget;
	QTabWidget* m_dataShowTabWidget;
	CustomTableWidget* m_timelyInfoTable;
	CustomTableWidget* m_deviceAlarmInfoTable;
    // ϵͳ�澯��Ϣ;
	QListWidget* m_systemAlarmInfoTable;

	// �����˿���;
	QWidget* m_robotControlWidget;
	QList<QPushButton*> m_controlButtonList;
	QList<QPushButton*> m_ModeButtonList;
    InputWidget* m_currentModeLabel;

	// ��ǰ��Ļ���;
	int m_sreenWidth;

	// ����widget;
	ControlBackWidget* m_radarControlWidget;
	ControlBackWidget* m_bodyControlWidget;
	ControlBackWidget* m_ptzControlWidget;
	ControlBackWidget* m_FireFightingControlWidget;

	/// �������˿���
	//1.������Ʋ���
	ControlBackWidget* m_bodyControlWidgetPutFire;
	CustomButton* m_bodyControlButtonPutFire;

	//2.����������
	QWidget			   *m_fire_robot_control_widget;
	InputWidget		   * m_fire_robot_image_check_widget;
	SwitchWidget	   *m_fire_robot_switch_control[5];//�ֱ��Ӧ�������ܡ�ˮ����ˮ��ɨ�䡢����˨6������

	QPushButton* nButtonFireRobotCheck;
	QPushButton* nButtonWellRobotCheckCamare;

	//3.��̨���Ʋ���
	ControlBackWidget* m_ptzControlWidgetPutFire;
	CustomButton* m_ptzControlButtonPutFir;

    // ���������ߣ���̨���ư�ť;
	CustomButton* m_bodyControlButton;
	CustomButton* m_ptzControlButton;
	CustomButton* m_fireFightingContiotButton;

    // �����˿��Ʋ��������߳�;
    RobotControlThread* m_robotControlThread;

	QStackedWidget* nTabWidgetControl;

    // ����BackWidget��
    QWidget*		m_switchBackWidget;
    SwitchWidget*	m_SwitchControl[6];
	QStackedWidget* m_switch_back_stacked_widget;

    // ��ǰtableҳ��;
    int m_realTimeInfoPageIndex;
    int m_deviceAlarmInfoPageIndex;
    int m_systemAlarmInfoPageIndex;

    // ʵʱ��Ϣtable����;
    QList<WheelInspectResultStruct> m_realTimeInfoList;
    // �豸�澯��Ϣtable����;
    QList<WheelInspectResultStruct> m_deviceAlarmInfoList;
    // ϵͳ�澯��Ϣtable����;
    QList<QString> m_systemAlarmInfoList;

    // ϵͳ�澯������ɫˢ��ʱ��;
    QTimer m_systemAlarmTextRefreshTimer;
    // ϵͳ�澯table�����Ƿ���ɫ;
    bool m_isSystemAlarmtTextRed;
    // ϵͳ�澯��ť�����Ƿ���ɫ;
    bool m_isSystemAlarmtButtonTextRed;

    // �������;
    DLWheelTaskCheck* m_taskCheckWidget;
    // ��ǰ��˵�index;
    int m_currentDeviceAlarmCheckIndex;
    // ��ǰδ��˵��豸�澯��Ϣ����;
    int m_unCheckedDeviceAlarmCount;

    // ��Ƶ�ط�;
    // ��Ƶ���Ŵ���;
    BaseWidget* m_videoBackWindow;
    // ������Ƶ�ؼ�;
    VideoPlayer* m_videoPlayWidget;
    // �������ƶ���;
    HCNetCameraInterface* m_hCNetCameraInterface;

    // ���������Խ�����;
    HCNetCameraInterface* m_hCNetCameraInterfaceForVoiceTalk;
	TalkVoiceCom* m_talkVoiceCom;

	//	����ʶ������
	QTimer *m_faceRecognitionTimer;
	bool bDoFaceRecognition;

};
