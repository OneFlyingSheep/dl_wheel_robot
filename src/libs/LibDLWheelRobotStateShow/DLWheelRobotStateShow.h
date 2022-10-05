#ifndef DL_WHEEL_ROBOT_STATE_SHOW_H
#define DL_WHEEL_ROBOT_STATE_SHOW_H

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QThread>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLWheelCustomWidget/ParseUrl.h"
#include <QMutex>

class BorderWidget;
class InputWidget;
class SwitchWidget;
class BatteryWidget;

/********IP����״̬����*********/

class CommunicationStatusTest : public QThread
{
    Q_OBJECT

public:
    CommunicationStatusTest(QObject *parent = Q_NULLPTR)
        : QThread(parent)
        , m_isStop(false)
        , m_isConnected(false)
        , m_strTestIp("")
    {

    }

    ~CommunicationStatusTest()
    {
        stopTest();
    }

    // ���õ�ǰ��Ҫ����IP��ַ��Port;
    void setTestIpAndPort(QString strIp, int iPort)
    {
        m_strTestIp = strIp;
        m_iPort = iPort;
    }

    // ֹͣ�߳�;
    void stopTest()
    {
        m_isStop = true;
        this->wait();
        this->quit();
    }

    // ��ȡ��ǰ����״̬;
    bool getCurrentConnectStatus()
    {
        return m_isConnected;
    }

private:

    void run()
    {
        while (!m_isStop)
        {
            m_isConnected = ParseUrl::getIpAndPortConnectState(m_strTestIp, m_iPort, 1000);
            QThread::sleep(1);
        }     
    }

private:
    // �Ƿ�ֹͣ;
    bool m_isStop;

    // ���ֵ�ǰ��Ҫ����IP��ַ��Port;
    QString m_strTestIp;
    int m_iPort;
	// ��ǰ����״̬;
    bool m_isConnected;
};

/*********������״̬��ʾҳ��**********/

class DLWheelRobotStateShow : public QWidget
{
    Q_OBJECT

public:
	DLWheelRobotStateShow(QWidget* parent = NULL);

	// ����Ϊ����ģʽ;
	void setDevelopMode();
	// ����ʵʱ��Ϣ;
	void onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);
	// ���·�ʵʱ��Ϣ;
	void onUpdateRobotNoneRealtimeStatus(WheelRobotNoneRealtimeStatus realTimeStatus);

	// ��ʼ������;
	void initWidget();

    // ���ø���ͨ��״̬���IP��Port;
    void setWirelessBaseStationIP(QString strIp, int iPort);
    void setControlSystemIP(QString strIp, int iPort);
    void setVisibleCameraIP(QString strIp, int iPort);
    void setChargeSystemIP(QString strIp, int iPort);
	void setInfraredCameraSIP(QString strIp, int iPort);
	void setFireRobotIP(QString strIp, int iPort);

    // �����Ƿ���ʾ�ϴ�bin�ļ�widget;
    void setIsShowUploadBinWidget(bool isShow);

	//��ʾ�ɼ��ⱶ��
	void SetVisableZoom(int iZoom);

signals:
    // �������淢�Ͳ�����Ϣ;
    void signalSendOperateMsg(QString);

    // ������ʾ����;
    void signalInfraredSwitch(bool isOpen);

    // �ɼ�����ʾ����;
    void signalVisibleSwitch(bool isOpen);


public slots:
    // ���¿ɼ�����Ƶ�Ŵ���;
	void onUpdateVisibleVideoParam(int visibleZoomValue);

private:
    // �ؼ���ʼ��;
	void initRunStateInfo();
	void initCommunicateStateInfo();
	void initBattertyStateInfo();
	void initRobotSelfModeInfo();
	void initLeftWidget();
	void initEnvironmentStateInfo();

	//void initControlStateInfo();

	void initDeviceStateInfo();

	void initRightWidget();
    void initUploadBinWidget();
	
	// ������̨�Ƕ�����ʱ��;
	void initUpdateRealTimeDataTimer();

    // ��ʼ��ͨ��״̬���;
    void initCommunicationStatusTest();

private:
	QWidget* m_leftBackWidget;
	BorderWidget* m_runStateInfoWidget;
	InputWidget* m_bodyTmpWidget;
	InputWidget* m_bodyHuiWidget;
	InputWidget* m_firePtzHorizontalPos;
	InputWidget* m_runSpeed;
	InputWidget* m_firePtzVerticalPos;
	InputWidget* m_cameraFocus;
	InputWidget* m_cameraZoom;
	InputWidget* m_doublePtzHorizontalPos;
	InputWidget* m_doublePtzVerticalPos;


	BorderWidget* m_communitcateStateInfoWidget;
	InputWidget* m_wirelessStation;
	InputWidget* m_controlSystem;
	InputWidget* m_visibleCamera;
	InputWidget* m_chargeSystem;
	InputWidget* m_infreredCamera;
	InputWidget* m_robotConnect;

	BorderWidget* m_batteryStateInfoWidget;
	InputWidget* m_currentBatteryValue;
	QLabel *m_batteryValueValue;

	BorderWidget* m_robotSelfModeInfoWidget;
	InputWidget* m_leftWheel;
	InputWidget* m_rightWheel;
	InputWidget* m_externaSupplyBattery;
	InputWidget* m_chargeState;
	QLabel* m_chargeStateLabel;
	InputWidget* m_chargeValue;

	InputWidget* m_robotDiskUsage;
	InputWidget* m_robotDiskTotal;
	InputWidget* m_runMileage;
	InputWidget* m_runTime;
	InputWidget* m_patrolDeviceCount;
	InputWidget* m_defectFindedCount;

	QWidget* m_rightBackWidget;
	BorderWidget* m_environmentStateInfoWidget;
	InputWidget* m_environmentTmp;
	InputWidget* m_environmentHumidity;
	InputWidget* m_environmentWindSpeed;

	BorderWidget* m_controlStateInfoWidget;
	InputWidget* m_infraredFunction;
	InputWidget* m_visibleFunction;

	InputWidget* m_firePotStatusMajor;
	InputWidget* m_firePotInfoMajor;
	InputWidget* m_firePotStatusAlt;
	InputWidget* m_firePotInfoAlt;
	InputWidget* m_rainBrushState;
	InputWidget* m_avoidObstacleFunction;
	InputWidget* m_carLightState;
	InputWidget* m_chargeRoom;
	InputWidget* m_robotState;

	/*SwitchWidget* m_infraredFunction;
	SwitchWidget* m_visibleFunction;
	SwitchWidget* m_rainBrushState;
	SwitchWidget* m_avoidObstacleFunction;
	SwitchWidget* m_carLightState;
	SwitchWidget* m_chargeRoom;
	SwitchWidget* m_robotState;*/

	// ��ذٷֱ�;
	BatteryWidget* m_batteryWidget;

	// �����ж�ʵʱ�����Ƿ�ʱ����;
	QTimer m_updateRealTimeDataTimer;

	// ��ǰ�Ƿ���¹�ʵʱ����;
	bool m_isUpdateRealTimeData;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ͨ��״̬����ʱ��;
    QTimer m_communicationStatusUpdateTimer;

    // ͨ��״̬���;
    CommunicationStatusTest* m_wirelessBaseStationStatus;
    CommunicationStatusTest* m_controlSystemStatus;
    CommunicationStatusTest* m_visibleCameraStatus;
    CommunicationStatusTest* m_chargeSystemStatus;
	CommunicationStatusTest* m_infraredCameraStatus;
	CommunicationStatusTest* m_fireRobotStatus;

    // ���̷߳��ʼ�����ֹ����;
    QMutex m_mutex;

    // ��ǰ���״̬;
    bool m_isCharge;

    // �ϴ�bin�ļ��ؼ�;
    QWidget* m_uploadBinBackWidget;
};

#endif
