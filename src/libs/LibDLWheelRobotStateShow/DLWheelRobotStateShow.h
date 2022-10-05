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

/********IP连接状态测试*********/

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

    // 设置当前需要检测的IP地址、Port;
    void setTestIpAndPort(QString strIp, int iPort)
    {
        m_strTestIp = strIp;
        m_iPort = iPort;
    }

    // 停止线程;
    void stopTest()
    {
        m_isStop = true;
        this->wait();
        this->quit();
    }

    // 获取当前连接状态;
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
    // 是否停止;
    bool m_isStop;

    // 保持当前需要检测的IP地址、Port;
    QString m_strTestIp;
    int m_iPort;
	// 当前连接状态;
    bool m_isConnected;
};

/*********机器人状态显示页面**********/

class DLWheelRobotStateShow : public QWidget
{
    Q_OBJECT

public:
	DLWheelRobotStateShow(QWidget* parent = NULL);

	// 设置为开发模式;
	void setDevelopMode();
	// 更新实时信息;
	void onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);
	// 更新非实时信息;
	void onUpdateRobotNoneRealtimeStatus(WheelRobotNoneRealtimeStatus realTimeStatus);

	// 初始化窗口;
	void initWidget();

    // 设置各个通信状态监测IP和Port;
    void setWirelessBaseStationIP(QString strIp, int iPort);
    void setControlSystemIP(QString strIp, int iPort);
    void setVisibleCameraIP(QString strIp, int iPort);
    void setChargeSystemIP(QString strIp, int iPort);
	void setInfraredCameraSIP(QString strIp, int iPort);
	void setFireRobotIP(QString strIp, int iPort);

    // 设置是否显示上传bin文件widget;
    void setIsShowUploadBinWidget(bool isShow);

	//显示可见光倍率
	void SetVisableZoom(int iZoom);

signals:
    // 向主界面发送操作信息;
    void signalSendOperateMsg(QString);

    // 红外显示开关;
    void signalInfraredSwitch(bool isOpen);

    // 可见光显示开关;
    void signalVisibleSwitch(bool isOpen);


public slots:
    // 更新可见光视频放大倍率;
	void onUpdateVisibleVideoParam(int visibleZoomValue);

private:
    // 控件初始化;
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
	
	// 更新云台角度数据时钟;
	void initUpdateRealTimeDataTimer();

    // 初始化通信状态检测;
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

	// 电池百分比;
	BatteryWidget* m_batteryWidget;

	// 用于判断实时数据是否及时更新;
	QTimer m_updateRealTimeDataTimer;

	// 当前是否更新过实时数据;
	bool m_isUpdateRealTimeData;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 通信状态更新时钟;
    QTimer m_communicationStatusUpdateTimer;

    // 通信状态检测;
    CommunicationStatusTest* m_wirelessBaseStationStatus;
    CommunicationStatusTest* m_controlSystemStatus;
    CommunicationStatusTest* m_visibleCameraStatus;
    CommunicationStatusTest* m_chargeSystemStatus;
	CommunicationStatusTest* m_infraredCameraStatus;
	CommunicationStatusTest* m_fireRobotStatus;

    // 多线程访问加锁防止崩溃;
    QMutex m_mutex;

    // 当前充电状态;
    bool m_isCharge;

    // 上传bin文件控件;
    QWidget* m_uploadBinBackWidget;
};

#endif
