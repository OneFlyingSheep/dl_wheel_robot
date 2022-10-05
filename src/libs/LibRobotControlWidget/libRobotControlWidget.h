#pragma once
/// 机器人控制界面

#include <QImage>
#include <stdio.h>
#include <atlbase.h>
#include <atlcom.h>
#include <sapi.h>
#include <QWidget>
#include <QString>
#include <QStringList>
#include <QGroupBox>
#include <QCoreApplication>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkRequest>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkInterface>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTimer>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"

#include "systeminformation.h"
#include "firehydrantinformation.h"
#include "chargingroominformation.h"
#include "InspectionRobotControlClass.h"
#include "firefightingrobotinformation.h"

#pragma execution_character_set("utf-8")

class SystemInformation;			///< 系统信息
class ChargingRoomInformation;		///< 充电房信息
class FireHydrantInformation;		///< 消防栓信息
class InspectionRobotControlClass;	///< 查打一体机器人信息
class FirefightingRobotInformation;	///< 消防机器人信息


/// 机器人信息显示以及控制界面

class LibRobotControlWidget : public QWidget
{
	Q_OBJECT

public:
	LibRobotControlWidget(QWidget *parent = NULL);
	~LibRobotControlWidget();

	void initWidget();
	QString getCurrIPNumber();
public slots:
	//void replyFinished(QNetworkReply*);
	void slotUpdataDataInfo();

	/// 系统信息相关槽函数
	void slotDiskSpaceforSystem(int pSpaceRemarning, int pSpaceTotal);	///< 当前系统磁盘空间
	void slotSignalStrengthforSystem(int pSignalStrength);	///< 信号强度

	/// 消防栓信息相关槽函数
	void slotValveSwitchforFireHydrant(bool pSwitch);	///< 阀门开关
	void slotValvePressureforFireHydrant(int Pressure);	///< 阀门压力...压强

	/// 充电房信息相关槽函数
	void slotSignalStrengthforChargingRoom(int nSignalStrength);	///< 充电房信号强度
	void slotChargingSwitchStateforChargingRoom(bool nChargingSwitchState);	///< 充电开关状态
	void slotWorkingStateforChargingRoom(int nWorkingState);

	/// 查打一体机器人相关槽函数
	void slotSetInfraredFocus(int);	///< 云台位置
	void slotSetInfraredQuantityOfElectricity(int value); ///< 电量


private:
	/// 系统信息界面
	SystemInformation* m_systemInformationWidget;
	/// 充电房信息界面
	ChargingRoomInformation* m_chargingRoomInformationWidget;
	/// 智能消防栓信息界面
	FireHydrantInformation* m_fireHydrantInformationWidget;
	/// 巡检、查打一体机器人控制界面
	InspectionRobotControlClass* m_inspectionRobotConClassWidget;
	/// 灭火机器人控制界面
	FirefightingRobotInformation* m_fireFightingRobotInfoWidget;


	/// 获取天气信息使用变量
	QNetworkAccessManager *manager;  //请求句柄
	QString fengli;       //风力
	QString wendu;        //温度
	QString weather_type;  //天气类型
	QTimer m_updataTimer;

	/// 双光云台信息设置
	int m_focusValue, m_panValue;
};
