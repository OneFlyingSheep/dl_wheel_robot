#pragma once

#include <QObject>
#include <QMutex>
#include "common/DLHangRailRobotGlobalDef.hpp"
class MessageNotifier : public QObject
{
	Q_OBJECT

public:
	static MessageNotifier* getInstance();

private:
	MessageNotifier();
	~MessageNotifier();

signals:
	// 地图设备双击进行跳转到智能报警页面;
	void signalMapDeviceDoubleClicked(QString areaName);
	// 智能巡检鼠标双击某一条任务，跳转到巡检记录页面;
	void signalIntelligencePatrolTaskTableDoubleClicked(QString strTaskId, QString strDeviceId);
	// 新增报警信息，或者处理报警信息通知地图区域设备是否闪烁;
	void signalNotifyMapAreaDeviceUpdateAlarmState();
	// 机器人异常报警信息通知;
	void signalRobotStatus(QList<QString> m_robotAlarmStatus);

	void signalEnviStatus(QList<thresholdEnviAlarm> m_enviAlarmsSignList);

	// 通知采集页面进行标零;
	void signalNotifySetZero();
private:
	static MessageNotifier* m_inst;

	static QMutex m_mutex;
};

