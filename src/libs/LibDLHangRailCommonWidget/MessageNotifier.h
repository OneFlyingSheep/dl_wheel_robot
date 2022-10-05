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
	// ��ͼ�豸˫��������ת�����ܱ���ҳ��;
	void signalMapDeviceDoubleClicked(QString areaName);
	// ����Ѳ�����˫��ĳһ��������ת��Ѳ���¼ҳ��;
	void signalIntelligencePatrolTaskTableDoubleClicked(QString strTaskId, QString strDeviceId);
	// ����������Ϣ�����ߴ�������Ϣ֪ͨ��ͼ�����豸�Ƿ���˸;
	void signalNotifyMapAreaDeviceUpdateAlarmState();
	// �������쳣������Ϣ֪ͨ;
	void signalRobotStatus(QList<QString> m_robotAlarmStatus);

	void signalEnviStatus(QList<thresholdEnviAlarm> m_enviAlarmsSignList);

	// ֪ͨ�ɼ�ҳ����б���;
	void signalNotifySetZero();
private:
	static MessageNotifier* m_inst;

	static QMutex m_mutex;
};

