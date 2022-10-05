#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h>

class DLWheelPatrolResultData : public Singleton<DLWheelPatrolResultData>
{
public:
	DLWheelPatrolResultData();
	~DLWheelPatrolResultData();

	//�豸�澯��ѯ_���ݶ�������ѯ
	QList<DeviceAlarmSearchStruct> getWheelDeviceAlarmSearchList(int m_page,int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);
	void getWheelDeviceAlarmSearchPage(int & m_page, int & m_count,int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);

	//�豸�澯��ѯ_����task_uuid��ѯ
	QList<DeviceAlarmSearchStruct> getWheelDeviceAlarmForTaskUUidList(int m_page, int m_pageCount,QString m_task_uuid, QString m_start_time, QString m_stop_time);
	void getWheelDeviceAlarmForTaskUUidPage(int & m_page, int & m_count,int m_pageCount, QString m_task_uuid, QString m_start_time, QString m_stop_time);

	//����device_uuid��task_uuid��ѯ�豸�澯��ѯ
	DeviceAlarmSearchStruct getWheelAlarmDataForDeviceTaskUUid(QString task_uuid, QString device_uuid);

	//�����豸id��ȡ�����б�
	QList<WheelTaskShow> getWheelTaskDataForDeviceUUid(QString deviceUUid = "");

	//�豸�澯��ѯȷ��
	QList<WheelRobortTaskSearchStruct> getDeviceAlarmSearchVerify(int m_page, int m_pageCount);
	void getDeviceAlarmSearchVerifyPage(int &m_page, int &m_count, int m_pageCount);

	//Ѳ�������
	QList<WheelPatrolResultStruct> getWheelPatrolResultList(int m_page, QString m_device_uuid, QString m_start_time, QString m_stop_time);
	void getWheelPatrolResultCount(int &m_page, int &m_count, QString m_device_uuid, QString m_start_time, QString m_stop_time);

	//�豸�Աȷ���
	QList<WheelPatrolResultCompareStruct> getWheelPatrolResultCompareList(int m_page, int m_pageCount, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time);
	void getWheelPatrolResultCompareCount(int &m_page, int &m_count, int m_pageCount, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time);

	//���չʾ
	QList<WheelRobortIntervalShowStruct> getWheelIntervalShow();
	QList<WheelRobortDeviceFromIntervalStruct> getWheelDeviceFromIntervalShow(QString m_equipment_interval_uuid);

	//�澯��ѯ�б�
	QList<WheelRobortAlarmSearchStruct> getWheelRobortAlarmSearchList(int m_page,int m_pageCount, QString m_start_time, QString m_stop_time);
	void getWheelRobortAlarmSearchCount(int &m_page, int &m_count, int m_pageCount, QString m_start_time, QString m_stop_time);

	//ʶ���쳣��λ��ѯ
	QList<DeviceAlarmSearchStruct> getWheelUnusualPointSearchList(int m_page, int m_pageCount, QStringList status);
	void getWheelUnusualPointSearchCount(int & m_page, int & m_count, int m_pageCount, QStringList status);

	//�жϵ�ǰ�����Ƿ�������
	bool isAuditFinish(QString task_uuid);

	//�Զ��屨���ѯ
	QList<QStringList> getCreateExcelAlarmSearch(int m_page, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);
	void getCreateExcelAlarmSearchCount(int & m_page, int & m_count, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);

	//�澯Ƶ�β�ѯ
	int alarmCountSearch(QString device_uuid);
};

#define WHEEL_PATROL_RESULT DLWheelPatrolResultData::GetSingleton()