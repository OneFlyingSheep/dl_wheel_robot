#include "DLWheelPatrolResultData.h"
#define TASK_PATROL_COUNT_SHOW	10


DLWheelPatrolResultData::DLWheelPatrolResultData()
{
}


DLWheelPatrolResultData::~DLWheelPatrolResultData()
{
}

QList<DeviceAlarmSearchStruct> DLWheelPatrolResultData::getWheelDeviceAlarmSearchList(int m_page,int m_pageCount, WheelPatrolParameter m_wheelPatrolPara)
{
	QList<DeviceAlarmSearchStruct> m_deviceAlarmSearchStru;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getWheelDeviceAlarmSearchDB(m_count, m_pageCount, m_wheelPatrolPara, m_deviceAlarmSearchStru);
	return m_deviceAlarmSearchStru;
}

void DLWheelPatrolResultData::getWheelDeviceAlarmSearchPage(int &m_page, int &m_count,int m_pageCount, WheelPatrolParameter m_wheelPatrolPara)
{
	QList<DeviceAlarmSearchStruct> m_deviceAlarmSearchStru;
	m_count = -1;
	WHEEL_ROBOT_DB.getWheelDeviceAlarmSearchDB(m_count, m_pageCount, m_wheelPatrolPara, m_deviceAlarmSearchStru);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}

QList<DeviceAlarmSearchStruct> DLWheelPatrolResultData::getWheelDeviceAlarmForTaskUUidList(int m_page, int m_pageCount, QString m_task_uuid, QString m_start_time="", QString m_stop_time="")
{
	QList<DeviceAlarmSearchStruct> m_deviceAlarmSearchStru;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getWheelDeviceAlarmForTaskUUidDB(m_count, m_pageCount, m_task_uuid, m_start_time, m_stop_time,m_deviceAlarmSearchStru);
	return m_deviceAlarmSearchStru;
}

void DLWheelPatrolResultData::getWheelDeviceAlarmForTaskUUidPage(int & m_page, int & m_count, int m_pageCount, QString m_task_uuid, QString m_start_time="", QString m_stop_time="")
{
	m_count = 0;
	WHEEL_ROBOT_DB.getWheelDeviceAlarmForTaskUUidCountDB(m_count, m_task_uuid, m_start_time, m_stop_time);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}

DeviceAlarmSearchStruct DLWheelPatrolResultData::getWheelAlarmDataForDeviceTaskUUid(QString task_uuid, QString device_uuid)
{
	DeviceAlarmSearchStruct stru;
	WHEEL_ROBOT_DB.getWheelAlarmDataForDeviceTaskUUidDB(task_uuid, device_uuid, stru);
	return stru;
}

QList<WheelTaskShow> DLWheelPatrolResultData::getWheelTaskDataForDeviceUUid(QString deviceUUid)
{
	QList<WheelTaskShow> taskData;
	WHEEL_ROBOT_DB.getWheelTaskDataForDeviceUUidDB(deviceUUid, taskData);
	return taskData;
}

QList<WheelRobortTaskSearchStruct> DLWheelPatrolResultData::getDeviceAlarmSearchVerify(int m_page, int m_pageCount)
{
	QList<WheelRobortTaskSearchStruct> m_wheelRobortTaskSearchStru;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getWheelDeviceAlarmSearchVerifyDB(m_count, m_pageCount, m_wheelRobortTaskSearchStru);
	return m_wheelRobortTaskSearchStru;
}

void DLWheelPatrolResultData::getDeviceAlarmSearchVerifyPage(int &m_page, int &m_count, int m_pageCount)
{
	m_count = 0;
	WHEEL_ROBOT_DB.getWheelDeviceAlarmSearchVerifyCountDB(m_count);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}
//.///////////////////////////////////////////////////////////////////////////
QList<WheelPatrolResultStruct> DLWheelPatrolResultData::getWheelPatrolResultList(int m_page, QString m_device_uuid, QString m_start_time, QString m_stop_time)
{
	QList<WheelPatrolResultStruct> m_wheelPatrolResultStru;
	int m_count = (m_page - 1) * TASK_PATROL_COUNT_SHOW;
	WHEEL_ROBOT_DB.getWheelPatrolResultDB(m_count, TASK_PATROL_COUNT_SHOW, m_wheelPatrolResultStru, m_device_uuid, m_start_time, m_stop_time);
	return m_wheelPatrolResultStru;
}

void DLWheelPatrolResultData::getWheelPatrolResultCount(int &m_page, int &m_count, QString m_device_uuid, QString m_start_time, QString m_stop_time)
{
	WHEEL_ROBOT_DB.getWheelPatrolResultCountDB(m_count, m_device_uuid, m_start_time, m_stop_time);
	m_page = m_count / TASK_PATROL_COUNT_SHOW;
	if (m_count % TASK_PATROL_COUNT_SHOW != 0)
	{
		m_page++;
	}
}
//.///////////////////////////////////////////////////////////////////////////
QList<WheelPatrolResultCompareStruct> DLWheelPatrolResultData::getWheelPatrolResultCompareList(int m_page, int m_pageCount, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time)
{
	QList<WheelPatrolResultCompareStruct> m_wheelPatrolResultCompareStru;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getWheelPatrolResultCompareDB(m_count, m_pageCount, m_wheelPatrolResultCompareStru, m_device_uuid, m_save_type_id, m_start_time, m_stop_time);
	return m_wheelPatrolResultCompareStru;
}

void DLWheelPatrolResultData::getWheelPatrolResultCompareCount(int & m_page, int & m_count, int m_pageCount, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time)
{
	WHEEL_ROBOT_DB.getWheelPatrolResultCompareCountDB(m_count, m_device_uuid, m_save_type_id, m_start_time, m_stop_time);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}
//.////////////////////////////////////////////////////////////////////////.//
QList<WheelRobortIntervalShowStruct> DLWheelPatrolResultData::getWheelIntervalShow()
{
	QList<WheelRobortIntervalShowStruct> m_wheelRobortIntervalShowStru;
	WHEEL_ROBOT_DB.getWheelIntervalShowDB(m_wheelRobortIntervalShowStru);
	return m_wheelRobortIntervalShowStru;
}

QList<WheelRobortDeviceFromIntervalStruct> DLWheelPatrolResultData::getWheelDeviceFromIntervalShow(QString m_equipment_interval_uuid)
{
	QList<WheelRobortDeviceFromIntervalStruct> m_wrDeviceFromIntervalStru;
	WHEEL_ROBOT_DB.getWheelDeviceFromIntervalShowDB(m_equipment_interval_uuid, m_wrDeviceFromIntervalStru);
	return m_wrDeviceFromIntervalStru;
}
//.////////////////////////////////////////////////////////////////////////.//
QList<WheelRobortAlarmSearchStruct> DLWheelPatrolResultData::getWheelRobortAlarmSearchList(int m_page, int m_pageCount, QString m_start_time, QString m_stop_time)
{
	QList<WheelRobortAlarmSearchStruct> m_alarmSearchStru;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getWheelRobortAlarmSearchDB(m_count, m_pageCount, m_alarmSearchStru , m_start_time, m_stop_time);
	return m_alarmSearchStru;
}

void DLWheelPatrolResultData::getWheelRobortAlarmSearchCount(int & m_page, int & m_count, int m_pageCount, QString m_start_time, QString m_stop_time)
{
	WHEEL_ROBOT_DB.getWheelRobortAlarmSearchCountDB(m_count, m_start_time, m_stop_time);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}
//.////////////////////////////////////////////////////////////////////////.//
QList<DeviceAlarmSearchStruct> DLWheelPatrolResultData::getWheelUnusualPointSearchList(int m_page, int m_pageCount,QStringList status)
{
	QList<DeviceAlarmSearchStruct> m_UnusualStru;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getWheelUnusualPointSearchListDB(m_count, m_pageCount, m_UnusualStru, status);
	return m_UnusualStru;
}

void DLWheelPatrolResultData::getWheelUnusualPointSearchCount(int & m_page, int & m_count, int m_pageCount, QStringList status)
{
	m_count = 0;
	WHEEL_ROBOT_DB.getWheelUnusualPointSearchCountDB(m_count,status);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}
//.////////////////////////////////////////////////////////////////////////.//
bool DLWheelPatrolResultData::isAuditFinish(QString task_uuid)
{
	return WHEEL_ROBOT_DB.isAuditFinishDB(task_uuid);
}
//.////////////////////////////////////////////////////////////////////////.//
QList<QStringList> DLWheelPatrolResultData::getCreateExcelAlarmSearch(int m_page, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara)
{
	int icount = 0;
	QList<QStringList> c_data;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getCreateExcelAlarmSearchDB(1, m_count, m_pageCount, icount, c_data, m_wheelPatrolPara);
	return c_data;
}
void DLWheelPatrolResultData::getCreateExcelAlarmSearchCount(int & m_page, int & m_count, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara)
{
	m_count = 0;
	QList<QStringList> c_data;
	WHEEL_ROBOT_DB.getCreateExcelAlarmSearchDB(0, 0, 0, m_count, c_data, m_wheelPatrolPara);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}

//.////////////////////////////////////////////////////////////////////////.//

int DLWheelPatrolResultData::alarmCountSearch(QString device_uuid)
{
	int count = 0;
	WHEEL_ROBOT_DB.getAlarmCountSearchDB(device_uuid, count);
	return count;
}


