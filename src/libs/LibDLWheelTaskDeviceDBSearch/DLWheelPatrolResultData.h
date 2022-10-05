#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h>

class DLWheelPatrolResultData : public Singleton<DLWheelPatrolResultData>
{
public:
	DLWheelPatrolResultData();
	~DLWheelPatrolResultData();

	//设备告警查询_根据多条件查询
	QList<DeviceAlarmSearchStruct> getWheelDeviceAlarmSearchList(int m_page,int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);
	void getWheelDeviceAlarmSearchPage(int & m_page, int & m_count,int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);

	//设备告警查询_根据task_uuid查询
	QList<DeviceAlarmSearchStruct> getWheelDeviceAlarmForTaskUUidList(int m_page, int m_pageCount,QString m_task_uuid, QString m_start_time, QString m_stop_time);
	void getWheelDeviceAlarmForTaskUUidPage(int & m_page, int & m_count,int m_pageCount, QString m_task_uuid, QString m_start_time, QString m_stop_time);

	//根据device_uuid和task_uuid查询设备告警查询
	DeviceAlarmSearchStruct getWheelAlarmDataForDeviceTaskUUid(QString task_uuid, QString device_uuid);

	//根据设备id获取任务列表
	QList<WheelTaskShow> getWheelTaskDataForDeviceUUid(QString deviceUUid = "");

	//设备告警查询确认
	QList<WheelRobortTaskSearchStruct> getDeviceAlarmSearchVerify(int m_page, int m_pageCount);
	void getDeviceAlarmSearchVerifyPage(int &m_page, int &m_count, int m_pageCount);

	//巡检结果浏览
	QList<WheelPatrolResultStruct> getWheelPatrolResultList(int m_page, QString m_device_uuid, QString m_start_time, QString m_stop_time);
	void getWheelPatrolResultCount(int &m_page, int &m_count, QString m_device_uuid, QString m_start_time, QString m_stop_time);

	//设备对比分析
	QList<WheelPatrolResultCompareStruct> getWheelPatrolResultCompareList(int m_page, int m_pageCount, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time);
	void getWheelPatrolResultCompareCount(int &m_page, int &m_count, int m_pageCount, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time);

	//间隔展示
	QList<WheelRobortIntervalShowStruct> getWheelIntervalShow();
	QList<WheelRobortDeviceFromIntervalStruct> getWheelDeviceFromIntervalShow(QString m_equipment_interval_uuid);

	//告警查询列表
	QList<WheelRobortAlarmSearchStruct> getWheelRobortAlarmSearchList(int m_page,int m_pageCount, QString m_start_time, QString m_stop_time);
	void getWheelRobortAlarmSearchCount(int &m_page, int &m_count, int m_pageCount, QString m_start_time, QString m_stop_time);

	//识别异常点位查询
	QList<DeviceAlarmSearchStruct> getWheelUnusualPointSearchList(int m_page, int m_pageCount, QStringList status);
	void getWheelUnusualPointSearchCount(int & m_page, int & m_count, int m_pageCount, QStringList status);

	//判断当前任务是否审核完成
	bool isAuditFinish(QString task_uuid);

	//自定义报表查询
	QList<QStringList> getCreateExcelAlarmSearch(int m_page, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);
	void getCreateExcelAlarmSearchCount(int & m_page, int & m_count, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);

	//告警频次查询
	int alarmCountSearch(QString device_uuid);
};

#define WHEEL_PATROL_RESULT DLWheelPatrolResultData::GetSingleton()