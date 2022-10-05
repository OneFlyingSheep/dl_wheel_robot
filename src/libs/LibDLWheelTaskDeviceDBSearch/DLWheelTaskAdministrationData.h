#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h>
#include <LibDLWheelRobotConfigData/DLWheelRobotTaskConfig.h>

class DLWheelTaskAdministrationData : public Singleton<DLWheelTaskAdministrationData>
{
public:
	DLWheelTaskAdministrationData();
	~DLWheelTaskAdministrationData();

	//获取check_box
	QList<WheelTaskAdminCheckBoxStruct> getTaskAdminCheckBoxData(int m_taskRunType);

	void setWheelDeviceAreaCheckBox(bool m_judge);
	void setWheelDeviceTypeCheckBox(bool m_judge, QStringList m_deviceType);
	void setWheelRecognitionTypeCheckBox(bool m_judge, bool m_judge_2, QStringList m_recognitionType);
	void setWheelMeterTypeCheckBox(bool m_judge, bool m_judge_2, QStringList m_meterType);
	void setWheelAlarmLevelCheckBox(bool m_judge, bool m_judge_2);
	void setWheelTaskStatusCheckBox(bool m_judge, bool m_judge_2);
	void setWheelDeviceFacadeCheckBox(bool m_judge, bool m_judge_2);

	
	QList<QString> getDeviceUUidForCheckBoxData(int checkBoxTypeEnum, QString checkBoxTypeUUid);

	//任务编制列表获取
	QList<WheelTaskEditStruct> getWheelTaskEditList(int m_page, int m_pageCount, WheelTaskAdminType m_task_edit_type_id);
	void getWheelTaskEditPage(int &m_page, int &m_count, int m_pageCount, WheelTaskAdminType m_task_edit_type_id);
	//根据任务编制返回设备列表
	QList<QString> getWheelTaskEditDeviceList(QString m_task_edit_uuid);
	//根据任务编制uuid返回编制结构体
	WheelTaskEditStruct getTaskEditStruForTaskTemplateUUid(QString m_task_edit_uuid);

	//任务列表获取，拆分组合
	QList<WheelTaskListShowStruct> getTaskListShowStru(WheelTaskListSearchIndex m_index);
	void getWheelTaskListPage(int & m_page, int & m_count, int m_pageCount, WheelTaskListSearchIndex m_index);
	QList<WheelTaskListShowStruct> getTaskListShow(int m_page, int m_pageCount);

	//设备uuid和编制类型
	void getDeviceUUidMapAndEditType(QMap<QString, QString> &m_deviceMap, WheelTaskAdminType &m_taskAdminType, WheelTaskEditStruct &m_taskEditStruct, QString m_edit_uuid);

	//获取设备uuid
	QStringList getDeviceUUidFromTaskDevicesList(QString m_edit_uuid);
	
	//日历数据
	QMap<int, QList<WheelCalendarData>> getCalendarTaskMap(QString m_start_time,QString m_end_time);

	//任务列表_任务的uuid及名字
	QList<WheelTaskShow> getWheelTaskListShow(QString m_start_time, QString m_end_time);
	//返回任务结构体
	WheelTaskShow getWheelTaskStru(QString task_uuid);

	//查询历史完成的任务
	QList<WheelTaskEditStruct> getWheelTaskHistoryStru(QString start_time, QString end_time, QString task_name);

	//巡检报告任务
	QList<WheelCreateReport> getCreateReportStru(QString start_time, QString end_time);

private:
	QList<WheelTaskAdminCheckBoxStruct> m_wheelTaskAdminCheckBoxStru;
	QList<WheelTaskListShowStruct> m_TaskListShowStru;
};

#define WHEEL_TASK_ADMINISTRATION DLWheelTaskAdministrationData::GetSingleton()
