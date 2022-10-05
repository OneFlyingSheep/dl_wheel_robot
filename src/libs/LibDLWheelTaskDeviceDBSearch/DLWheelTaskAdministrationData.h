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

	//��ȡcheck_box
	QList<WheelTaskAdminCheckBoxStruct> getTaskAdminCheckBoxData(int m_taskRunType);

	void setWheelDeviceAreaCheckBox(bool m_judge);
	void setWheelDeviceTypeCheckBox(bool m_judge, QStringList m_deviceType);
	void setWheelRecognitionTypeCheckBox(bool m_judge, bool m_judge_2, QStringList m_recognitionType);
	void setWheelMeterTypeCheckBox(bool m_judge, bool m_judge_2, QStringList m_meterType);
	void setWheelAlarmLevelCheckBox(bool m_judge, bool m_judge_2);
	void setWheelTaskStatusCheckBox(bool m_judge, bool m_judge_2);
	void setWheelDeviceFacadeCheckBox(bool m_judge, bool m_judge_2);

	
	QList<QString> getDeviceUUidForCheckBoxData(int checkBoxTypeEnum, QString checkBoxTypeUUid);

	//��������б��ȡ
	QList<WheelTaskEditStruct> getWheelTaskEditList(int m_page, int m_pageCount, WheelTaskAdminType m_task_edit_type_id);
	void getWheelTaskEditPage(int &m_page, int &m_count, int m_pageCount, WheelTaskAdminType m_task_edit_type_id);
	//����������Ʒ����豸�б�
	QList<QString> getWheelTaskEditDeviceList(QString m_task_edit_uuid);
	//�����������uuid���ر��ƽṹ��
	WheelTaskEditStruct getTaskEditStruForTaskTemplateUUid(QString m_task_edit_uuid);

	//�����б��ȡ��������
	QList<WheelTaskListShowStruct> getTaskListShowStru(WheelTaskListSearchIndex m_index);
	void getWheelTaskListPage(int & m_page, int & m_count, int m_pageCount, WheelTaskListSearchIndex m_index);
	QList<WheelTaskListShowStruct> getTaskListShow(int m_page, int m_pageCount);

	//�豸uuid�ͱ�������
	void getDeviceUUidMapAndEditType(QMap<QString, QString> &m_deviceMap, WheelTaskAdminType &m_taskAdminType, WheelTaskEditStruct &m_taskEditStruct, QString m_edit_uuid);

	//��ȡ�豸uuid
	QStringList getDeviceUUidFromTaskDevicesList(QString m_edit_uuid);
	
	//��������
	QMap<int, QList<WheelCalendarData>> getCalendarTaskMap(QString m_start_time,QString m_end_time);

	//�����б�_�����uuid������
	QList<WheelTaskShow> getWheelTaskListShow(QString m_start_time, QString m_end_time);
	//��������ṹ��
	WheelTaskShow getWheelTaskStru(QString task_uuid);

	//��ѯ��ʷ��ɵ�����
	QList<WheelTaskEditStruct> getWheelTaskHistoryStru(QString start_time, QString end_time, QString task_name);

	//Ѳ�챨������
	QList<WheelCreateReport> getCreateReportStru(QString start_time, QString end_time);

private:
	QList<WheelTaskAdminCheckBoxStruct> m_wheelTaskAdminCheckBoxStru;
	QList<WheelTaskListShowStruct> m_TaskListShowStru;
};

#define WHEEL_TASK_ADMINISTRATION DLWheelTaskAdministrationData::GetSingleton()
