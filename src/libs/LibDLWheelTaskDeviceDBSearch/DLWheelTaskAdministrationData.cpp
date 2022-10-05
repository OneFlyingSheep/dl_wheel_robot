#include "DLWheelTaskAdministrationData.h"
#include "LibDLWheelCountDateTime/CountDateTime.h"
#include <QDebug>
#include <QList>
#include <QtAlgorithms>
#define TASK_PATROL_COUNT_SHOW	10

bool CapitySort(const WheelTaskListShowStruct & msVideoFirst, const WheelTaskListShowStruct & msVideoSecond)
{
	return (QDateTime::fromString(msVideoFirst.task_start_time, "yyyy-MM-dd hh:mm:ss") < QDateTime::fromString(msVideoSecond.task_start_time, "yyyy-MM-dd hh:mm:ss"));
}

DLWheelTaskAdministrationData::DLWheelTaskAdministrationData()
{
}

DLWheelTaskAdministrationData::~DLWheelTaskAdministrationData()
{
}

QList<WheelTaskAdminCheckBoxStruct> DLWheelTaskAdministrationData::getTaskAdminCheckBoxData(int m_taskRunType)
{
	QStringList m_deviceType;
	QStringList m_recognitionType;
	QStringList m_meterType;
	m_wheelTaskAdminCheckBoxStru.clear();
	switch (m_taskRunType)
	{
	case WHEEL_TOTAL_PATROL:
		setWheelDeviceAreaCheckBox(true);
		setWheelDeviceTypeCheckBox(true, m_deviceType);
		setWheelRecognitionTypeCheckBox(true, false, m_recognitionType);
		setWheelMeterTypeCheckBox(true, true, m_meterType);
		break;

	case WHEEL_ROUTINE_PATROL:
		setWheelDeviceAreaCheckBox(true);
		setWheelDeviceTypeCheckBox(true, m_deviceType);
		m_recognitionType << "表计读数"<<"位置状态识别"<<"设备外观查看"<<"声音检测";
		setWheelRecognitionTypeCheckBox(false, false, m_recognitionType);
		setWheelMeterTypeCheckBox(true, true, m_meterType);
		break;

	case WHEEL_INFRARED_THERMOMETRY:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		m_recognitionType << "红外测温";
		setWheelRecognitionTypeCheckBox(false, false, m_recognitionType);
		break;

	case WHEEL_OIL_LEVEL_THERMOMETRY:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		m_recognitionType << "表计读取";
		setWheelRecognitionTypeCheckBox(false, false, m_recognitionType);
		m_meterType << "油位表" << "油温表";
		setWheelMeterTypeCheckBox(false, false, m_meterType);
		break;

	case WHEEL_ARRESTER_METER_COPY:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		m_recognitionType << "表计读取";
		setWheelRecognitionTypeCheckBox(false, false, m_recognitionType);
		m_meterType << "避雷器动作次数表" << "泄露电流表";
		setWheelMeterTypeCheckBox(false, false, m_meterType);
		break;

	case WHEEL_SF6_PRESSURE_GAGE:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		m_recognitionType << "表计读取";
		setWheelRecognitionTypeCheckBox(false, false, m_recognitionType);
		m_meterType << "SF6压力表";
		setWheelMeterTypeCheckBox(false, false, m_meterType);
		break;

	case WHEEL_HYDRAULIC_PRESSURE_GAUGE:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		m_recognitionType << "表计读取";
		setWheelRecognitionTypeCheckBox(false, false, m_recognitionType);
		m_meterType << "液压表";
		setWheelMeterTypeCheckBox(false, false, m_meterType);
		break;

	case WHEEL_LOCATION_STATE_DISCERN:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		break;

	case WHEEL_SCURVINESS_WEATHER_PAROL:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);
		break;

	case WHEEL_FLAW_TAIL_AFTER:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);
		break;

	case WHEEL_DISTANT_ABNORMAL_ALARM:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);
		break;

	case WHEEL_DISTANT_STATE_AFFIRM:
		setWheelDeviceAreaCheckBox(false);
		m_deviceType << "断路器"<<"隔离开关"<<"开关柜";
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);
		break;

	case WHEEL_SECURITY_LINKAGE:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);
		break;

	case WHEEL_ASSIST_ACCIDENT_DISPOSE:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);
		break;

	case WHEEL_USER_DEFINED_TASK:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);
		break;

	case WHEEL_TASK_SHOW:
		setWheelTaskStatusCheckBox(false, true);
		break;

	case WHEEL_DEVICE_ALARM_MES:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelAlarmLevelCheckBox(false, true);
		break;

	case WHEEL_CREATE_EXCEL_REPORT:
		setWheelDeviceAreaCheckBox(false);
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		setWheelRecognitionTypeCheckBox(false, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);
		setWheelDeviceFacadeCheckBox(false, false);
		break;

	case WHEEL_STANDARD_PATROL_VINDICATE:
		setWheelDeviceTypeCheckBox(false, m_deviceType);
		break;

	case WHEEL_PATROL_POINT_SET:
		setWheelDeviceAreaCheckBox(true);
		setWheelDeviceTypeCheckBox(true, m_deviceType);
		setWheelRecognitionTypeCheckBox(true, true, m_recognitionType);
		setWheelMeterTypeCheckBox(false, true, m_meterType);

	default:
		break;
	}
	return m_wheelTaskAdminCheckBoxStru;
}

void DLWheelTaskAdministrationData::setWheelDeviceAreaCheckBox(bool m_judge)
{
	WheelTaskAdminCheckBoxStruct c_dataStruct;
	std::map<QString, QString> c_deviceArea = WHEEL_DEVICE_CONFIG.getWheelDeviceAreaDataMap();
	WheelPackageCheckBoxStruct c_packageCheckBox;

	c_dataStruct.CheckBoxTypeEnum = WHEEL_DEVICE_AREA_NAME;
	c_dataStruct.typeName = "设备区域";

	std::map<QString, QString>::iterator iter;
	for (iter = c_deviceArea.begin(); iter != c_deviceArea.end(); iter++)
	{
		c_packageCheckBox.checkbox_uuid = iter->first;
		c_packageCheckBox.checkbox_name = iter->second;
		c_packageCheckBox.is_enabled = true;
		if (m_judge)
		{
			c_packageCheckBox.is_check = true;
		}
		else
		{
			c_packageCheckBox.is_check = false;
		}

		c_dataStruct.PackageCheckBoxStru.append(c_packageCheckBox);
	}
	m_wheelTaskAdminCheckBoxStru.append(c_dataStruct);
}

void DLWheelTaskAdministrationData::setWheelDeviceTypeCheckBox(bool m_judge, QStringList m_deviceType)
{
	WheelTaskAdminCheckBoxStruct c_dataStruct;
	std::map<QString, QString> c_deviceType = WHEEL_DEVICE_CONFIG.getWheelDeviceTypeDataMap();
	WheelPackageCheckBoxStruct c_packageCheckBox;

	c_dataStruct.CheckBoxTypeEnum = WHEEL_DEVICE_TYPE;
	c_dataStruct.typeName = "设备类型";

	std::map<QString, QString>::iterator iter;
	for (iter = c_deviceType.begin(); iter != c_deviceType.end(); iter++)
	{
		c_packageCheckBox.checkbox_uuid = iter->first;
		c_packageCheckBox.checkbox_name = iter->second;
		c_packageCheckBox.is_enabled = true;
		if (m_judge)
		{
			c_packageCheckBox.is_check = true;
		}
		else
		{
			c_packageCheckBox.is_check = false;
			for(int i = 0; i < m_deviceType.size(); i++)
			{
				if (iter->second == m_deviceType.at(i))
				{
					c_packageCheckBox.is_check = true;
				}
			}
		}
		c_dataStruct.PackageCheckBoxStru.append(c_packageCheckBox);
	}
	m_wheelTaskAdminCheckBoxStru.append(c_dataStruct);
}

void DLWheelTaskAdministrationData::setWheelRecognitionTypeCheckBox(bool m_judge, bool m_judge_2, QStringList m_recognitionType)
{
	WheelTaskAdminCheckBoxStruct c_dataStruct;
	std::map<int, QString> c_recognitionType = WHEEL_DEVICE_CONFIG.getWheelRecognitionTypeDataMap();
	WheelPackageCheckBoxStruct c_packageCheckBox;

	c_dataStruct.CheckBoxTypeEnum = WHEEL_RECOGNITION_TYPE;
	c_dataStruct.typeName = "识别类型";

	std::map<int, QString>::iterator iter;
	for (iter = c_recognitionType.begin(); iter != c_recognitionType.end(); iter++)
	{
		c_packageCheckBox.checkbox_uuid = QString("%1").arg(iter->first);
		c_packageCheckBox.checkbox_name = iter->second;
		if (m_judge)
		{
			c_packageCheckBox.is_enabled = true;
			c_packageCheckBox.is_check = true;
		}
		else
		{
			if (m_judge_2)
			{
				c_packageCheckBox.is_enabled = true;
				c_packageCheckBox.is_check = false;
			}
			else
			{
				c_packageCheckBox.is_enabled = false;
				c_packageCheckBox.is_check = false;
				for (int i = 0; i < m_recognitionType.size(); i++)
				{
					if (iter->second == m_recognitionType.at(i))
					{
						c_packageCheckBox.is_enabled = true;
						c_packageCheckBox.is_check = true;
					}
				}
			}
		}
		c_dataStruct.PackageCheckBoxStru.append(c_packageCheckBox);
	}
	m_wheelTaskAdminCheckBoxStru.append(c_dataStruct);
}

void DLWheelTaskAdministrationData::setWheelMeterTypeCheckBox(bool m_judge, bool m_judge_2, QStringList m_meterType)
{
	WheelTaskAdminCheckBoxStruct c_dataStruct;
	std::map<int, WheelRobortMeterTypeStruct> c_meterType = WHEEL_DEVICE_CONFIG.getWheelMeterTypeDataMap();
	WheelPackageCheckBoxStruct c_packageCheckBox;

	c_dataStruct.CheckBoxTypeEnum = WHEEL_METER_TYPE;
	c_dataStruct.typeName = "表计类型";

	std::map<int, WheelRobortMeterTypeStruct>::iterator iter;
	for (iter = c_meterType.begin(); iter != c_meterType.end(); iter++)
	{
		c_packageCheckBox.checkbox_uuid = QString("%1").arg(iter->first);
		c_packageCheckBox.checkbox_name = iter->second.meter_type_name;
		if (m_judge)
		{
			c_packageCheckBox.is_enabled = true;
			c_packageCheckBox.is_check = true;
		}
		else
		{
			if (m_judge_2)
			{
				c_packageCheckBox.is_enabled = false;
				c_packageCheckBox.is_check = false;
			}
			else
			{
				c_packageCheckBox.is_enabled = false;
				c_packageCheckBox.is_check = false;
				for (int i = 0; i < m_meterType.size(); i++)
				{
					if (iter->second.meter_type_name == m_meterType.at(i))
					{
						c_packageCheckBox.is_enabled = true;
						c_packageCheckBox.is_check = true;
					}
				}
			}
		}
		c_dataStruct.PackageCheckBoxStru.append(c_packageCheckBox);
	}
	m_wheelTaskAdminCheckBoxStru.append(c_dataStruct);
}
void DLWheelTaskAdministrationData::setWheelAlarmLevelCheckBox(bool m_judge, bool m_judge_2)
{
	
	WheelTaskAdminCheckBoxStruct c_dataStruct;
	std::map<int, QString> c_alarm_level = WHEEL_DEVICE_CONFIG.getWheelAlarmLevelDataMap();
	WheelPackageCheckBoxStruct c_packageCheckBox;

	c_dataStruct.CheckBoxTypeEnum = WHEEL_ALARM_LEVEL;
	c_dataStruct.typeName = "告警等级";

	std::map<int, QString>::iterator iter;
	for (iter = c_alarm_level.begin(); iter != c_alarm_level.end(); iter++)
	{
		c_packageCheckBox.checkbox_uuid = QString("%1").arg(iter->first);
		c_packageCheckBox.checkbox_name = iter->second;
		if (m_judge)
		{
			c_packageCheckBox.is_enabled = true;
			c_packageCheckBox.is_check = true;
		}
		else
		{
			if (m_judge_2)
			{
				c_packageCheckBox.is_enabled = true;
				c_packageCheckBox.is_check = false;
			}
			else
			{
				c_packageCheckBox.is_enabled = false;
				c_packageCheckBox.is_check = false;
			}
		}
		c_dataStruct.PackageCheckBoxStru.append(c_packageCheckBox);
	}
	m_wheelTaskAdminCheckBoxStru.append(c_dataStruct);
}

void DLWheelTaskAdministrationData::setWheelTaskStatusCheckBox(bool m_judge, bool m_judge_2)
{
	WheelTaskAdminCheckBoxStruct c_dataStruct;
	std::map<int, QString> c_task_status = WHEEL_TASK_END_ACTION.getWheelTaskStatusDataMap();
	WheelPackageCheckBoxStruct c_packageCheckBox;

	c_dataStruct.CheckBoxTypeEnum = WHEEL_TASK_STATUS;
	c_dataStruct.typeName = "任务状态";

	std::map<int, QString>::iterator iter;
	for (iter = c_task_status.begin(); iter != c_task_status.end(); iter++)
	{
		c_packageCheckBox.checkbox_uuid = QString("%1").arg(iter->first);
		c_packageCheckBox.checkbox_name = iter->second;
		if (m_judge)
		{
			c_packageCheckBox.is_enabled = true;
			c_packageCheckBox.is_check = true;
		}
		else
		{
			if (m_judge_2)
			{
				c_packageCheckBox.is_enabled = true;
				c_packageCheckBox.is_check = false;
			}
			else
			{
				c_packageCheckBox.is_enabled = false;
				c_packageCheckBox.is_check = false;
			}
		}
		c_dataStruct.PackageCheckBoxStru.append(c_packageCheckBox);
	}
	m_wheelTaskAdminCheckBoxStru.append(c_dataStruct);
}
void DLWheelTaskAdministrationData::setWheelDeviceFacadeCheckBox(bool m_judge, bool m_judge_2)
{
	WheelTaskAdminCheckBoxStruct c_dataStruct;
	WheelPackageCheckBoxStruct c_packageCheckBox;

	QStringList c_name;
	c_name << "电子围栏" << "水位线"<<"排水泵"<<"沉降监测点"<<"电容器外观"<<"避雷器外观";

	for (int i = 0; i < c_name.size(); i++)
	{
		c_packageCheckBox.checkbox_name = c_name[i];
		c_packageCheckBox.is_enabled = false;
		c_packageCheckBox.is_check = false;
		c_dataStruct.PackageCheckBoxStru.append(c_packageCheckBox);
	}
	c_dataStruct.typeName = WHEEL_DEVICE_FACADE;
	c_dataStruct.typeName = "设备外观检查";
	m_wheelTaskAdminCheckBoxStru.append(c_dataStruct);
}
//..///////////////////////////////////////////////////////////////////////////////
QList<QString> DLWheelTaskAdministrationData::getDeviceUUidForCheckBoxData(int checkBoxTypeEnum, QString checkBoxTypeUUid)
{
	QList<QString> m_devices;
	QString m_choose;
	switch (checkBoxTypeEnum)
	{
	case WHEEL_DEVICE_AREA_NAME:
		m_choose = "device_area";
		break;
	case WHEEL_DEVICE_TYPE:
		m_choose = "device_type";
		break;
	case WHEEL_RECOGNITION_TYPE:
		m_choose = "recognition_type";
		break;
	case WHEEL_METER_TYPE:
		m_choose = "meter_type";
		break;
	case WHEEL_DEVICE_APPEARANCE_TYPE:
		m_choose = "";
		break;
	default:
		break;
	}

	WHEEL_ROBOT_DB.getDeviceUUidForCheckBoxDB(m_choose,checkBoxTypeUUid,m_devices);
	return m_devices;
}
//..///////////////////////////////////////////////////////////////////////////////
QList<WheelTaskEditStruct> DLWheelTaskAdministrationData::getWheelTaskEditList(int m_page, int m_pageCount, WheelTaskAdminType m_task_edit_type_id)
{
	QList<WheelTaskEditStruct> m_wheelTaskEditStruct;
	int m_count = (m_page - 1) * m_pageCount;

	WHEEL_ROBOT_DB.getTaskEditListDB(m_count, m_pageCount, m_task_edit_type_id, m_wheelTaskEditStruct);
	return m_wheelTaskEditStruct;
}

void DLWheelTaskAdministrationData::getWheelTaskEditPage(int &m_page, int &m_count, int m_pageCount, WheelTaskAdminType m_task_edit_type_id)
{
	m_count = 0;
	WHEEL_ROBOT_DB.getTaskEditPageDB(m_count, m_task_edit_type_id);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}

QList<QString> DLWheelTaskAdministrationData::getWheelTaskEditDeviceList(QString m_task_edit_uuid)
{
	QList<QString> m_device_uuid;
	WHEEL_ROBOT_DB.getTaskDeviceDataDB(m_task_edit_uuid, m_device_uuid);
	return m_device_uuid;
}

WheelTaskEditStruct DLWheelTaskAdministrationData::getTaskEditStruForTaskTemplateUUid(QString m_task_edit_uuid)
{
	WheelTaskEditStruct taskEdit;
	WHEEL_ROBOT_DB.getTaskEditStruForTaskTemplateUUidDB(m_task_edit_uuid, taskEdit);
	return taskEdit;
}
//..///////////////////////////////////////////////////////////////////////////////
QList<WheelTaskListShowStruct> DLWheelTaskAdministrationData::getTaskListShowStru(WheelTaskListSearchIndex m_index)
{
	QList<WheelTaskListShowStruct> m_wheelTaskListShowStru;
	QList<WheelTaskTemplateList> m_taskTemplateList;
	WHEEL_ROBOT_DB.getTaskListShowDB(m_wheelTaskListShowStru, m_index, m_taskTemplateList);

	QDateTime current_date_time = QDateTime::currentDateTime();
 	QString current_date = current_date_time.toString("yyyy-MM-dd hh:mm:ss");
	QString curr_hms_now_string = current_date.mid(11, 8);

	QTime curr_hms_now_time = QTime::fromString(curr_hms_now_string, "hh:mm:ss");

	QDate startTime = QDate::fromString(m_index.m_start_time, "yyyy-MM-dd");
	QDate endTime = QDate::fromString(m_index.m_stop_time, "yyyy-MM-dd");
	QDate nowTime = QDate::fromString(current_date.mid(0,10), "yyyy-MM-dd");

	QDate startChoo;
	QDate endChoo;

	bool brun = false;

	if (nowTime >= startTime && nowTime <= endTime)
	{
		startChoo = QDate::fromString(current_date.mid(0, 10), "yyyy-MM-dd");
		endChoo = QDate::fromString(m_index.m_stop_time, "yyyy-MM-dd");
	}
	else if (nowTime > endTime)
	{
		if (m_wheelTaskListShowStru.size() != 0)
		{
			qSort(m_wheelTaskListShowStru.begin(), m_wheelTaskListShowStru.end(), CapitySort);
		}
		return m_wheelTaskListShowStru;
	}
	else if (nowTime < startTime)
	{
		startChoo = QDate::fromString(m_index.m_start_time, "yyyy-MM-dd");
		endChoo = QDate::fromString(m_index.m_stop_time, "yyyy-MM-dd");
	}
	for (int k = 0; k < m_index.m_task_status_id.size(); k++)
	{
		if (m_index.m_task_status_id[k] == "1")
		{
			brun = true;
		}
	}
	if (m_index.m_task_status_id.size() == 0)
	{
		brun = true;
	}

	if (brun)
	{
		QStringList dateReturn;
		for (int i = 0; i < m_taskTemplateList.size(); i++)
		{
			int cho = m_taskTemplateList[i].task_loop_type_id;
			switch (cho)
			{
			case -1:
				dateReturn.clear();
				dateReturn = WHEEL_COUNT_DATE.getCountTimingTaskChoose(startChoo, endChoo, m_taskTemplateList[i].task_start_date, m_taskTemplateList[i].task_start_time);
				break;
			case 1:
				dateReturn.clear();
				dateReturn = WHEEL_COUNT_DATE.getCountTwoDateForAddOneDay(startChoo, endChoo, m_taskTemplateList[i].task_start_date, m_taskTemplateList[i].task_start_time);
				break;
			case 2:
				dateReturn.clear();
				dateReturn = WHEEL_COUNT_DATE.getCountChooseWeekList(startChoo, endChoo, m_taskTemplateList[i].task_start_date, m_taskTemplateList[i].task_start_time);
				break;
			case 3:
				dateReturn.clear();
				dateReturn = WHEEL_COUNT_DATE.getCountMonthlyOneDayList(startChoo, endChoo, m_taskTemplateList[i].task_start_date, m_taskTemplateList[i].task_start_time);
				break;
			case 4:
				dateReturn.clear();
				dateReturn = WHEEL_COUNT_DATE.getCountFixationIntervalTimeList(QDate::fromString(m_taskTemplateList[i].task_start_date, "yyyy-MM-dd"), startChoo, endChoo, m_taskTemplateList[i].task_repeat_duration, m_taskTemplateList[i].task_start_time);
				break;
			case 5:
				dateReturn.clear();
				dateReturn = WHEEL_COUNT_DATE.getCountFixationDateTimeList(startChoo, endChoo, m_taskTemplateList[i].task_start_date, m_taskTemplateList[i].task_start_time);
				break;
			default:
				break;
			}
			for (int k = 0; k < dateReturn.size(); k++)
			{
				WheelTaskListShowStruct groupStru;
				groupStru.task_edit_uuid = m_taskTemplateList[i].task_edit_uuid;
				groupStru.task_template_uuid = m_taskTemplateList[i].task_template_uuid;
				groupStru.task_uuid = "";
				groupStru.task_edit_type_id = (WheelTaskAdminType)m_taskTemplateList[i].task_edit_type_id;
				groupStru.task_name = m_taskTemplateList[i].task_template_name;
				groupStru.task_start_time = dateReturn[k];
				groupStru.task_status_name = m_taskTemplateList[i].task_status_name;
				m_wheelTaskListShowStru.append(groupStru);
			}
		}
	}
	
	if (m_wheelTaskListShowStru.size() == 0)
	{
		return m_wheelTaskListShowStru;
	}
	qSort(m_wheelTaskListShowStru.begin(), m_wheelTaskListShowStru.end(), CapitySort);
	return m_wheelTaskListShowStru;
}

void DLWheelTaskAdministrationData::getWheelTaskListPage(int &m_page, int &m_count, int m_pageCount, WheelTaskListSearchIndex m_index)
{
	m_TaskListShowStru = getTaskListShowStru(m_index);
	m_count = m_TaskListShowStru.size();
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
	 	m_page++;
	}
}
QList<WheelTaskListShowStruct> DLWheelTaskAdministrationData::getTaskListShow(int m_page, int m_pageCount)
{
	QList<WheelTaskListShowStruct> mtr;
	if (m_page == 0)
	{
		return mtr;
	}
	if (m_TaskListShowStru.size() == 0)
	{
		return mtr;
	}
	int m_count = (m_page - 1) * m_pageCount;
	for (int i = 0; i < m_pageCount; i++)
	{
		mtr.append(m_TaskListShowStru[m_count]);
		m_count++;
		if (m_count == m_TaskListShowStru.size())
			break;
	}
	return mtr;
}
//..///////////////////////////////////////////////////////////////////////////////
void DLWheelTaskAdministrationData::getDeviceUUidMapAndEditType(QMap<QString, QString>& m_deviceMap, WheelTaskAdminType & m_taskAdminType, WheelTaskEditStruct &m_taskEditStruct, QString m_edit_uuid)
{
	WHEEL_ROBOT_DB.getEditDevicesForEditUUidDB(m_deviceMap, m_edit_uuid);
	WHEEL_ROBOT_DB.getTaskEditTypeIdDB(m_taskAdminType, m_edit_uuid);
	WHEEL_ROBOT_DB.getTaskEditDB(m_taskEditStruct, m_edit_uuid);
}

QStringList DLWheelTaskAdministrationData::getDeviceUUidFromTaskDevicesList(QString m_edit_uuid)
{
	QStringList m_deviceMap;
	WHEEL_ROBOT_DB.getEditDevicesForEditUUidListDB(m_deviceMap, m_edit_uuid);
	return m_deviceMap;
}
//..///////////////////////////////////////////////////////////////////////////////
QMap<int, QList<WheelCalendarData>> DLWheelTaskAdministrationData::getCalendarTaskMap(QString m_start_time, QString m_end_time)
{
	QMap<int, QList<WheelCalendarData>> taskMap;
	WheelCalendarData calend;
	QList<WheelCalendarData> calendList;
	WheelTaskListSearchIndex m_index;
	m_index.m_start_time = m_start_time+"-01";
	m_index.m_stop_time = m_end_time+"-01";

	QList<WheelTaskListShowStruct> mtr = getTaskListShowStru(m_index);
	if (mtr.size() == 0)
	{
		return taskMap;
	}
	QString str = mtr[0].task_start_time.mid(0, 10);
	for (int i = 0; i < mtr.size(); i++)
	{
	
		QString task_name = mtr[i].task_name;
		QString task_time = mtr[i].task_start_time;
		QString task_status = mtr[i].task_status_name;
		QString dateTime = task_time.mid(0, 10);
		int task_month = task_time.mid(8, 2).toInt();

		QString task_name_add;
        if (task_time.mid(11, 2).toInt()>12)
        {
            task_name_add = task_time.mid(11, 5) + QString("下午 ") + task_name;
        }
        else
        {
            task_name_add = task_time.mid(11, 5) + QString("上午 ") + task_name;
        }
		
		if (dateTime == str)
		{
			calend.task_name = task_name_add;
			calend.task_status = task_status;
			calendList.append(calend);

			if (i == mtr.size() - 1)
			{
				taskMap.insert(task_month, calendList);
			}
		}
		else
		{
			taskMap.insert(mtr[i-1].task_start_time.mid(8, 2).toInt(), calendList);
			calendList.clear();

			calend.task_name = task_name_add;
			calend.task_status = task_status;
			calendList.append(calend);

			if (i == mtr.size() - 1)
			{
				taskMap.insert(task_month, calendList);
			}
		}
		str = dateTime;
	}
	return taskMap;
}
//..///////////////////////////////////////////////////////////////////////////////
QList<WheelTaskShow> DLWheelTaskAdministrationData::getWheelTaskListShow(QString m_start_time, QString m_end_time)
{
	QList<WheelTaskShow> taskList;
	WHEEL_ROBOT_DB.getWheelTaskListShowDB(m_start_time, m_end_time, taskList);
	return taskList;
}

WheelTaskShow DLWheelTaskAdministrationData::getWheelTaskStru(QString task_uuid)
{
	WheelTaskShow taskStru;
	WHEEL_ROBOT_DB.getWheelTaskStruShowDB(taskStru, task_uuid);
	return taskStru;
}
//..///////////////////////////////////////////////////////////////////////////////
QList<WheelTaskEditStruct> DLWheelTaskAdministrationData::getWheelTaskHistoryStru(QString start_time, QString end_time, QString task_name)
{
	QList<WheelTaskEditStruct> stru;
	WHEEL_ROBOT_DB.getWheelTaskHistoryStruDB(start_time, end_time, task_name, stru);
	return stru;
}
//..///////////////////////////////////////////////////////////////////////////////
QList<WheelCreateReport> DLWheelTaskAdministrationData::getCreateReportStru(QString start_time, QString end_time)
{
	QList<WheelCreateReport> data;
	WHEEL_ROBOT_DB.getCreateReportDB(data, start_time, end_time);
	return data;
}