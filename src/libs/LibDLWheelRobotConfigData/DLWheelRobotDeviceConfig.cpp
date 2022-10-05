#include "DLWheelRobotDeviceConfig.h"
#include <QApplication>
#include <QDir>
#include <QTextStream>
#include <boost/uuid/uuid_generators.hpp>  
#include <boost/uuid/uuid_io.hpp>  
#include <boost/uuid/uuid.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include <QTableWidget>
#include <QHeaderView>
#include <QHBoxLayout>

DLWheelRobotDeviceConfig::DLWheelRobotDeviceConfig()
{
	m_noteMsgCountF = 0;
	m_noteMsgCountS = 0;
    refreshData();
}

DLWheelRobotDeviceConfig::~DLWheelRobotDeviceConfig()
{
}

void DLWheelRobotDeviceConfig::refreshData()
{
	loadDeviceSnData();
    loadWheelDeviceAreaData();
    loadWheelDevicePointNameData();
    loadWheelSubDeviceTypeData();
    loadWheelDeviceTypeData();
    loadWheelVoltageLevelData();
    loadWheelFeverTypeData();
    loadWheelMeterTypeData();
    loadWheelRecognitionTypeData();
    loadWheelAlarmLevelData();
    loadWheelSaveTypeData();
    loadWheelEquipmentIntervalData();
}

void DLWheelRobotDeviceConfig::loadDeviceSnData()
{
	m_WheelDeviceSnData.clear();
	WHEEL_ROBOT_DB.getDeviceSn(m_WheelDeviceSnData);
}

QString DLWheelRobotDeviceConfig::getDeviceSnNameQString(QString device_name)
{
	return QString();
}

///////////////////////////////////////////////////////////////////////////////////

void DLWheelRobotDeviceConfig::loadWheelDeviceAreaData()
{
	m_WheelDeviceAreaData.clear();
	WHEEL_ROBOT_DB.getWheelDeviceAreaDataDB(m_WheelDeviceAreaData);
}

std::map<QString, QString> DLWheelRobotDeviceConfig::getWheelDeviceAreaDataMap()
{
	return m_WheelDeviceAreaData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelDeviceAreaNameQList()
{
	m_WheelDeviceAreaQList.clear();
	std::map<QString, QString>::iterator iter;
	for (iter = m_WheelDeviceAreaData.begin(); iter != m_WheelDeviceAreaData.end(); iter++)
	{
		m_WheelDeviceAreaQList.append(iter->second);
	}
	return m_WheelDeviceAreaQList;
}

QString DLWheelRobotDeviceConfig::getWheelDeviceAreaNameQString(QString m_device_area_uuid)
{
	QString m_dev_area_name;
	if (m_WheelDeviceAreaData.find(m_device_area_uuid) != m_WheelDeviceAreaData.end())
	{
		m_dev_area_name = m_WheelDeviceAreaData.at(m_device_area_uuid);
	}
	return m_dev_area_name;
}

QString DLWheelRobotDeviceConfig::getWheelDeviceAreaUUidQString(QString m_device_area_name)
{
	QString m_dev_area_uuid;
	auto find_item = std::find_if(m_WheelDeviceAreaData.begin(), m_WheelDeviceAreaData.end(),
		[m_device_area_name](const std::map<QString, QString>::value_type item)
	{
		return item.second == m_device_area_name;
	});
	if (find_item != m_WheelDeviceAreaData.end())
	{
		m_dev_area_uuid = (*find_item).first;
	}
	return m_dev_area_uuid;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelDevicePointNameData()
{
	m_WheelDevicePointNameData.clear();
	WHEEL_ROBOT_DB.getWheelDevicePointNameDB(m_WheelDevicePointNameData);
}

std::map<QString, WheelDevicePointNameStruct> DLWheelRobotDeviceConfig::getWheelDevicePointNameDataMap()
{
	return m_WheelDevicePointNameData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelDevicePointNameQList()
{
	m_WheelDevicePointNameQList.clear();
	std::map<QString, WheelDevicePointNameStruct>::iterator iter;
	for (iter = m_WheelDevicePointNameData.begin(); iter != m_WheelDevicePointNameData.end(); iter++)
	{
		m_WheelDevicePointNameQList.append(iter->second.device_point_type_name);
	}
	return m_WheelDevicePointNameQList;
}

QString DLWheelRobotDeviceConfig::getWheelSubDeviceTypeUUidFromPointName(QString m_device_point_type_uuid)
{
	QString m_sub_device_type_uuid;
	if (m_WheelDevicePointNameData.find(m_device_point_type_uuid) != m_WheelDevicePointNameData.end())
	{
		m_sub_device_type_uuid = m_WheelDevicePointNameData.at(m_device_point_type_uuid).sub_device_type_uuid;
	}
	return m_sub_device_type_uuid;
}

QString DLWheelRobotDeviceConfig::getWheelDevicePointNameQString(QString m_device_point_type_uuid)
{
	QString m_device_point_type_name;
	if (m_WheelDevicePointNameData.find(m_device_point_type_uuid) != m_WheelDevicePointNameData.end())
	{
		m_device_point_type_name = m_WheelDevicePointNameData.at(m_device_point_type_uuid).device_point_type_name;
	}
	return m_device_point_type_name;
}

QString DLWheelRobotDeviceConfig::getWheelDevicePointTypeUUidQString(QString m_device_point_type_name)
{
	QString m_device_point_type_uuid;
	auto find_item = std::find_if(m_WheelDevicePointNameData.begin(), m_WheelDevicePointNameData.end(),
		[m_device_point_type_name](const std::map<QString, WheelDevicePointNameStruct>::value_type item)
	{
		return item.second.device_point_type_name == m_device_point_type_name;
	});
	if (find_item != m_WheelDevicePointNameData.end())
	{
		m_device_point_type_uuid = (*find_item).first;
	}
	return m_device_point_type_uuid;
}

std::map<QString, WheelDevicePointNameStruct> DLWheelRobotDeviceConfig::getWheelDevicePointNameFromSubDeviceType(QString m_sub_device_uuid)
{
	std::map<QString, WheelDevicePointNameStruct> m_wheelDevicePointName;
	std::map<QString, WheelDevicePointNameStruct>::iterator iter;
	for (iter = m_WheelDevicePointNameData.begin(); iter != m_WheelDevicePointNameData.end(); iter++)
	{
		if (iter->second.sub_device_type_uuid == m_sub_device_uuid)
		{
			WheelDevicePointNameStruct devicePointStru;
			devicePointStru.device_point_type_uuid = iter->second.device_point_type_uuid;
			devicePointStru.device_point_type_name = iter->second.device_point_type_name;
			devicePointStru.recognition_type_id = iter->second.recognition_type_id;
			devicePointStru.meter_type_id = iter->second.meter_type_id;
			devicePointStru.fever_type_id = iter->second.fever_type_id;
			devicePointStru.save_type_id = iter->second.save_type_id;
			devicePointStru.unit_type_id = iter->second.unit_type_id;
	
			m_wheelDevicePointName.insert(std::make_pair(iter->first, devicePointStru));
		}
	}
	return m_wheelDevicePointName;
}
/////////////////////////////////////////////////////////////////////////////////////
WheelJudgeTakePhoto DLWheelRobotDeviceConfig::getWheelChooseRecognitionBool(QString m_device_point_type_uuid)
{
	WheelJudgeTakePhoto judge = IniNull;

	bool m_bool = true;
	int recoType;
	int saveType;
	auto find_item = std::find_if(m_WheelDevicePointNameData.begin(), m_WheelDevicePointNameData.end(),
		[m_device_point_type_uuid](const std::map<QString, WheelDevicePointNameStruct>::value_type item)
	{
		return item.first == m_device_point_type_uuid;
	});
	if (find_item != m_WheelDevicePointNameData.end())
	{
		recoType = (*find_item).second.recognition_type_id;
		saveType = (*find_item).second.save_type_id;

		if (saveType == 1 || recoType == 1)
		{
			judge = VisibleLightJudge;
		}
		if (saveType == 2 || saveType == 4 || recoType == 4 || recoType == 6)
		{
			judge = InfraredLightJudge;
		}
		if (saveType == 5 || recoType == 5)
		{
			judge = VoiceJudge;
		}
		if (saveType == 6)
		{
			judge = VideoJudge;
		}
	}
	return judge;
}
WheelJudgeTakePhoto DLWheelRobotDeviceConfig::getWheelChooseRecForDeviceUUidBool(QString m_device_uuid)
{
	QString device_point_type_uuid;
	WHEEL_ROBOT_DB.getDevicePointTypeUUidFromDevices(m_device_uuid, device_point_type_uuid);
	return getWheelChooseRecognitionBool(device_point_type_uuid);
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelSubDeviceTypeData()
{
	m_WheelSubDeviceTypeData.clear();
	WHEEL_ROBOT_DB.getWheelSubDeviceTypeDB(m_WheelSubDeviceTypeData);
}

std::map<QString, WheelSubDeviceTypeStruct> DLWheelRobotDeviceConfig::getWheelSubDeviceTypeDataMap()
{
	return m_WheelSubDeviceTypeData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelSubDeviceNameQList()
{
	m_WheelSubDeviceNameQList.clear();
	std::map<QString, WheelSubDeviceTypeStruct>::iterator iter;
	for (iter = m_WheelSubDeviceTypeData.begin(); iter != m_WheelSubDeviceTypeData.end(); iter++)
	{
		m_WheelSubDeviceNameQList.append(iter->second.sub_device_name);
	}
	return m_WheelSubDeviceNameQList;
}

QString DLWheelRobotDeviceConfig::getWheelDeviceTypeUUidFromSubDevice(QString m_sub_device_type_uuid)
{
	QString m_device_type_uuid;
	if (m_WheelSubDeviceTypeData.find(m_sub_device_type_uuid) != m_WheelSubDeviceTypeData.end())
	{
		m_device_type_uuid = m_WheelSubDeviceTypeData.at(m_sub_device_type_uuid).device_type_uuid;
	}
	return m_device_type_uuid;
}

QString DLWheelRobotDeviceConfig::getWheelSubDeviceNameQString(QString m_sub_device_type_uuid)
{
	QString m_sub_device_name;
	if (m_WheelSubDeviceTypeData.find(m_sub_device_type_uuid) != m_WheelSubDeviceTypeData.end())
	{
		m_sub_device_name = m_WheelSubDeviceTypeData.at(m_sub_device_type_uuid).sub_device_name;
	}
	return m_sub_device_name;
}

QString DLWheelRobotDeviceConfig::getWheelSubDeviceTypeUUidQString(QString m_sub_device_name)
{
	QString sub_device_type_uuid;
	auto find_item = std::find_if(m_WheelSubDeviceTypeData.begin(), m_WheelSubDeviceTypeData.end(),
		[m_sub_device_name](const std::map<QString, WheelSubDeviceTypeStruct>::value_type item)
	{
		return item.second.sub_device_name == m_sub_device_name;
	});
	if (find_item != m_WheelSubDeviceTypeData.end())
	{
		sub_device_type_uuid = (*find_item).first;
	}
	return sub_device_type_uuid;
}

std::map<QString, QString> DLWheelRobotDeviceConfig::getWheelSubDeviceNameFromDeviceType(QString m_device_type_uuid)
{
	std::map<QString, QString> m_wheelSubDeviceName;
	std::map<QString, WheelSubDeviceTypeStruct>::iterator iter;
	for (iter = m_WheelSubDeviceTypeData.begin(); iter != m_WheelSubDeviceTypeData.end(); iter++)
	{
		if (iter->second.device_type_uuid == m_device_type_uuid)
		{
			m_wheelSubDeviceName.insert(std::make_pair(iter->first, iter->second.sub_device_name));
		}
	}
	return m_wheelSubDeviceName;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelDeviceTypeData()
{
	m_WheelDeviceTypeData.clear();
	WHEEL_ROBOT_DB.getWheelDeviceTypeDB(m_WheelDeviceTypeData);
}

std::map<QString, QString> DLWheelRobotDeviceConfig::getWheelDeviceTypeDataMap()
{
	return m_WheelDeviceTypeData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelDeviceTypeNameQList()
{
	m_WheelDeviceTypeNameQList.clear();
	std::map<QString, QString>::iterator iter;
	for (iter = m_WheelDeviceTypeData.begin(); iter != m_WheelDeviceTypeData.end(); iter++)
	{
		m_WheelDeviceTypeNameQList.append(iter->second);
	}
	return m_WheelDeviceTypeNameQList;
}

QString DLWheelRobotDeviceConfig::getWheelDeviceTypeNameQString(QString m_device_type_uuid)
{
	QString m_device_type_name;
	if (m_WheelDeviceTypeData.find(m_device_type_uuid) != m_WheelDeviceTypeData.end())
	{
		m_device_type_name = m_WheelDeviceTypeData.at(m_device_type_uuid);
	}
	return m_device_type_name;
}

QString DLWheelRobotDeviceConfig::getWheelDeviceTypeUUidQString(QString m_device_type_name)
{
	QString m_device_type_uuid;
	auto find_item = std::find_if(m_WheelDeviceTypeData.begin(), m_WheelDeviceTypeData.end(),
		[m_device_type_name](const std::map<QString, QString>::value_type item)
	{
		return item.second == m_device_type_name;
	});
	if (find_item != m_WheelDeviceTypeData.end())
	{
		m_device_type_uuid = (*find_item).first;
	}
	return m_device_type_uuid;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelVoltageLevelData()
{
	m_WheelVoltageLevelData.clear();
	WHEEL_ROBOT_DB.getWheelVoltageLevelDB(m_WheelVoltageLevelData);
}

std::map<QString, QString> DLWheelRobotDeviceConfig::getWheelVoltageLevelDataMap()
{
	return m_WheelVoltageLevelData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelVoltageLevelNameQList()
{
	m_WheelVoltageLevelNameQList.clear();
	std::map<QString, QString>::iterator iter;
	for (iter = m_WheelVoltageLevelData.begin(); iter != m_WheelVoltageLevelData.end(); iter++)
	{
		m_WheelVoltageLevelNameQList.append(iter->second);
	}
	return m_WheelVoltageLevelNameQList;
}

QString DLWheelRobotDeviceConfig::getWheelVoltageLevelNameQString(QString m_voltage_level_id)
{
	QString m_voltage_level_name;
	if (m_WheelVoltageLevelData.find(m_voltage_level_id) != m_WheelVoltageLevelData.end())
	{
		m_voltage_level_name = m_WheelVoltageLevelData.at(m_voltage_level_id);
	}
	return m_voltage_level_name;
}

QString DLWheelRobotDeviceConfig::getWheelVoltageLevelIdInt(QString m_voltage_level_name)
{
	QString m_voltage_level_id;
	if (m_voltage_level_name.isEmpty())
	{
		m_voltage_level_id = -1;
	}
	else
	{
		auto find_item = std::find_if(m_WheelVoltageLevelData.begin(), m_WheelVoltageLevelData.end(),
			[m_voltage_level_name](const std::map<QString, QString>::value_type item)
		{
			return item.second == m_voltage_level_name;
		});
		if (find_item != m_WheelVoltageLevelData.end())
		{
			m_voltage_level_id = (*find_item).first;
		}
	}
	
	return m_voltage_level_id;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelFeverTypeData()
{
	m_WheelFeverTypeData.clear();
	WHEEL_ROBOT_DB.getWheelFeverTypeDB(m_WheelFeverTypeData);
}

std::map<int, QString> DLWheelRobotDeviceConfig::getWheelFeverTypeDataMap()
{
	return m_WheelFeverTypeData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelFeverTypeNameQList()
{
	m_WheelFeverTypeNameQList.clear();
	std::map<int, QString>::iterator iter;
	for (iter = m_WheelFeverTypeData.begin(); iter != m_WheelFeverTypeData.end(); iter++)
	{
		m_WheelFeverTypeNameQList.append(iter->second);
	}
	return m_WheelFeverTypeNameQList;
}

QString DLWheelRobotDeviceConfig::getWheelFeverTypeNameQString(int m_fever_type_id)
{
	QString m_fever_type_name;
	if (m_WheelFeverTypeData.find(m_fever_type_id) != m_WheelFeverTypeData.end())
	{
		m_fever_type_name = m_WheelFeverTypeData.at(m_fever_type_id);
	}
	return m_fever_type_name;
}

int DLWheelRobotDeviceConfig::getWheelFeverTypeIdInt(QString m_fever_type_name)
{
	int m_fever_type_id = -1;
	if (m_fever_type_name.isEmpty())
	{
		m_fever_type_id = -1;
	}
	else
	{
		auto find_item = std::find_if(m_WheelFeverTypeData.begin(), m_WheelFeverTypeData.end(),
			[m_fever_type_name](const std::map<int, QString>::value_type item)
		{
			return item.second == m_fever_type_name;
		});
		if (find_item != m_WheelFeverTypeData.end())
		{
			m_fever_type_id = (*find_item).first;
		}
	}
	return m_fever_type_id;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelMeterTypeData()
{
	m_WheelMeterTypeData.clear();
	WHEEL_ROBOT_DB.getWheelMeterTypeDB(m_WheelMeterTypeData);
}

std::map<int, WheelRobortMeterTypeStruct> DLWheelRobotDeviceConfig::getWheelMeterTypeDataMap()
{
	return m_WheelMeterTypeData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelMeterTypeNameQList()
{
	m_WheelMeterTypeNameQList.clear();
	std::map<int, WheelRobortMeterTypeStruct>::iterator iter;
	for (iter = m_WheelMeterTypeData.begin(); iter != m_WheelMeterTypeData.end(); iter++)
	{
		m_WheelMeterTypeNameQList.append(iter->second.meter_type_name);
	}
	return m_WheelMeterTypeNameQList;
}

QString DLWheelRobotDeviceConfig::getWheelMeterTypeNameQString(int m_meter_type_id)
{
	QString m_meter_type_name;
	if (m_WheelMeterTypeData.find(m_meter_type_id) != m_WheelMeterTypeData.end())
	{
		m_meter_type_name = m_WheelMeterTypeData.at(m_meter_type_id).meter_type_name;
	}
	return m_meter_type_name;
}

int DLWheelRobotDeviceConfig::getWheelMeterTypeIdInt(QString m_meter_type_name)
{
	int m_meter_type_id = -1;
	if (m_meter_type_name.isEmpty())
	{
		m_meter_type_id = -1;
	}
	else
	{
		auto find_item = std::find_if(m_WheelMeterTypeData.begin(), m_WheelMeterTypeData.end(),
			[m_meter_type_name](const std::map<int, WheelRobortMeterTypeStruct>::value_type item)
		{
			return item.second.meter_type_name == m_meter_type_name;
		});
		if (find_item != m_WheelMeterTypeData.end())
		{
			m_meter_type_id = (*find_item).first;
		}
	}
	
	return m_meter_type_id;
}
QString DLWheelRobotDeviceConfig::getWheelThresholdUUidFromMeterType(int m_meter_type_id)
{
	QString m_threshold_filename;
	if (m_WheelMeterTypeData.find(m_meter_type_id) != m_WheelMeterTypeData.end())
	{
		m_threshold_filename = m_WheelMeterTypeData.at(m_meter_type_id).threshold_filename;
	}
	return m_threshold_filename;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelRecognitionTypeData()
{
	m_WheelRecognitionTypeData.clear();
	WHEEL_ROBOT_DB.getWheelRecognitionTypeDB(m_WheelRecognitionTypeData);
}

std::map<int, QString> DLWheelRobotDeviceConfig::getWheelRecognitionTypeDataMap()
{
	return m_WheelRecognitionTypeData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelRecognitionTypeQList()
{
	m_WheelRecognitionTypeQList.clear();
	std::map<int, QString>::iterator iter;
	for (iter = m_WheelRecognitionTypeData.begin(); iter != m_WheelRecognitionTypeData.end(); iter++)
	{
		m_WheelRecognitionTypeQList.append(iter->second);
	}
	return m_WheelRecognitionTypeQList;
}

QString DLWheelRobotDeviceConfig::getWheelRecognitionTypeNameQString(int recognition_type_id)
{
	QString m_recognition_type_name;
	if (m_WheelRecognitionTypeData.find(recognition_type_id) != m_WheelRecognitionTypeData.end())
	{
		m_recognition_type_name = m_WheelRecognitionTypeData.at(recognition_type_id);
	}
	return m_recognition_type_name;
}

int DLWheelRobotDeviceConfig::getWheelRecognitionTypeIdInt(QString recognition_type_name)
{
	int m_recognition_type_id = -1;
	if (recognition_type_name.isEmpty())
	{
		m_recognition_type_id = -1;
	}
	else
	{
		auto find_item = std::find_if(m_WheelRecognitionTypeData.begin(), m_WheelRecognitionTypeData.end(),
			[recognition_type_name](const std::map<int, QString>::value_type item)
		{
			return item.second == recognition_type_name;
		});
		if (find_item != m_WheelRecognitionTypeData.end())
		{
			m_recognition_type_id = (*find_item).first;
		}
	}

	return m_recognition_type_id;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelAlarmLevelData()
{
	m_WheelAlarmLevelData.clear();
	WHEEL_ROBOT_DB.getWheelAlarmLevelDB(m_WheelAlarmLevelData);
}

std::map<int, QString> DLWheelRobotDeviceConfig::getWheelAlarmLevelDataMap()
{
	return m_WheelAlarmLevelData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelAlarmLevelDataQList()
{
	m_WheelAlarmLevelQList.clear();
	std::map<int, QString>::iterator iter;
	for (iter = m_WheelAlarmLevelData.begin(); iter != m_WheelAlarmLevelData.end(); iter++)
	{
		m_WheelAlarmLevelQList.append(iter->second);
	}
	return m_WheelAlarmLevelQList;
}

int DLWheelRobotDeviceConfig::getWheelAlarmLevelId(QString alarm_level_name)
{
	int m_alarm_level_id;
	if (alarm_level_name.isEmpty())
	{
		m_alarm_level_id = -1;
	}
	else
	{
		auto find_item = std::find_if(m_WheelAlarmLevelData.begin(), m_WheelAlarmLevelData.end(),
			[alarm_level_name](const std::map<int, QString>::value_type item)
		{
			return item.second == alarm_level_name;
		});
		if (find_item != m_WheelAlarmLevelData.end())
		{
			m_alarm_level_id = (*find_item).first;
		}
	}
	return m_alarm_level_id;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelSaveTypeData()
{
	m_WheelSaveTypeData.clear();
	WHEEL_ROBOT_DB.getWheelSaveTypeDB(m_WheelSaveTypeData);
}

std::map<int, QString> DLWheelRobotDeviceConfig::getWheelSavaTypeDataMap()
{
	return m_WheelSaveTypeData;
}

QList<QString> DLWheelRobotDeviceConfig::getWheelSavaTypeDataQList()
{
	m_WheelSaveTypeQList.clear();
	std::map<int, QString>::iterator iter;
	for (iter = m_WheelSaveTypeData.begin(); iter != m_WheelSaveTypeData.end(); iter++)
	{
		m_WheelSaveTypeQList.append(iter->second);
	}
	return m_WheelSaveTypeQList;
}

QString DLWheelRobotDeviceConfig::getWheelSaveTypeNameQString(int save_type_id)
{
	QString m_save_type_name;
	if (m_WheelSaveTypeData.find(save_type_id) != m_WheelSaveTypeData.end())
	{
		m_save_type_name = m_WheelSaveTypeData.at(save_type_id);
	}
	return m_save_type_name;
}

int DLWheelRobotDeviceConfig::getWheelSaveTypeIdInt(QString save_type_name)
{
	int m_save_type_id = -1;
	if (save_type_name.isEmpty())
	{
		m_save_type_id = -1;
	}
	else
	{
		auto find_item = std::find_if(m_WheelSaveTypeData.begin(), m_WheelSaveTypeData.end(),
			[save_type_name](const std::map<int, QString>::value_type item)
		{
			return item.second == save_type_name;
		});
		if (find_item != m_WheelSaveTypeData.end())
		{
			m_save_type_id = (*find_item).first;
		}
	}
	return m_save_type_id;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::loadWheelEquipmentIntervalData()
{
	m_WheelEquipmentIntervalData.clear();
	WHEEL_ROBOT_DB.getWheelEquipmentIntervalDB(m_WheelEquipmentIntervalData);
}

std::map<QString, WheelRobortEquipmentIntervalStruct> DLWheelRobotDeviceConfig::getWheelRobortEquipmentIntervalDataMap()
{
	return m_WheelEquipmentIntervalData;
}

std::map<QString, QString> DLWheelRobotDeviceConfig::getWheelRobortEquipmentIntervalFromVoltageLevelId(QString m_voltage_level_id)
{
	std::map<QString, QString> m_wheelEquipmentIntervalName;
	std::map<QString, WheelRobortEquipmentIntervalStruct>::iterator iter;
	for (iter = m_WheelEquipmentIntervalData.begin(); iter != m_WheelEquipmentIntervalData.end(); iter++)
	{
		if (iter->second.voltage_level_id == m_voltage_level_id)
		{
			m_wheelEquipmentIntervalName.insert(std::make_pair(iter->first, iter->second.equipment_interval_name));
		}
	}
	return m_wheelEquipmentIntervalName;
}
QString DLWheelRobotDeviceConfig::getWheelEquipmentIntervalQString(QString equipment_interval_name)
{
	QString m_equipment_interval_uuid;
	std::map<QString, WheelRobortEquipmentIntervalStruct>::iterator iter;
	for (iter = m_WheelEquipmentIntervalData.begin(); iter != m_WheelEquipmentIntervalData.end(); iter++)
	{
		if (iter->second.equipment_interval_name == equipment_interval_name)
		{
			m_equipment_interval_uuid = iter->first;
		}
	}
	return m_equipment_interval_uuid;
}

/////////////////////////////////////////////////////////////////////////////////////
QList<WheelStandardPatrolVindicateStruct> DLWheelRobotDeviceConfig::getWheelStandardPatrolVindicate(QStringList m_device_type_uuid)
{
	QList<WheelStandardPatrolVindicateStruct> m_standardPatrolVindicateStru;
	WHEEL_ROBOT_DB.getWheelStandardPatrolVindicateDB(m_device_type_uuid, m_standardPatrolVindicateStru);
	return m_standardPatrolVindicateStru;
}

WheelDevicePointNameStruct DLWheelRobotDeviceConfig::getPointUUidForStandardPatrolVin(WheelStandardPatrolVindicateStruct m_standPatrol)
{
	WheelDevicePointNameStruct c_devicePointNameStru;
	c_devicePointNameStru.device_point_type_uuid = m_standPatrol.m_device_point_uuid;
	c_devicePointNameStru.sub_device_type_uuid = getWheelSubDeviceTypeUUidQString(m_standPatrol.m_sub_device_name);
	c_devicePointNameStru.device_point_type_name = m_standPatrol.m_device_point_name;
	c_devicePointNameStru.recognition_type_id = getWheelRecognitionTypeIdInt(m_standPatrol.m_recognition_type_name);
	c_devicePointNameStru.meter_type_id = getWheelMeterTypeIdInt(m_standPatrol.m_meter_type_name);
	c_devicePointNameStru.fever_type_id = getWheelFeverTypeIdInt(m_standPatrol.m_fever_type_name);
	c_devicePointNameStru.save_type_id = getWheelSaveTypeIdInt(m_standPatrol.m_save_type_name);
	return c_devicePointNameStru;
}
/////////////////////////////////////////////////////////////////////////////////////
WheelRobortDeviceParameterStruct DLWheelRobotDeviceConfig::getWheelDeviceParameterData(QString m_device_uuid)
{
	WheelRobortDeviceParameterStruct m_deviceParameter;
	WHEEL_ROBOT_DB.getWheelDeviceParameterDB(m_device_uuid,m_deviceParameter);
	return m_deviceParameter;
}
/////////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotDeviceConfig::getWheelDeviceParameterDataToText()
{
	QList<QStringList> m_devP;
	QStringList lisErrorMsg;
	WHEEL_ROBOT_DB.getWheelDeviceParameterToTextDB(m_devP, lisErrorMsg);
    if (m_devP.size() == 0)
    {
        return;
    }
    else
	{
		QWidget *widget = new QWidget;
		widget->setWindowModality(Qt::ApplicationModal);
		widget->setWindowFlags(Qt::Widget | Qt::WindowStaysOnTopHint);
		widget->resize(QSize(380, 500));

		QTableWidget *table = new QTableWidget(0, 1);
		table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
		table->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);

		QStringList headerList;
		headerList << QString("异常设备列表").toLocal8Bit();
		table->setHorizontalHeaderLabels(headerList);
		for (int row = 0; row < lisErrorMsg.size(); row++)
		{
			table->insertRow(row);
			table->setItem(row, 0, new QTableWidgetItem(lisErrorMsg[row]));
			table->item(row, 0)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
		}

		QHBoxLayout *layout = new QHBoxLayout(widget);
		layout->addWidget(table);
		widget->show();
	}

	QDir *TEST = new QDir;
	bool exist = TEST->exists("TEST");
	if (!exist)
		bool ok = TEST->mkdir("TEST");

	QString fileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/device_parameter/device_parameter.txt";

	QString strFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/device_parameter";
	QDir downloadDir;
	if (!downloadDir.exists(strFilePath))
	{
		downloadDir.mkpath(strFilePath);
	}


	QFile file(fileName);
	//if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
    if (!file.open(QIODevice::WriteOnly))
	{
		return;
	}
	file.resize(0);
	QTextStream in(&file);
	in.setCodec("utf-8");
	for (int i = 0; i < m_devP.size(); i++)
	{
		for (int j = 0; j < m_devP[i].size(); j++)
		{
			in << m_devP[i][j];
			if (j == m_devP[i].size() - 1)
			{
				in << "\n";
			}
			else
			{
				in << ",";
			}
		}
	}
	file.close();
}
/////////////////////////////////////////////////////////////////////////////////////
QStringList DLWheelRobotDeviceConfig::getWheelAllDeviceUUid()
{
	QStringList m_dev;
	WHEEL_ROBOT_DB.getWheelAllDeviceUUidDB(m_dev);
	return m_dev;
}
/////////////////////////////////////////////////////////////////////////////////////

QString DLWheelRobotDeviceConfig::dealNameLetter(QString name, int &phase_id)
{
	QString Msg = name;
	phase_id = 1;
	if (name.contains("A", Qt::CaseSensitive))
	{
	//	Msg = name.replace(QRegExp("A"), "X");
		phase_id = 2;
	}
	if (name.contains("B", Qt::CaseSensitive))
	{
	//	Msg = name.replace(QRegExp("B"), "X");
		phase_id = 3;
	}
	if (name.contains("C", Qt::CaseSensitive))
	{
	//	Msg = name.replace(QRegExp("C"), "X");
		phase_id = 4;
	}
	return Msg;
}

QString DLWheelRobotDeviceConfig::getUUid()
{
	boost::uuids::random_generator rgen;//���������  
	boost::uuids::uuid ssid = rgen();//����һ�������UUID
	std::string tmp = boost::lexical_cast<std::string>(ssid);
	boost::erase_all(tmp, "-");
	QString qsid = QString::fromStdString(tmp);
	return qsid;
}

//./////////////////////////////////////////////////////////////////////////////////.//
QList<WheelPatrolPointSet> DLWheelRobotDeviceConfig::getPatrolPointSetData(int m_page, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara)
{
	QList<WheelPatrolPointSet> data;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getPatrolPointSetDataDB(m_count, m_pageCount, data, m_wheelPatrolPara);
	return data;
}

void DLWheelRobotDeviceConfig::getPatrolPointSetDataCount(int & m_page, int & m_count, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara)
{
	m_count = 0;
	WHEEL_ROBOT_DB.getPatrolPointSetDataCountDB(m_count, m_wheelPatrolPara);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}
//./////////////////////////////////////////////////////////////////////////////////.//

QString DLWheelRobotDeviceConfig::getDeviceTypeNameForDeviceUUid(QString device_uuid)
{
	QString deviceType;
	WHEEL_ROBOT_DB.getDeviceTypeNameForDeviceUUidDB(device_uuid, deviceType);
	return deviceType;
}

WheelRobotMeterType DLWheelRobotDeviceConfig::getMeterEnumWithDeviceUUid(QString device_uuid)
{
	WheelRobotMeterType meterId;
	WHEEL_ROBOT_DB.getMeterEnumWithDeviceUUidDB(device_uuid, meterId);
	return meterId;
}

//./////////////////////////////////////////////////////////////////////////////////.//

// QList<WheelNoteMessage> DLWheelRobotDeviceConfig::getAlarmMessageSubscribe(WheelAlarmMessageIf msg)
// {
// 	QList<WheelNoteMessage> data;
// 	WHEEL_ROBOT_DB.getAlarmMessageSubscribeDB(data, msg);
// 	return data;
// }

QList<WheelNoteMessage> DLWheelRobotDeviceConfig::getAlarmMessageSubscribe(int m_page, int m_pageCount, WheelAlarmMessageIf msg)
{
	QList<WheelNoteMessage> data;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getAlarmMessageSubscribeDB(m_count, m_pageCount, data, msg);
	return data;
}

void DLWheelRobotDeviceConfig::getAlarmMessageSubscribeCount(int & m_page, int & m_count, int m_pageCount, WheelAlarmMessageIf msg)
{
	m_count = 0;
	WHEEL_ROBOT_DB.getAlarmMessageSubscribeCountDB(m_count, msg);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}

QList<WheelNoteMessage> DLWheelRobotDeviceConfig::getSystemMessageSubscribe(int m_page, int m_pageCount, WheelAlarmMessageIf msg)
{
	QList<WheelNoteMessage> data;
	int m_count = (m_page - 1) * m_pageCount;
	WHEEL_ROBOT_DB.getSystemMessageSubscribeDB(m_count, m_pageCount, data, msg);
	return data;
}

void DLWheelRobotDeviceConfig::getSystemMessageSubscribeCount(int & m_page, int & m_count, int m_pageCount, WheelAlarmMessageIf msg)
{
	m_count = 0;
	WHEEL_ROBOT_DB.getSystemMessageSubscribeCountDB(m_count, msg);
	m_page = m_count / m_pageCount;
	if (m_count % m_pageCount != 0)
	{
		m_page++;
	}
}

QList<WeelSystemAlarm> DLWheelRobotDeviceConfig::getSystemNameAndUUid()
{
	QList<WeelSystemAlarm> data;
	WHEEL_ROBOT_DB.getSystemNameAndUUidDB(data);
	return data;
}
