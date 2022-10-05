#include "DLWheelRobotTaskConfig.h"



DLWheelRobotTaskConfig::DLWheelRobotTaskConfig()
{
	loadWheelTaskEndActionData();
	loadWheelTaskEndTypeData();
	loadWheelTaskTypeData();
	loadWheelTaskStatusData();
}


DLWheelRobotTaskConfig::~DLWheelRobotTaskConfig()
{
}
///////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotTaskConfig::loadWheelTaskEndActionData()
{
	m_WheelTaskEndActionData.clear();
	WHEEL_ROBOT_DB.getWheelTaskEndActionDB(m_WheelTaskEndActionData);
}

std::map<int, QString> DLWheelRobotTaskConfig::getWheelTaskEndActionDataMap()
{
	return m_WheelTaskEndActionData;
}

QList<QString> DLWheelRobotTaskConfig::getWheelTaskEndActionNameQList()
{
	m_WheelTaskEndActionNameQList.clear();
	std::map<int, QString>::iterator iter;
	for (iter = m_WheelTaskEndActionData.begin(); iter != m_WheelTaskEndActionData.end(); iter++)
	{
		m_WheelTaskEndActionNameQList.append(iter->second);
	}
	return m_WheelTaskEndActionNameQList;
}

QString DLWheelRobotTaskConfig::getWheelTaskEndActionNameQString(int m_task_end_action_id)
{
	QString m_task_end_action_name;
	if (m_WheelTaskEndActionData.find(m_task_end_action_id) != m_WheelTaskEndActionData.end())
	{
		m_task_end_action_name = m_WheelTaskEndActionData.at(m_task_end_action_id);
	}
	return m_task_end_action_name;
}

int DLWheelRobotTaskConfig::getWheelTaskEndActionIdInt(QString m_task_end_action_name)
{
	int m_task_end_action_id;
	auto find_item = std::find_if(m_WheelTaskEndActionData.begin(), m_WheelTaskEndActionData.end(),
		[m_task_end_action_name](const std::map<int, QString>::value_type item)
	{
		return item.second == m_task_end_action_name;
	});
	if (find_item != m_WheelTaskEndActionData.end())
	{
		m_task_end_action_id = (*find_item).first;
	}
	return m_task_end_action_id;
}
///////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotTaskConfig::loadWheelTaskEndTypeData()
{
	m_WheelTaskEndTypeData.clear();
	WHEEL_ROBOT_DB.getWheelTaskEndTypeDB(m_WheelTaskEndTypeData);
}

std::map<int, QString> DLWheelRobotTaskConfig::getWheelTaskEndTypeDataMap()
{
	return m_WheelTaskEndTypeData;
}

QList<QString> DLWheelRobotTaskConfig::getWheelTaskEndTypeNameQList()
{
	m_WheelTaskEndTypeQList.clear();
	std::map<int, QString>::iterator iter;
	for (iter = m_WheelTaskEndTypeData.begin(); iter != m_WheelTaskEndTypeData.end(); iter++)
	{
		m_WheelTaskEndTypeQList.append(iter->second);
	}
	return m_WheelTaskEndTypeQList;
}

QString DLWheelRobotTaskConfig::getWheelTaskEndTypeNameQString(int m_task_end_type_id)
{
	QString m_task_end_type_name;
	if (m_WheelTaskEndTypeData.find(m_task_end_type_id) != m_WheelTaskEndTypeData.end())
	{
		m_task_end_type_name = m_WheelTaskEndTypeData.at(m_task_end_type_id);
	}
	return m_task_end_type_name;
}

int DLWheelRobotTaskConfig::getWheelTaskEndTypeIdInt(QString m_task_end_type_name)
{
	int m_task_end_type_id;
	auto find_item = std::find_if(m_WheelTaskEndTypeData.begin(), m_WheelTaskEndTypeData.end(),
		[m_task_end_type_name](const std::map<int, QString>::value_type item)
	{
		return item.second == m_task_end_type_name;
	});
	if (find_item != m_WheelTaskEndTypeData.end())
	{
		m_task_end_type_id = (*find_item).first;
	}
	return m_task_end_type_id;
}
///////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotTaskConfig::loadWheelTaskTypeData()
{
	m_WheelTaskTypeData.clear();
	WHEEL_ROBOT_DB.getWheelTaskTypeDB(m_WheelTaskTypeData);
}

std::map<int, QString> DLWheelRobotTaskConfig::getWheelTaskTypeDataMap()
{
	return m_WheelTaskTypeData;
}

QList<QString> DLWheelRobotTaskConfig::getWheelTaskTypeNameQList()
{
	m_WheelTaskTypeQList.clear();
	std::map<int, QString>::iterator iter;
	for (iter = m_WheelTaskTypeData.begin(); iter != m_WheelTaskTypeData.end(); iter++)
	{
		m_WheelTaskTypeQList.append(iter->second);
	}
	return m_WheelTaskTypeQList;
}

QString DLWheelRobotTaskConfig::getWheelTaskTypeNameQString(int m_task_type_id)
{
	QString m_task_type_name;
	if (m_WheelTaskTypeData.find(m_task_type_id) != m_WheelTaskTypeData.end())
	{
		m_task_type_name = m_WheelTaskTypeData.at(m_task_type_id);
	}
	return m_task_type_name;
}

int DLWheelRobotTaskConfig::getWheelTaskTypeIdInt(QString m_task_type_name)
{
	int m_task_type_id;
	auto find_item = std::find_if(m_WheelTaskTypeData.begin(), m_WheelTaskTypeData.end(),
		[m_task_type_name](const std::map<int, QString>::value_type item)
	{
		return item.second == m_task_type_name;
	});
	if (find_item != m_WheelTaskTypeData.end())
	{
		m_task_type_id = (*find_item).first;
	}
	return m_task_type_id;
}
///////////////////////////////////////////////////////////////////////////////////
void DLWheelRobotTaskConfig::loadWheelTaskStatusData()
{
	m_WheelTaskStatusData.clear();
	WHEEL_ROBOT_DB.getWheelTaskStatusDB(m_WheelTaskStatusData);
}
std::map<int, QString> DLWheelRobotTaskConfig::getWheelTaskStatusDataMap()
{
	return m_WheelTaskStatusData;
}