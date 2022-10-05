#include "DLWheelRobotStationConfig.h"



DLWheelRobotStationConfig::DLWheelRobotStationConfig()
{
	loadWheelStationConfigData();
}


DLWheelRobotStationConfig::~DLWheelRobotStationConfig()
{
}

std::map<int, WheelStationConfigStruct> DLWheelRobotStationConfig::getWheelStationConfigData()
{
	return m_WheelStationConfigData;
}

QList<WheelUserConfig> DLWheelRobotStationConfig::getWheelUserConfigData(WheelUserType nowType)
{
	QList<WheelUserConfig> conData;
	WHEEL_ROBOT_DB.getWheelUserConfigDataDB(conData, nowType);
	return conData;
}

QList<WheelUserConfig> DLWheelRobotStationConfig::getWheelAllUserConfigData()
{
	QList<WheelUserConfig> conData;
	WHEEL_ROBOT_DB.getWheelAllUserConfigDataDB(conData);
	return conData;
}

void DLWheelRobotStationConfig::loadWheelStationConfigData()
{
	m_WheelStationConfigData.clear();
	WHEEL_ROBOT_DB.getWheelStationConfigDataDB(m_WheelStationConfigData);
}