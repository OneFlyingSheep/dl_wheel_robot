#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "common/Singleton.hpp"

class DLWheelRobotStationConfig : public Singleton<DLWheelRobotStationConfig>
{
public:
	DLWheelRobotStationConfig();
	~DLWheelRobotStationConfig();

public:
	std::map<int, WheelStationConfigStruct> getWheelStationConfigData();

	QList<WheelUserConfig> getWheelUserConfigData(WheelUserType nowType);
	QList<WheelUserConfig> getWheelAllUserConfigData();

private:
	void loadWheelStationConfigData();

private:
	std::map<int, WheelStationConfigStruct> m_WheelStationConfigData;
};

#define WHEEL_STATION_CONFIG DLWheelRobotStationConfig::GetSingleton()