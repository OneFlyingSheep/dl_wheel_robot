#pragma once
#include <lua.hpp>
#include <common/DLWheelRobotGlobalDef.hpp>

class LuaThreshold
{
public:
	LuaThreshold();
	~LuaThreshold();

public:
    DeviceAlarmLevel thresholdCheck(QString res, QString device_uuid, WheelRobotMeterType meter_type_id);
    DeviceAlarmLevel thresholdThreePhase(QString aPhase, QString bPhase, QString cPhase, float &max_temp_diff);

    int checkLuaScript(QString luaPath, QString &errMsg);
	DeviceAlarmLevel virtualLuaScript(QString res, QString virtual_uuid);

	//·µ»Ø¼ÙµÄÖµ
	QString phonyValueScript(QString uuid_);

	bool getLoginData(QStringList &_data);
};

