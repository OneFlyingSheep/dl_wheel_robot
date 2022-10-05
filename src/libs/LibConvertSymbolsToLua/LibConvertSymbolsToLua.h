#ifndef __LIB_CONVERT_STRING_TO_LUA__
#define __LIB_CONVERT_STRING_TO_LUA__

#include <QString>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <QVector>
#include "LibDLLuaScript/LuaThreshold.h"

class conventSymbols2Lua
{
public:
    conventSymbols2Lua();
    ~conventSymbols2Lua();

    bool convert(QVector<THRESHOLD_ELEMENT> &elementList, QString &luaString, DeviceAlarmLevel level, QString &errMsg);

    bool getLuaScript(QVector<THRESHOLD_ELEMENT> alarmNormalList, QVector<THRESHOLD_ELEMENT> alarmWarningList, 
        QVector<THRESHOLD_ELEMENT> alarmCommonList, QVector<THRESHOLD_ELEMENT> alarmSerialList, QVector<THRESHOLD_ELEMENT> alarmDangerList, 
        QString &luaString, QString &errMsg);

    bool getStructFromJson(QString jsonString, QVector<THRESHOLD_ELEMENT> &element);

private:
    inline void getLevelString(QVector<THRESHOLD_ELEMENT> &elementList, QString &luaString, DeviceAlarmLevel level, QString &errMsg);

    LuaThreshold m_luaScriptCheck;
};
#endif