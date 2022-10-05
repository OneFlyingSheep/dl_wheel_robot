#pragma once
#include <map>
#include <string>
#include "common/DLWheelRobotGlobalDef.hpp"

class AlarmMessageManager
{
public:
    static AlarmMessageManager *GetInstance();
    void GetAlarmMessage(int iId, AlarmMessage &alarmMessage);

private:
    AlarmMessageManager();
    ~AlarmMessageManager();
    void Init();
    void InsertAlarmMessage(int iID, std::string strSpeakAlarmMessage, std::string strDisplayAlarmMessage, std::string strErrorCode = "");

private:
    std::map<int, AlarmMessage> m_mapID2AlarmMessage;
    static AlarmMessageManager *m_pAlarmMsgManager;
};
