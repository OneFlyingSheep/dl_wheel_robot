#ifndef __DL_HANG_ROBOT_ENV_STATUS_H__
#define __DL_HANG_ROBOT_ENV_STATUS_H__

#include "common/DLHangRailRobotGlobalDef.hpp"
#include "LibDLHangRailRobotDBOperation/LibDLHangRailRobotDBOperation.h"
#include "common/Singleton.hpp"


class DLHangRailEnvStatusData : public Singleton<DLHangRailEnvStatusData>
{
public:
    DLHangRailEnvStatusData();
    ~DLHangRailEnvStatusData();

public:
    void setEnvStatusTemperature(float temperature);
    void setEnvStatusHumidity(float hum);
    void setEnvStatusSf6(float sf6);
    void setEnvStatusO3(float o3);

    envType getEnvStatus();

private:
    envType m_robotEnvStatus;
};

#define ROBOT_ENV_STATUS DLHangRailEnvStatusData::GetSingleton()

#endif