#include "DLHangRailEnvStatusData.h"

DLHangRailEnvStatusData::DLHangRailEnvStatusData()
{

}

DLHangRailEnvStatusData::~DLHangRailEnvStatusData()
{

}

void DLHangRailEnvStatusData::setEnvStatusTemperature(float temperature)
{
    m_robotEnvStatus.envTemperature = temperature;
}

void DLHangRailEnvStatusData::setEnvStatusHumidity(float hum)
{
    m_robotEnvStatus.envHumidity = hum;
}

void DLHangRailEnvStatusData::setEnvStatusSf6(float sf6)
{
    m_robotEnvStatus.envSf6 = sf6;
}

void DLHangRailEnvStatusData::setEnvStatusO3(float o3)
{
    m_robotEnvStatus.envO3 = o3;
}

envType DLHangRailEnvStatusData::getEnvStatus()
{
    return m_robotEnvStatus;
}
