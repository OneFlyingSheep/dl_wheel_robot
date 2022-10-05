#pragma once
#include "common/DLWheelRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <QFile>
#include <QApplication>


#define CORE_CFG_FILE_PATH "/ConfigData/WheelRobotCfg/CoreCfg.ini"
#define ROBOT_CFG_FILE_PATH "/ConfigData/WheelRobotCfg/RobotCfg.ini"
#define BJSOCKET_CFG_FILE_PATH "/ConfigData/WheelRobotCfg/CoreBJCfg.ini"

class DLWheelRobotCoreConfig : public Singleton<DLWheelRobotCoreConfig>
{
public:
    DLWheelRobotCoreConfig();
    ~DLWheelRobotCoreConfig();

    void loadFromFile(QString cfgFilePath);
    WheelRobotCoreConfigStruct getCfg();
    WheelRobotCoreRobotConfig getCurrentRobotCfg();
    QList<WheelRobotCoreRobotConfig> getAllRobotCfg();
    bool changeCurrentRobot(QString robotName);

    void loadBJFromFile();
    BJWheelSocketConfig getBJSocketCfg();

private:
    WheelRobotCoreConfigStruct m_coreCfg;
    QList<WheelRobotCoreRobotConfig> m_allRobotCfg;
    WheelRobotCoreRobotConfig m_currentRobotCfg;
    BJWheelSocketConfig m_bjSocketCfg;
};

#define WHEEL_ROBOT_CORE_CONFIG DLWheelRobotCoreConfig::GetSingleton()

