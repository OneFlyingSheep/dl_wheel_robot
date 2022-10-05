#pragma once
#include "common/DLWheelRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <QFile>
#include <QApplication>

#define BACKGROUND_CFG_FILE_PATH "/ConfigData/WheelRobotCfg/BackgroundCfg.ini"

class DLWheelRobotBackgroundConfig : public Singleton<DLWheelRobotBackgroundConfig>
{
public:
    DLWheelRobotBackgroundConfig();
    ~DLWheelRobotBackgroundConfig();

    void loadFromFile(QString cfgFilePath);
    WheelRobotBackgroundConfigStruct getCfg();
    WheelRobotBackgroundCoreCfg getCoreCfg();
    void initCoreCfg(WheelRobotBackgroundCoreCfg coreCfg);

    WheelRobotCoreRobotConfig getCoreRobotCfg();
    void initCoreRobotCfg(WheelRobotCoreRobotConfig cfg);

private:
    WheelRobotBackgroundConfigStruct m_backgroundCfg;
    WheelRobotBackgroundCoreCfg m_coreCfg;
    WheelRobotCoreRobotConfig m_coreRobotCfg;
};

#define WHEEL_ROBOT_BACKGROUND_CONFIG DLWheelRobotBackgroundConfig::GetSingleton()

