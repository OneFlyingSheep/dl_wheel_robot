#pragma once
#include "common/DLHangRailRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <QFile>
#include <QApplication>

#define HANG_RAIL_BACKGROUND_CFG_FILE_PATH "/ConfigData/HangRailRobotCfg/BackgroundCfg.ini"
#define HANG_RAIL_BACKGROUND_CFG_Pad_FILE_PATH "/ConfigData/HangRailRobotCfg/BackgroundCfgForPad.ini"

class DLHangRailRobotConfigData : public Singleton<DLHangRailRobotConfigData>
{
public:
    DLHangRailRobotConfigData();
    ~DLHangRailRobotConfigData();

    bool getIsLoadSuccess();

    void loadFromFile(QString filePath = HANG_RAIL_BACKGROUND_CFG_FILE_PATH);
    void loadFromFileForPad(QString filePath = HANG_RAIL_BACKGROUND_CFG_Pad_FILE_PATH);

    HangRailRobotBackgroundConfigStruct getCfg();
    HangRailRobotBackgroundConfigStructForPad getCfgForPad();

    HangRailCoreBackConfigStruct getCoreBackCfg();

    void initCoreBackCfg(HangRailCoreBackConfigStruct coreCfg);

private:
    HangRailRobotBackgroundConfigStructForPad m_backgroundCfg;
    HangRailCoreBackConfigStruct m_coreCfg;
    bool m_isLoadSuccess;
};

#define HANG_RAIL_ROBOT_BACKGROUND_CONFIG DLHangRailRobotConfigData::GetSingleton()

