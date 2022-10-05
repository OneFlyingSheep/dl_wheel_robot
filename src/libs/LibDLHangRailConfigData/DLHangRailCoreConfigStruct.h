#pragma once
#include "common/DLHangRailRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <QFile>
#include <QApplication>

#define CORE_CFG_FILE_PATH "/ConfigData/HangRailRobotCfg/CoreCfg.ini"
#define CORE_BACK_CFG_FILE_PATH "/ConfigData/HangRailRobotCfg/CoreBackCfg.ini"

class DLHangRailCoreConfigStruct : public Singleton<DLHangRailCoreConfigStruct>
{
public:
	DLHangRailCoreConfigStruct();
    ~DLHangRailCoreConfigStruct();

	void loadFromFile();
	void loadCoreFromFile();
	void loadCoreBackFromFile();

	HangRailCoreConfigStruct getCoreCfg();
	HangRailCoreBackConfigStruct getCoreBackCfg();

	void initCoreBackCfg(HangRailCoreBackConfigStruct coreBackCfg);

private:

	HangRailCoreConfigStruct m_coreCfg;
	HangRailCoreBackConfigStruct m_coreBackCfg;
};

#define HANGRAIL_ROBOT_CORE_CONFIG DLHangRailCoreConfigStruct::GetSingleton()

