#include "DLHangRailRobotConfigData.h"
#include <QSettings>
#include <QFile>

DLHangRailRobotConfigData::DLHangRailRobotConfigData()
{
    m_isLoadSuccess = false;
}


DLHangRailRobotConfigData::~DLHangRailRobotConfigData()
{

}

bool DLHangRailRobotConfigData::getIsLoadSuccess()
{
    return m_isLoadSuccess;
}

///////////////////////////////////////////////////////////////////////////////////


void DLHangRailRobotConfigData::loadFromFile(QString filePath /* = HANG_RAIL_BACKGROUND_CFG_FILE_PATH */)
{
    m_isLoadSuccess = true;
    QString strFilePath = QApplication::applicationDirPath() + filePath;
    bool isExist = QFile::exists(strFilePath);
    if (!isExist)
    {
        m_isLoadSuccess = false;
        return;
    }

    QSettings settings(strFilePath, QSettings::IniFormat);

    int i = 0;

    QString coreHeader = "core";
    QStringList coreFieldList;
    coreFieldList << coreHeader  + "/coreServerIp" << coreHeader + "/coreServerPort" << coreHeader + "/rootPath" << coreHeader + "/isShowAssistVideo";

    for (int i = 0; i < coreFieldList.count(); i++)
    {
        QString strValue = settings.value(coreFieldList[i]).toString();
        if (strValue.isEmpty())
        {
            m_isLoadSuccess = false;
            return;
        }
    }

    i = 0;
    m_backgroundCfg.coreServerIp = settings.value(coreFieldList[i++]).toByteArray();
    m_backgroundCfg.coreServerPort = settings.value(coreFieldList[i++]).toInt();
    m_backgroundCfg.strRootPath = settings.value(coreFieldList[i++]).toString();
    m_backgroundCfg.isShowAssistVideo = settings.value(coreFieldList[i++]).toBool();


}

void DLHangRailRobotConfigData::loadFromFileForPad(QString filePath /* = HANG_RAIL_BACKGROUND_CFG_Pad_FILE_PATH */)
{
    loadFromFile(filePath);
    if (m_isLoadSuccess)
    {
        QString strFilePath = QApplication::applicationDirPath() + filePath;
        QSettings settings(strFilePath, QSettings::IniFormat);

        QString coreHeader = "Login";
        QStringList coreFieldList;
        coreFieldList << coreHeader + "/LoginName" << coreHeader + "/LoginPsd";

        for (int i = 0; i < coreFieldList.count(); i++)
        {
            QString strValue = settings.value(coreFieldList[i]).toString();
            if (strValue.isEmpty())
            {
                m_isLoadSuccess = false;
                return;
            }
        }

        int i = 0;
        m_backgroundCfg.strLoginName = settings.value(coreFieldList[i++]).toByteArray();
        m_backgroundCfg.strLoginPsd = settings.value(coreFieldList[i++]).toByteArray();
    }
}

HangRailRobotBackgroundConfigStruct DLHangRailRobotConfigData::getCfg()
{
    return m_backgroundCfg;
}

HangRailRobotBackgroundConfigStructForPad DLHangRailRobotConfigData::getCfgForPad()
{
    return m_backgroundCfg;
}

HangRailCoreBackConfigStruct DLHangRailRobotConfigData::getCoreBackCfg()
{
    return m_coreCfg;
}

void DLHangRailRobotConfigData::initCoreBackCfg(HangRailCoreBackConfigStruct coreCfg)
{
    m_coreCfg = coreCfg;
}
