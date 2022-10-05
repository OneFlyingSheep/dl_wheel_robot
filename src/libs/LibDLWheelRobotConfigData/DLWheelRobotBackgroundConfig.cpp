#include "DLWheelRobotBackgroundConfig.h"
#include <QSettings>

DLWheelRobotBackgroundConfig::DLWheelRobotBackgroundConfig()
{

}


DLWheelRobotBackgroundConfig::~DLWheelRobotBackgroundConfig()
{

}
///////////////////////////////////////////////////////////////////////////////////


void DLWheelRobotBackgroundConfig::loadFromFile(QString cfgFilePath)
{
    QString strFilePath = QApplication::applicationDirPath() + BACKGROUND_CFG_FILE_PATH;
    QSettings settings(strFilePath, QSettings::IniFormat);

    int i = 0;

    QString coreHeader = "cfg";
    QStringList coreFieldList;
    coreFieldList 
        << coreHeader + "/" + "coreServerIp" 
        << coreHeader + "/" + "coreServerPort" 
        << coreHeader + "/" + "rootPath";

    i = 0;
    m_backgroundCfg.coreServerIp = settings.value(coreFieldList[i++]).toByteArray();
    m_backgroundCfg.coreServerPort = settings.value(coreFieldList[i++]).toInt();
    m_backgroundCfg.rootPath = QApplication::applicationDirPath() + "/" + settings.value(coreFieldList[i++]).toByteArray();

    i = 0;
    QString voiceTalkHeader = "VoiceTalk";
    QStringList voiceTalkFieldList;
    voiceTalkFieldList
        << voiceTalkHeader + "/" + "cameraIp"
        << voiceTalkHeader + "/" + "cameraPort"
        << voiceTalkHeader + "/" + "cameraUser"
        << voiceTalkHeader + "/" + "cameraPassword";

    i = 0;
    m_backgroundCfg.voiceTalkCameraIp = settings.value(voiceTalkFieldList[i++]).toByteArray();
    m_backgroundCfg.voiceTalkCameraPort = settings.value(voiceTalkFieldList[i++]).toByteArray();
    m_backgroundCfg.voiceTalkCameraUser = settings.value(voiceTalkFieldList[i++]).toByteArray();
    m_backgroundCfg.voiceTalkCameraPassword = settings.value(voiceTalkFieldList[i++]).toByteArray();

    i = 0;
    QString communicationHeader = "CommunicationStatus";
    QStringList communicationIPList;
    communicationIPList
        << communicationHeader + "/" + "wirelessStationIp"
        << communicationHeader + "/" + "controlSystemIp"
        << communicationHeader + "/" + "visibleCameraIp"
        << communicationHeader + "/" + "chargeSystemIp"
		<< communicationHeader + "/" + "infraredCameraIp" 
		<< communicationHeader + "/" + "firRobotIp"; 

    i = 0;
    m_backgroundCfg.wirelessStationIp = settings.value(communicationIPList[i++]).toByteArray();
    m_backgroundCfg.controlSystemIp = settings.value(communicationIPList[i++]).toByteArray();
    m_backgroundCfg.visibleCameraIp = settings.value(communicationIPList[i++]).toByteArray();
    m_backgroundCfg.chargeSystemIp = settings.value(communicationIPList[i++]).toByteArray();
	m_backgroundCfg.infraredCameraIp = settings.value(communicationIPList[i++]).toByteArray();
	m_backgroundCfg.firRobotIp = settings.value(communicationIPList[i++]).toByteArray();

	m_backgroundCfg.strDefaultMapName = settings.value("DefaultMap/mapname").toByteArray();				//默认地图的名字
	m_backgroundCfg.strDefaultBgName = settings.value("DefaultBgImage/bgname").toByteArray();			//默认背景图的名字

	//加载zoom表
	m_backgroundCfg.strCheckZoomTxt = settings.value("ZoomCaptureTxt/checkZoomTxt").toByteArray();

	m_backgroundCfg.strRtpIp = settings.value("RtpClient/rtpIp").toByteArray();
	m_backgroundCfg.iRtpSendPort = settings.value("RtpClient/rtpSendPort").toInt();
	m_backgroundCfg.iRtpReceivePort = settings.value("RtpClient/rtpReceivePort").toInt();

}

WheelRobotBackgroundConfigStruct DLWheelRobotBackgroundConfig::getCfg()
{
    return m_backgroundCfg;
}

WheelRobotBackgroundCoreCfg DLWheelRobotBackgroundConfig::getCoreCfg()
{
    return m_coreCfg;
}

void DLWheelRobotBackgroundConfig::initCoreCfg(WheelRobotBackgroundCoreCfg coreCfg)
{
    m_coreCfg = coreCfg;
}

WheelRobotCoreRobotConfig DLWheelRobotBackgroundConfig::getCoreRobotCfg()
{
    return m_coreRobotCfg;
}

void DLWheelRobotBackgroundConfig::initCoreRobotCfg(WheelRobotCoreRobotConfig cfg)
{
    m_coreRobotCfg = cfg;
}
