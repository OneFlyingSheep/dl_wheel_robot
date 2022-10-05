#include "DLWheelRobotCoreConfig.h"
#include <QSettings>

DLWheelRobotCoreConfig::DLWheelRobotCoreConfig()
{
    loadFromFile("");
    loadBJFromFile();
}


DLWheelRobotCoreConfig::~DLWheelRobotCoreConfig()
{

}
///////////////////////////////////////////////////////////////////////////////////


void DLWheelRobotCoreConfig::loadFromFile(QString cfgFilePath)
{
    QString coreFilePath = QApplication::applicationDirPath() + CORE_CFG_FILE_PATH;
    QSettings coreSettings(coreFilePath, QSettings::IniFormat);

    int i = 0;
    
    QString coreServerHeader = "coreServer";
    QStringList coreServerFieldList;
    coreServerFieldList
        << coreServerHeader + "/" + "coreServerIp"
        << coreServerHeader + "/" + "coreServerPort"
        << coreServerHeader + "/" + "rcfServerPort"
        << coreServerHeader + "/" + "rootPath"
        << coreServerHeader + "/" + "databaseLocal"
        << coreServerHeader + "/" + "databaseRemoteIp"
        << coreServerHeader + "/" + "databasePort"
        << coreServerHeader + "/" + "databaseUsername"
        << coreServerHeader + "/" + "databasePassword"
        << coreServerHeader + "/" + "databaseName"
        << coreServerHeader + "/" + "defaultRobotName"
        << coreServerHeader + "/" + "infraredManufacturer"
        << coreServerHeader + "/" + "temporaryDoorIp"
        << coreServerHeader + "/" + "temporaryDoorPort"
		<< coreServerHeader + "/" + "ftpServerIp"
		<< coreServerHeader + "/" + "ftpServerPort"
		<< coreServerHeader + "/" + "ftpUserName"
		<< coreServerHeader + "/" + "ftpUserPasswd"
		<< coreServerHeader + "/" + "recordPeriod";

    i = 0;

    m_coreCfg.coreServerIp = coreSettings.value(coreServerFieldList[i++]).toByteArray();
    m_coreCfg.coreServerPort = coreSettings.value(coreServerFieldList[i++]).toInt();
    m_coreCfg.rcfServerPort = coreSettings.value(coreServerFieldList[i++]).toInt();
    m_coreCfg.rootPath = QApplication::applicationDirPath() + "/" + coreSettings.value(coreServerFieldList[i++]).toByteArray();
    m_coreCfg.databaseLocal = coreSettings.value(coreServerFieldList[i++]).toByteArray();
    m_coreCfg.databaseRemoteIp = coreSettings.value(coreServerFieldList[i++]).toByteArray();
    m_coreCfg.databasePort = coreSettings.value(coreServerFieldList[i++]).toInt();
    m_coreCfg.databaseUsername = coreSettings.value(coreServerFieldList[i++]).toByteArray();
    m_coreCfg.databasePassword = QByteArray::fromBase64(coreSettings.value(coreServerFieldList[i++]).toByteArray());
    m_coreCfg.databaseName = coreSettings.value(coreServerFieldList[i++]).toByteArray();
    m_coreCfg.defaultRobotName = coreSettings.value(coreServerFieldList[i++]).toByteArray();
    m_coreCfg.infraredManufacturer = coreSettings.value(coreServerFieldList[i++]).toByteArray().split('|').first().toInt();
    m_coreCfg.temporaryDoorIp = coreSettings.value(coreServerFieldList[i++]).toByteArray();
    m_coreCfg.temporaryDoorPort = coreSettings.value(coreServerFieldList[i++]).toInt();
	m_coreCfg.ftpServerIp = coreSettings.value(coreServerFieldList[i++]).toByteArray();
	m_coreCfg.ftpServerPort = coreSettings.value(coreServerFieldList[i++]).toByteArray().toInt();
	m_coreCfg.ftpUserName = coreSettings.value(coreServerFieldList[i++]).toByteArray();
	m_coreCfg.ftpUserPasswd = coreSettings.value(coreServerFieldList[i++]).toByteArray();
	m_coreCfg.recordPeriod = coreSettings.value(coreServerFieldList[i++]).toByteArray().toInt();

    QString robotFilePath = QApplication::applicationDirPath() + ROBOT_CFG_FILE_PATH;
    QSettings robotSettings(robotFilePath, QSettings::IniFormat);
    robotSettings.setIniCodec("GBK");
    QStringList robotList = robotSettings.childGroups();

    for (int j = 0; j < robotList.size(); j++)
    {
        WheelRobotCoreRobotConfig tmp;
        QString coreServerHeader = robotList[j];
        QStringList coreServerFieldList;
        coreServerFieldList
            << coreServerHeader + "/" + "robotIp"
            << coreServerHeader + "/" + "robotName"
            << coreServerHeader + "/" + "robotPort"

            << coreServerHeader + "/" + "hcUserName"
            << coreServerHeader + "/" + "hcPassword"

			<< coreServerHeader + "/" + "hcIP"
			<< coreServerHeader + "/" + "hcCtrlPort"
			<< coreServerHeader + "/" + "hcRtspPort"

			<< coreServerHeader + "/" + "hcIPFront"
			<< coreServerHeader + "/" + "hcFrontCtrlPort"
			<< coreServerHeader + "/" + "hcFrontRtspPort"


			<< coreServerHeader + "/" + "hcIPBack"
			<< coreServerHeader + "/" + "hcBackCtrlPort"
			<< coreServerHeader + "/" + "hcBackRtspPort"


			<< coreServerHeader + "/" + "infraredNeed"
			<< coreServerHeader + "/" + "infraredCameraIp"
			<< coreServerHeader + "/" + "infraredCtrlPort"
			<< coreServerHeader + "/" + "infraredRtspPort";

        i = 0;
        tmp.robotIp = robotSettings.value(coreServerFieldList[i++]).toByteArray();
        tmp.robotName = robotSettings.value(coreServerFieldList[i++]).toString();
        tmp.robotPort = robotSettings.value(coreServerFieldList[i++]).toInt();
        tmp.hcUserName = robotSettings.value(coreServerFieldList[i++]).toByteArray();
        tmp.hcPassword = robotSettings.value(coreServerFieldList[i++]).toByteArray();

		tmp.hcIP = robotSettings.value(coreServerFieldList[i++]).toByteArray();
		tmp.hcCtrlPort = robotSettings.value(coreServerFieldList[i++]).toInt();
		tmp.hcRtspPort = robotSettings.value(coreServerFieldList[i++]).toInt();

		tmp.hcIPFront = robotSettings.value(coreServerFieldList[i++]).toByteArray();
		tmp.hcFrontCtrlPort = robotSettings.value(coreServerFieldList[i++]).toInt();
		tmp.hcFrontRtspPort = robotSettings.value(coreServerFieldList[i++]).toInt();

		tmp.hcIPBack = robotSettings.value(coreServerFieldList[i++]).toByteArray();
		tmp.hcBackCtrlPort = robotSettings.value(coreServerFieldList[i++]).toInt();
		tmp.hcBackRtspPort = robotSettings.value(coreServerFieldList[i++]).toInt();

		tmp.infraredNeed = robotSettings.value(coreServerFieldList[i++]).toInt();
		tmp.infraredCameraIp = robotSettings.value(coreServerFieldList[i++]).toByteArray();
		tmp.infraredCtrlPort = robotSettings.value(coreServerFieldList[i++]).toInt();
		tmp.infraredRtspPort = robotSettings.value(coreServerFieldList[i++]).toInt();

        m_allRobotCfg.push_back(tmp);

        if (robotList[j] == m_coreCfg.defaultRobotName)
        {
            m_currentRobotCfg = tmp;
        }
    }
}

WheelRobotCoreConfigStruct DLWheelRobotCoreConfig::getCfg()
{
    return m_coreCfg;
}

WheelRobotCoreRobotConfig DLWheelRobotCoreConfig::getCurrentRobotCfg()
{
    return m_currentRobotCfg;
}

QList<WheelRobotCoreRobotConfig> DLWheelRobotCoreConfig::getAllRobotCfg()
{
    return m_allRobotCfg;
}

bool DLWheelRobotCoreConfig::changeCurrentRobot(QString robotName)
{
    bool findFlag = false;
    for (int i = 0; i < m_allRobotCfg.size(); i++)
    {
        if (m_allRobotCfg[i].robotName == robotName)
        {
            findFlag = true;
            m_currentRobotCfg = m_allRobotCfg[i];
        }
    }

    return findFlag;
}

void DLWheelRobotCoreConfig::loadBJFromFile()
{
    QString robotFilePath = QApplication::applicationDirPath() + BJSOCKET_CFG_FILE_PATH;
    QSettings settings(robotFilePath, QSettings::IniFormat);

    int i = 0;

    m_bjSocketCfg.socket_connect_ip = settings.value("BJSocket/socket_connect_ip").toByteArray();
    m_bjSocketCfg.socket_connect_port = settings.value("BJSocket/socket_connect_port").toByteArray();
    m_bjSocketCfg.send_code = settings.value("BJSocket/send_code").toByteArray();
    m_bjSocketCfg.receive_code = settings.value("BJSocket/receive_code").toByteArray();
    m_bjSocketCfg.robot_name = settings.value("BJSocket/robot_name").toByteArray();
    m_bjSocketCfg.robot_code = settings.value("BJSocket/robot_code").toByteArray();
    m_bjSocketCfg.robot_main_code = settings.value("BJSocket/robot_main_code").toByteArray();
    m_bjSocketCfg.station_code = settings.value("BJSocket/station_code").toByteArray();

    m_bjSocketCfg.ftps_connect_ip = settings.value("Ftps/ftps_connect_ip").toByteArray();
    m_bjSocketCfg.ftps_connect_port = settings.value("Ftps/ftps_connect_port").toByteArray();
    m_bjSocketCfg.ftps_username = settings.value("Ftps/ftps_username").toByteArray();
    m_bjSocketCfg.ftps_passwd = settings.value("Ftps/ftps_passwd").toByteArray();

    m_bjSocketCfg.imageResolution = settings.value("Map/ImageResolution").toDouble();
    m_bjSocketCfg.centerX = settings.value("Map/CentreX").toDouble();
    m_bjSocketCfg.centerY = settings.value("Map/CentreY").toDouble();
}

BJWheelSocketConfig DLWheelRobotCoreConfig::getBJSocketCfg()
{
    return m_bjSocketCfg;
}
