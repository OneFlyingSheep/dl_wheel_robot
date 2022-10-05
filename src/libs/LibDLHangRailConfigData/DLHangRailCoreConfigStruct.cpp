#include "DLHangRailCoreConfigStruct.h"
#include <QSettings>

DLHangRailCoreConfigStruct::DLHangRailCoreConfigStruct()
{
    loadFromFile();
}

DLHangRailCoreConfigStruct::~DLHangRailCoreConfigStruct()
{

}
///////////////////////////////////////////////////////////////////////////////////
void DLHangRailCoreConfigStruct::loadFromFile()
{
	loadCoreFromFile();
	loadCoreBackFromFile();
}

void DLHangRailCoreConfigStruct::loadCoreFromFile()
{
	QString strFilePath = QApplication::applicationDirPath() + CORE_CFG_FILE_PATH;
	QSettings settings(strFilePath, QSettings::IniFormat);

	int i = 0;
	QString RCFHeader = "RCF_Server";
	QStringList RCFFiledList;
	RCFFiledList << RCFHeader + "/" + "Rcf_server_Ip" << RCFHeader + "/" + "Rcf_server_port"
		<< RCFHeader + "/" + "Rcf_Server_Path";
	m_coreCfg.Rcf_server_Ip = settings.value(RCFFiledList[i++]).toByteArray();
	m_coreCfg.Rcf_server_port = settings.value(RCFFiledList[i++]).toInt();

	i = 0;
	QString coreHeader = "Core";
	QStringList coreFiledList;
	coreFiledList << coreHeader + "/" + "corePcIp" << coreHeader + "/" + "coreServerPort" << coreHeader + "/" + "coreEnvPort" << coreHeader + "/" + "coreTftpServerPort";
	m_coreCfg.corePcIp = settings.value(coreFiledList[i++]).toByteArray();
	m_coreCfg.coreServerPort = settings.value(coreFiledList[i++]).toInt();
    m_coreCfg.coreEnvPort = settings.value(coreFiledList[i++]).toInt();
    m_coreCfg.coreTftpServerPort = settings.value(coreFiledList[i++]).toInt();

	i = 0;
	QString walkHeader = "Walk";
	QStringList walkFileList;
	walkFileList << walkHeader + "/" + "robotWalkBoxIpAddr" << walkHeader + "/" + "robotWalkBoxCtrlPort"
		<< walkHeader + "/" + "robotWalkBoxQueryPort";
	m_coreCfg.robotWalkBoxIpAddr = settings.value(walkFileList[i++]).toByteArray();
	m_coreCfg.robotWalkBoxCtrlPort = settings.value(walkFileList[i++]).toInt();
	m_coreCfg.robotWalkBoxQueryPort = settings.value(walkFileList[i++]).toInt();

	i = 0;
	QString clouHeader = "Cloud";
	QStringList cloudFileList;
	cloudFileList << clouHeader + "/" + "robotSensorIpAddr" << clouHeader + "/" + "robotSensorBoxCtrlPort"
		<< clouHeader + "/" + "robotSensorBoxQueryPort";
	m_coreCfg.robotSensorIpAddr = settings.value(cloudFileList[i++]).toByteArray();
	m_coreCfg.robotSensorBoxCtrlPort = settings.value(cloudFileList[i++]).toInt();
	m_coreCfg.robotSensorBoxQueryPort = settings.value(cloudFileList[i++]).toInt();

	i = 0;
	QString dbHeader = "database";
	QStringList dbFileList;
	dbFileList << dbHeader + "/" + "databaseLocalhost" << dbHeader + "/" + "databasePort"
		<< dbHeader + "/" + "databaseUsername" << dbHeader + "/" + "databasePassword"
		<< dbHeader + "/" + "databaseName";
	m_coreCfg.databaseLocalhost = settings.value(dbFileList[i++]).toByteArray();
	m_coreCfg.databasePort = settings.value(dbFileList[i++]).toInt();
	m_coreCfg.databaseUsername = settings.value(dbFileList[i++]).toByteArray();
	m_coreCfg.databasePassword = settings.value(dbFileList[i++]).toByteArray();
	m_coreCfg.databaseName = settings.value(dbFileList[i++]).toByteArray();

	i = 0;
	QString HCCameraHeader = "HCNetCamera";
	QStringList HCCameraFieldList;
	HCCameraFieldList
		<< HCCameraHeader + "/" + "hcIpaddr"
		<< HCCameraHeader + "/" + "hcPort"
		<< HCCameraHeader + "/" + "hcUserName"
		<< HCCameraHeader + "/" + "hcPassword";
	m_coreCfg.hcIpaddr = settings.value(HCCameraFieldList[i++]).toByteArray();
	m_coreCfg.hcPort = settings.value(HCCameraFieldList[i++]).toInt();
	m_coreCfg.hcUserName = settings.value(HCCameraFieldList[i++]).toByteArray();
	m_coreCfg.hcPassword = settings.value(HCCameraFieldList[i++]).toByteArray();

	i = 0;
	QString magHeader = "Magnity";
	QStringList magFieldList;
	magFieldList << magHeader + "/" + "magIpaddr";
	m_coreCfg.magIpaddr = settings.value(magFieldList[i++]).toByteArray();

	i = 0;
	QString fileHeader = "FilePath";
	QStringList filePathList;
	filePathList << fileHeader + "/" + "rootPath";
	m_coreCfg.rootPath = settings.value(filePathList[i++]).toByteArray();

}
void DLHangRailCoreConfigStruct::loadCoreBackFromFile()
{
	QString strFilePath = QApplication::applicationDirPath() + CORE_BACK_CFG_FILE_PATH;
	QSettings settings(strFilePath, QSettings::IniFormat);

	int i = 0;
	QString RouHeader = "Router";
	QStringList RouFiledList;
	RouFiledList << RouHeader + "/" + "RouterIp";
	m_coreBackCfg.routerIp = settings.value(RouFiledList[i++]).toByteArray();

	i = 0;
	QString RCFHeader = "RCF_Server";
	QStringList RCFFiledList;
	RCFFiledList << RCFHeader + "/" + "RcfServerPort";
	m_coreBackCfg.RcfServerPort = settings.value(RCFFiledList[i++]).toInt();

	i = 0;
	QString dbHeader = "database";
	QStringList dbFileList;
	dbFileList << dbHeader + "/" + "databaseLocalhost" << dbHeader + "/" + "databasePort"
		<< dbHeader + "/" + "databaseUsername" << dbHeader + "/" + "databasePassword"
		<< dbHeader + "/" + "databaseName";
	m_coreBackCfg.databaseLocalhost = settings.value(dbFileList[i++]).toByteArray();
	m_coreBackCfg.databasePort = settings.value(dbFileList[i++]).toInt();
	m_coreBackCfg.databaseUsername = settings.value(dbFileList[i++]).toByteArray();
	m_coreBackCfg.databasePassword = settings.value(dbFileList[i++]).toByteArray();
	m_coreBackCfg.databaseName = settings.value(dbFileList[i++]).toByteArray();

	i = 0;
	QString HCCameraHeader = "HCNetCamera";
	QStringList HCCameraFieldList;
	HCCameraFieldList
		<< HCCameraHeader + "/" + "hcnetPort"
		<< HCCameraHeader + "/" + "hcnetRTSPPort"
		<< HCCameraHeader + "/" + "hcnetUserName"
		<< HCCameraHeader + "/" + "hcnetPassword";
	m_coreBackCfg.hcnetPort = settings.value(HCCameraFieldList[i++]).toInt();
	m_coreBackCfg.hcnetRTSPPort = settings.value(HCCameraFieldList[i++]).toInt();
	m_coreBackCfg.hcnetUserName = settings.value(HCCameraFieldList[i++]).toByteArray();
	m_coreBackCfg.hcnetPassword = settings.value(HCCameraFieldList[i++]).toByteArray();
}

HangRailCoreConfigStruct DLHangRailCoreConfigStruct::getCoreCfg()
{
	return m_coreCfg;
}

HangRailCoreBackConfigStruct DLHangRailCoreConfigStruct::getCoreBackCfg()
{
    return m_coreBackCfg;
}

void DLHangRailCoreConfigStruct::initCoreBackCfg(HangRailCoreBackConfigStruct coreBackCfg)
{
	m_coreBackCfg = coreBackCfg;
}
