#include "DLHangRailCommonTools.h"
#include <QSettings>
#include <QFile>
#include <QDebug>
#include <QApplication>
#include <QWidget>

QByteArray DLHangRailCommonTools::s_ConfigDBNam = "";

DBInfoData_T DLHangRailCommonTools::getDBInfoData()
{
	DBInfoData_T dbInfoData;
	QString strFilePath = QApplication::applicationDirPath() + DB_CONFIG_DATA_PATH;
	QSettings settings(strFilePath, QSettings::IniFormat);
	QString infoHeader = encryption(s_ConfigDBNam);
	QStringList fieldList;
	fieldList << infoHeader + "/" + encryption("HostName") << infoHeader + "/" + encryption("DBName")
		<< infoHeader + "/" + encryption("UserName") << infoHeader + "/" + encryption("PassWord");

	dbInfoData.hostName = decryption(settings.value(fieldList[0]).toByteArray());
	dbInfoData.dbName = decryption(settings.value(fieldList[1]).toByteArray());
	dbInfoData.userName = decryption(settings.value(fieldList[2]).toByteArray());
	dbInfoData.password = decryption(settings.value(fieldList[3]).toByteArray());

	return dbInfoData;
}

void DLHangRailCommonTools::setDBInfoData(DBInfoData_T dbInfo)
{
	QString strFilePath = QApplication::applicationDirPath() + DB_CONFIG_DATA_PATH;
	QFile file(strFilePath);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
	{
		qDebug() << "打开失败";
	}
	QSettings *setIni = new QSettings(strFilePath, QSettings::IniFormat);

	//设置键值信息;
	QString iniHeader = encryption(s_ConfigDBNam);
	setIni->beginGroup(iniHeader);
	setIni->setValue(encryption("HostName"), encryption(dbInfo.hostName.toUtf8()));
	setIni->setValue(encryption("DBName"), encryption(dbInfo.dbName.toUtf8()));
	setIni->setValue(encryption("UserName"), encryption(dbInfo.userName.toUtf8()));
	setIni->setValue(encryption("PassWord"), encryption(dbInfo.password.toUtf8()));
	setIni->endGroup();
}

bool DLHangRailCommonTools::checkConfigFile(QByteArray configDBName)
{
	// 检查配置文件信息时把配置文件中要查找的数据库名称保存;
	s_ConfigDBNam = configDBName;
	QString strFilePath = QApplication::applicationDirPath() + DB_CONFIG_DATA_PATH;
	bool isExist = QFile::exists(strFilePath);
	if (!isExist)
	{
		return false;
	}

	QString infoHeader = encryption(s_ConfigDBNam);
	QStringList fieldList;
	fieldList << infoHeader + "/" + encryption("HostName") << infoHeader + "/" + encryption("DBName")
		<< infoHeader + "/" + encryption("UserName") << infoHeader + "/" + encryption("PassWord");

	QSettings settings(strFilePath, QSettings::IniFormat);
	for (int i = 0; i < fieldList.count(); i++)
	{
		QString strField = settings.value(fieldList[i]).toString();
		if (strField.isEmpty())
		{
			return false;
		}
	}

	return true;
}

QString DLHangRailCommonTools::encryption(QByteArray strText)
{
	return strText.toBase64();
}

QString DLHangRailCommonTools::decryption(QByteArray strText)
{
	return QByteArray::fromBase64(strText);
}

void DLHangRailCommonTools::getNetWorkInfo(QString& strIP, QString& strPort)
{
	DBInfoData_T dbInfoData;
	QString strFilePath = QApplication::applicationDirPath() + NETWORK_CONFIG_DATA_PATH;
	QSettings settings(strFilePath, QSettings::IniFormat);
	QString infoHeader = "NetWorkInfo";
	QStringList fieldList;
	fieldList << infoHeader + "/IP" << infoHeader + "/Port";

	strIP = settings.value(fieldList[0]).toString();
	strPort = settings.value(fieldList[1]).toString();
}

void DLHangRailCommonTools::saveNetWorkInfo(const QString& strIP, const QString& strPort)
{
	QString strFilePath = QApplication::applicationDirPath() + NETWORK_CONFIG_DATA_PATH;
	QFile file(strFilePath);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
	{
		qDebug() << "打开失败";
	}
	QSettings *setIni = new QSettings(strFilePath, QSettings::IniFormat);

	//设置键值信息;
	QString iniHeader = "NetWorkInfo";
	setIni->beginGroup(iniHeader);
	setIni->setValue("IP", strIP);
	setIni->setValue("Port", strPort);
	setIni->endGroup();
}

void DLHangRailCommonTools::loadStyleSheet(QWidget* widget, QString styleFilePath)
{
	QFile file(styleFilePath);
	file.open(QFile::ReadOnly);
	if (file.isOpen())
	{
		if (widget != NULL)
		{
			QString styleSheet = widget->styleSheet();
			styleSheet += QLatin1String(file.readAll());
			widget->setStyleSheet(styleSheet);
		}
	}
}