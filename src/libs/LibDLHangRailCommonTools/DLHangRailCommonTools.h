#ifndef __DL_HANG_ROBOT_COMMON_TOOLS_H__
#define __DL_HANG_ROBOT_COMMON_TOOLS_H__

#include <QObject>

#define DB_CONFIG_DATA_PATH				 "/ConfigData/config.ini"			// 数据库配置文件路径;
#define NETWORK_CONFIG_DATA_PATH		 "/ConfigData/network.ini"			// 网络配置文件路径;

typedef struct DBInfoData
{
	QString hostName;
	QString dbName;
	QString userName;
	QString password;
}DBInfoData_T;

class DLHangRailCommonTools : public QObject
{
	Q_OBJECT

public:
	// 获取数据库连接信息;
	static DBInfoData_T getDBInfoData();
	// 设置数据库连接信息;
	static void setDBInfoData(DBInfoData_T dbInfo);
	// 检查配置文件数据是否完整;
	static bool checkConfigFile(QByteArray configDBName);

	// 字符串加密;
	static QString encryption(QByteArray strText);
	// 字符串解密;
	static QString decryption(QByteArray strText);

	// 从配置文件中读取网络设置信息;
	static void getNetWorkInfo(QString& strIP, QString& strPort);
	// 将网络设置信息写入配置文件中;
	static void saveNetWorkInfo(const QString& strIP, const QString& strPort);

	// 加载样式表;
	static void loadStyleSheet(QWidget* widget, QString styleFilePath);
private:
	static QByteArray s_ConfigDBNam;
};

#endif