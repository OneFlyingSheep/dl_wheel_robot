#ifndef __DL_HANG_ROBOT_COMMON_TOOLS_H__
#define __DL_HANG_ROBOT_COMMON_TOOLS_H__

#include <QObject>

#define DB_CONFIG_DATA_PATH				 "/ConfigData/config.ini"			// ���ݿ������ļ�·��;
#define NETWORK_CONFIG_DATA_PATH		 "/ConfigData/network.ini"			// ���������ļ�·��;

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
	// ��ȡ���ݿ�������Ϣ;
	static DBInfoData_T getDBInfoData();
	// �������ݿ�������Ϣ;
	static void setDBInfoData(DBInfoData_T dbInfo);
	// ��������ļ������Ƿ�����;
	static bool checkConfigFile(QByteArray configDBName);

	// �ַ�������;
	static QString encryption(QByteArray strText);
	// �ַ�������;
	static QString decryption(QByteArray strText);

	// �������ļ��ж�ȡ����������Ϣ;
	static void getNetWorkInfo(QString& strIP, QString& strPort);
	// ������������Ϣд�������ļ���;
	static void saveNetWorkInfo(const QString& strIP, const QString& strPort);

	// ������ʽ��;
	static void loadStyleSheet(QWidget* widget, QString styleFilePath);
private:
	static QByteArray s_ConfigDBNam;
};

#endif