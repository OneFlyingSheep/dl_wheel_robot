#include "FTPSClient.h"
#include <QCoreApplication>
#include <QDebug>
#include <windows.h>

const int kCost1 = 10; //直移一格消耗 
const int kCost2 = 14; //斜移一格消耗 


FTPSClient::FTPSClient(QObject *parnt)
{
	handle_ = new QProcess(this);
	connect(handle_, SIGNAL(finished(int, QProcess::ExitStatus)), SLOT(slot_on_finished(int, QProcess::ExitStatus)));
}


FTPSClient::~FTPSClient()
{
	handle_->close();
	if (handle_ != NULL) {
		delete handle_;
		handle_ = NULL;
	}
}

void FTPSClient::initClient(const QString &ip, int port, const QString &user, const QString &passwd)
{
	ip_ = ip;
	port_ = port;
	user_ = user;
	passwd_ = passwd;
}


void FTPSClient::upLoadFile(const QString &local_file, const QString &remote_file)
{
	if (ip_.isEmpty() || ip_.isNull())
		return;

	//"curl --trace -n -v --ssl-reqd -k  -u alex:hw657324 -T D:\FTP\settings.py ftp://127.0.0.1:21/1.py";
	QString program = QCoreApplication::applicationDirPath() + "/curl.exe ";


//     QString remote_dir = remote_file.mid(0, remote_file.lastIndexOf('/')) + "/";
// 
     QString remote_dir = "test_2/test_3";



//    QString mkdir_cmd = program + QString("--trace -n -v --ssl-reqd -k  -u %1:%2 ftp://%3:%4/robot/ -X \"MKD %5\"")
    QString mkdir_cmd = program + QString("--trace -n -v --ssl-reqd -k  -u %1:%2 ftp://%3:%4 -X \"MKD %5\"")
        .arg(user_).arg(passwd_).arg(ip_).arg(QString::number(port_)).arg(remote_dir);
    system(mkdir_cmd.toStdString().c_str());
//     Sleep(1000);
// 
// 	QString cmd = program + QString("--trace -n -v --ssl-reqd -k  -u %1:%2 -T %3 ftp://%4:%5/%6")\
// 		.arg(user_).arg(passwd_).arg(local_file).arg(ip_).arg(QString::number(port_)).arg(remote_file);
// 	qDebug() << program << " ===== " << cmd;
// 	system(cmd.toStdString().c_str());

}


void FTPSClient::upLoadFile(const QString &local_file, const QString &remote_file, const QString &remote_name)
{
    if (ip_.isEmpty() || ip_.isNull())
        return;

    //"curl --trace -n -v --ssl-reqd -k  -u alex:hw657324 -T D:\FTP\settings.py ftp://127.0.0.1:21/1.py";
    QString program = QCoreApplication::applicationDirPath() + "/curl.exe ";
    QStringList pathList = remote_file.split("/");
    QString remote_dir = "";
    for (int i = 0; i < pathList.size(); i++)
    {
        if (i == 0)
        {
            remote_dir = pathList[i];
        }
        else
        {
            remote_dir = remote_dir + "/" + pathList[i];
        }
        QString mkdir_cmd = program + QString("--trace -n -v --ssl-reqd -k  -u %1:%2 ftp://%3:%4 -X \"MKD %5\"")
            .arg(user_).arg(passwd_).arg(ip_).arg(QString::number(port_)).arg(remote_dir);
        system(mkdir_cmd.toStdString().c_str());
        Sleep(300);
    }

    QString cmd = program + QString("--trace -n -v --ssl-reqd -k  -u %1:%2 -T %3 ftp://%4:%5/%6")\
        .arg(user_).arg(passwd_).arg(local_file).arg(ip_).arg(QString::number(port_)).arg(remote_file + "/" + remote_name);
	qDebug() << program << " ===== " << cmd;
	system(cmd.toStdString().c_str());
}

void FTPSClient::slot_on_finished(int exitCode, QProcess::ExitStatus exitStatus)
{
	emit sigUploadState(exitCode);
}


