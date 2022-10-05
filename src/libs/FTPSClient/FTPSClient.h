#ifndef FTPSCLINET_H
#define FTPSCLINET_H

#include<QObject>
#include <QProcess>


class FTPSClient : public QObject
{
	Q_OBJECT

public:
	FTPSClient(QObject *parnt = 0);
	~FTPSClient();
	void initClient(const QString &ip, int port, const QString &user, const QString &passwd_);
    void upLoadFile(const QString &local_file, const QString &remote_file);	//local_file本地带文件名绝对路径，remote_file远端服务器根目录相对路径带文件名
    void upLoadFile(const QString &local_file, const QString &remote_file, const QString &remote_name);	//local_file本地带文件名绝对路径，remote_file远端服务器根目录相对路径带文件名

signals:
	void sigUploadState(int retcode);	//0成功，1失败

	private slots:
	void slot_on_finished(int exitCode, QProcess::ExitStatus exitStatus);

private:
	QProcess * handle_;
	QString ip_;
	int port_;
	QString user_;
	QString passwd_;
};





#endif // !ASTARCAL_H