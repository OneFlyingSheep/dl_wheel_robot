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
    void upLoadFile(const QString &local_file, const QString &remote_file);	//local_file���ش��ļ�������·����remote_fileԶ�˷�������Ŀ¼���·�����ļ���
    void upLoadFile(const QString &local_file, const QString &remote_file, const QString &remote_name);	//local_file���ش��ļ�������·����remote_fileԶ�˷�������Ŀ¼���·�����ļ���

signals:
	void sigUploadState(int retcode);	//0�ɹ���1ʧ��

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