#ifndef RCF_CLIENT_H
#define RCF_CLIENT_H

#include <QObject>
#include <QThread>

enum RCFOperation
{
    RCF_Upload,
    RCF_Download_File,
    RCF_Download_Device_Image,
};

class RCFClient : public QThread
{
	Q_OBJECT

public:
    RCFClient(QObject* parent = NULL);

    ~RCFClient();

    bool getCurrentDownloadStatus();

    void uploadFile(QString strCoreFilePath, QString strLocalFilePath);

    void downloadFile(int downloadType, QString strCoreFilePath, QString strFileName, QString strLocalFilePath);

    void downloadDeviceImage(int downloadType, QString strCoreFilePath, QString strFileName, QString strLocalFilePath);

signals:
    // ���µ�ǰRCF����Ľ���;
    void signalUpdateTaskProgress(int);
    // ��ǰRCF�ϴ��������;
    void signalTaskUploadFinished(bool isSuccess);
    // ��ǰRCF�����������;
    void signalTaskDownloadFinished(bool isSuccess, int downloadType, QString strLocalFilePath);

private:
    void run();   
    
signals:
  

private:
    bool m_isDownloading;
    RCFOperation m_RCFOperation;

    // ��ǰ��������;
    int m_currentDownloadType;

    QString m_strCoreFilePath;
    QString m_strFileName;
    QString m_strLocalFilePath;

};

#endif