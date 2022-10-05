#pragma once

#include <QObject>
#include <QThread>
#include "LibZip/LibZip.h"
#include "LibDLCreateExcel/CreateExcelForQt5.h"

class RCFImageDownload : public QThread
{
	Q_OBJECT

public:
    RCFImageDownload(QObject* parent = NULL);

    ~RCFImageDownload();

	void SetTaskExportImage(QString strTaskId, QString strTaskName, QString strExportPath);

    // ����ͼƬ�б�;
    void setImageInfoList(QStringList strTaskIdList, QStringList strDeviceIdList);

    // ֹͣ��ǰ����;
    void stopDownloadImage();

private:
    void run();

    // ���ؿɼ���ͼƬ;
    void downloadVisibleImageFile(QString strTaskId, QString strDeviceId, int imageIndex);
    void downloadInfraredImageFile(QString strTaskId, QString strDeviceId, int imageIndex);

	void DownloadTaskImageFromCore();					//�������������ͼƬ������
	void CopyTaskImage();								//���Ƶ�ǰ�����µ�ͼƬ��ָ��Ŀ¼��
    
signals:
    // �����������;
    void signalDowmloadFinished(int result, int imageIndex, QString strImagePath);

    // ����ͼƬ�������;
    void signalAllDownloaded();

private:
    bool m_bIsStopDownloadImage;						//�Ƿ�ֹͣ����ͼƬ

    QString m_strCoreFileDir;							//cor�ļ�·��
    QString m_strDownloadFilePath;						//���ص��ļ�·��

    QStringList m_strTaskIdList;					//��������id
    QStringList m_strDeviceIdList;					//�����豸��id

	bool m_bIsDownloadTaskImage;					//�Ƿ��������������ͼƬ
	QString m_strTaskID;							//��ǰ����id
	QString m_strTaskName;							//��ǰ��������
    LibZip m_pZip;
    CreateExcelForQt5 m_pExcel;
};