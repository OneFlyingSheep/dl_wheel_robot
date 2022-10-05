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

    // 下载图片列表;
    void setImageInfoList(QStringList strTaskIdList, QStringList strDeviceIdList);

    // 停止当前下载;
    void stopDownloadImage();

private:
    void run();

    // 下载可见光图片;
    void downloadVisibleImageFile(QString strTaskId, QString strDeviceId, int imageIndex);
    void downloadInfraredImageFile(QString strTaskId, QString strDeviceId, int imageIndex);

	void DownloadTaskImageFromCore();					//下载整个任务的图片到本地
	void CopyTaskImage();								//复制当前任务下的图片到指定目录下
    
signals:
    // 单张下载完成;
    void signalDowmloadFinished(int result, int imageIndex, QString strImagePath);

    // 所有图片下载完成;
    void signalAllDownloaded();

private:
    bool m_bIsStopDownloadImage;						//是否停止下载图片

    QString m_strCoreFileDir;							//cor文件路径
    QString m_strDownloadFilePath;						//下载的文件路径

    QStringList m_strTaskIdList;					//保存任务id
    QStringList m_strDeviceIdList;					//保存设备的id

	bool m_bIsDownloadTaskImage;					//是否下载整个任务的图片
	QString m_strTaskID;							//当前任务id
	QString m_strTaskName;							//当前任务名称
    LibZip m_pZip;
    CreateExcelForQt5 m_pExcel;
};