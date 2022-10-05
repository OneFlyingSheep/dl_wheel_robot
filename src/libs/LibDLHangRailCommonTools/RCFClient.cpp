#include "RCFClient.h"
#include <QDir>
#include <QDebug>
#include "LibProtoClient/ProtoClient.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"

#pragma execution_character_set("utf-8")

RCFClient::RCFClient(QObject* parent /*= NULL*/)
    : QThread(parent)
    , m_isDownloading(false)
{
    GFILE_TRANSFER_CLIENT.signal_tran_percentage.connect(boost::bind(&RCFClient::signalUpdateTaskProgress, this, _1));
    connect(this, &RCFClient::signalUpdateTaskProgress, this, [=](int iProgress) {
        qDebug() << "RCFClient::signalUpdateTaskProgress : " << iProgress;
    });
}

RCFClient::~RCFClient()
{
    this->quit();
    this->wait();
}

bool RCFClient::getCurrentDownloadStatus()
{
    return m_isDownloading;
}

void RCFClient::uploadFile(QString strCoreFilePath, QString strLocalFilePath)
{
    if (m_isDownloading)
    {
        return;
    }
    m_RCFOperation = RCF_Upload;
    m_strCoreFilePath = strCoreFilePath;
    m_strLocalFilePath = strLocalFilePath;
    this->start();
}

void RCFClient::downloadFile(int downloadType, QString strCoreFilePath, QString strFileName, QString strLocalFilePath)
{
    if (m_isDownloading)
    {
        return;
    }
    m_currentDownloadType = downloadType;
    m_RCFOperation = RCF_Download_File;
    m_strFileName = strFileName;
    m_strCoreFilePath = strCoreFilePath;
    m_strLocalFilePath = strLocalFilePath;
    this->start();
}

void RCFClient::downloadDeviceImage(int downloadType, QString strCoreFilePath, QString strFileName, QString strLocalFilePath)
{
    if (m_isDownloading)
    {
        return;
    }
    m_currentDownloadType = downloadType;
    m_RCFOperation = RCF_Download_Device_Image;
    m_strFileName = strFileName;
    m_strCoreFilePath = strCoreFilePath;
    m_strLocalFilePath = strLocalFilePath;
    this->start();
}

void RCFClient::run()
{
    m_isDownloading = true;
    int result = -1;
    switch (m_RCFOperation)
    {
    case RCF_Upload:
    {
        result = GFILE_TRANSFER_CLIENT.uploadFile(m_strLocalFilePath.toLocal8Bit().data(), m_strCoreFilePath.toLocal8Bit().data());
        bool isSuccess = result == 0;
        emit signalTaskUploadFinished(isSuccess);
    }
        break;
    case RCF_Download_File:
    {
        result = GFILE_TRANSFER_CLIENT.downloadFile(m_strCoreFilePath.toLocal8Bit().data(), m_strFileName.toLocal8Bit().data(), m_strLocalFilePath.toLocal8Bit().data());
        bool isSuccess = result == 0;
        emit signalTaskDownloadFinished(isSuccess, m_currentDownloadType, m_strLocalFilePath + "/" + m_strFileName);
    }
    break;
    case RCF_Download_Device_Image:
    {
        result = GFILE_TRANSFER_CLIENT.downloadFile(m_strCoreFilePath.toLocal8Bit().data(), "", m_strLocalFilePath.toLocal8Bit().data());
        bool isSuccess = result == 0;
        emit signalTaskDownloadFinished(isSuccess, m_currentDownloadType, m_strFileName);
    }
    break;
    default:
        break;
    } 
    
    m_isDownloading = false;
}