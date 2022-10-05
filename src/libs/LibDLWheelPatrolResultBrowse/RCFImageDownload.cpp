#include <QDir>
#include <QDebug>
#include "RCFImageDownload.h"
#include "LibProtoClient/ProtoClient.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"

#pragma execution_character_set("utf-8")

RCFImageDownload::RCFImageDownload(QObject* parent /*= NULL*/)
    : QThread(parent)
    , m_bIsStopDownloadImage(false)
	, m_bIsDownloadTaskImage(false)
	,m_strTaskID("")
{
    
}

RCFImageDownload::~RCFImageDownload()
{
    this->quit();
    this->wait();
}

void RCFImageDownload::SetTaskExportImage(QString strTaskId, QString strTaskName, QString strExportPath)
{
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");
	m_bIsDownloadTaskImage = true;
	m_strTaskID = strTaskId;
    m_strTaskName = strTaskName + "_" + current_date;
    m_strDownloadFilePath = strExportPath;
}


void RCFImageDownload::setImageInfoList(QStringList strTaskIdList, QStringList strDeviceIdList)
{
    m_strTaskIdList = strTaskIdList;
    m_strDeviceIdList = strDeviceIdList;
}

void RCFImageDownload::stopDownloadImage()
{
    m_bIsStopDownloadImage = true;
}

void RCFImageDownload::run()
{
	if (m_bIsDownloadTaskImage)
	{//�����������������ͼƬ
		m_bIsDownloadTaskImage = false;

        if (!m_pZip.is_exits_file(m_strTaskID))
        {
            DownloadTaskImageFromCore();							//�����ص�����
        }
        m_pZip.unzip_taskfile_zip(m_strTaskID);
		CopyTaskImage();										//�ٸ��Ƶ�ָ��Ŀ¼
        m_pExcel.export_excel_result(m_strTaskID, m_strDownloadFilePath + "/" + m_strTaskName);
		emit signalAllDownloaded();								// ����ͼƬ�������;
	}
	else
	{
		for (int i = 0; i < m_strTaskIdList.count(); i++)
		{
			if (m_bIsStopDownloadImage)
			{
				return;
			}

			// ����û�У���ȥ����;
			WheelJudgeTakePhoto type = WHEEL_DEVICE_CONFIG.getWheelChooseRecForDeviceUUidBool(m_strDeviceIdList[i]);
			if (type == VisibleLightJudge)
			{
				downloadVisibleImageFile(m_strTaskIdList[i], m_strDeviceIdList[i], i);
			}
			else if (type == InfraredLightJudge)
			{
				downloadInfraredImageFile(m_strTaskIdList[i], m_strDeviceIdList[i], i);
			}
		}

		// ����ͼƬ�������;
		emit signalAllDownloaded();
	}
	}
   

void RCFImageDownload::downloadVisibleImageFile(QString strTaskId, QString strDeviceId, int imageIndex)
{
    m_strDownloadFilePath = QString("%1/DeviceImage/%2").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath).arg(strTaskId);
    m_strCoreFileDir = QString("task/%1/%2").arg(strTaskId).arg(strDeviceId);

    QString strImagePath = QString("%1/%2/%3_result.jpg").arg(m_strDownloadFilePath).arg(strDeviceId).arg(strDeviceId);
    // �ȿ�������û��_result�ļ������û��ȥcore������;
    if (QFile::exists(strImagePath))
    {
        // �����У���ֱ��֪ͨ������ʾͼƬ;
        emit signalDowmloadFinished(0, imageIndex, strImagePath);
        return;
    }

    QDir downloadDir;
    if (!downloadDir.exists(m_strDownloadFilePath))
    {
        downloadDir.mkpath(m_strDownloadFilePath);
    }

    QString strRootPath = m_strDownloadFilePath + "/" + strDeviceId;
    QString strOriPath = strDeviceId + ".jpg";
    QString strResPath = strDeviceId + "_result.jpg";
//    int downloadResult = GFILE_TRANSFER_CLIENT.downloadFile(m_strCoreFileDir.toLocal8Bit().data(), "", m_strDownloadFilePath.toLocal8Bit().data());
    int downloadResult = GFILE_TRANSFER_CLIENT.downloadFile(m_strCoreFileDir.toLocal8Bit().data(), strOriPath.toLocal8Bit().data(), strRootPath.toLocal8Bit().data());
    downloadResult = GFILE_TRANSFER_CLIENT.downloadFile(m_strCoreFileDir.toLocal8Bit().data(), strResPath.toLocal8Bit().data(), strRootPath.toLocal8Bit().data());
    // ��core��������֮�����ж��Ƿ����_result�ļ�;
    if (QFile::exists(strImagePath))
    {
        emit signalDowmloadFinished(0, imageIndex, strImagePath);
    }
    else
    {
        QString strOriginalImagePath = strImagePath.remove("_result");
        if (QFile::exists(strOriginalImagePath))
        {
            emit signalDowmloadFinished(0, imageIndex, strOriginalImagePath);
        }
        else
        {
            emit signalDowmloadFinished(-1, imageIndex, "");
        }
    }
}

void RCFImageDownload::downloadInfraredImageFile(QString strTaskId, QString strDeviceId, int imageIndex)
{
    m_strDownloadFilePath = QString("%1/DeviceImage/%2").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath).arg(strTaskId);
    m_strCoreFileDir = QString("task/%1/%2").arg(strTaskId).arg(strDeviceId);

    QString strImagePath = QString("%1/%2/%3-temp.jpg").arg(m_strDownloadFilePath).arg(strDeviceId).arg(strDeviceId);
    // �ȿ�������û��_result�ļ������û��ȥcore������;
    if (QFile::exists(strImagePath))
    {
        // �����У���ֱ��֪ͨ������ʾͼƬ;
        emit signalDowmloadFinished(0, imageIndex, strImagePath);
        return;
    }

    QDir downloadDir;
    if (!downloadDir.exists(m_strDownloadFilePath))
    {
        downloadDir.mkpath(m_strDownloadFilePath);
    }

    int downloadResult = GFILE_TRANSFER_CLIENT.downloadFile(m_strCoreFileDir.toLocal8Bit().data(), "", m_strDownloadFilePath.toLocal8Bit().data());
    if (QFile::exists(strImagePath))
    {
        emit signalDowmloadFinished(0, imageIndex, strImagePath);
    }
    else
    {
        QString strOriginalImagePath = strImagePath.replace("temp", "raw");
        if (QFile::exists(strOriginalImagePath))
        {
            emit signalDowmloadFinished(0, imageIndex, strOriginalImagePath);
        }
        else
        {
            emit signalDowmloadFinished(-1, imageIndex, "");
        }
    }
}

void RCFImageDownload::DownloadTaskImageFromCore()
{
// 	QString strDownloadFilePath = QString("%1/DeviceImage").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath);
// 	QString strCoreFileDir = QString("task/%1").arg(m_strTaskID);
// 	int downloadResult = GFILE_TRANSFER_CLIENT.downloadFile(strCoreFileDir.toLocal8Bit().data(), "", strDownloadFilePath.toLocal8Bit().data());			
    //��core��ͼƬ���ص�����
    QString strDownloadFilePath = QString("%1/task_zip").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath);
    QString strCoreFileDir = QString("task_zip");
    QString strFileName = QString(m_strTaskID + ".zip");
    int downloadResult = GFILE_TRANSFER_CLIENT.downloadFile(strCoreFileDir.toLocal8Bit().data(), strFileName.toLocal8Bit().data(), strDownloadFilePath.toLocal8Bit().data());
}

void RCFImageDownload::CopyTaskImage()
{//���Ƶ�ǰ�����µ�����ͼƬ
	QString strCopyDir = QString("%1/%2").arg(m_strDownloadFilePath).arg(m_strTaskName);						//���ƽ����ŵ�Ŀ¼
 	QDir dirCopy(strCopyDir);
	QFileInfo fileInfoCopy(strCopyDir);
	if (!fileInfoCopy.isDir() || !dirCopy.exists())
	{//�ж�ѡ���Ŀ¼�Ƿ����
		dirCopy.mkpath(strCopyDir);
	}

	QMap<QString, QString> mapEquipID2Name;
	WHEEL_ROBOT_DB.getDeviceNameForTaskUUid(m_strTaskID, mapEquipID2Name);				//��ȡ��ǰ�����µ������豸��id��Ӧ��name

	//����Ŀ¼�������ļ�
	QString strDownloadFilePath = QString("%1/DeviceImage/%2").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath).arg(m_strTaskID);
	QDir dir(strDownloadFilePath);
	QFileInfoList list = dir.entryInfoList(QDir::Dirs | QDir::Files | QDir::Readable | QDir::Writable | QDir::Hidden | QDir::NoDotAndDotDot, QDir::Name);
	foreach(QFileInfo dirInfo, list)
	{//��������Ŀ¼
		QDir dirFile(dirInfo.filePath());
		QFileInfoList lstFiles = dirFile.entryInfoList(QDir::Dirs | QDir::Files | QDir::Readable | QDir::Writable | QDir::Hidden | QDir::NoDotAndDotDot, QDir::Name);
		QString strId = dirInfo.fileName();						//��ȡ��ǰ�豸id
		QString strEquipName = "Ĭ��";							//ͨ���豸id��ȡ�豸��
		QMap<QString, QString>::const_iterator it = mapEquipID2Name.find(strId);
		if (it != mapEquipID2Name.end())
		{
			strEquipName = it.value();
		}

		foreach(QFileInfo fileInfo, lstFiles)
		{
			QString strNewFile = QString("%1/%2").arg(strCopyDir).arg(fileInfo.fileName().replace(strId, strEquipName));				//���ļ���
            if (strNewFile.contains("scaled") || strNewFile.contains("type"))
            {
                continue;
            }
			QFile file(strNewFile);
			if (file.exists())
			{//�ļ��Ƿ���ڣ����ڵĻ������ļ�������������ʱ��
				if (strNewFile.contains("_result.jpg"))
				{
					strNewFile = QString("%1/%2_result_%3.jpg").arg(strCopyDir).arg(strEquipName).arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss"));
				}
				else
				{
					strNewFile = QString("%1/%2_%3.jpg").arg(strCopyDir).arg(strEquipName).arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss"));
				}
				//strNewFile = strNewFile.insert(strNewFile.size() - 5, QString("_%1").arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss")));
			}
			QFile::copy(fileInfo.absoluteFilePath(), strNewFile);				//�����ļ���ָ����Ŀ¼
		}
	}
}

