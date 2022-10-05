#include "ClearOutDateFileObject.h"
#include <windows.h>  
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QDateTime>
#include <QFile>

#define OUT_DATE_DAYS 365                       // �ļ�����ʱ��;
#define REFRESH_TIME 60 * 60 * 1000             // ���ˢ��ʱ��;
#define OUT_SPACE_USE_SCALE 90                  // �������̵�ʹ�ñ���;
#define OUT_SPACE_DELETE_FILE_SCALE 2           // �������̵�ɾ���ļ��ı���;

// ��ȡ��Ӧ·��������Ϣ;
void getDiskFreeSpace(QString driver, quint64& freeSpace, quint64& totalSpace) 
{
    LPCWSTR lpcwstrDriver = (LPCWSTR)driver.utf16();

    ULARGE_INTEGER liFreeBytesAvailable, liTotalBytes, liTotalFreeBytes;

    if (!GetDiskFreeSpaceEx(lpcwstrDriver, &liFreeBytesAvailable, &liTotalBytes, &liTotalFreeBytes))
    {
        qDebug() << "ERROR: Call to GetDiskFreeSpaceEx() failed.";
        return ;
    }
    totalSpace = (quint64)liTotalBytes.QuadPart / 1024 / 1024 / 1024;
    freeSpace = (quint64)liTotalFreeBytes.QuadPart / 1024 / 1024 / 1024;
}

ClearOutDateFileObject::ClearOutDateFileObject(QObject *parent)
    : QThread(parent)
    , m_fileOutDateDays(OUT_DATE_DAYS)
    , m_refreshTimeCount(REFRESH_TIME)
    , m_outSpaceUseScale(OUT_SPACE_USE_SCALE)
    , m_outSpaceDeleteFileScale(OUT_SPACE_DELETE_FILE_SCALE)
{
    m_refreshTimer.setInterval(REFRESH_TIME);
    connect(&m_refreshTimer, &QTimer::timeout, this, [=] {
        this->start();
    });
}

ClearOutDateFileObject::~ClearOutDateFileObject()
{
}

void ClearOutDateFileObject::setClearFileList(QStringList strFilePathList)
{
    m_strFilePathList = strFilePathList;
}

void ClearOutDateFileObject::setFileOutDateDays(int days)
{
    if (days <= 0)
    {
        return;
    }
    m_fileOutDateDays = days;
}

void ClearOutDateFileObject::setRefreshTime(quint64 time)
{
    if (time <= 0)
    {
        return;
    }
    m_refreshTimeCount = time;
    m_refreshTimer.setInterval(m_refreshTimeCount);
}

void ClearOutDateFileObject::setOutSpaceUserScale(int useScale)
{
    if (useScale <= 0)
    {
        return;
    }
    m_outSpaceUseScale = useScale;
}

void ClearOutDateFileObject::setOutSpaceDeleteFileScale(int deleteFileScale)
{
    if (deleteFileScale <= 0)
    {
        return;
    }
    m_outSpaceDeleteFileScale = deleteFileScale;
}

void ClearOutDateFileObject::startClear()
{
    this->start();
    m_refreshTimer.start();
}

void ClearOutDateFileObject::run()
{
    for each (QString strfilePath in m_strFilePathList)
    {
        // ��ɾ�������ռ��ļ�����ɾ�������ļ�;
        deleteOutSpaceFile(strfilePath);
        deleteOutDateFile(strfilePath);
    }
}

// ɾ���ļ���;
bool ClearOutDateFileObject::deleteDir(const QString &path)
{
    if (path.isEmpty()) {
        return false;
    }
    QDir dir(path);
    if (!dir.exists()) {
        return true;
    }
    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot); //���ù���;
    QFileInfoList fileList = dir.entryInfoList(); // ��ȡ���е��ļ���Ϣ;
    foreach(QFileInfo file, fileList)
    {
        //�����ļ���Ϣ;
        if (file.isFile())
        {
            // ���ļ���ɾ��;
            file.dir().remove(file.fileName());
        }
        else
        {
            // �ݹ�ɾ��;
            deleteDir(file.absoluteFilePath());
        }
    }
    return dir.rmpath(dir.absolutePath()); // ɾ���ļ���;
}

void ClearOutDateFileObject::deleteOutSpaceFile(QString strFilePath)
{
    if (QFile::exists(strFilePath))
    {
        quint64 freeSpace = 0;
        quint64 totalSpace = 0;
        getDiskFreeSpace(strFilePath, freeSpace, totalSpace);
        // ȷ��������ȷ;
        if (totalSpace > 0)
        {
            // ����ʹ�ñ���;
            int userScale = 100 - 1.0 * 100 * freeSpace / totalSpace;
            if (userScale >= m_outSpaceUseScale)
            {
                // ɾ�������ļ�;
                deleteUnnecessaryFile(strFilePath);
            }
        }
    }
}

void ClearOutDateFileObject::deleteUnnecessaryFile(QString strFilePath)
{
    QDir dir(strFilePath);
    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);

    QFileInfoList fileInfoList = dir.entryInfoList(QDir::NoFilter, QDir::Time);
    // ����ļ���Ϊ��ֱ���˳�;
    if (fileInfoList.count() <= 0)
    {
        return;
    }

    int fileTotalCount = fileInfoList.count();
    int deleteFileCouint = fileTotalCount / m_outSpaceDeleteFileScale;
    if (deleteFileCouint == 0)
    {
        deleteFileCouint = 1;
    }
    bool isSuccess = false;
    for (int i = fileTotalCount - 1; i >= fileTotalCount - deleteFileCouint; i--)
    {
        if (fileInfoList[i].isFile())
        {
            // ���ļ���ֱ��ɾ��;
            isSuccess = fileInfoList[i].dir().remove(fileInfoList[i].fileName());
        }
        else
        {
            // ���ļ��У������ɾ��;
            isSuccess = deleteDir(fileInfoList[i].filePath());
        }
        qDebug() << "IsSuccess : " << isSuccess << fileInfoList[i].filePath();
    }
}

void ClearOutDateFileObject::deleteOutDateFile(QString strFilePath)
{
    QDir dir(strFilePath);
    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);

    QFileInfoList fileInfoList = dir.entryInfoList(QDir::NoFilter, QDir::Time);
    for (int i = 0; i < fileInfoList.count(); i++)
    {
        QDate dateTime = fileInfoList[i].lastModified().date();
        int dayToToDay = dateTime.daysTo(QDate::currentDate());
        bool isSuccess = false;
        if (dayToToDay > m_fileOutDateDays)
        {
            if (fileInfoList[i].isFile())
            {
                // ���ļ���ֱ��ɾ��;
                isSuccess = fileInfoList[i].dir().remove(fileInfoList[i].fileName());
            }
            else
            {
                // ���ļ��У������ɾ��;
                isSuccess = deleteDir(fileInfoList[i].filePath());
            }
            qDebug() << "IsSuccess : " << isSuccess << fileInfoList[i].filePath() << "--" << dateTime << dateTime.daysTo(QDate::currentDate());
        }
        qDebug() << fileInfoList[i].filePath() << "-" << dateTime << dateTime.daysTo(QDate::currentDate());
    }
}
