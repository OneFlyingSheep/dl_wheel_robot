#pragma once

#include <QThread>
#include <QTimer>

class ClearOutDateFileObject : public QThread
{
    Q_OBJECT

public:
    ClearOutDateFileObject(QObject *parent = NULL);
    ~ClearOutDateFileObject();

    // 设置要清除的文件路径列表;
    void setClearFileList(QStringList strFilePathList);

    // 设置要删除文件的超出天数;
    void setFileOutDateDays(int days);
    // 设置检测时钟刷新时间;
    void setRefreshTime(quint64 time);
    // 设置超出该盘的使用比例;
    void setOutSpaceUserScale(int useScale);
    // 超出该盘的删除文件的比例;
    void setOutSpaceDeleteFileScale(int deleteFileScale);

    // 开始清理(必须调用此函数才可以进行自动清理，且调用之后，会立马进行一次清理);
    void startClear();

private:
    // 线程中进行删除;
    void run();

    // 删除文件夹及文件夹下全部文件;
    bool deleteDir(const QString &path);

    // 超出该盘空间百分之九十清理四分之一文件(百分比可设置);
    void deleteOutSpaceFile(QString strFilePath);
    // 删除多余文件;
    void deleteUnnecessaryFile(QString strFilePath);

    // 删除超期文件;
    void deleteOutDateFile(QString strFilePath);

private:
    int m_fileOutDateDays;                      // 文件超期时间;
    quint64 m_refreshTimeCount;                 // 监测刷新时间;
    int m_outSpaceUseScale;                     // 超出该盘的使用比例;
    int m_outSpaceDeleteFileScale;              // 超出该盘的删除文件的比例;
    QTimer m_refreshTimer;                      // 检测刷新时钟;
    QStringList m_strFilePathList;              // 删除文件夹列表;
};
