#pragma once

#include <QThread>
#include <QTimer>

class ClearOutDateFileObject : public QThread
{
    Q_OBJECT

public:
    ClearOutDateFileObject(QObject *parent = NULL);
    ~ClearOutDateFileObject();

    // ����Ҫ������ļ�·���б�;
    void setClearFileList(QStringList strFilePathList);

    // ����Ҫɾ���ļ��ĳ�������;
    void setFileOutDateDays(int days);
    // ���ü��ʱ��ˢ��ʱ��;
    void setRefreshTime(quint64 time);
    // ���ó������̵�ʹ�ñ���;
    void setOutSpaceUserScale(int useScale);
    // �������̵�ɾ���ļ��ı���;
    void setOutSpaceDeleteFileScale(int deleteFileScale);

    // ��ʼ����(������ô˺����ſ��Խ����Զ������ҵ���֮�󣬻��������һ������);
    void startClear();

private:
    // �߳��н���ɾ��;
    void run();

    // ɾ���ļ��м��ļ�����ȫ���ļ�;
    bool deleteDir(const QString &path);

    // �������̿ռ�ٷ�֮��ʮ�����ķ�֮һ�ļ�(�ٷֱȿ�����);
    void deleteOutSpaceFile(QString strFilePath);
    // ɾ�������ļ�;
    void deleteUnnecessaryFile(QString strFilePath);

    // ɾ�������ļ�;
    void deleteOutDateFile(QString strFilePath);

private:
    int m_fileOutDateDays;                      // �ļ�����ʱ��;
    quint64 m_refreshTimeCount;                 // ���ˢ��ʱ��;
    int m_outSpaceUseScale;                     // �������̵�ʹ�ñ���;
    int m_outSpaceDeleteFileScale;              // �������̵�ɾ���ļ��ı���;
    QTimer m_refreshTimer;                      // ���ˢ��ʱ��;
    QStringList m_strFilePathList;              // ɾ���ļ����б�;
};
