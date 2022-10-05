#ifndef _XML_ANALYZE_H
#define _XML_ANALYZE_H

#include <QObject>
#include <QXmlStreamReader>
#include <QMutex>

struct YcSetMsgData
{
    QString strName;
    QString strUnit;
    YcSetMsgData(QString name, QString unit)
    {
        strName = name;
        strUnit = unit;
    }

    YcSetMsgData()
    {}
};

class XmlAnalyze : public QObject
{
    Q_OBJECT

public:
    static XmlAnalyze& getInstance();

    // ��ʼ��ȡxml�ļ�;
    bool readFile(const QString &fileName);
    // ��ȡң������;
    QVector<YcSetMsgData> getYcMsgData();
    // ��ȡң������;
    QVector<QString> getYxMsgData();
    // ��ȡң������;
    QVector<QString> getYkMsgData();

private:
    XmlAnalyze(QObject *parent = Q_NULLPTR);
   
    // ��ȡrobot�ֶ�;
    void readRobotMembers();
    // ��ȡYcSetMsg�ֶ�;
    void readYcSetMsg();
    // ��ȡYxSetMsg�ֶ�;
    void readYxSetMsg();
    // ��ȡYkSetMsg�ֶ�;
    void readYkSetMsg();
    // ���������ֶ�;
    void skipUnknownElement();

private:
    static XmlAnalyze* m_inst;
    static QMutex m_mutex;

    QVector<YcSetMsgData> m_ycMsgDataVector;
    QVector<QString> m_yxMsgDataVector;
    QVector<QString> m_ykMsgDataVector;
    QXmlStreamReader m_reader;
};

#define XML_ANALYZE XmlAnalyze::getInstance()

#endif
