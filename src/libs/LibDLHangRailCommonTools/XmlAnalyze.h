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

    // 开始读取xml文件;
    bool readFile(const QString &fileName);
    // 获取遥测数据;
    QVector<YcSetMsgData> getYcMsgData();
    // 获取遥信数据;
    QVector<QString> getYxMsgData();
    // 获取遥控数据;
    QVector<QString> getYkMsgData();

private:
    XmlAnalyze(QObject *parent = Q_NULLPTR);
   
    // 读取robot字段;
    void readRobotMembers();
    // 读取YcSetMsg字段;
    void readYcSetMsg();
    // 读取YxSetMsg字段;
    void readYxSetMsg();
    // 读取YkSetMsg字段;
    void readYkSetMsg();
    // 调过无用字段;
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
