#include "XmlAnalyze.h"
#include <QFile>
#include <QDebug>
#include <QApplication>

XmlAnalyze* XmlAnalyze::m_inst = NULL;
QMutex XmlAnalyze::m_mutex;

XmlAnalyze& XmlAnalyze::getInstance()
{
    m_mutex.lock();
    if (m_inst == NULL)
    {
        m_inst = new XmlAnalyze;
        QString strFileName = QApplication::applicationDirPath() + "/ConfigData/HangRailRobotCfg/yh_104_gd.xml";
        m_inst->readFile(strFileName);
    }
    m_mutex.unlock();

    return *m_inst;
}

QVector<YcSetMsgData> XmlAnalyze::getYcMsgData()
{
    return m_ycMsgDataVector;
}

QVector<QString> XmlAnalyze::getYxMsgData()
{
    return m_yxMsgDataVector;
}

QVector<QString> XmlAnalyze::getYkMsgData()
{
    return m_ykMsgDataVector;
}

XmlAnalyze::XmlAnalyze(QObject *parent)
    : QObject(parent)
{
}

bool XmlAnalyze::readFile(const QString &fileName)
{
    QFile file(fileName);

    if (!file.open(QFile::ReadOnly | QFile::Text))
    {
        qDebug() << "error";
        return false;
    }

    m_reader.setDevice(&file);
    while (!m_reader.atEnd()) {
        if (m_reader.isStartElement())
        {
            if (m_reader.name() == "Root") 
            {
                readRobotMembers();
            }
            else 
            {
                m_reader.raiseError(tr("Not a valid book file"));
            }
        }
        else 
        {
            m_reader.readNext();
        }
    }

    file.close();
    if (m_reader.hasError())
    {
        return false;
    }
    else if (file.error() != QFile::NoError)
    {
        return false;
    }
    return true;
}

void XmlAnalyze::readRobotMembers()
{
    m_reader.readNext();
    while (!m_reader.atEnd())
    {
        if (m_reader.isEndElement()) 
        {
            if (m_reader.name().toString() == "Root")
            {
                break;
            }
        }

        if (m_reader.isStartElement()) 
        {
            if (m_reader.name() == "YcTran") 
            {
                readYcSetMsg();
            }
            else if (m_reader.name() == "YxTran")
            {
                readYxSetMsg();
            }
            else if (m_reader.name() == "YkTran")
            {
                readYkSetMsg();
            }
            else
            {
                skipUnknownElement();
            }
        }
        else 
        {
            m_reader.readNext();
        }
    }
}

void XmlAnalyze::readYcSetMsg()
{
    m_reader.readNext();
    while (!m_reader.atEnd())
    {
        if (m_reader.isEndElement())
        {
            if (m_reader.name().toString() == "YcTran")
            {
                break;
            }            
        }

        if (m_reader.isStartElement())
        {
            if (m_reader.name() == "YcData")
            {
                m_ycMsgDataVector.append(YcSetMsgData(m_reader.attributes().value("Name").toString(), m_reader.attributes().value("Unit").toString()));
            }
            else
            {
                skipUnknownElement();
            }
        }
        m_reader.readNext();
    }
}

void XmlAnalyze::readYxSetMsg()
{
    m_reader.readNext();
    while (!m_reader.atEnd())
    {
        if (m_reader.isEndElement())
        {
            if (m_reader.name().toString() == "YxTran")
            {
                break;
            }
        }

        if (m_reader.isStartElement())
        {
            if (m_reader.name() == "YxData")
            {
                m_yxMsgDataVector.append(m_reader.attributes().value("Name").toString());
            }
            else
            {
                skipUnknownElement();
            }
        }
        m_reader.readNext();
    }
}

void XmlAnalyze::readYkSetMsg()
{
    m_reader.readNext();
    while (!m_reader.atEnd())
    {
        if (m_reader.isEndElement())
        {
            if (m_reader.name().toString() == "YkTran")
            {
                break;
            }
        }

        if (m_reader.isStartElement())
        {
            if (m_reader.name() == "YkData")
            {
                m_ykMsgDataVector.append(m_reader.attributes().value("Name").toString());
            }
            else
            {
                skipUnknownElement();
            }
        }
        m_reader.readNext();
    }
}

void XmlAnalyze::skipUnknownElement()
{
    m_reader.readNext();
    while (!m_reader.atEnd())
    {
        if (m_reader.isEndElement())
        {
            m_reader.readNext();
            break;
        }

        if (m_reader.isStartElement())
        {
            skipUnknownElement();
        }
        else
        {
            m_reader.readNext();
        }
    }
}