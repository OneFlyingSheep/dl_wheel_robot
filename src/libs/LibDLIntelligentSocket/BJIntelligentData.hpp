#pragma once
#include <QMap>
#include <QFile>
#include <QTextStream>
#include <stdio.h>
#include "LibDLNewInfraredSDK/GuiDeNewInfrared.h"
#include <QDebug>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

class BJIntelligentData
{
private:
    QMap<QString, QString> m_deviceTypeMap;
    QMap<QString, QString> m_meterTypeMap;
    QMap<QString, QString> m_appearanceTypeMap;
    QMap<QString, QString> m_recognitionTypeMap;
public:
    BJIntelligentData()
    {
        init_device_type_map();
        init_meter_type_map();
        init_appearance_type_map();
        init_recognition_type_map();
    }
    ~BJIntelligentData() {}

    QString contrast_device_type_with_key(QString key)
    {
        if (m_deviceTypeMap.contains(key))
        {
            return m_deviceTypeMap[key];
        }
        return "";
    }

    QString contrast_meter_type_with_key(QString key)
    {
        if (m_meterTypeMap.contains(key))
        {
            return m_meterTypeMap[key];
        }
        return "";
    }
    QString contrast_appearance_type_with_key(QString key)
    {
        if (m_appearanceTypeMap.contains(key))
        {
            return m_appearanceTypeMap[key];
        }
        return "";
    }
    QString contrast_phase_with_string(QString value)
    {
        if ("A" == value)
        {
            return "1";
        }
        else if ("B" == value)
        {
            return "2";
        }
        else if ("C" == value)
        {
            return "3";
        }
        else
        {
            return "";
        }
    }
    QString contrast_recognition_type_with_key(QString key)
    {
        if (m_recognitionTypeMap.contains(key))
        {
            return m_recognitionTypeMap[key];
        }
        return "";
    }
    QString contrast_collect_type_with_recognition_type(QString id)
    {
        if (id == "4")
        {
            return "1";
        }
        else if (id == "1" || id == "2" || id == "3")
        {
            return "2";
        }
        else if (id == "5")
        {
            return "3";
        }
        else if (id == "3,4")
        {
            return "1,2";
        }
        else
        {
            return "";
        }
    }

public:
    void init_device_type_map()
    {
        m_deviceTypeMap.insert("�ͽ�ʽ��ѹ�����翹����", "1");
        m_deviceTypeMap.insert("��·��", "2");
        m_deviceTypeMap.insert("��ϵ���", "3");
        m_deviceTypeMap.insert("���뿪��", "4");
        m_deviceTypeMap.insert("���ع�", "5");

        m_deviceTypeMap.insert("����������", "6");
        m_deviceTypeMap.insert("��ѹ������", "7");
        m_deviceTypeMap.insert("������", "8");
        m_deviceTypeMap.insert("������������", "9");
        m_deviceTypeMap.insert("��ʽ�翹��", "10");

        m_deviceTypeMap.insert("��������װ��", "11");
        m_deviceTypeMap.insert("ĸ�߼���Ե��", "12");
        m_deviceTypeMap.insert("��ǽ�׹�", "13");
        m_deviceTypeMap.insert("������Ȧ", "14");
        m_deviceTypeMap.insert("��Ƶ�沨��", "15");

        m_deviceTypeMap.insert("��ϵ�����", "16");
        m_deviceTypeMap.insert("��ѹ�۶���", "17");
        m_deviceTypeMap.insert("���Ե��ֱװ��", "18");
        m_deviceTypeMap.insert("�ӵ�װ��", "19");
        m_deviceTypeMap.insert("�����估���޵�Դ��", "20");

        m_deviceTypeMap.insert("վ�ñ�ѹ��", "21");
        m_deviceTypeMap.insert("վ�ý�����Դϵͳ", "22");
        m_deviceTypeMap.insert("վ��ֱ����Դϵͳ", "23");
        m_deviceTypeMap.insert("�豸����", "24");
        m_deviceTypeMap.insert("������ʩ", "25");

        m_deviceTypeMap.insert("������ʩ", "26");
        m_deviceTypeMap.insert("����������", "27");
        m_deviceTypeMap.insert("����������������", "28");
    }
    void init_meter_type_map()
    {
        m_meterTypeMap.insert("��λ��", "1");
        m_meterTypeMap.insert("����������������", "2");
        m_meterTypeMap.insert("й©������", "3");
        m_meterTypeMap.insert("SF6ѹ����", "4");
        m_meterTypeMap.insert("Һѹ��", "5");
        m_meterTypeMap.insert("���ض���������", "6");
        m_meterTypeMap.insert("���±�", "7");
        m_meterTypeMap.insert("��λ��", "8");
        m_meterTypeMap.insert("��ѹ��", "9");
    }
    void init_appearance_type_map()
    {
        m_appearanceTypeMap.insert("����Χ��", "1");
        m_appearanceTypeMap.insert("�������", "2");
        m_appearanceTypeMap.insert("��ĭ����", "3");
        m_appearanceTypeMap.insert("����ˮ��", "4");
        m_appearanceTypeMap.insert("����˨", "5");

        m_appearanceTypeMap.insert("������", "6");
        m_appearanceTypeMap.insert("�豸��", "7");
        m_appearanceTypeMap.insert("������", "8");
        m_appearanceTypeMap.insert("����ͷ", "9");
        m_appearanceTypeMap.insert("ˮλ��", "10");

        m_appearanceTypeMap.insert("��ˮ��", "11");
        m_appearanceTypeMap.insert("��������", "12");
    }

    void init_recognition_type_map()
    {
        m_recognitionTypeMap.insert("��ƶ�ȡ", "1");
        m_recognitionTypeMap.insert("λ��״̬ʶ��", "2");
        m_recognitionTypeMap.insert("�豸��۲鿴", "3");
        m_recognitionTypeMap.insert("�������", "4");
        m_recognitionTypeMap.insert("�������", "5");
        m_recognitionTypeMap.insert("��˸���", "6");
        m_recognitionTypeMap.insert("�������+�豸��۲鿴", "3,4");
    }
};