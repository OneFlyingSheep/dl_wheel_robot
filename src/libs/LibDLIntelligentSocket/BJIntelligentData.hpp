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
        m_deviceTypeMap.insert("油浸式变压器（电抗器）", "1");
        m_deviceTypeMap.insert("断路器", "2");
        m_deviceTypeMap.insert("组合电器", "3");
        m_deviceTypeMap.insert("隔离开关", "4");
        m_deviceTypeMap.insert("开关柜", "5");

        m_deviceTypeMap.insert("电流互感器", "6");
        m_deviceTypeMap.insert("电压互感器", "7");
        m_deviceTypeMap.insert("避雷器", "8");
        m_deviceTypeMap.insert("并联电容器组", "9");
        m_deviceTypeMap.insert("干式电抗器", "10");

        m_deviceTypeMap.insert("串联补偿装置", "11");
        m_deviceTypeMap.insert("母线及绝缘子", "12");
        m_deviceTypeMap.insert("穿墙套管", "13");
        m_deviceTypeMap.insert("消弧线圈", "14");
        m_deviceTypeMap.insert("高频祖波器", "15");

        m_deviceTypeMap.insert("耦合电容器", "16");
        m_deviceTypeMap.insert("高压熔断器", "17");
        m_deviceTypeMap.insert("中性点隔直装置", "18");
        m_deviceTypeMap.insert("接地装置", "19");
        m_deviceTypeMap.insert("端子箱及检修电源箱", "20");

        m_deviceTypeMap.insert("站用变压器", "21");
        m_deviceTypeMap.insert("站用交流电源系统", "22");
        m_deviceTypeMap.insert("站用直流电源系统", "23");
        m_deviceTypeMap.insert("设备构架", "24");
        m_deviceTypeMap.insert("辅助设施", "25");

        m_deviceTypeMap.insert("土建设施", "26");
        m_deviceTypeMap.insert("独立避雷针", "27");
        m_deviceTypeMap.insert("避雷器动作次数表", "28");
    }
    void init_meter_type_map()
    {
        m_meterTypeMap.insert("油位表", "1");
        m_meterTypeMap.insert("避雷器动作次数表", "2");
        m_meterTypeMap.insert("泄漏电流表", "3");
        m_meterTypeMap.insert("SF6压力表", "4");
        m_meterTypeMap.insert("液压表", "5");
        m_meterTypeMap.insert("开关动作次数表", "6");
        m_meterTypeMap.insert("油温表", "7");
        m_meterTypeMap.insert("档位表", "8");
        m_meterTypeMap.insert("气压表", "9");
    }
    void init_appearance_type_map()
    {
        m_appearanceTypeMap.insert("电子围栏", "1");
        m_appearanceTypeMap.insert("红外对射", "2");
        m_appearanceTypeMap.insert("泡沫喷淋", "3");
        m_appearanceTypeMap.insert("消防水泵", "4");
        m_appearanceTypeMap.insert("消防栓", "5");

        m_appearanceTypeMap.insert("消防室", "6");
        m_appearanceTypeMap.insert("设备室", "7");
        m_appearanceTypeMap.insert("照明灯", "8");
        m_appearanceTypeMap.insert("摄像头", "9");
        m_appearanceTypeMap.insert("水位线", "10");

        m_appearanceTypeMap.insert("排水泵", "11");
        m_appearanceTypeMap.insert("沉降监测点", "12");
    }

    void init_recognition_type_map()
    {
        m_recognitionTypeMap.insert("表计读取", "1");
        m_recognitionTypeMap.insert("位置状态识别", "2");
        m_recognitionTypeMap.insert("设备外观查看", "3");
        m_recognitionTypeMap.insert("红外测温", "4");
        m_recognitionTypeMap.insert("声音检测", "5");
        m_recognitionTypeMap.insert("闪烁检测", "6");
        m_recognitionTypeMap.insert("红外测温+设备外观查看", "3,4");
    }
};