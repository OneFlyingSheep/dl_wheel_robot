#include "LibConvertSymbolsToLua/LibConvertSymbolsToLua.h"
#include <QFile>
#include <QTextStream>
#include <JSON/json.h>

conventSymbols2Lua::conventSymbols2Lua()
{

}

conventSymbols2Lua::~conventSymbols2Lua()
{

}

bool conventSymbols2Lua::convert(QVector<THRESHOLD_ELEMENT> &elementList, QString &luaString, DeviceAlarmLevel level, QString &errMsg)
{
    luaString.clear();
    QString tmpString;
    int count = 0;
    
    QStringList parameterList;

    luaString += "function checkResult(result, CURRENT_VARIABLE)\n";

    for (int i = 0; i < elementList.size(); i++)
    {
        if (elementList[i].type == THRESHOLD_TYPE_VARIABLE ||
            elementList[i].type == THRESHOLD_TYPE_INIT_VAL ||
            elementList[i].type == THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE ||
            elementList[i].type == THRESHOLD_TYPE_CHANGE_RATE)
        {
            if (!parameterList.contains(elementList[i].val[0]))
            {
                parameterList.push_back(elementList[i].val[0]);
                luaString += QString("local %1 = 0;\n").arg(elementList[i].val[0]);
            }
        }
    }

    getLevelString(elementList, luaString, level, errMsg);

    luaString += "return 0;\nend\n";

    QFile file("./tmpLua.lua");
    file.open(QIODevice::ReadWrite | QIODevice::Truncate);

    QTextStream f(&file);
    f << luaString;
    file.close();
    QString filePath = file.fileName();

    return m_luaScriptCheck.checkLuaScript(filePath, errMsg) == 0;
}

bool conventSymbols2Lua::getLuaScript(QVector<THRESHOLD_ELEMENT> alarmNormalList, QVector<THRESHOLD_ELEMENT> alarmWarningList,
    QVector<THRESHOLD_ELEMENT> alarmCommonList, QVector<THRESHOLD_ELEMENT> alarmSerialList, QVector<THRESHOLD_ELEMENT> alarmDangerList,
    QString &luaString, QString &errMsg)
{
    luaString.clear();
    QString tmpString;
    int count = 0;

    QStringList parameterList;

    luaString += "function checkResult(CURRENT_VARIABLE)\n";

    for (int i = 0; i < alarmNormalList.size(); i++)
    {
        if (alarmNormalList[i].type == THRESHOLD_TYPE_VARIABLE ||
            alarmNormalList[i].type == THRESHOLD_TYPE_INIT_VAL ||
            alarmNormalList[i].type == THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE ||
            alarmNormalList[i].type == THRESHOLD_TYPE_CHANGE_RATE)
        {
            if (!parameterList.contains(alarmNormalList[i].val[0]))
            {
                parameterList.push_back(alarmNormalList[i].val[0]);
                luaString += QString("local %1 = 0;\n").arg(alarmNormalList[i].val[0]);
            }
        }
    }

    for (int i = 0; i < alarmWarningList.size(); i++)
    {
        if (alarmWarningList[i].type == THRESHOLD_TYPE_VARIABLE ||
            alarmWarningList[i].type == THRESHOLD_TYPE_INIT_VAL ||
            alarmWarningList[i].type == THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE ||
            alarmWarningList[i].type == THRESHOLD_TYPE_CHANGE_RATE)
        {
            if (!parameterList.contains(alarmWarningList[i].val[0]))
            {
                parameterList.push_back(alarmWarningList[i].val[0]);
                luaString += QString("local %1 = 0;\n").arg(alarmWarningList[i].val[0]);
            }
        }
    }

    for (int i = 0; i < alarmCommonList.size(); i++)
    {
        if (alarmCommonList[i].type == THRESHOLD_TYPE_VARIABLE ||
            alarmCommonList[i].type == THRESHOLD_TYPE_INIT_VAL ||
            alarmCommonList[i].type == THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE ||
            alarmCommonList[i].type == THRESHOLD_TYPE_CHANGE_RATE)
        {
            if (!parameterList.contains(alarmCommonList[i].val[0]))
            {
                parameterList.push_back(alarmCommonList[i].val[0]);
                luaString += QString("local %1 = 0;\n").arg(alarmCommonList[i].val[0]);
            }
        }
    }

    for (int i = 0; i < alarmSerialList.size(); i++)
    {
        if (alarmSerialList[i].type == THRESHOLD_TYPE_VARIABLE ||
            alarmSerialList[i].type == THRESHOLD_TYPE_INIT_VAL ||
            alarmSerialList[i].type == THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE ||
            alarmSerialList[i].type == THRESHOLD_TYPE_CHANGE_RATE)
        {
            if (!parameterList.contains(alarmSerialList[i].val[0]))
            {
                parameterList.push_back(alarmSerialList[i].val[0]);
                luaString += QString("local %1 = 0;\n").arg(alarmSerialList[i].val[0]);
            }
        }
    }

    for (int i = 0; i < alarmDangerList.size(); i++)
    {
        if (alarmDangerList[i].type == THRESHOLD_TYPE_VARIABLE ||
            alarmDangerList[i].type == THRESHOLD_TYPE_INIT_VAL ||
            alarmDangerList[i].type == THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE ||
            alarmDangerList[i].type == THRESHOLD_TYPE_CHANGE_RATE)
        {
            if (!parameterList.contains(alarmDangerList[i].val[0]))
            {
                parameterList.push_back(alarmDangerList[i].val[0]);
                luaString += QString("local %1 = 0;\n").arg(alarmDangerList[i].val[0]);
            }
        }
    }

    getLevelString(alarmNormalList, luaString, Alarm_Normal,errMsg);
    getLevelString(alarmWarningList, luaString, Alarm_Waring, errMsg);
    getLevelString(alarmCommonList, luaString, Alarm_Common, errMsg);
    getLevelString(alarmSerialList, luaString, Alarm_Serious, errMsg);
    getLevelString(alarmDangerList, luaString, Alarm_Dangerous, errMsg);

    luaString += "return 0;\nend\n";
    QFile file("./tmpLua.lua");
    file.open(QIODevice::ReadWrite | QIODevice::Truncate);

    QTextStream f(&file);
    f << luaString;
    file.close();
    QString filePath = file.fileName();

    return m_luaScriptCheck.checkLuaScript(filePath, errMsg) == 0;
}

bool conventSymbols2Lua::getStructFromJson(QString jsonString, QVector<THRESHOLD_ELEMENT> &element)
{
    Json::Value jsonRoot;
    element.clear();

    Json::Reader read;
    read.parse(jsonString.toStdString().c_str(), jsonRoot);

    for (int i = 0; i < jsonRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)jsonRoot[i]["type"].asInt();
        for (int j = 0; j < jsonRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(jsonRoot[i]["val"][j].asString().c_str()));
        }
        element.push_back(ele);
    }

    return true;
}

void conventSymbols2Lua::getLevelString(QVector<THRESHOLD_ELEMENT> &elementList, QString &luaString, DeviceAlarmLevel level, QString &errMsg)
{
    luaString += "if (";

    for (int i = 0; i < elementList.size(); i++)
    {
        switch (elementList[i].type)
        {
        case THRESHOLD_TYPE_PLUS:
        {
            luaString += "+ ";
            break;
        }
        case THRESHOLD_TYPE_MINUS:
        {
            luaString += "- ";
            break;
        }
        case THRESHOLD_TYPE_MUTIPLE:
        {
            luaString += "* ";
            break;
        }
        case THRESHOLD_TYPE_DEVIDED:
        {
            luaString += "/ ";
            break;
        }
        case THRESHOLD_TYPE_EQUAL:
        {
            luaString += "== ";
            break;
        }
        case THRESHOLD_TYPE_GREATER_THEN:
        {
            luaString += "> ";
            break;
        }
        case THRESHOLD_TYPE_LESS_THEN:
        {
            luaString += "< ";
            break;
        }
        case THRESHOLD_TYPE_LEFT_PARENTHESES:
        {
            luaString += "( ";
            break;
        }
        case THRESHOLD_TYPE_RIGHT_PARENTHESES:
        {
            luaString += ") ";
            break;
        }
        case THRESHOLD_TYPE_LEFT_BRACKET:
        {
            luaString += "[ ";
            break;
        }
        case THRESHOLD_TYPE_RIGHT_BRACKET:
        {
            luaString += "] ";
            break;
        }
        case THRESHOLD_TYPE_NOT_EQUAL:
        {
            luaString += "!= ";
            break;
        }
        case THRESHOLD_TYPE_GREATER_EQUAL:
        {
            luaString += ">= ";
            break;
        }
        case THRESHOLD_TYPE_LESS_EQUAL:
        {
            luaString += "<= ";
            break;
        }
        case THRESHOLD_TYPE_AND:
        {
            luaString += "and ";
            break;
        }
        case THRESHOLD_TYPE_OR:
        {
            luaString += "or ";
            break;
        }
        case THRESHOLD_TYPE_INCLUDE:
        {
            //                 if (i > 0 && (elementList[i - 1].type == THRESHOLD_TYPE_VARIABLE || elementList[i - 1].type == THRESHOLD_TYPE_CURRENT_VARIABLE || elementList[i - 1].type == THRESHOLD_TYPE_VARIABLE
            //                     || elementList[i - 1].type == THRESHOLD_TYPE_INIT_VAL || elementList[i - 1].type == THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE || elementList[i - 1].type == THRESHOLD_TYPE_INITIAL_DIFFERENCE_VALUE
            //                     || elementList[i - 1].type == THRESHOLD_TYPE_DIFFERENCE_TEMPERATURE_BETWEEN_PHASES))
            //                 {
            //                     luaString
            //                 }
            //                 luaString += "";
            break;
        }
        case THRESHOLD_TYPE_EXCLUDE:
        {
            break;
        }
        case THRESHOLD_TYPE_IN_RANGE:
        {
            break;
        }
        case THRESHOLD_TYPE_NOT_IN_RANGE:
        {
            break;
        }
        case THRESHOLD_TYPE_VARIABLE:
        {
            luaString += elementList[i].val[0] + " ";
            break;
        }
        case THRESHOLD_TYPE_CURRENT_VARIABLE:
        {
            luaString += elementList[i].val[0] + " ";
            break;
        }
        case THRESHOLD_TYPE_NUMBER:
        {
            luaString += QString("%1 ").arg(elementList[i].val[0].toFloat());
            break;
        }
        case THRESHOLD_TYPE_INIT_VAL:
        {
            luaString += elementList[i].val[0] + " ";
            break;
        }
        case THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE:
        {
            luaString += elementList[i].val[0] + " ";
            break;
        }
        case THRESHOLD_TYPE_INITIAL_DIFFERENCE_VALUE:
        {
            luaString += elementList[i].val[0] + " ";
            break;
        }
        case THRESHOLD_TYPE_DIFFERENCE_TEMPERATURE_BETWEEN_PHASES:
        {
            luaString += elementList[i].val[0] + " ";
            break;
        }
        case THRESHOLD_TYPE_CHANGE_RATE:
        {
            luaString += elementList[i].val[0] + " ";
            break;
        }
        case THRESHOLD_TYPE_ABS_FUNC:
        {
            break;
        }
        default:
            break;
        }
    }

    luaString += QString(") then \nreturn %1;\nend\n").arg(level);
}

