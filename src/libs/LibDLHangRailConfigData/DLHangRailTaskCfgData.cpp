#include "DLHangRailTaskCfgData.h"
#include <fstream>
#include <LibCommonFileOperation/LibCommonFileOperation.h>
#include <QDebug>
#include <boost/uuid/uuid_generators.hpp>  
#include <boost/uuid/uuid_io.hpp>  
#include <boost/uuid/uuid.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
DLHangRailTaskCfgData::DLHangRailTaskCfgData()
{
    loadTask();
    loadTaskExecType();
}

DLHangRailTaskCfgData::~DLHangRailTaskCfgData()
{
    saveTask();
}

bool DLHangRailTaskCfgData::insertSingleTaskTemplate(taskTemplateType task)
{
    bool bRet = false;
    task.task_template_ssid = getSessionId();
    bRet = ROBOTDB_DB.insertSingleTaskTemplate(task);
    if (bRet)
    {
        loadTask();
    }
    return bRet;

}

bool DLHangRailTaskCfgData::deleteTaskTemplateBySsId(QString Ssid)
{
    return ROBOTDB_DB.deleteSingleTaskBySsid(Ssid);
}

bool DLHangRailTaskCfgData::getTaskTemplateBySsId(QString Ssid, taskTemplateType &task)
{
    //QVector<taskConfigType>::iterator itr = m_taskList.begin();
    //for (itr; itr != m_taskList.end();)
    //{
    //    if (itr->taskId == id)
    //    {
    //        task = *itr;
    //        return true;
    //    }
    //    else
    //    {
    //        itr++;
    //    }
    //}
    return false;
}

bool DLHangRailTaskCfgData::updateTodayTimedTask(QList<taskTemplateType> &tasks)
{
    bool bRet = ROBOTDB_DB.getTodayTimeTask(tasks);
    if (bRet)
    {
        m_todayTimedTaskList.clear();
        m_todayTimedTaskList = tasks;
    }
    return bRet;
    
}

bool DLHangRailTaskCfgData::getNextTaskRemainTime(int &intVal)
{
    if (m_todayTimedTaskList.size() > 0)
    {
        taskTemplateType nextTask = m_todayTimedTaskList.front();
        //intVal = nextTask.task_start_data - getCurrentTime();
        return true;
    }
    else
    {
        return false;
    }
}

bool DLHangRailTaskCfgData::getAllDevicesTask(taskTemplateType &task)
{
    bool bRet = false;
    task.task_type_id = 1;
    task.task_template_ssid = "";
    task.task_template_name = "全站巡检";
    bRet = ROBOTDB_DB.getAllDevicesDeviceSsId(task.task_template_device_list);
    task.task_end_action_id = 1;
    return true;
}

QList<taskTemplateType> DLHangRailTaskCfgData::getAlltask()
{
    return m_taskList;
}

QList<taskTemplateType> DLHangRailTaskCfgData::getTodayTimedTask()
{
    return m_todayTimedTaskList;
}

std::map<int, taskExecuteType> DLHangRailTaskCfgData::getTaskExecType()
{
    return m_taskExecTypeList;
}

void DLHangRailTaskCfgData::loadTask()
{
    m_todayTimedTaskList.clear();
    m_taskList.clear();

    ROBOTDB_DB.getAllTaskTemplate(m_taskList);

    updateTodayTimedTask(m_todayTimedTaskList);
}

void DLHangRailTaskCfgData::saveTask()
{
    //QString taskFilePath = "config/taskList.txt";
    //std::fstream f;
    //if (bFileExists(taskFilePath))
    //{
    //    f.open(taskFilePath.c_str(), std::ios::out | std::ios::trunc);
    //    if (!f.is_open())
    //    {
    //        return;
    //    }
    //    QVector<taskConfigType>::iterator itr = m_taskList.begin();
    //    while (itr != m_taskList.end())
    //    {
    //        char buff[COMMON_WR_BUFF_LENGTH];
    //        QString deviceIdListStr;
    //        for (int i = 0; i < itr->deviceIdList.size(); i++)
    //        {
    //            deviceIdListStr += getString(itr->deviceIdList[i]) + " ";
    //        }
    //        //sprintf(buff, "%d,%s,%d,%d,%d,%d,%s", itr->taskId, itr->taskName.c_str(), (int)itr->taskType,
    //        //    itr->startTime, itr->loopTimes, itr->loopInterVal, deviceIdListStr.c_str());
    //        f << buff << std::endl;
    //        itr++;
    //    }
    //    f.flush();
    //    f.close();
    //}
}

void DLHangRailTaskCfgData::loadTaskExecType()
{
    m_taskExecTypeList.clear();
    ROBOTDB_DB.getTaskExecType(m_taskExecTypeList);
}

int DLHangRailTaskCfgData::getCurrentTime()
{
    time_t time(NULL);
    return (int)time;
}

QString DLHangRailTaskCfgData::getSessionId()
{
    boost::uuids::random_generator rgen;//随机生成器  
    boost::uuids::uuid ssid = rgen();//生成一个随机的UUID
    std::string tmp = boost::lexical_cast<std::string>(ssid);
    boost::erase_all(tmp,"-");
    QString qsid = QString::fromStdString(tmp);
    return qsid;
}
