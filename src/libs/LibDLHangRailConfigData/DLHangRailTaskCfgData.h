#ifndef __DL_HANG_ROBOT_TASK_DATA_H__
#define __DL_HANG_ROBOT_TASK_DATA_H__

#include "common/DLHangRailRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <LibDLHangRailRobotDBOperation/LibDLHangRailRobotDBOperation.h>
#include <boost/thread/mutex.hpp>
#include <QString>
#include <QVector>
#include <map>

class DLHangRailTaskCfgData : public Singleton<DLHangRailTaskCfgData>
{
public:
    DLHangRailTaskCfgData();
    ~DLHangRailTaskCfgData();

public:
    bool insertSingleTaskTemplate(taskTemplateType task);
    bool deleteTaskTemplateBySsId(QString Ssid);
    bool getTaskTemplateBySsId(QString Ssid, taskTemplateType &task);

    bool updateTodayTimedTask(QList<taskTemplateType> &tasks);

    bool getNextTaskRemainTime(int &intVal);

    bool getAllDevicesTask(taskTemplateType &task);

    QList<taskTemplateType> getAlltask();
    QList<taskTemplateType> getTodayTimedTask();

    std::map<int, taskExecuteType> getTaskExecType();

private:
    void loadTask();
    void saveTask();

    void loadTaskExecType();

    int getCurrentTime();

    QString getSessionId();

private:
    QList<taskTemplateType> m_taskList;
    QList<taskTemplateType> m_todayTimedTaskList;

    std::map<int, taskExecuteType> m_taskExecTypeList;
};

#define ROBOTTASKCFG DLHangRailTaskCfgData::GetSingleton()

//#define ROBOTTASKCFG_ADD_TASK(task) ROBOTTASKCFG.addInTask(task)
//
//#define ROBOTTASKCFG_DEL_TASK_BY_ID(id) ROBOTTASKCFG.deleteTaskById(id)
//#define ROBOTTASKCFG_DEL_TASK_BY_NAME(name) ROBOTTASKCFG.deleteTaskByName(name)
//
//#define ROBOTTASKCFG_GET_TASK_BY_ID(id, task) ROBOTTASKCFG.getTaskById(id, task)
//#define ROBOTTASKCFG_GET_TASK_BY_NAME(name, task) ROBOTTASKCFG.getTaskByName(name, task)
//
//#define ROBOTTASKCFG_GET_NEXT_TASK_REMAIN_TIME(val) ROBOTTASKCFG.getNextTaskRemainTime(val)
//
//#define ROBOTTASKCFG_GET_ALL_TASK ROBOTTASKCFG.getAlltask()

#endif