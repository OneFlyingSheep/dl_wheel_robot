#ifndef __DL_COMMON_WHEEL_ROBOT_TASK_H__
#define __DL_COMMON_WHEEL_ROBOT_TASK_H__

#include "common/DLWheelRobotGlobalDef.hpp"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreServer.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include <QObject>
#include "LibDLLuaScript/LuaThreshold.h"
#include "LibDLSMSMessage/DLSMSMessage.h"
#include "LibZip/LibZip.h"
#include "VisionAlgoDef.h"
#include <boost/signals2.hpp>

class MeterAction;

class LibDLWheelRobotTask : public QObject
{
    Q_OBJECT

public:
    LibDLWheelRobotTask(QObject *parent = 0);

    ~LibDLWheelRobotTask();

    void slot_taskCollectFinish(WheelRobotDeviceCollectStatus status);

    void slot_updateTimedTask();
    void slot_deviceUploadFinish(WheelRobotDeviceCollectData data);
    void slot_setCurrentTask(WheelRobotTaskBegin task);
    void slot_setCurrentDevice(QString task_uuid, QString device_uuid, int device_count);

    WheelRobotCurrentTaskInfomation slot_getCurrentTask();
    void slot_countTodayTasks();

public:
	//	巡检项记录推送
	boost::signals2::signal<void(WheelInspectResultStruct)> signal_inspect_notify;

	//	任务完成
	boost::signals2::signal<void()> signal_updateTimedTask;

	//	任务开始
	boost::signals2::signal<void(WheelRobotDeviceCollectData)> signal_deviceUploadFinish;


private:

    int getVisionListSize();
    void doVisionCheck(WheelRobotDeviceCollectData data);
    void visionFunc();
    void timedTaskFunc();
    WheelRobotDeviceCollectData getAndRemoveVisionFrontData();
    int bGetAndRemoveVisionFrontData(WheelRobotDeviceCollectData &data);
    int getTimedTaskDataListSize(WheelTaskTemplateStruct &task);
    WheelTaskTemplateStruct getTimedTaskFrontData();
    void removeTimedTaskFrontData();
    void clearTimedTaskData();
    QList<WheelTaskTemplateStruct> getTodayTasks();

    
    void setTodayTasks(QList<WheelTaskTemplateStruct> tasks);
	void initVisionInterface();

	bool judgeDeviceProperty(QString taskUUid, QString deviceUUid, QString &compareValue);
	QString compareInspectResultValue(QStringList devValue);

private:
    boost::thread *m_visionThread;
    QList<WheelRobotDeviceCollectData> m_visionCheckDataList;
    QList<WheelTaskTemplateStruct> m_timedTaskDataList;
    boost::mutex m_visionCheckMutex;
    boost::mutex m_timedTaskMutex;
    bool bVisionRunning;
    boost::thread *m_timedTaskThread;
    bool bTimedTaskRunning;
    bool bUpdateTimedTask;

    boost::thread *m_currentTaskInfoThread;
    void currentTaskFunc();
    bool bCurrentTaskRunning;
    bool bNewTask;

    LuaThreshold luaCheck;

    WheelRobotCurrentTaskInfomation m_currentTaskInfo;
    boost::mutex m_currentTaskInfoLock;
    bool m_bInfraredType = true;
    LibZip m_pZip;

};
#endif