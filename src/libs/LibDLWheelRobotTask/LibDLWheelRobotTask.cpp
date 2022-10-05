//#pragma push_macro("slots")
//#undef slots
//#include <Python.h>
//#pragma pop_macro("slots")

#include "LibDLWheelRobotTask.h"
#include <QtSql/QSqlError>
#include <QDebug>
#include <QSettings>
#include <QApplication>
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include "LibDLNewInfraredSDK/InfraredChoose.h"
#include <ctime>  
#include "image_recognition_sdk.h"



LibDLWheelRobotTask::LibDLWheelRobotTask(QObject *parent /*= 0*/)
{
    bUpdateTimedTask = false;
    slot_countTodayTasks();
    bVisionRunning = true;
    bTimedTaskRunning = true;
    bCurrentTaskRunning = true;
    bNewTask = false;	


    if (WHEEL_ROBOT_CORE_CONFIG.getCfg().infraredManufacturer >= 1)
    {
        INFRARED_BASE.setInfraredType(SDKInfraredType::SDK_JUGE_INFRARED);
        m_bInfraredType = false;
    }
    else
    {
        INFRARED_BASE.setInfraredType(SDKInfraredType::SDK_GUIDE_INFRARED);
        m_bInfraredType = true;
    }
    
	initVisionInterface();

    m_visionThread = new boost::thread(boost::bind(&LibDLWheelRobotTask::visionFunc, this));
    m_timedTaskThread = new boost::thread(boost::bind(&LibDLWheelRobotTask::timedTaskFunc, this));
    m_currentTaskInfoThread = new boost::thread(boost::bind(&LibDLWheelRobotTask::currentTaskFunc, this));

}

LibDLWheelRobotTask::~LibDLWheelRobotTask()
{
    if (NULL != m_visionThread)
    {
        bVisionRunning = false;
        m_visionThread->join();
        m_visionThread = NULL;
    }

    if (NULL != m_timedTaskThread)
    {
        bTimedTaskRunning = false;
        m_timedTaskThread->join();
        m_timedTaskThread = NULL;
    }

    if (NULL != m_currentTaskInfoThread)
    {
        bCurrentTaskRunning = false;
        m_currentTaskInfoThread->join();
        m_currentTaskInfoThread = NULL;
    }

}


void LibDLWheelRobotTask::initVisionInterface()
{
	std::string script_path = (QCoreApplication::applicationDirPath() + "/script").toLocal8Bit().constData();
	std::string locate_model_path = (QCoreApplication::applicationDirPath() + "/script/model").toLocal8Bit().constData();

	ROS_INFO("Load python scripts successfully,script path:%s", script_path.c_str());

	//QString rootPath = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath;
	//QString visionLogPath = rootPath + "/log/Vision/";
	//QString modelPath = rootPath + "/models/";
	//char result_image_path[512] = { 0 };
	//QString imgCompressPath = "";
	//QString imgPath = "D:/hl_ws/common_vision_tools/2-数据标注/表计识别标注/标定工具/标定文件/d1f944b995a1cfafc7c47a15de9e0cb4/source_0.jpg";

	//QString templatePath = "D:/hl_ws/common_vision_tools/2-数据标注/表计识别标注/标定工具/标定文件/d1f944b995a1cfafc7c47a15de9e0cb4/";

	//std::vector<CMeterIdentificationResult> recognition_result;

	//meter_identification_algorithm((char*)imgPath.toLocal8Bit().toStdString().c_str(),
	//	(char*)templatePath.toLocal8Bit().toStdString().c_str(),
	//	(char*)visionLogPath.toLocal8Bit().toStdString().c_str(),
	//	(char*)modelPath.toLocal8Bit().toStdString().c_str(),
	//	0,
	//	result_image_path,
	//	recognition_result);
}

void LibDLWheelRobotTask::slot_taskCollectFinish(WheelRobotDeviceCollectStatus status)
{
    boost::mutex::scoped_lock lock(m_currentTaskInfoLock);
    bNewTask = false;
    WheelRobotCurrentTaskInfomation t;
    m_currentTaskInfo = t;
    //memset(&m_currentTaskInfo, 0, sizeof(m_currentTaskInfo));
}

void LibDLWheelRobotTask::slot_deviceUploadFinish(WheelRobotDeviceCollectData data)
{
    boost::mutex::scoped_lock lock(m_visionCheckMutex);
    m_visionCheckDataList.push_back(data);
}

void LibDLWheelRobotTask::slot_setCurrentTask(WheelRobotTaskBegin task)
{
    boost::mutex::scoped_lock lock(m_currentTaskInfoLock);

    if (bNewTask)
    {
        ROS_WARN("former task not stop while new task comming : former task:%s, current task:%s!", task.task_uuid.toStdString().c_str(), m_currentTaskInfo.task_uuid.toStdString().c_str());
    }

    bNewTask = true;
    m_currentTaskInfo.task_uuid = task.task_uuid;
    WHEEL_ROBOT_DB.getTaskNameByTaskUuid(task.task_uuid, m_currentTaskInfo.task_name);
    m_currentTaskInfo.start_time = task.start_time;
    m_currentTaskInfo.predict_duration = task.predict_duration;
    WHEEL_ROBOT_DB.getDevicesByTaskUuid(m_currentTaskInfo.devices, task.task_uuid);
    m_currentTaskInfo.points = task.points;
    m_currentTaskInfo.alarmDeviceCount = task.abnormal_device;
    m_currentTaskInfo.current_device_name = "";
    m_currentTaskInfo.current_device_uuid = "";
    
}

void LibDLWheelRobotTask::slot_setCurrentDevice(QString task_uuid, QString device_uuid, int device_count)
{
    boost::mutex::scoped_lock lock(m_currentTaskInfoLock);

    if (!bNewTask)
    {
        ROS_ERROR("robot not doing task");
        return;
    }

    if (task_uuid != m_currentTaskInfo.task_uuid)
    {
        ROS_ERROR("error task_uuid!!");
    }
    else
    {
        m_currentTaskInfo.current_device_uuid = device_uuid;
        m_currentTaskInfo.device_count = device_count;
        WHEEL_ROBOT_DB.getDeviceFullNameByDeviceUuid(device_uuid, m_currentTaskInfo.current_device_name);
    }
}

WheelRobotCurrentTaskInfomation LibDLWheelRobotTask::slot_getCurrentTask()
{
    boost::mutex::scoped_lock lock(m_currentTaskInfoLock);
    return m_currentTaskInfo;
}

void LibDLWheelRobotTask::slot_updateTimedTask()
{
    bUpdateTimedTask = true;
}

int LibDLWheelRobotTask::getVisionListSize()
{
    boost::mutex::scoped_lock lock(m_visionCheckMutex);
    return m_visionCheckDataList.size();
}

WheelRobotDeviceCollectData LibDLWheelRobotTask::getAndRemoveVisionFrontData()
{
    boost::mutex::scoped_lock lock(m_visionCheckMutex);
    WheelRobotDeviceCollectData data = m_visionCheckDataList.front();
    m_visionCheckDataList.removeFirst();
    return data;
}

int LibDLWheelRobotTask::bGetAndRemoveVisionFrontData(WheelRobotDeviceCollectData &data)
{
    boost::mutex::scoped_lock lock(m_visionCheckMutex);
    int tsize = m_visionCheckDataList.size();
    if (tsize > 0)
    {
        data = m_visionCheckDataList.front();
        m_visionCheckDataList.removeFirst();
    }
    return tsize;
}

int LibDLWheelRobotTask::getTimedTaskDataListSize(WheelTaskTemplateStruct &task)
{
    boost::mutex::scoped_lock lock(m_timedTaskMutex);
    int tsize = m_timedTaskDataList.size();
    if (tsize > 0)
    {
        task = m_timedTaskDataList.front();
    }
    return tsize;
}

WheelTaskTemplateStruct LibDLWheelRobotTask::getTimedTaskFrontData()
{
    boost::mutex::scoped_lock lock(m_timedTaskMutex);
    WheelTaskTemplateStruct tmpData;
    tmpData = m_timedTaskDataList.front();
    return tmpData;
}

void LibDLWheelRobotTask::removeTimedTaskFrontData()
{
    boost::mutex::scoped_lock lock(m_timedTaskMutex);
    m_timedTaskDataList.removeFirst();
}

void LibDLWheelRobotTask::clearTimedTaskData()
{
    boost::mutex::scoped_lock lock(m_timedTaskMutex);
    m_timedTaskDataList.clear();
}

QList<WheelTaskTemplateStruct> LibDLWheelRobotTask::getTodayTasks()
{
    QList<WheelTaskTemplateStruct> tasks;
    WHEEL_ROBOT_DB.getWheelTaskTemplateAllTimedTasksSortByStartTimeAfterCurrent(tasks);
    QList<WheelTaskTemplateStruct> todayTask;
    QList<WheelTaskTemplateStruct>::iterator itr = tasks.begin();
    QDate currentDate = QDate::currentDate();
    for (; itr != tasks.end(); itr++)
    {
        if (WHEEL_ROBOT_TASK_IMMEDIATELY_TASK == itr->task_type_id)
        {
            ROS_ERROR("immediately task, error task type, maybe getWheelTaskTemplateAllTimedTasks func error");
        }
        else if (WHEEL_ROBOT_TASK_TIMED_TASK == itr->task_type_id)
        {
            QDate taskDate = QDate::fromString(itr->task_start_date, "yyyy-MM-dd");
            if (currentDate.daysTo(taskDate) == 0)
            {
                todayTask.push_back(*itr);
            }
        }
        else if (WHEEL_ROBOT_TASK_LOOP_TASK == itr->task_type_id)
        {
            if (WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_DAY == itr->task_loop_type_id)
            {
                todayTask.push_back(*itr);
            }
            else if (WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_WEEK == itr->task_loop_type_id)
            {
                if (currentDate.dayOfWeek() == itr->task_start_date.toInt())
                {
                    todayTask.push_back(*itr);
                }
            }
            else if (WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_MONTH == itr->task_loop_type_id)
            {
                if (currentDate.day() == itr->task_start_date.toInt())
                {
                    todayTask.push_back(*itr);
                }
            }
            else if (WHEEL_ROBOT_TASK_LOOP_TYPE_FIXED_INTERVAL_DAYS == itr->task_loop_type_id)
            {
                QDate startDate = QDate::fromString(itr->task_start_date, "yyyy-MM-dd");
                if ((currentDate.daysTo(startDate) % itr->task_repeat_duration) == 0)
                {
                    todayTask.push_back(*itr);
                }
            }
            else if (WHEEL_ROBOT_TASK_LOOP_TYPE_CALENDAR == itr->task_loop_type_id)
            {
                QList<QDate> dateList;
                QStringList dateStringList = itr->task_start_date.split(",");
                for (int i = 0; i < dateStringList.size(); i++)
                {
                    QDate splitData = QDate::fromString(dateStringList[i], "yyyy-MM-dd");
                    if (splitData.month() == currentDate.month() && splitData.day() == currentDate.day())
                    {
                        todayTask.push_back(*itr);
                        break;
                    }
                }
            }
            else
            {
                ROS_ERROR("error task loop type");
            }
        }
        else
        {
            ROS_ERROR("unknown task type");
        }
    }

    return todayTask;
}

void LibDLWheelRobotTask::slot_countTodayTasks()
{
    setTodayTasks(getTodayTasks());
}

void LibDLWheelRobotTask::setTodayTasks(QList<WheelTaskTemplateStruct> tasks)
{
    boost::mutex::scoped_lock lock(m_timedTaskMutex);
    m_timedTaskDataList.clear();
    m_timedTaskDataList = tasks;
}

void LibDLWheelRobotTask::currentTaskFunc()
{
    while (bCurrentTaskRunning)
    {
        Sleep(1000);
        if (bNewTask)
        {
            boost::mutex::scoped_lock lock(m_currentTaskInfoLock);
            WheelRobotCurrentTaskInfoShow showTask;
            showTask.task_uuid = m_currentTaskInfo.task_uuid;
            showTask.task_name = m_currentTaskInfo.task_name;
            showTask.predict_duration = m_currentTaskInfo.predict_duration;
            showTask.current_device_uuid = m_currentTaskInfo.current_device_uuid;
            showTask.current_device_name = m_currentTaskInfo.current_device_name;
            showTask.total_devices = m_currentTaskInfo.devices.size();
            showTask.checked_devices = m_currentTaskInfo.device_count;
            showTask.percent = (showTask.total_devices != 0) ? (showTask.checked_devices * 1.0f / showTask.total_devices * 1.0f) : 0;
            ROS_INFO("task currentTaskFunc: showTask.total_devices:%d   showTask.checked_devicesP:%d", showTask.total_devices, showTask.checked_devices);
            showTask.alarmDeviceCount = m_currentTaskInfo.alarmDeviceCount;
            WHEEL_ROBOT_DB.getTaskPropertyByTaskUuid(showTask.task_uuid, showTask.task_property);
            WHEEL_CORE_SERVER.Remote_robot_current_task_status(showTask);
        }
        else
        {
            boost::mutex::scoped_lock lock(m_currentTaskInfoLock);
            WheelRobotCurrentTaskInfoShow showTask;

            WHEEL_CORE_SERVER.Remote_robot_current_task_status(showTask);
        }
    }
}

void LibDLWheelRobotTask::doVisionCheck(WheelRobotDeviceCollectData data)
{
	//qDebug() << "===========================> task_uuid :" << data.task_uuid << ", device_uuid: " << data.device_uuid;
	WheelRobotDeviceStruct dev;

	WHEEL_ROBOT_DB.queryDeviceByDeviceUuid(data.device_uuid, dev);
	QString rootPath = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath;
	QString visionLogPath = rootPath + "/log/Vision/";
	QString modelPath = rootPath + "/models/";
	char result_image_path[512] = { 0 };
	QString imgCompressPath = rootPath + "/task/" + data.task_uuid + "/" + data.device_uuid + "/" + data.device_uuid;
	QString imgPath = imgCompressPath + ".jpg";
	QString imgInfrad = imgCompressPath + ".jpg";
	if ((dev.recognition_type_id == WHEEL_ROBOT_RECOGNIZE_TYPE_INFRARED) || (dev.recognition_type_id == WHEEL_ROBOT_RECOGNIZE_TYPE_INF_AND_SURFACE))
	{
		if (m_bInfraredType)
		{
			imgPath = imgInfrad.replace(".jpg", "-temp.jpg");
		}
		else
		{
			imgPath = imgInfrad.replace(".jpg", "-temp.jpg");
		}
	}
	QString templatePath = rootPath + "/template/" + data.device_uuid + "/";
	//cv::Mat result_mat;
	ROS_INFO("Start recognition device result!");
	ROS_INFO("Recognition device imgPath:%s", imgPath.toLocal8Bit().toStdString().c_str());
	ROS_INFO("Recognition device templatePath:%s", templatePath.toLocal8Bit().toStdString().c_str());
	ROS_INFO("Recognition device type id:%d", int(dev.recognition_type_id));

	std::vector<CMeterIdentificationResult> recognition_result;

	meter_identification_algorithm((char*)imgPath.toLocal8Bit().toStdString().c_str(),
		(char*)templatePath.toLocal8Bit().toStdString().c_str(),
		(char*)visionLogPath.toLocal8Bit().toStdString().c_str(),
		(char*)modelPath.toLocal8Bit().toStdString().c_str(),
		0,
		result_image_path,
		recognition_result);

	QString result_img = imgCompressPath + QString("_result.jpg");
	QFile::copy(result_image_path, result_img);

	for (int i = 0; i < 3; i++)
	{
		Sleep(300);
		QFile file;
		if (file.exists(imgPath))
		{
			ROS_INFO("Photo exists!Path:%s", imgPath.toLocal8Bit().toStdString().c_str());
			break;
		}
		if (i == 2)
		{
			ROS_INFO("Photo not exists!");
			return;
		}
	}

	WheelInspectResultStruct inspectRes;
	DeviceAlarmLevel level_id = Alarm_NONE;

	inspectRes.task_uuid = data.task_uuid;
	inspectRes.device_uuid = dev.device_uuid;
	inspectRes.inspect_status_id = (WheelRobotInspectResultStatusType)data.deviceCollectStatus;
	inspectRes.inspect_time = data.date_time;
	inspectRes.is_dealed = false;
	if (!recognition_result.empty()){
		inspectRes.inspect_result = QString::number(recognition_result[0].value);
	}
	else {
		inspectRes.inspect_result = "No target";
	}

	switch (dev.recognition_type_id)
	{
	case WHEEL_ROBOT_RECOGNIZE_TYPE_METER:
	{
		switch (dev.meter_type_id)
		{
		case WHEEL_ROBOT_METER_TYPE_LIGHTNING_PROTECTOR_TIMES:
		case WHEEL_ROBOT_METER_TYPE_SWITCHING_TIMES:
		{
			// ocr
			ROS_INFO("Recognition 1! re:%d  me:%d", dev.recognition_type_id, dev.meter_type_id);



			ROS_INFO("Recognition 1 end!");
			break;
		}

		default:
		{
			ROS_INFO("Recognition 2!");
			ROS_INFO("Recognition 2 end!");
			break;
		}
		}

		// threshole check;
		//if (meter_result.ret_msg_.err_code_ != 0 || meter_result.results_.empty())
		//{
		//	ROS_INFO("Recognition 3!");
		//	level_id = Alarm_NoIdentifyAbnormal;
		//	inspectRes.inspect_result = "异常";
		//	ROS_INFO("Recognition 3 end!");
		//}
		//else
		//{
		//	ROS_INFO("Recognition 4!");
		//	level_id = luaCheck.thresholdCheck(QString::number(meter_result.results_.front().value_), dev.device_uuid, dev.meter_type_id);
		//	inspectRes.inspect_result = QString::number(meter_result.results_.front().value_);
		//	ROS_INFO("Recognition 4 end!");
		//}
		break;
	}
	case WHEEL_ROBOT_RECOGNIZE_TYPE_STATUS:
	{
		ROS_INFO("Recognition 5!");
		//meter_identification_algorithm(std::string(templatePath.toLocal8Bit()), std::string(imgPath.toLocal8Bit()), meter_result);
		ROS_INFO("Recognition 5 end!");

		//if (meter_result.ret_msg_.err_code_ != 0 || meter_result.results_.empty())
		//{
		//	level_id = Alarm_NoIdentifyAbnormal;
		//	inspectRes.inspect_result = "异常";
		//}
		//else
		//{
		//	level_id = luaCheck.thresholdCheck(QString::number(meter_result.results_.front().value_), dev.device_uuid, dev.meter_type_id);
		//	inspectRes.inspect_result = QString::number(meter_result.results_.front().value_);
		//}

		break;
	}
	case WHEEL_ROBOT_RECOGNIZE_TYPE_INFRARED:
	case WHEEL_ROBOT_RECOGNIZE_TYPE_INF_AND_SURFACE:
    {
		break;
	}
	case WHEEL_ROBOT_RECOGNIZE_TYPE_SURFACE:
	{
		ROS_INFO("Recognition 7!");
		level_id = Alarm_Normal;
		inspectRes.inspect_result = "正常";
		// nothing;
		break;
	}
	case WHEEL_ROBOT_RECOGNIZE_TYPE_AUDIO:
	{
		ROS_INFO("Recognition 8!");
		// null
		level_id = Alarm_Normal;
		inspectRes.inspect_result = "正常";
		break;
	}
	case WHEEL_ROBOT_RECOGNIZE_TYPE_FACE:
	{
		ROS_INFO("Recognition 9!");
		level_id = Alarm_Normal;
		inspectRes.inspect_result = "正常";
		break;
	}
	case WHEEL_ROBOT_RECOGNIZE_TYPE_FIRE:
	{
		ROS_INFO("Recognition 10!");
		level_id = Alarm_Normal;
		inspectRes.inspect_result = "正常";
		break;
	}
	case WHEEL_ROBOT_RECOGNIZE_TYPE_FIRE_HOSE:
	{
		ROS_INFO("Recognition 11!");
		level_id = Alarm_Normal;
		inspectRes.inspect_result = "正常";
		break;
	}
	default:
		ROS_INFO("Recognition 12!");
		ROS_ERROR("unknown recognition type:%d", (int)dev.recognition_type_id);
		level_id = Alarm_NoIdentifyAbnormal;
		inspectRes.inspect_result = "异常";
		break;
	}
	WHEEL_ROBOT_DB.getDeviceNameWithDeviceUUid(inspectRes.device_uuid, inspectRes.virtual_name);
    inspectRes.alarm_level_id = (level_id == Alarm_NONE) ? Alarm_NoIdentifyAbnormal : level_id;
	if (inspectRes.alarm_level_id > Alarm_NoIdentifyAbnormal)
	{
		inspectRes.alarm_level_id = Alarm_NoIdentifyAbnormal;
	}
    WHEEL_ROBOT_DB.updateDeviceAlarmLevel(inspectRes.alarm_level_id, inspectRes.device_uuid);
    WHEEL_ROBOT_DB.insertInspectResultDB(inspectRes);
    WHEEL_CORE_SERVER.Remote_robot_inspect_result(inspectRes);

	WHEEL_ROBOT_DB.getDeviceSnForDeviceUUidDB(inspectRes.device_uuid, inspectRes.deal_info_uuid, inspectRes.virtual_name);
	signal_inspect_notify(inspectRes);

	QImage img;
	img.load(imgCompressPath + QString("_result.jpg"));
	QImage result = img.scaled(960, 540, Qt::KeepAspectRatio, Qt::FastTransformation).scaled(91, 54, Qt::KeepAspectRatio, Qt::SmoothTransformation);
	bool isSuccess = result.save(imgCompressPath + QString("_scaled.jpg"), "JPEG", 20);

    do 
    {
        boost::mutex::scoped_lock lock(m_currentTaskInfoLock);
        if (bNewTask && data.task_uuid == m_currentTaskInfo.task_uuid && inspectRes.alarm_level_id != Alarm_Normal)
        {
            m_currentTaskInfo.alarmDeviceCount++;
            WheelRobotCurrentTaskInfoShow showTask;
            showTask.task_uuid = m_currentTaskInfo.task_uuid;
            showTask.task_name = m_currentTaskInfo.task_name;
            showTask.predict_duration = m_currentTaskInfo.predict_duration;
            showTask.current_device_uuid = m_currentTaskInfo.current_device_uuid;
            showTask.current_device_name = m_currentTaskInfo.current_device_name;
            showTask.total_devices = m_currentTaskInfo.devices.size();
            showTask.checked_devices = m_currentTaskInfo.device_count;
            showTask.percent = (showTask.total_devices != 0) ? (showTask.checked_devices * 1.0f / showTask.total_devices * 1.0f) : 0;
            ROS_INFO("task currentTaskFunc: showTask.total_devices:%d   showTask.checked_devicesP:%d", showTask.total_devices, showTask.checked_devices);
            showTask.alarmDeviceCount = m_currentTaskInfo.alarmDeviceCount;
            WHEEL_ROBOT_DB.getTaskPropertyByTaskUuid(showTask.task_uuid, showTask.task_property);
            WHEEL_CORE_SERVER.Remote_robot_current_task_status(showTask);
        }
    } while (0);

}

void LibDLWheelRobotTask::visionFunc()
{
	/*
	char* src_image_path = "d://test/1.jpg";
	char parameter_path[512] = "d://test/1/";
	char log_path[512] = "d://test/Log/";
	char* model_path = "d://test/Model/";
	int step = 0;
	char result_image_path[512] = { 0 };

	std::vector<CMeterIdentificationResult> recognition_result;

	meter_identification_algorithm(src_image_path, parameter_path, log_path, model_path, step, result_image_path, recognition_result);
	*/


    while (bVisionRunning)
    {
        Sleep(100);

        WheelRobotDeviceCollectData data;
        int size = bGetAndRemoveVisionFrontData(data);

        if (size > 0)
        {
            if (data.bTaskFinish)
            {
                //get environment;
                WheelTaskStruct finishedTask;
                finishedTask.task_status_id = (WHEEL_ROBOT_TASK_END_TYPE_NORMAL == data.taskEndStatus) ? WHEEL_ROBOT_TASK_STATUS_FINISH : WHEEL_ROBOT_TASK_STATUS_ABORT;
                finishedTask.task_end_type_id = data.taskEndStatus;
                finishedTask.task_uuid = data.task_uuid;
                finishedTask.task_end_time = data.date_time.isEmpty() ? QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") : data.date_time;
                // more info
                ROS_INFO("task finish: task_uuid:%s, status: %d, end_type:%d;", finishedTask.task_uuid.toStdString().c_str(), (int)finishedTask.task_status_id, (int)finishedTask.task_end_type_id);
                WHEEL_ROBOT_DB.updateTaskEndStatus(finishedTask);

                m_pZip.create_taskfile_zip(data.task_uuid);
                continue;
            }
            else
            {
                try
                {
                    doVisionCheck(data);
                }
                catch (...)
                {
                    ROS_ERROR("vision check error!!, taskId:%s, devId:%s", data.task_uuid.toStdString().c_str(), data.device_uuid.toStdString().c_str());
                }
            }

			ROS_INFO("inspect virtual:start virtual detection!");
			QString compareValue = "";
			bool retInpect = judgeDeviceProperty(data.task_uuid, data.device_uuid, compareValue);
			if (retInpect)
			{
				WheelInspectResultStruct stru;
				stru.task_uuid = data.task_uuid;
				virtualDeviceConfig cfg;
				WHEEL_ROBOT_DB.getVirtualDeviceUUid(data.device_uuid, cfg);
				stru.device_uuid = cfg.virtual_uuid;
				stru.inspect_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
				stru.inspect_result = compareValue;
				stru.inspect_status_id = WHEEL_ROBOT_INSPECT_RES_STATUS_NORMAL;
				stru.is_dealed = 0;
				DeviceAlarmLevel alarm_level = luaCheck.virtualLuaScript(compareValue, cfg.virtual_uuid);
				stru.alarm_level_id = alarm_level;
				stru.deal_info_uuid = "";
				stru.virtual_name = cfg.sub_device_type_name.replace("A", "(虚拟设备)").replace("B", "(虚拟设备)").replace("C", "(虚拟设备)");

				ROS_INFO("inspect virtual:virtual detection retInpect true!");
				WheelInspectResultStruct inspectRes;
				inspectRes.task_uuid = data.task_uuid;
				inspectRes.device_uuid = cfg.virtual_uuid;
				inspectRes.inspect_time = stru.inspect_time;

				inspectRes.inspect_result = compareValue;
				inspectRes.inspect_status_id = WHEEL_ROBOT_INSPECT_RES_STATUS_NORMAL;
				inspectRes.is_dealed = 1;
				inspectRes.alarm_level_id = alarm_level;

				WHEEL_ROBOT_DB.insertInspectResultDB(inspectRes);
				WHEEL_CORE_SERVER.Remote_robot_inspect_result(stru);
			}
        }
    }

}

void LibDLWheelRobotTask::timedTaskFunc()
{
    QDateTime formerData = QDateTime::currentDateTime();
    QDateTime currentData;

    while (bTimedTaskRunning)
    {
        Sleep(1000);
        currentData = QDateTime::currentDateTime();
        if ((currentData.date() != formerData.date()) || bUpdateTimedTask)
        {
            bUpdateTimedTask = false;
            clearTimedTaskData();
            setTodayTasks(getTodayTasks());
        }

        WheelTaskTemplateStruct task;

        if (getTimedTaskDataListSize(task) <= 0)
        {
            ROS_INFO("none timed tasks");
            continue;
        }
        
        currentData = QDateTime::currentDateTime();
        int remainSecs = currentData.time().secsTo(task.task_start_time);
        ROS_INFO("timed task remain secs:%d", remainSecs);
        if (remainSecs <= 0)
        {
            // send
            WHEEL_CORE_SERVER.robot_task_assign(task);
            removeTimedTaskFrontData();
        }        
    }
}

bool LibDLWheelRobotTask::judgeDeviceProperty(QString taskUUid, QString deviceUUid, QString &compareValue)
{
	ROS_INFO("inspect virtual:taskUUid:%s ,devicesUUid:%s",taskUUid.toStdString(),deviceUUid.toStdString());
	QString releDev = "";
	QString virtualUUid = "";
	bool ret = WHEEL_ROBOT_DB.getVirtualRelevanceDeviceUUid(taskUUid, deviceUUid, releDev, virtualUUid);
	ROS_INFO("inspect virtual:releDev:%s ,virtualUUid:%s", releDev.toStdString(), virtualUUid.toStdString());
	if (!ret)
	{
		return ret;
	}
	QStringList devList= releDev.split(" ");
	QStringList devValue;

	if (devList.size() < 2)
	{
		ROS_INFO("inspect virtual:devList.size():%d", devList.size());
		return false;
	}

	for (int i = 0; i < devList.size(); i++)
	{
		QString resultValue = "";
		bool retValue = WHEEL_ROBOT_DB.getInspectResultValueForDeviceUUid(taskUUid, devList[i], resultValue);
		if (!retValue)
		{
			return retValue;
		}
		else
		{
			devValue.append(resultValue);
		}
	}
	//compareValue = compareInspectResultValue(devValue);
	if (devValue.size() == 2)
	{
		//泄露电流表Lua
		//暂时不用
		return false;
	}
	if (devValue.size() == 3)
	{
		//三项温差Lua
		compareValue = compareInspectResultValue(devValue);
		ROS_INFO("inspect virtual:compareValue:%d", compareValue.toStdString());
	}
	return true;
}

QString LibDLWheelRobotTask::compareInspectResultValue(QStringList devValue)
{
	float minValue = 0.0;
	float maxValue = 0.0;
	minValue = devValue[0].toFloat();
	maxValue = devValue[0].toFloat();
	for (int i = 0; i < devValue.size(); i++)
	{
		if (minValue < devValue[i].toFloat())
		{
		}
		else
		{
			minValue = devValue[i].toFloat();
		}
		if (maxValue < devValue[i].toFloat())
		{
			maxValue = devValue[i].toFloat();
		}
		else
		{
		}
	}
	float diffValue = maxValue - minValue;
	return QString("%1").arg(diffValue);
}