#include "LibDLWheelRobotCoreServer.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include <QUuid>
#include <LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <LibDLWheelRobotConfigData/DLWheelRobotVirtualDevice.h>
#include "LibDLWheelRobotBimCore/url_def.h"

#define PI (3.14159)
#define RESOLUTION (0.1)
static uint16_t wheelRobotCoreServerMsgId = 0;

static boost::asio::io_service io_service;
static boost::asio::io_service::work io_work(io_service);
WheelRobotCurrentTaskInfoShow g_last_task_status;

LibDLWheelRobotCoreServer::LibDLWheelRobotCoreServer()
{
	bStatusRealtimeRunning = true;
	bStatusNoneRealtimeRunning = true;
	bheartbeatRunning = true;
	m_current_robotID = -1;
	for (int i = 0; i < WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg().size(); ++i)
	{
		LibDLWheelRobotMsg *robot_socket = new LibDLWheelRobotMsg();
		robot_socket->runMessageLoop(WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].robotIp.toStdString(),
										WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].robotPort, SOCK_TYPE_CLIENT);
		m_robotSocketMap[WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].robotName.toInt()] = robot_socket;
	}

}

LibDLWheelRobotCoreServer::~LibDLWheelRobotCoreServer()
{
    disconnect();
}

void LibDLWheelRobotCoreServer::reconnectToRobot()
{
	for (auto itor = m_robotSocketMap.begin(); itor != m_robotSocketMap.end(); ++itor)
	{
		//std::string address = itor->first;
		//std::size_t pos = address.find_first_of(":");
		//std::string ip = address.substr(0, pos);
		//int port = atoi(address.substr(pos).c_str());
		//itor->second->renewConnect(ip, port);
	}
}

void LibDLWheelRobotCoreServer::disconnect()
{
	for (auto itor = m_robotSocketMap.begin(); itor != m_robotSocketMap.end(); ++itor)
	{
		if (NULL != itor->second)
		{
			delete itor->second;
			itor->second = NULL;
		}
	}
}

LibDLWheelRobotMsg* LibDLWheelRobotCoreServer::getCurrentRobotSocket()
{
	auto itor = m_robotSocketMap.find(m_current_robotID);
	return (itor == m_robotSocketMap.end()) ? NULL : itor->second;
}


void LibDLWheelRobotCoreServer::startServer()
{
	ROS_INFO("LibDLWheelRobotCoreServer::ip_string=%s, port = %d", WHEEL_ROBOT_CORE_CONFIG.getCfg().coreServerIp.toStdString().c_str(), WHEEL_ROBOT_CORE_CONFIG.getCfg().coreServerPort);
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string(WHEEL_ROBOT_CORE_CONFIG.getCfg().coreServerIp.toStdString()), WHEEL_ROBOT_CORE_CONFIG.getCfg().coreServerPort);
    m_serverSocket = boost::shared_ptr<CCoreServer>(new CCoreServer(endpoint, io_service));
    registerBack2CoreHandles();
    registerRobot2CoreHandels();
    m_serverSocket->initSocket();
    boost::thread * t1 = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
    m_heartbeatThread = new boost::thread(boost::bind(&LibDLWheelRobotCoreServer::robotheartbeatFunc, this));
}

void LibDLWheelRobotCoreServer::Remote_robot_msg(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_msg==Str:%s", msg->msgBody.getJsonString().c_str());
	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->postClientMsg(msg);
}

void LibDLWheelRobotCoreServer::Remote_robot_run_msg(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_run_msg==Str:%s", msg->msgBody.getJsonString().c_str());
	if (!getCurrentRobotSocket())
		return;
    getCurrentRobotSocket()->postClientMsg(msg);
}

void LibDLWheelRobotCoreServer::Remote_robot_switch_msg(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_switch_msg==Str:%s", msg->msgBody.getJsonString().c_str());
	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->postClientMsg(msg);
}

void LibDLWheelRobotCoreServer::Remote_robot_ptz_msg(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_ptz_msg==Str:%s", msg->msgBody.getJsonString().c_str());
	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->postClientMsg(msg);
}

void LibDLWheelRobotCoreServer::Remote_robot_ptz_infradred_msg(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_ptz_infradred_msg==Str:%s", msg->msgBody.getJsonString().c_str());
	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->postClientMsg(msg);
}

void LibDLWheelRobotCoreServer::Remote_robot_task_assign_msg(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_task_assign_msg==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_task_assign_msg retVal = NULL");
        return;
    }

    WheelRobotAssignTask assignTask;
    assignTask.task_uuid = QString::fromStdString(val["task_uuid"].asString());
    assignTask.task_name = QString::fromStdString(val["task_name"].asString());
    assignTask.task_edit_uuid = QString::fromStdString(val["task_edit_uuid"].asString());
    assignTask.task_template_uuid = QString::fromStdString(val["task_template_uuid"].asString());
    assignTask.priority = (WheelRobotTaskpriority)val["priority"].asInt();
    assignTask.task_end_action = (WheelRobotTaskEndActionType)val["task_end_action"].asInt();
	int creturn = val["return"].asInt();

    Json::Value dev_val = val["devices"];

    for (int i = 0; i < dev_val.size(); i++)
    {
        assignTask.devices.push_back(dev_val[i].asString());
    }    

    WheelTaskStruct task;
    task.task_uuid = assignTask.task_uuid;
    task.task_name = assignTask.task_name;
    task.task_edit_uuid = assignTask.task_edit_uuid;
    task.task_template_uuid = assignTask.task_template_uuid;
    task.task_type_id = WHEEL_ROBOT_TASK_IMMEDIATELY_TASK;
    task.task_status_id = WHEEL_ROBOT_TASK_STATUS_EXEC;
    task.task_start_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    task.task_end_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");

    QString errmsg;

    bool bRet = WHEEL_ROBOT_DB.insertTaskDB(task, errmsg);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_task_assign + WHEELROBOT_PointADD);
    body.jsonAppendElement("task_uuid", std::string(task.task_uuid.toLocal8Bit()));
    body.jsonAppendElement("task_name", std::string(task.task_name.toLocal8Bit()));
    body.jsonAppendElement("task_edit_uuid", std::string(task.task_edit_uuid.toLocal8Bit()));
    body.jsonAppendElement("task_template_uuid", std::string(task.task_template_uuid.toLocal8Bit()));
	body.jsonAppendElement("return", creturn);

    if (!bRet)
    {
        body.jsonAppendElement("status", false);
        body.jsonAppendElement("errmsg", std::string(errmsg.toLocal8Bit()).c_str());
    }
    else
    {
        body.jsonAppendElement("status", true);
		if (getCurrentRobotSocket()) {
			getCurrentRobotSocket()->postClientMsg(msg);
		}
    }

    m_serverSocket->postMsg2AllConnection(body);

}

void LibDLWheelRobotCoreServer::robot_env_sensor_info(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::robot_env_sensor_info==Str:%s", msg->msgBody.getJsonString().c_str());
	//	不是本台机器人不做转发
	if (msg->msgBody.getJsonVal()["robot_id"].asInt() != m_current_robotID) {
		return;
	}

	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::robot_env_sensor_info retVal = NULL");
		return;
	}

	robotEnvInfoSignal(((int)m_current_robotID), msg->msgBody.getJsonString());
}

void LibDLWheelRobotCoreServer::robot_task_traj_record(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::robot_task_traj_record==Str:%s", msg->msgBody.getJsonString().c_str());
	if (msg->msgBody.getJsonVal()["robot_id"].asInt() != m_current_robotID) {
		return;
	}

	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::robot_env_sensor_info retVal = NULL");
		return;
	}
	robotTaskTrajSignal(((int)m_current_robotID), msg->msgBody.getJsonString());
}

void LibDLWheelRobotCoreServer::robot_task_Direct(boost::shared_ptr<roboKitMsg> msg)
{
	////	不是本台机器人不做转发
	//if (msg->msgBody.getJsonVal()["robot_id"].asInt() != m_current_robotID) {
	//	return;
	//}
    m_serverSocket->postMsg2AllConnection(msg->msgBody);
}

void LibDLWheelRobotCoreServer::robot_task_Begin(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::robot_task_Begin==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();
    robot_task_Direct(msg);
    if (val.isNull())
    {
        return;
    }

    WheelRobotTaskBegin taskBegin;

    taskBegin.task_uuid = QString::fromStdString(val["task_uuid"].asString());
    m_nowRunningTaskUuid = taskBegin.task_uuid;
    taskBegin.start_time = QString::fromStdString(val["start_time"].asString());
    taskBegin.predict_duration = val["predict_duration"].asInt();

    Json::Value::iterator pointItr = val["points"].begin();
    for (; pointItr != val["points"].end(); pointItr++)
    {
        taskBegin.points.push_back(QString::fromStdString(pointItr->asString()));
    }

    std::vector<std::string> pointCooedVec;
    for (int i = 0; i < val["points_coord"].size(); i++)
    {
        pointCooedVec.push_back(val["points_coord"][i].asString());
    }

    taskBegin.abnormal_device = val["device_list"].size();
//    QStringList deviceList;
    WheelInspectResultStruct inspectRes;
    inspectRes.task_uuid = taskBegin.task_uuid;
    
    inspectRes.inspect_time = taskBegin.start_time;
    inspectRes.inspect_result = "异常";
    inspectRes.inspect_status_id = WHEEL_ROBOT_INSPECT_RES_STATUS_NORMAL;
    inspectRes.is_dealed = 0;
    inspectRes.alarm_level_id = Alarm_DeviceRecondition;
    inspectRes.deal_info_uuid = "";
    for (int i = 0; i < val["device_list"].size(); i++)
    {
        inspectRes.device_uuid = QString::fromStdString(val["device_list"][i].asString());
        WHEEL_ROBOT_DB.insertInspectResultDB(inspectRes);
/*        deviceList.append(QString::fromStdString(val["device_list"][i].asString()));*/
    }

    if (taskBegin.abnormal_device != 0)
    {
        inspectRes.isHangOut = true;
        inspectRes.alarm_count = taskBegin.abnormal_device;
        WHEEL_CORE_SERVER.Remote_robot_inspect_result(inspectRes);
    }

    bjRobotPatrolLineSignal(pointCooedVec);

    wheelRobotTaskCallback(taskBegin);

    WHEEL_ROBOT_DB.updateTaskDB(taskBegin.task_uuid, taskBegin.start_time);
    WheelRobotCurrentTaskInfoShow task;
    task.task_uuid = taskBegin.task_uuid;
    bjRobotTaskStatusDataSignal(task);
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_update_task_table_signal_req);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::robot_task_Current_Task_Device(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    QString task_uuid = QString::fromStdString(val["task_uuid"].asString());
    QString device_uuid = QString::fromStdString(val["device_uuid"].asString());
    int device_count = val["device_count"].asInt();

    wheelRobotCurrentDeviceCallback(task_uuid, device_uuid, device_count);
}

void LibDLWheelRobotCoreServer::robot_heart_beat_msg(boost::shared_ptr<roboKitMsg> msg)
{
    //Do nothing
    return;
}

bool LibDLWheelRobotCoreServer::robot_task_assign(WheelTaskTemplateStruct assignTask)
{
    QString errmsg;
    WheelTaskStruct task;

    task.task_uuid = QUuid::createUuid().toString().remove("{").remove("}").remove("-");
    task.task_name = assignTask.task_template_name;
    task.task_edit_uuid = assignTask.task_edit_uuid;
    task.task_template_uuid = assignTask.task_template_uuid;
    task.task_type_id = assignTask.task_type_id;
    task.task_status_id = WHEEL_ROBOT_TASK_STATUS_EXEC;
    task.task_start_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");

    bool bRet = WHEEL_ROBOT_DB.insertTaskDB(task, errmsg);


    if (!bRet)
    {
        ROS_ERROR(std::string(errmsg.toLocal8Bit()).c_str());
        return bRet;
    }

    QList<QString> devices;
    bRet = WHEEL_ROBOT_DB.getTaskDeviceDataDB(task.task_edit_uuid, devices);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_task_assign);

    Json::Value dev;

    for (int i = 0; i < devices.size(); i++)
    {
        dev[i] = devices[i].toStdString();
    }

    body.jsonAppendElement("task_uuid", std::string(task.task_uuid.toLocal8Bit()));
    body.jsonAppendElement("task_name", std::string(task.task_name.toLocal8Bit()));
    body.jsonAppendElement("task_edit_uuid", std::string(task.task_edit_uuid.toLocal8Bit()));
    body.jsonAppendElement("task_template_uuid", std::string(task.task_template_uuid.toLocal8Bit()));


    body.jsonAppendElement("priority", (int)WHEEL_ROBOT_TASK_1ST);
    body.jsonAppendElement("devices", dev);
    body.jsonAppendElement("dev_optimize", true);
    body.jsonAppendElement("task_end_action", (int)task.task_end_action);
    body.jsonAppendElement("breakTask", false);

	if (getCurrentRobotSocket())
	{
		getCurrentRobotSocket()->baseMsgPostMsg(body);
	}
    return bRet;    
}

void LibDLWheelRobotCoreServer::robot_task_query_task_list_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::robot_task_query_task_list_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    QString taskUUid = QString::fromStdString(val["task_uuid"].asString());
    QStringList taskQList;
    //Json::Value taskList = val["task_list"];
    for (int i = 0; i < val["task_list"].size(); i++)
    {
        QString data = QString::fromStdString(val["task_list"][i].asString());
        if (data != taskUUid)
        {
            taskQList.append(data);
        }
    }
    ROS_INFO("LibDLWheelRobotCoreServer::robot_task_query_task_list_resp %d",taskQList.size());
    WHEEL_ROBOT_DB.updateTaskDB(taskUUid, taskQList);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_update_task_table_signal_req);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::robot_task_device_finish_resp(boost::shared_ptr<roboKitMsg> msg)
{
    m_serverSocket->postMsg2AllConnection(msg->msgBody);
    ROS_INFO("LibDLWheelRobotCoreServer::robot_task_device_finish_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();
    
    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::robot_task_device_finish_resp retVal = NULL");
        return;
    }
    
    WheelRobotDeviceCollectData toDoData;
    
    toDoData.task_uuid = QString::fromStdString(val["task_uuid"].asString());
    toDoData.device_uuid = QString::fromStdString(val["device_uuid"].asString());
    toDoData.date_time = QString::fromStdString(val["date_time"].asString());
    toDoData.deviceCollectStatus = (WheelRobotDeviceCollectStatus)val["status"].asInt();
    toDoData.bTaskFinish = false;
    int iStatusError = val["status_error"].asInt();

	if (!val["api_code"].isNull()) {
		Json::Value root;
		root["api_code"] = val["7001"];
		root["distance_a"] = val["distance_a"];
		root["distance_b"] = val["distance_b"];
		root["AngleA"] = val["AngleA"];
		root["AngleB"] = val["AngleB"];
		root["AngleA3"] = val["AngleA3"];
		root["AngleB3"] = val["AngleB3"];
		root["distance_c"] = val["distance_c"];
		root["AngleA2"] = val["AngleA2"];
		root["AngleB2"] = val["AngleB2"];

		toDoData.other_info = QString::fromStdString(root.toStyledString());
	}

    if (toDoData.deviceCollectStatus == WHEEL_ROBOT_DEVICE_COLLECT_NORMAL)
    {
        signal_deviceUploadFinish(toDoData);
    }

    if (toDoData.deviceCollectStatus == WHEEL_ROBOT_DEVICE_COLLECT_ERROR)
    {
        WheelInspectResultStruct inspectRes;
        inspectRes.task_uuid = toDoData.task_uuid;
        inspectRes.device_uuid = toDoData.device_uuid;
        inspectRes.inspect_time = toDoData.date_time;
        inspectRes.inspect_result = "异常";
        inspectRes.inspect_status_id = WheelRobotInspectResultStatusType(toDoData.deviceCollectStatus);
        inspectRes.is_dealed = 0;
        inspectRes.alarm_level_id = DeviceAlarmLevel(iStatusError + 6);
        inspectRes.deal_info_uuid = "";
        WHEEL_ROBOT_DB.insertInspectResultDB(inspectRes);
        Remote_robot_inspect_result(inspectRes);
    }

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_device_finish + WHEELROBOT_PointADD);

	body.jsonAppendElement("task_uuid", toDoData.task_uuid.toStdString());
	body.jsonAppendElement("device_uuid", toDoData.device_uuid.toStdString());

	ROS_INFO("device_return:send success!");
	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->baseMsgPostMsg(body);
}

void LibDLWheelRobotCoreServer::robot_task_finish_resp(boost::shared_ptr<roboKitMsg> msg)
{
    m_serverSocket->postMsg2AllConnection(msg->msgBody);
    ROS_INFO("LibDLWheelRobotCoreServer::robot_task_finish_res==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::robot_task_finish_res retVal = NULL");
        return;
    }

    std::string task_uuid = val["task_uuid"].asString();
    std::string date_time = val["date_time"].asString();
    WheelRobotDeviceCollectStatus status = (WheelRobotDeviceCollectStatus)val["status"].asInt();

	if (!val["result_msg"].isNull()) {
		ROS_ERROR("==========================================%s", val["result_msg"].asString().c_str());
	}

    signal_taskCollectFinish(status);

    
    int ret_code;
    string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotCoreServer::robot_task_finish_res, task_uuid:%s, date_time:%ds, status:%d", task_uuid.c_str(), date_time.c_str(), (int)status);
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::robot_task_finish_res error : code:%d, msg:%s", ret_code, err_msg);
    }
}

void LibDLWheelRobotCoreServer::robot_task_all_finish_resp(boost::shared_ptr<roboKitMsg> msg)
{
    //m_serverSocket->postDirectMsg2AllConnection(msg);
    ROS_INFO("LibDLWheelRobotCoreServer::robot_task_all_finish_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::robot_task_all_finish_resp retVal = NULL");
        return;
    }

    std::string task_uuid = val["task_uuid"].asString();
    std::string date_time = val["date_time"].asString();
    WheelRobotTaskEndType status = (WheelRobotTaskEndType)val["status"].asInt();

    WheelRobotDeviceCollectData data;
    data.bTaskFinish = true;
    data.date_time = QString::fromStdString(date_time);
    data.task_uuid = QString::fromStdString(task_uuid);
    data.taskEndStatus = status;

    signal_deviceUploadFinish(data);

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_all_finish + WHEELROBOT_PointADD);

	body.jsonAppendElement("task_uuid", task_uuid);

	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->baseMsgPostMsg(body);
}

void LibDLWheelRobotCoreServer::robot_status_real_time_req()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_status_real_time);

	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->baseMsgPostMsg(body);
}

void LibDLWheelRobotCoreServer::robot_status_none_real_time_req()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_status_none_real_time);

	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->baseMsgPostMsg(body);
}

void LibDLWheelRobotCoreServer::robot_status_alarm_req()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_status_alarm);

	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->baseMsgPostMsg(body);
}

void LibDLWheelRobotCoreServer::robot_status_Direct_resp(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::robot_status_Direct_resp==Str:%s", msg->msgBody.getJsonString().c_str());
	//	不是本台机器人不做转发
	if (msg->msgBody.getJsonVal()["robot_id"].asInt() != m_current_robotID) {
		return;
	}

	Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::robot_status_real_time retVal = NULL");
        return;
    }

	jsonBody base_info_content;
	int battery_level = val["battery_level"].asFloat() * 100.0;
	int battery_life = battery_level / 12.5;
	float speed = val["vx"].asFloat();
	if (abs(val["vx"].asFloat() - 0.001) < 0.00001) {
		speed = 0.0;
	}
	
	base_info_content.jsonAppendElement("id", std::to_string((int)m_current_robotID));
	base_info_content.jsonAppendElement("battery", std::to_string(battery_level));
	base_info_content.jsonAppendElement("battery_life", std::to_string(battery_life));
	base_info_content.jsonAppendElement("vel", std::to_string(speed));

	jsonBody pos_content;
	pos_content.jsonAppendElement("id", std::to_string((int)m_current_robotID));
	pos_content.jsonAppendElement("x", std::to_string(val["x"].asFloat()));
	pos_content.jsonAppendElement("y", std::to_string(val["y"].asFloat()));
	pos_content.jsonAppendElement("z", std::to_string(val["angle"].asFloat()));
	pos_content.jsonAppendElement("orientation_x", std::to_string(val["qx"].asFloat()));
	pos_content.jsonAppendElement("orientation_y", std::to_string(val["qy"].asFloat()));
	pos_content.jsonAppendElement("orientation_z", std::to_string(val["qz"].asFloat()));
	pos_content.jsonAppendElement("orientation_w", std::to_string(val["qw"].asFloat()));
	pos_content.jsonAppendElement("time", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString());

	robotBodyInfoSignal(((int)m_current_robotID), base_info_content.getJsonString());
	robotPosSignal(((int)m_current_robotID), pos_content.getJsonString());
    m_serverSocket->postDirectMsg2AllConnection(msg);
}

void LibDLWheelRobotCoreServer::robot_config_Direct_resp(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::robot_config_Direct_resp==Str:%s", msg->msgBody.getJsonString().c_str());
	//	不是本台机器人不做转发
	if (msg->msgBody.getJsonVal()["robot_id"].asInt() != m_current_robotID) {
		return;
	}
    m_serverSocket->postDirectMsg2AllConnection(msg);
}


void LibDLWheelRobotCoreServer::robot_status_real_time(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::robot_status_real_time==Str:%s", msg->msgBody.getJsonString().c_str());
    robot_status_Direct_resp(msg);
}

void LibDLWheelRobotCoreServer::robot_status_none_real_time(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::robot_status_none_real_time==Str:%s", msg->msgBody.getJsonString().c_str());
    robot_status_Direct_resp(msg);
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::robot_status_none_real_time retVal = NULL");
        return;
    }
}

void LibDLWheelRobotCoreServer::robot_status_alarm_initiative(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::robot_config_Direct_resp==Str:%s", msg->msgBody.getJsonString().c_str());
	//	不是本台机器人不做转发
	if (msg->msgBody.getJsonVal()["robot_id"].asInt() != m_current_robotID) {
		return;
	}

    m_serverSocket->postDirectMsg2AllConnection(msg);
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::robot_status_alarm_initiative retVal = NULL");
        return;
    }

    int iDeivceCount = 0;
    int iAlarmCount = 0;
//    QList<int> errorList;
    Json::Value error_val = val["warnMsgs"];
    for (int i = 0; i < error_val.size(); i++)
    {
        int error_id = error_val[i]["errorId"].asInt();
        if (error_id >= 1101 && error_id <= 1406)
        {
            iDeivceCount++;
            iAlarmCount++;
        }
        if (error_id >= 2107 && error_id <= 2202)
        {
            iAlarmCount++;
        }
        if (error_id >= 3101 && error_id <= 3303)
        {
            iAlarmCount++;
        }
    }
}

void LibDLWheelRobotCoreServer::robot_config_time_sync()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_time_sync);

    body.jsonAppendElement("time", std::string(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toLocal8Bit()));

	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->baseMsgPostMsg(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_device_insert_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_insert_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_insert_req retVal = NULL");
        return;
    }
    WheelRobotInsertDeviceStruct dev;

    dev.device.device_uuid = QString::fromStdString(val["device_uuid"].asString());
    dev.device.voltage_level_id = QString::fromStdString(val["voltage_level_id"].asString());
    dev.device.equipment_interval_uuid = QString::fromStdString(val["equipment_interval_uuid"].asString());
    dev.device.device_area_uuid = QString::fromStdString(val["device_area_uuid"].asString());
    dev.device.device_type_uuid = QString::fromStdString(val["device_type_uuid"].asString());
    dev.device.sub_device_type_uuid = QString::fromStdString(val["sub_device_type_uuid"].asString());
    dev.device.device_point_type_uuid = QString::fromStdString(val["device_point_type_uuid"].asString());
    dev.device.unit_type_uuid = QString::fromStdString(val["unit_type_uuid"].asString());
    dev.device.recognition_type_id = (WheelRobotRecognizeType)val["recognition_type_id"].asInt();
    dev.device.save_type_id = (WheelRobotSaveType)val["save_type_id"].asInt();
    dev.device.meter_type_id = (WheelRobotMeterType)val["meter_type_id"].asInt();
    dev.device.fever_type_id = (WheelRobotFeverType)val["fever_type_id"].asInt();
    dev.device.threshold_filename = QString::fromStdString(val["threshold_filename"].asString());
    dev.device.device_phase_id = (WheelRobotPhaseType)val["device_phase_id"].asInt();
    dev.device.alarm_level_id = (DeviceAlarmLevel)val["alarm_level_id"].asInt();


    dev.paratmeter.device_uuid = QString::fromStdString(val["device_uuid"].asString());
    dev.paratmeter.point_id = val["point_id"].asInt();
    dev.paratmeter.ptz_pan = val["ptz_pan"].asInt();
    dev.paratmeter.ptz_tilt = val["ptz_tilt"].asInt();
    dev.paratmeter.hc_zoom_near = val["hc_zoom_near"].asInt();
    dev.paratmeter.hc_focus_near = val["hc_focus_near"].asInt();
    dev.paratmeter.hc_zoom_far = val["hc_zoom_far"].asInt();
    dev.paratmeter.hc_focus_far = val["hc_focus_far"].asInt();
    dev.paratmeter.mag_focus = val["mag_focus"].asInt();
    dev.paratmeter.video_length = val["video_length"].asInt();
    dev.paratmeter.audio_length = val["audio_length"].asInt();

    QString retMsg;
    bool bRet = WHEEL_ROBOT_DB.insertDevice(dev, retMsg);
    Remote_robot_device_insert_resp(dev.device.device_uuid.toStdString(), bRet, std::string(retMsg.toLocal8Bit()));
    int ret_code;
    string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_insert_req, device_uuid:%s", dev.device.device_uuid.toStdString().c_str());
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_insert_req error : code:%d, msg:%s", ret_code, err_msg);
    }
}

void LibDLWheelRobotCoreServer::Remote_robot_device_insert_resp(std::string device_uuid, bool bInserted, std::string insertMsg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_insert + WHEELROBOT_PointADD);

    body.jsonAppendElement("device_uuid", device_uuid);
    body.jsonAppendElement("bInserted", bInserted);
    body.jsonAppendElement("insertMsg", insertMsg);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_device_update_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_update_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_update_req retVal = NULL");
        return;
    }
    WheelRobortDeviceParameterStruct dev;

    dev.device_uuid = QString::fromStdString(val["device_uuid"].asString());
    dev.point_id = val["point_id"].asInt();
    dev.ptz_pan = val["ptz_pan"].asInt();
    dev.ptz_tilt = val["ptz_tilt"].asInt();
    dev.hc_zoom_near = val["hc_zoom_near"].asInt();
    dev.hc_focus_near = val["hc_focus_near"].asInt();
    dev.hc_zoom_far = val["hc_zoom_far"].asInt();
    dev.hc_focus_far = val["hc_focus_far"].asInt();
    dev.mag_focus = val["mag_focus"].asInt();
    dev.video_length = val["video_length"].asInt();
    dev.audio_length = val["audio_length"].asInt();


    QString retMsg;
    bool bRet = WHEEL_ROBOT_DB.updateDeviceParameter(dev, retMsg);
	QStringList nodeUUid;
	bRet = WHEEL_ROBOT_DB.selectDeviceNodeUUidForDeviceUUid(dev.device_uuid, nodeUUid);
	if (bRet)
	{
		jsonBody body;
		body.setMsgNumber(getMsgId());
		body.setMsgType((int)Request_robot_delete_devices_fromlist_req + WHEELROBOT_PointADD);
		Json::Value root;
		root.append(nodeUUid[0].toStdString());
		root.append(nodeUUid[1].toStdString());
		root.append(nodeUUid[2].toStdString());
		root.append(nodeUUid[3].toStdString());
		body.jsonAppendElement("bRet", bRet);
		body.jsonAppendElement("nodeUUidList", root);
		m_serverSocket->postMsg2AllConnection(body);
	}

    Remote_robot_device_update_resp(dev.device_uuid.toStdString(), bRet, std::string(retMsg.toLocal8Bit()));
    int ret_code;
    string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_update_req, device_uuid:%s", dev.device_uuid.toStdString().c_str());
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_update_req error : code:%d, msg:%s", ret_code, err_msg);
    }
}

void LibDLWheelRobotCoreServer::Remote_robot_device_update_resp(std::string device_uuid, bool bUpdated, std::string updateMsg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_update + WHEELROBOT_PointADD);

    body.jsonAppendElement("device_uuid", device_uuid);
    body.jsonAppendElement("bUpdated", bUpdated);
    body.jsonAppendElement("updateMsg", updateMsg);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_device_delete_single_device_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_delete_single_device_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_single_device_req retVal = NULL");
        return;
    }
    
    std::string device_uuid = val["device_uuid"].asString();
    QString retMsg;

    bool bRet = WHEEL_ROBOT_DB.deleteDeviceFromDeviceUUid(QString::fromStdString(device_uuid), retMsg);
    Remote_robot_device_delete_operation_resp(bRet, std::string(retMsg.toLocal8Bit()));
    
    int ret_code;
    string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_delete_single_device_req, device_uuid:%s", device_uuid.c_str());
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_single_device_req error : code:%d, msg:%s", ret_code, err_msg);
    }
}

void LibDLWheelRobotCoreServer::Remote_robot_device_delete_operation_resp(bool bDeleted, std::string deleteMsg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_delete + WHEELROBOT_PointADD);

    body.jsonAppendElement("bDeleted", bDeleted);
    body.jsonAppendElement("deleteMsg", deleteMsg);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_device_delete_device_type_deivce_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_delete_device_type_deivce_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_device_type_deivce_req retVal = NULL");
        return;
    }

    std::string interval_uuid = val["interval_uuid"].asString();
    std::string device_uuid = val["device_uuid"].asString();
    QString retMsg;

    bool bRet = WHEEL_ROBOT_DB.deleteDeviceTypeByIntervalAndDeviceType(QString::fromStdString(interval_uuid), QString::fromStdString(device_uuid), retMsg);
    Remote_robot_device_delete_operation_resp(bRet, std::string(retMsg.toLocal8Bit()));

    int ret_code;
    string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_delete_device_type_deivce_req, interval_uuid:%s, device_uuid:%s", interval_uuid.c_str(), device_uuid.c_str());
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_device_type_deivce_req error : code:%d, msg:%s", ret_code, err_msg);
    }
}

void LibDLWheelRobotCoreServer::Remote_robot_device_delete_interval_deivce_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_delete_interval_deivce_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_interval_deivce_req retVal = NULL");
        return;
    }

    std::string interval_uuid = val["interval_uuid"].asString();
    QString retMsg;

    bool bRet = WHEEL_ROBOT_DB.deleteIntervalByIntervalUuid(QString::fromStdString(interval_uuid), retMsg);
    Remote_robot_device_delete_operation_resp(bRet, std::string(retMsg.toLocal8Bit()));

    int ret_code;
    string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_delete_interval_deivce_req, interval_uuid:%s", interval_uuid.c_str());
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_interval_deivce_req error : code:%d, msg:%s", ret_code, err_msg);
    }
}

void LibDLWheelRobotCoreServer::Remote_robot_device_delete_voltage_deivce_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_delete_voltage_deivce_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_voltage_deivce_req retVal = NULL");
        return;
    }

    QString voltage_id = val["voltage_id"].asString().c_str();
    QString retMsg;

    bool bRet = WHEEL_ROBOT_DB.deleteVoltageDevicesByVoltageId(voltage_id, retMsg);
    Remote_robot_device_delete_operation_resp(bRet, std::string(retMsg.toLocal8Bit()));

    int ret_code;
    string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_device_delete_voltage_deivce_req, voltage_id:%d", voltage_id);
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_voltage_deivce_req error : code:%d, msg:%s", ret_code, err_msg);
    }
}

void LibDLWheelRobotCoreServer::Remote_robot_control_remote_upgrade_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::robot_control_remote_upgrade_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_device_delete_voltage_deivce_req retVal = NULL");
        return;
    }
    QString fileName = val["fileName"].asString().c_str();

    int icount = 0;
    while (icount++)
    {
        QFile _file;
        if (_file.exists(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/" + fileName))
        {
            break;
        }
        if (icount > 3)
        {
            boost::shared_ptr<roboKitMsg> msg(new roboKitMsg());
            jsonBody body;
            body.setMsgNumber(getMsgId());
            body.setMsgType(int(Request_robot_Control_Remote_Upgrade + WHEELROBOT_PointADD));

            body.jsonAppendElement("ret_code", 1);
            body.jsonAppendElement("err_msg", "文件上传失败！");
            msg->msgBody = body;
            Remote_robot_control_remote_upgrade_resp(msg);
            return;
        }
        Sleep(400);
    }
	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->postClientMsg(msg);
}

void LibDLWheelRobotCoreServer::Remote_robot_control_remote_upgrade_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_control_remote_upgrade_resp==Str:%s", msg->msgBody.getJsonString().c_str());
	//	不是本台机器人不做转发
	if (msg->msgBody.getJsonVal()["robot_id"].asInt() != m_current_robotID) {
		return;
	}
    m_serverSocket->postMsg2AllConnection(msg->msgBody);
}

void LibDLWheelRobotCoreServer::Remote_robot_task_edit_insert_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_task_edit_insert_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_task_edit_insert_req retVal = NULL");
        return;
    }
    WheelTaskEditStruct edit;

    edit.task_edit_uuid = QString::fromStdString(val["task_edit_uuid"].asString());
    edit.task_edit_name = QString::fromStdString(val["task_edit_name"].asString());
    edit.task_edit_date = QString::fromStdString(val["task_edit_date"].asString());
    edit.task_edit_type_id = (WheelTaskAdminType)val["task_edit_type_id"].asInt();

    QString retMsg;

    QList<QString> device_uuid;
    Json::Value::iterator devItr = val["device_uuid"].begin();
    for (; devItr != val["device_uuid"].end(); devItr++)
    {
        device_uuid.append(QString::fromStdString(devItr->asString()));
    }

    
	bool bRet = WHEEL_ROBOT_DB.insertTaskEditAndInsertTaskDevicesDB(edit, device_uuid, retMsg);

	Remote_robot_task_edit_insert_resp((int)edit.task_edit_type_id, bRet, std::string(retMsg.toLocal8Bit()));

}

void LibDLWheelRobotCoreServer::Remote_robot_task_edit_insert_resp(int task_edit_type_id, bool bInserted, std::string insertMsg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_task_edit_insert + WHEELROBOT_PointADD);

    body.jsonAppendElement("task_edit_type_id", task_edit_type_id);
    body.jsonAppendElement("bInserted", bInserted);
    body.jsonAppendElement("insertMsg", insertMsg);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_task_edit_update_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_task_edit_updata_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_task_edit_updata_req retVal = NULL");
        return;
    }
    WheelTaskEditStruct edit;

    edit.task_edit_uuid = QString::fromStdString(val["task_edit_uuid"].asString());
    edit.task_edit_name = QString::fromStdString(val["task_edit_name"].asString());
    edit.task_edit_date = QString::fromStdString(val["task_edit_date"].asString());
    edit.task_edit_type_id = (WheelTaskAdminType)val["task_edit_type_id"].asInt();

    QList<QString> device_uuid;
    Json::Value::iterator devItr = val["device_uuid"].begin();
    for (; devItr != val["device_uuid"].end(); devItr++)
    {
        device_uuid.append(QString::fromStdString(devItr->asString()));
    }

    QString retMsg;

	bool bRecrt = WHEEL_ROBOT_DB.updataTaskEditAndUpdataTaskDevicesDB(edit, device_uuid, retMsg);

	Remote_robot_task_edit_update_resp((int)edit.task_edit_type_id, bRecrt, std::string(retMsg.toLocal8Bit()));

}

void LibDLWheelRobotCoreServer::Remote_robot_task_edit_update_resp(int task_edit_type_id, bool bInserted, std::string deleteMsg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_task_edit_update + WHEELROBOT_PointADD);

    body.jsonAppendElement("task_edit_type_id", task_edit_type_id);
    body.jsonAppendElement("bInserted", bInserted);
    body.jsonAppendElement("insertMsg", deleteMsg);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_task_edit_delete_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_task_edit_delete_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_task_edit_delete_req retVal = NULL");
        return;
    }

    QString task_edit_uuid = QString::fromStdString(val["task_edit_uuid"].asString());
    int edit_task_type_id = val["edit_task_type_id"].asInt();

    QString retMsg;

    bool bRecrt = WHEEL_ROBOT_DB.deleteTaskEditDB(task_edit_uuid, retMsg);

    Remote_robot_task_edit_delete_resp(edit_task_type_id, bRecrt, std::string(retMsg.toLocal8Bit()));

}

void LibDLWheelRobotCoreServer::Remote_robot_task_edit_delete_resp(int task_edit_type_id, bool bInserted, std::string deleteMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_edit_delete + WHEELROBOT_PointADD);

	body.jsonAppendElement("task_edit_type_id", task_edit_type_id);
	body.jsonAppendElement("bInserted", bInserted);
	body.jsonAppendElement("insertMsg", deleteMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_db_task_delete_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_db_task_delete_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_db_task_delete_req = NULL");
        return;
    }

    QString retMsg;
    QString c_retMsg;
    QString task_delete_uuid;
    WheelTaskChoose taskChoose;
    bool bRet;
    for (int i = 0; i < val["task_choose"].size(); i++)
    {
        task_delete_uuid = QString::fromStdString(val["task_choose"][i]["task_uuid"].asString());
        taskChoose = (WheelTaskChoose)val["task_choose"][i]["taskChoose"].asInt();
        if (taskChoose == Task_Choo)
        {
            bRet = WHEEL_ROBOT_DB.deleteTaskDB(task_delete_uuid, retMsg);
            c_retMsg = c_retMsg + retMsg;
            if (!bRet)
                break;
        }
        else if (taskChoose == Task_Template_Choo)
        {
            bRet = WHEEL_ROBOT_DB.deleteTaskTemplateDB(task_delete_uuid, retMsg);
            c_retMsg = c_retMsg + retMsg;
            if (!bRet)
                break;
        }
    }

    Remote_robot_db_task_delete_resp(task_delete_uuid.toStdString(), bRet, std::string((c_retMsg).toLocal8Bit()));

    signal_updateTimedTask();
}

void LibDLWheelRobotCoreServer::Remote_robot_db_task_delete_resp(std::string task_delete_uuid, bool bInserted, std::string insertMsg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_db_task_delete + WHEELROBOT_PointADD);

    body.jsonAppendElement("task_edit_uuid", task_delete_uuid);
    body.jsonAppendElement("bInserted", bInserted);
    body.jsonAppendElement("insertMsg", insertMsg);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_partrol_result_verify_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_partrol_result_verify_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_partrol_result_verify_req = NULL");
        return;
    }
    QString retMsg;
    QString rectMsg;

    bool bRet;
    int dealed_choo = val["is_dealed"].asInt();

	WheelPartolResult resultVerify;
	resultVerify.task_uuid = QString::fromStdString(val["task_uuid"].asString());
	resultVerify.device_uuid = QString::fromStdString(val["device_uuid"].asString());
	resultVerify.inspect_status_id=val["inspect_status_id"].asInt();
	resultVerify.is_dealed=val["is_dealed"].asInt();
	resultVerify.alarm_level_id=val["alarm_level_id"].asInt();
	resultVerify.deal_info_uuid=QString::fromStdString(val["deal_info_uuid"].asString());
	resultVerify.dealed_info = QString::fromStdString(val["dealed_info"].asString());
	resultVerify.dealed_result = QString::fromStdString(val["dealed_result"].asString());
	resultVerify.dealed_status_id = val["dealed_status_id"].asInt();

	int choose = val["choose"].asInt();

    if (dealed_choo == 0)
    {
		bRet = WHEEL_ROBOT_DB.updataInspectResultAndInsertDealInfoDB(resultVerify, rectMsg);
		if (WHEEL_ROBOT_DB.isAuditFinishDB(resultVerify.task_uuid))
		{
			WHEEL_ROBOT_DB.updataTaskForAuditIdDB(resultVerify.task_uuid);
		}
    }
    else if (dealed_choo == 1)
    {
		bRet = WHEEL_ROBOT_DB.updataInspectResultAndUpdataDealInfoDB(resultVerify, rectMsg);
    }

    Remote_robot_partrol_result_verify_resp(choose, bRet, std::string((retMsg + rectMsg).toLocal8Bit()));

    int ret_code;
    string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();

    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_partrol_result_verify_req, device_uuid:%s", resultVerify.device_uuid.toStdString().c_str());
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_partrol_result_verify_req error : code:%d, msg:%s", ret_code, err_msg);
    }
}

void LibDLWheelRobotCoreServer::Remote_robot_partrol_result_verify_resp(int choose, bool bInserted, std::string insertMsg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_partrol_result_verify + WHEELROBOT_PointADD);

	body.jsonAppendElement("choose", choose);
    body.jsonAppendElement("bInserted", bInserted);
    body.jsonAppendElement("insertMsg", insertMsg);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_task_template_insert_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_task_template_insert_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_task_template_insert_req = NULL");
		return;
	}
	QString retMsg;
	bool bRet;

	WheelTaskTemplateStruct temp;
	temp.task_template_uuid = QString::fromStdString(val["task_template_uuid"].asString());
	temp.task_edit_uuid = QString::fromStdString(val["task_edit_uuid"].asString());
	temp.task_type_id = (WheelRobotTaskType)val["task_type_id"].asInt();
	temp.task_end_action_id = (WheelRobotTaskEndActionType)val["task_end_action_id"].asInt();
	temp.task_template_name = QString::fromStdString(val["task_template_name"].asString());
	temp.task_start_date = QString::fromStdString(val["task_start_date"].asString());
	temp.task_repeat_duration = val["task_repeat_duration"].asInt();
	temp.task_status_id = (WheelRobotTaskStatusType)val["task_status_id"].asInt();
	temp.task_loop_type_id = (WheelRobotTaskLoopType)val["task_loop_type_id"].asInt();
	temp.task_start_time = QTime::fromString(QString::fromStdString(val["task_start_time"].asString()),"hh:mm:ss");
	bRet = WHEEL_ROBOT_DB.insertTaskTemplateDB(temp, retMsg);

	Remote_robot_task_template_insert_resq(temp.task_template_uuid.toStdString(), bRet, std::string((retMsg).toLocal8Bit()));

    signal_updateTimedTask();
}

void LibDLWheelRobotCoreServer::Remote_robot_task_template_insert_resq(std::string task_template_uuid, bool bInserted, std::string insertMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_template_insert + WHEELROBOT_PointADD);

	body.jsonAppendElement("task_template_uuid", task_template_uuid);
	body.jsonAppendElement("bInserted", bInserted);
	body.jsonAppendElement("insertMsg", insertMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_config_insert_voltage_level_req(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    std::string voltage_level_name = val["voltage_level_name"].asString();
    QString errMsg;
    bool bRet = WHEEL_ROBOT_DB.insertVoltageLevelDB(QString::fromLocal8Bit(voltage_level_name.c_str()), errMsg);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_insert_voltage_level + WHEELROBOT_PointADD);

    body.jsonAppendElement("status", bRet);
    body.jsonAppendElement("errMsg", std::string(errMsg.toLocal8Bit()));

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_config_insert_area_req(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    WheelRobortDeviceAreaStruct dev_area;
    dev_area.device_area_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');

    dev_area.device_area_name = QString::fromLocal8Bit(val["device_area_name"].asString().c_str());

    QString errMsg;
    
    bool bRet = WHEEL_ROBOT_DB.insertDeviceAreaDB(dev_area, errMsg);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_insert_area + WHEELROBOT_PointADD);

    body.jsonAppendElement("status", bRet);
    body.jsonAppendElement("errMsg", std::string(errMsg.toLocal8Bit()));

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_config_insert_interval_req(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    WheelRobortEquipmentIntervalStruct interval;
    interval.equipment_interval_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
    interval.voltage_level_id = QString::fromLocal8Bit(val["voltage_level_id"].asString().c_str());
    interval.equipment_interval_name = QString::fromLocal8Bit(val["equipment_interval_name"].asString().c_str());

    QString errMsg;

    bool bRet = WHEEL_ROBOT_DB.insertEquipmentIntervalDB(interval, errMsg);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_insert_interval + WHEELROBOT_PointADD);

    body.jsonAppendElement("status", bRet);
    body.jsonAppendElement("errMsg", std::string(errMsg.toLocal8Bit()));

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_config_delete_voltage_level_req(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    std::string voltage_level_id = val["voltage_level_id"].asString();
    QString errMsg;
    bool bRet = WHEEL_ROBOT_DB.deleteVoltageLevelDB(QString::fromLocal8Bit(voltage_level_id.c_str()), errMsg);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_delete_voltage_level + WHEELROBOT_PointADD);

    body.jsonAppendElement("status", bRet);
    body.jsonAppendElement("errMsg", std::string(errMsg.toLocal8Bit()));

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_config_delete_area_req(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    std::string device_area_uuid = val["device_area_uuid"].asString();
    QString errMsg;

    bool bRet = WHEEL_ROBOT_DB.deleteDeviceAreaDB(QString::fromLocal8Bit(device_area_uuid.c_str()), errMsg);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_delete_area + WHEELROBOT_PointADD);

    body.jsonAppendElement("status", bRet);
    body.jsonAppendElement("errMsg", std::string(errMsg.toLocal8Bit()));

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_config_delete_interval_req(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    std::string equipment_interval_uuid = val["equipment_interval_uuid"].asString();
    QString errMsg;

    bool bRet = WHEEL_ROBOT_DB.deleteEquipmentIntervalDB(QString::fromLocal8Bit(equipment_interval_uuid.c_str()), errMsg);

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_delete_interval + WHEELROBOT_PointADD);

    body.jsonAppendElement("status", bRet);
    body.jsonAppendElement("errMsg", std::string(errMsg.toLocal8Bit()));

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_inspect_result(WheelInspectResultStruct res)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_inspect_result_res);

    body.jsonAppendElement("task_uuid", std::string(res.task_uuid.toLocal8Bit()));
    body.jsonAppendElement("device_uuid", std::string(res.device_uuid.toLocal8Bit()));
    body.jsonAppendElement("inspect_time", std::string(res.inspect_time.toLocal8Bit()));
    body.jsonAppendElement("inspect_result", std::string(res.inspect_result.toLocal8Bit()));
    body.jsonAppendElement("inspect_status_id", (int)res.inspect_status_id);
    body.jsonAppendElement("is_dealed", res.is_dealed);
    body.jsonAppendElement("alarm_level_id", (int)res.alarm_level_id);
	body.jsonAppendElement("deal_info_uuid", std::string(res.deal_info_uuid.toLocal8Bit()));
    body.jsonAppendElement("virtual_name", std::string(res.virtual_name.toLocal8Bit()));

    body.jsonAppendElement("is_hangout", res.isHangOut);
    body.jsonAppendElement("alarm_count", res.alarm_count);

    m_serverSocket->postMsg2AllConnection(body);

    bjRobotDeviceResultSendSignal(res.task_uuid, res.device_uuid);
}

void LibDLWheelRobotCoreServer::Remote_robot_compare_inspect_result(CompareDeviceInspectResult data)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_send_compare_device_inspect_req);

	body.jsonAppendElement("task_uuid", data.task_uuid.toStdString());
	body.jsonAppendElement("device_uuid", data.device_uuid.toStdString());
	body.jsonAppendElement("inspect_time", data.inspect_time.toStdString());
	body.jsonAppendElement("device_point_name", data.device_point_name.toStdString());
	body.jsonAppendElement("inspect_result", data.inspect_result.toStdString());
	body.jsonAppendElement("alarm_level", data.alarm_level.toStdString());

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_current_task_status(WheelRobotCurrentTaskInfoShow task)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_current_task_infomation);

    body.jsonAppendElement("task_uuid", std::string(task.task_uuid.toLocal8Bit()));
    body.jsonAppendElement("task_name", std::string(task.task_name.toLocal8Bit()));
    body.jsonAppendElement("predict_duration", task.predict_duration);
    body.jsonAppendElement("current_device_uuid", std::string(task.current_device_uuid.toLocal8Bit()));
    body.jsonAppendElement("current_device_name", std::string(task.current_device_name.toLocal8Bit()));
    body.jsonAppendElement("total_devices", task.total_devices);
    body.jsonAppendElement("checked_devices", task.checked_devices);
    body.jsonAppendElement("percent", task.percent);
    body.jsonAppendElement("alarmDeviceCount", task.alarmDeviceCount);
    body.jsonAppendElement("task_property", std::string(task.task_property.toLocal8Bit()));
    
	if (!task.task_uuid.isEmpty())
    {
        bjRobotTaskStatusDataSignal(task);
    }
    if (m_serverSocket != NULL)
    {
        m_serverSocket->postMsg2AllConnection(body);
    }

	if (task.task_uuid.isEmpty() || task.current_device_uuid.isEmpty() || 
		(g_last_task_status.task_uuid == task.task_uuid && g_last_task_status.checked_devices == task.checked_devices))
	{
		return;
	}
	jsonBody pos_content;
	std::string status;
	if (task.checked_devices == task.total_devices) {
		status = "Finished";
	}
	else if (task.checked_devices == 1 && task.total_devices > 1) {
		status = "Started";
	}
	else if (task.total_devices > 1 && task.checked_devices < task.total_devices) {
		status = "GoalReached";
	}
	ROS_ERROR("showTask.total_devices:%d   showTask.checked_devices:%d", task.total_devices, task.checked_devices);

	pos_content.jsonAppendElement("id", std::to_string((int)m_current_robotID));
	pos_content.jsonAppendElement("status", status);
	pos_content.jsonAppendElement("dev_id", std::string(task.current_device_uuid.toLocal8Bit()));
	pos_content.jsonAppendElement("dev_name", std::string(task.current_device_name.toLocal8Bit()));
	pos_content.jsonAppendElement("time", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString());

	robotPatrolStatusSignal(((int)m_current_robotID), pos_content.getJsonString());
	g_last_task_status = task;
}

void LibDLWheelRobotCoreServer::Remote_robot_patrol_result_audit_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_patrol_result_audit_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_patrol_result_audit_req = NULL");
		return;
	}
	QString retMsg;
	bool bRet;

	QString task_uuid = QString::fromStdString(val["task_uuid"].asString());
	QString dealed_time = QString::fromStdString(val["dealed_time"].asString());
	QString dealed_user = QString::fromStdString(val["dealed_user"].asString());

	bRet = WHEEL_ROBOT_DB.updataDealInfoForUserDB(task_uuid, dealed_time, dealed_user, retMsg);

	Remote_robot_patrol_result_audit_resp(bRet, task_uuid, std::string((retMsg).toLocal8Bit()));

	int ret_code;
	string err_msg;

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		ROS_INFO("no error");
		return;
	}

	ret_code = val["ret_code"].asInt();
	err_msg = val["err_msg"].asString();

	if (0 == ret_code)
	{
		ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_patrol_result_audit_req, task_uuid:%s", task_uuid.toStdString().c_str());
		return;
	}
	else
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_patrol_result_audit_req error : code:%d, msg:%s", ret_code, err_msg);
	}
}

void LibDLWheelRobotCoreServer::Remote_robot_patrol_result_audit_resp(bool bInserted,QString task_uuid, std::string insertMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_patrol_result_audit + WHEELROBOT_PointADD);

	body.jsonAppendElement("bInserted", bInserted);
	body.jsonAppendElement("task_uuid", task_uuid.toStdString());
	body.jsonAppendElement("insertMsg", insertMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_excel_import_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_excel_import_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_excel_import_req = NULL");
		return;
	}

	QStringList c_data;
	for (int i = 0; i < val["excelData"].size(); i++)
	{
		c_data.append(QString::fromStdString(val["excelData"][i].asString()));
	}

	QString retMsg;
	bool bRet = false;
	QString uuid;
	pointInsertDB(uuid, c_data, bRet, retMsg);
		
	Remote_robot_excel_import_resp(bRet, std::string((retMsg).toLocal8Bit()));
	
	int ret_code;
	string err_msg;

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		ROS_INFO("no error");
		return;
	}

	ret_code = val["ret_code"].asInt();
	err_msg = val["err_msg"].asString();

	if (0 == ret_code)
	{
		ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_excel_import_req");
		return;
	}
	else
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_excel_import_req error : code:%d, msg:%s", ret_code, err_msg);
	}
}

void LibDLWheelRobotCoreServer::Remote_robot_excel_import_resp(bool bInserted, std::string insertMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_excel_import_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bInserted", bInserted);
	body.jsonAppendElement("insertMsg", insertMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_insert_standard_patrol_vindicate_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_insert_standard_patrol_vindicate_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_insert_standard_patrol_vindicate_req = NULL");
		return;
	}

	QStringList c_data;
	for (int i = 0; i < val["excelData"].size(); i++)
	{
		c_data.append(QString::fromStdString(val["excelData"][i].asString()));
	}
	QString uuid = QString::fromStdString(val["uuid"].asString());

	QString retMsg;
	bool bRet = false;

	pointInsertDB(uuid, c_data, bRet, retMsg);

	Remote_robot_insert_standard_patrol_vindicate_resp(bRet, std::string((retMsg).toLocal8Bit()));

	int ret_code;
	string err_msg;

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		ROS_INFO("no error");
		return;
	}

	ret_code = val["ret_code"].asInt();
	err_msg = val["err_msg"].asString();

	if (0 == ret_code)
	{
		ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_insert_standard_patrol_vindicate_req");
		return;
	}
	else
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_insert_standard_patrol_vindicate_req error : code:%d, msg:%s", ret_code, err_msg);
	}
}

void LibDLWheelRobotCoreServer::Remote_robot_insert_standard_patrol_vindicate_resp(bool bInserted, std::string insertMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_insert_standard_spot_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bInserted", bInserted);
	body.jsonAppendElement("insertMsg", insertMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_updata_standard_patrol_vindicate_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_standard_patrol_vindicate_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_standard_patrol_vindicate_req retVal = NULL");
		return;
	}
	WheelStandardPatrolVindicateStruct data;
	data.m_device_point_uuid = QString::fromStdString(val["device_point_uuid"].asString());
	data.m_device_type_name = QString::fromStdString(val["device_type_name"].asString());
	data.m_sub_device_name = QString::fromStdString(val["sub_device_name"].asString());
	data.m_device_point_name = QString::fromStdString(val["device_point_name"].asString());
	data.m_recognition_type_name = QString::fromStdString(val["recognition_type_name"].asString());
	data.m_meter_type_name = QString::fromStdString(val["meter_type_name"].asString());
	data.m_fever_type_name = QString::fromStdString(val["fever_type_name"].asString());
	data.m_save_type_name = QString::fromStdString(val["save_type_name"].asString());

	WheelStandardPatrolVindicateStruct c_data;
	bool bRet = WHEEL_ROBOT_DB.getWheelStandardForPointUUidDB(c_data, data.m_device_point_uuid);

	//设备类型判断
	QString deviceTypeName = data.m_device_type_name;
	QString deviceTypeUUid = WHEEL_DEVICE_CONFIG.getWheelDeviceTypeUUidQString(deviceTypeName);
	if (data.m_device_type_name == c_data.m_device_type_name)
	{
	}
	else
	{
		if (deviceTypeUUid.isEmpty())
		{
			deviceTypeUUid = WHEEL_DEVICE_CONFIG.getUUid();
			WHEEL_ROBOT_DB.insertDeviceType(deviceTypeUUid, deviceTypeName);
			WHEEL_DEVICE_CONFIG.loadWheelDeviceTypeData();
		}
		else
		{
		}
	}

	//小设备类型判断
	int iNull;
	QString subDeviceTypeName = WHEEL_DEVICE_CONFIG.dealNameLetter(data.m_sub_device_name,iNull);
	QString subDeviceTypeUUid = WHEEL_DEVICE_CONFIG.getWheelSubDeviceTypeUUidQString(subDeviceTypeName);
	if (data.m_sub_device_name == c_data.m_sub_device_name)
	{
		WHEEL_ROBOT_DB.updataSubDeviceType(subDeviceTypeUUid, deviceTypeUUid);
	}
	else
	{
		if (subDeviceTypeUUid.isEmpty())
		{
			subDeviceTypeUUid = WHEEL_DEVICE_CONFIG.getUUid();
			WHEEL_ROBOT_DB.insertSubDeviceType(subDeviceTypeUUid, deviceTypeUUid, subDeviceTypeName);
			WHEEL_DEVICE_CONFIG.loadWheelSubDeviceTypeData();
		}
		else
		{
			WHEEL_ROBOT_DB.updataSubDeviceType(subDeviceTypeUUid, deviceTypeUUid);
		}
	}

	//识别类型
	QString recognitionTypeName = data.m_recognition_type_name;
	int recognitionTypeId = -2;
	if (!recognitionTypeName.isEmpty())
	{
		recognitionTypeId = WHEEL_DEVICE_CONFIG.getWheelRecognitionTypeIdInt(recognitionTypeName);
		if (recognitionTypeId == -1)
		{
			WHEEL_ROBOT_DB.insertRecognitionType(recognitionTypeName);
			WHEEL_DEVICE_CONFIG.loadWheelRecognitionTypeData();
			recognitionTypeId = WHEEL_DEVICE_CONFIG.getWheelRecognitionTypeIdInt(recognitionTypeName);
		}
	}

	//表计类型
	QString meterTypeName = data.m_meter_type_name;
	int meterTypeId = -2;
	if (!meterTypeName.isEmpty())
	{
		meterTypeId = WHEEL_DEVICE_CONFIG.getWheelMeterTypeIdInt(meterTypeName);
		if (meterTypeId == -1)
		{
			WHEEL_ROBOT_DB.insertMeterType(meterTypeName);
			WHEEL_DEVICE_CONFIG.loadWheelMeterTypeData();
			meterTypeId = WHEEL_DEVICE_CONFIG.getWheelMeterTypeIdInt(meterTypeName);
		}
	}

	//发热类型
	QString feverTypeName = data.m_fever_type_name;
	int feverTypeId = -2;
	if (!feverTypeName.isEmpty())
	{
		feverTypeId = WHEEL_DEVICE_CONFIG.getWheelFeverTypeIdInt(feverTypeName);
		if (feverTypeId == -1)
		{
			WHEEL_ROBOT_DB.insertFeverType(feverTypeName);
			WHEEL_DEVICE_CONFIG.loadWheelFeverTypeData();
			feverTypeId = WHEEL_DEVICE_CONFIG.getWheelFeverTypeIdInt(feverTypeName);
		}
	}

	//保存类型
	QString saveTypeName = data.m_save_type_name;
	int saveTypeId = -2;
	if (!saveTypeName.isEmpty())
	{
		saveTypeId = WHEEL_DEVICE_CONFIG.getWheelSaveTypeIdInt(saveTypeName);
		if (saveTypeId == -1)
		{
			WHEEL_ROBOT_DB.insertSaveType(saveTypeName);
			WHEEL_DEVICE_CONFIG.loadWheelSaveTypeData();
			saveTypeId = WHEEL_DEVICE_CONFIG.getWheelSaveTypeIdInt(saveTypeName);
		}
	}

	//点位名
	iNull;
	QString devicePointName = WHEEL_DEVICE_CONFIG.dealNameLetter(data.m_device_point_name,iNull);
	QString devicePointUUid = data.m_device_point_uuid;
	QString retMsg;
	QStringList insertData;
	insertData.append(devicePointUUid);
	insertData.append(subDeviceTypeUUid);
	insertData.append(devicePointName);
	insertData.append(QString("%1").arg(recognitionTypeId));
	insertData.append(QString("%1").arg(meterTypeId));
	insertData.append(QString("%1").arg(feverTypeId));
	insertData.append(QString("%1").arg(saveTypeId));

	bRet = WHEEL_ROBOT_DB.updataDevicePointName(insertData, retMsg);
	WHEEL_DEVICE_CONFIG.loadWheelDevicePointNameData();

	Remote_robot_updata_standard_patrol_vindicate_resp(bRet, std::string(retMsg.toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_updata_standard_patrol_vindicate_resp(bool bInserted, std::string insertMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_updata_standard_spot_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bInserted", bInserted);
	body.jsonAppendElement("insertMsg", insertMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_delete_standard_patrol_vindicate_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_standard_patrol_vindicate_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_standard_patrol_vindicate_req retVal = NULL");
		return;
	}
	QString Msg;
	QString uuid = QString::fromStdString(val["device_point_uuid"].asString());
	bool bRet = WHEEL_ROBOT_DB.deleteDeviceWithDevicePointUUid(uuid, Msg);
	Remote_robot_delete_standard_patrol_vindicate_resp(uuid.toStdString(), bRet, std::string(Msg.toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_delete_standard_patrol_vindicate_resp(std::string uuid, bool bInserted, std::string insertMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_standard_spot_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("uuid", uuid);
	body.jsonAppendElement("bInserted", bInserted);
	body.jsonAppendElement("insertMsg", insertMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_task_edit_import_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_task_edit_import_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_task_edit_import_req retVal = NULL");
		return;
	}
	QString Msg;
	QString c_task_edit_uuid_old = QString::fromStdString(val["task_edit_uuid_old"].asString());
	QString c_task_edit_name = QString::fromStdString(val["task_edit_name"].asString());
	QString c_task_edit_date = QString::fromStdString(val["task_edit_date"].asString());
	QString c_task_edit_uuid_new = WHEEL_DEVICE_CONFIG.getUUid();
	int c_task_edit_type_id = 15;

	WheelTaskEditStruct stru;
	stru.task_edit_uuid = c_task_edit_uuid_new;
	stru.task_edit_name = c_task_edit_name;
	stru.task_edit_date = c_task_edit_date;
	stru.task_edit_type_id = WheelTaskAdminType(c_task_edit_type_id);

	bool bRet = WHEEL_ROBOT_DB.importTaskEditDB(c_task_edit_uuid_old, stru, Msg);

	Remote_robot_task_edit_import_resp(bRet, std::string(Msg.toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_task_edit_import_resp(bool bImport, std::string importMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_edit_import_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bImport", bImport);
	body.jsonAppendElement("importMsg", importMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_user_config_add_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_user_config_add_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_user_config_add_req retVal = NULL");
		return;
	}

	WheelUserConfig data;
	QString Msg;
	data.user_uuid = QString::fromStdString(val["user_uuid"].asString());
	data.user_name = QString::fromStdString(val["user_name"].asString());
	data.user_role = (WheelUserType)val["user_role"].asInt();
	data.user_authority = val["user_authority"].asInt();
	data.user_password= QString::fromStdString(val["user_password"].asString());
	data.user_telephone = QString::fromStdString(val["user_telephone"].asString());

	bool bRet = WHEEL_ROBOT_DB.addUserConfigDB(data, Msg);

	Remote_robot_user_config_add_resp(bRet, std::string(Msg.toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_user_config_add_resp(bool bAdd, std::string addMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_user_add_config_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bAdd", bAdd);
	body.jsonAppendElement("addMsg", addMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_user_config_delete_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_user_config_delete_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_user_config_delete_req retVal = NULL");
		return;
	}
	QString Msg;
	QString user_uuid = QString::fromStdString(val["user_uuid"].asString());

	bool bRet = WHEEL_ROBOT_DB.deleteUserConfigDB(user_uuid, Msg);

	Remote_robot_user_config_delete_resp(bRet, std::string(Msg.toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_user_config_delete_resp(bool bDelete, std::string deleteMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_user_delete_config_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bDelete", bDelete);
	body.jsonAppendElement("deleteMsg", deleteMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_delete_patrol_point_set_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_delete_patrol_point_set_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_delete_patrol_point_set_req = NULL");
		return;
	}

	QStringList c_data;
	for (int i = 0; i < val["device_uuid"].size(); i++)
	{
		c_data.append(QString::fromStdString(val["device_uuid"][i].asString()));
	}
	bool bRet = WHEEL_ROBOT_DB.deleteDeviceFromDeviceList(c_data);
	QString retMsg;
	Remote_robot_delete_patrol_point_set_resp(bRet, std::string((retMsg).toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_delete_patrol_point_set_resp(bool bRet, std::string retMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_patrol_point_set_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", retMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_start_using_status_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_start_using_status_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_start_using_status_req = NULL");
		return;
	}

	QStringList c_data;
	for (int i = 0; i < val["device_uuid"].size(); i++)
	{
		c_data.append(QString::fromStdString(val["device_uuid"][i].asString()));
	}
	int start_using = val["start_using"].asInt();
	QString retMsg;
	bool bRet = WHEEL_ROBOT_DB.updataDeviceStartUsingDB(c_data, start_using, retMsg);
	Remote_robot_start_using_status_resp(bRet, std::string((retMsg).toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_start_using_status_resp(bool bRet, std::string retMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_start_using_status_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", retMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_patrol_point_add_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_patrol_point_add_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_patrol_point_add_req = NULL");
		return;
	}
	WheelPatrolPointSet data;
	data.device_uuid = QString::fromStdString(val["device_uuid"].asString());
	data.station_name = QString::fromStdString(val["station_name"].asString());
	data.voltahe_level_name = QString::fromStdString(val["voltahe_level_name"].asString());
	data.equipment_interval_name = QString::fromStdString(val["equipment_interval_name"].asString());
	data.device_area_name = QString::fromStdString(val["device_area_name"].asString());
	data.device_point_type_name = QString::fromStdString(val["device_point_type_name"].asString());

	QString voltaheLevelName = data.voltahe_level_name;
	QString voltaheLevelUUid = WHEEL_DEVICE_CONFIG.getWheelVoltageLevelIdInt(voltaheLevelName);
	if (voltaheLevelUUid.isEmpty())
	{
		voltaheLevelUUid = WHEEL_DEVICE_CONFIG.getUUid();
		WHEEL_ROBOT_DB.insertVoltageLevel(voltaheLevelUUid, voltaheLevelName);
		WHEEL_DEVICE_CONFIG.loadWheelVoltageLevelData();
	}
	else
	{
	}
	
	QString equipmentIntervalName = data.equipment_interval_name;
	QString equipmentIntervalUUid = WHEEL_DEVICE_CONFIG.getWheelEquipmentIntervalQString(equipmentIntervalName);
	if (equipmentIntervalUUid.isEmpty())
	{
		equipmentIntervalUUid = WHEEL_DEVICE_CONFIG.getUUid();
		WHEEL_ROBOT_DB.insertEquipmentInterval(equipmentIntervalUUid, voltaheLevelUUid, equipmentIntervalName);
		WHEEL_DEVICE_CONFIG.loadWheelEquipmentIntervalData();
	}
	else
	{
	}

	QString deviceAreaName = data.device_area_name;
	QString deviceAreaUUid = WHEEL_DEVICE_CONFIG.getWheelDeviceAreaUUidQString(deviceAreaName);
	if (deviceAreaUUid.isEmpty())
	{
		deviceAreaUUid = WHEEL_DEVICE_CONFIG.getUUid();
		WHEEL_ROBOT_DB.insertDeviceArea(deviceAreaUUid, deviceAreaName);
		WHEEL_DEVICE_CONFIG.loadWheelDeviceAreaData();
	}
	else
	{
	}

	//点位名
	int choPhase;
	QString devicePointName = WHEEL_DEVICE_CONFIG.dealNameLetter(data.device_point_type_name, choPhase);
	QString devicePointUUid = WHEEL_DEVICE_CONFIG.getWheelDevicePointTypeUUidQString(devicePointName);
	QString retMsg;
	QStringList insertData;
	
	insertData.append(data.device_uuid);
	insertData.append(devicePointUUid);
	insertData.append(voltaheLevelUUid);
	insertData.append(equipmentIntervalUUid);
	insertData.append(deviceAreaUUid);
	insertData.append(QString("%1").arg(choPhase));

	bool bRet = WHEEL_ROBOT_DB.insertPatrolPointForDevices(insertData, retMsg);
	Remote_robot_patrol_point_add_resp(bRet, std::string((retMsg).toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_patrol_point_add_resp(bool bRet, std::string retMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_patrol_point_add_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", retMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_patrol_point_updata_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_patrol_point_updata_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_patrol_point_updata_req = NULL");
		return;
	}
	WheelPatrolPointSet data;
	data.device_uuid = QString::fromStdString(val["device_uuid"].asString());
	data.station_name = QString::fromStdString(val["station_name"].asString());
	data.voltahe_level_name = QString::fromStdString(val["voltahe_level_name"].asString());
	data.equipment_interval_name = QString::fromStdString(val["equipment_interval_name"].asString());
	data.device_area_name = QString::fromStdString(val["device_area_name"].asString());
	data.device_point_type_name = QString::fromStdString(val["device_point_type_name"].asString());
	
	QString voltaheLevelName = data.voltahe_level_name;
	QString voltaheLevelUUid = WHEEL_DEVICE_CONFIG.getWheelVoltageLevelIdInt(voltaheLevelName);
	if (voltaheLevelUUid.isEmpty())
	{
		voltaheLevelUUid = WHEEL_DEVICE_CONFIG.getUUid();
		WHEEL_ROBOT_DB.insertVoltageLevel(voltaheLevelUUid, voltaheLevelName);
		WHEEL_DEVICE_CONFIG.loadWheelVoltageLevelData();
	}
	else
	{
	}

	QString equipmentIntervalName = data.equipment_interval_name;
	QString equipmentIntervalUUid = WHEEL_DEVICE_CONFIG.getWheelEquipmentIntervalQString(equipmentIntervalName);
	if (equipmentIntervalUUid.isEmpty())
	{
		equipmentIntervalUUid = WHEEL_DEVICE_CONFIG.getUUid();
		WHEEL_ROBOT_DB.insertEquipmentInterval(equipmentIntervalUUid, voltaheLevelUUid, equipmentIntervalName);
		WHEEL_DEVICE_CONFIG.loadWheelEquipmentIntervalData();
	}
	else
	{
	}

	QString deviceAreaName = data.device_area_name;
	QString deviceAreaUUid = WHEEL_DEVICE_CONFIG.getWheelDeviceAreaUUidQString(deviceAreaName);
	if (deviceAreaUUid.isEmpty())
	{
		deviceAreaUUid = WHEEL_DEVICE_CONFIG.getUUid();
		WHEEL_ROBOT_DB.insertDeviceArea(deviceAreaUUid, deviceAreaName);
		WHEEL_DEVICE_CONFIG.loadWheelDeviceAreaData();
	}
	else
	{
	}

	//点位名
	int choPhase;
	QString devicePointName = WHEEL_DEVICE_CONFIG.dealNameLetter(data.device_point_type_name, choPhase);
	QString devicePointUUid = WHEEL_DEVICE_CONFIG.getWheelDevicePointTypeUUidQString(devicePointName);
	QString retMsg;
	QStringList insertData;

	insertData.append(data.device_uuid);
	insertData.append(devicePointUUid);
	insertData.append(voltaheLevelUUid);
	insertData.append(equipmentIntervalUUid);
	insertData.append(deviceAreaUUid);
	insertData.append(QString("%1").arg(choPhase));

	bool bRet = WHEEL_ROBOT_DB.updatePatrolPointForDevices(insertData, retMsg);
	Remote_robot_patrol_point_updata_resp(bRet, std::string((retMsg).toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_patrol_point_updata_resp(bool bRet, std::string retMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_patrol_point_updata_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", retMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_update_task_status_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_update_task_status_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_update_task_status_req = NULL");
		return;
	}

	QString task_uuid = QString::fromStdString(val["task_uuid"].asString());
	int task_status = val["status"].asInt();

    QString retMsg;
    bool bRet = false;

    switch (WheelRobotTaskStatusType(task_status))
    {
//     case WHEEL_ROBOT_TASK_STATUS_WAIT:
//         bRet = WHEEL_ROBOT_DB.updateTaskStatusDB(task_uuid, task_status, retMsg);
//         break;
    case WHEEL_ROBOT_TASK_STATUS_FINISH:
        bRet = WHEEL_ROBOT_DB.updateTaskStatusDB(task_uuid, task_status, retMsg);
        robot_task_delete_req(task_uuid);
        break;
//     case WHEEL_ROBOT_TASK_STATUS_EXEC:
//         bRet = WHEEL_ROBOT_DB.updateTaskStatusDB(task_uuid, task_status, retMsg);
//         break;
    case WHEEL_ROBOT_TASK_STATUS_ABORT:
        bRet = WHEEL_ROBOT_DB.updateTaskStatusDB(task_uuid, task_status, retMsg);
        robot_task_delete_req(task_uuid);
        break;
//     case WHEEL_ROBOT_TASK_STATUS_OVERTIME:
//         bRet = WHEEL_ROBOT_DB.updateTaskStatusDB(task_uuid, task_status, retMsg);
//         break;
    case WHEEL_ROBOT_TASK_STATUS_HANGUP:
        bRet = WHEEL_ROBOT_DB.updateTaskTemplateDB(task_uuid, int(WHEEL_ROBOT_TASK_STATUS_HANGUP), retMsg);
        countTodyTaskCallback();
        break;
    case WHEEL_ROBOT_TASK_STATUS_HANGDOWN:
        bRet = WHEEL_ROBOT_DB.updateTaskTemplateDB(task_uuid, int(WHEEL_ROBOT_TASK_STATUS_WAIT), retMsg);
        countTodyTaskCallback();
        break;
    default:
        break;
    }

	Remote_robot_update_task_status_resp(bRet, std::string((retMsg).toLocal8Bit()));

}
void LibDLWheelRobotCoreServer::Remote_robot_update_task_status_resp(bool bRet, std::string retMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_update_task_status_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", retMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_insert_note_message_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_insert_note_message_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_insert_note_message_req = NULL");
		return;
	}

	QString user_receive_uuid = QString::fromStdString(val["user_receive_uuid"].asString());
	QString alarm_level_id = QString::fromStdString(val["alarm_level_id"].asString());
	QString send_time = QString::fromStdString(val["send_time"].asString());
	QString send_freq_id = QString::fromStdString(val["send_freq_id"].asString());
	QString fault_name_uuid = QString::fromStdString(val["fault_name_uuid"].asString());
	WheelDeviceTreePath type = (WheelDeviceTreePath)val["device_tree_path"].asInt();

	WheelSubMsgInsert noteMsg;
	noteMsg.user_receive_uuid = user_receive_uuid;
	noteMsg.alarm_level_id = alarm_level_id;
	noteMsg.send_time = send_time;
	noteMsg.send_freq_id = send_freq_id;
	QString noteUUid = WHEEL_DEVICE_CONFIG.getUUid();
	bool bRet;
	QString retMsg;
	QStringList fault_uuid;
	switch (type)
	{
	case Company_And_Station:
		WHEEL_ROBOT_DB.getWheelAllDeviceUUidDB(fault_uuid);
		bRet = WHEEL_ROBOT_DB.insertNoteMessageDB(noteUUid, noteMsg, fault_uuid, retMsg);
        break;
	case Voltage_Level:
		WHEEL_ROBOT_DB.getDeviceUUidWithType(fault_name_uuid, fault_uuid, type);
		bRet = WHEEL_ROBOT_DB.insertNoteMessageDB(noteUUid, noteMsg, fault_uuid, retMsg);
        break;
	case Equipment_Interval:
		WHEEL_ROBOT_DB.getDeviceUUidWithType(fault_name_uuid, fault_uuid, type);
		bRet = WHEEL_ROBOT_DB.insertNoteMessageDB(noteUUid, noteMsg, fault_uuid, retMsg);
        break;
	case Device_Type:
		WHEEL_ROBOT_DB.getDeviceUUidWithType(fault_name_uuid, fault_uuid, type);
		bRet = WHEEL_ROBOT_DB.insertNoteMessageDB(noteUUid, noteMsg, fault_uuid, retMsg);
        break;
	case Device_Point:
		fault_uuid.append(fault_name_uuid);
		bRet = WHEEL_ROBOT_DB.insertNoteMessageDB(noteUUid, noteMsg, fault_uuid, retMsg); 
        break;
	case Robot_System_Msg:
		fault_uuid.append(fault_name_uuid);
		bRet = WHEEL_ROBOT_DB.insertNoteMessageDB(noteUUid, noteMsg, fault_uuid, retMsg);
        break;
	default:
		break;
	}

	Remote_robot_insert_note_message_resp(bRet, std::string((retMsg).toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_insert_note_message_resp(bool bRet, std::string retMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_insert_note_message_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", retMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_delete_note_message_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_delete_note_message_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_delete_note_message_req = NULL");
		return;
	}

	QString noteUUid = QString::fromStdString(val["noteUUid"].asString());
	QString fault_name_uuid = QString::fromStdString(val["fault_name_uuid"].asString());

	QString retMsg;
	bool bRet = WHEEL_ROBOT_DB.deleteNoteMessageDB(noteUUid, fault_name_uuid, retMsg);

	Remote_robot_delete_note_message_resp(bRet, std::string((retMsg).toLocal8Bit()));
}

void LibDLWheelRobotCoreServer::Remote_robot_delete_note_message_resp(bool bRet, std::string retMsg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_note_message_req + WHEELROBOT_PointADD);

	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", retMsg);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_connect_2_new_robot_req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_connect_2_new_robot_req==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_connect_2_new_robot_req = NULL");
        return;
    }

    QString robot_name = QString::fromLocal8Bit(val["robot_name"].asString().c_str());

    bool bRet = WHEEL_ROBOT_CORE_CONFIG.changeCurrentRobot(robot_name);
    if (bRet)
    {
        reconnectToRobot();
    }
    
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_reconnect_2_new_robot_req + WHEELROBOT_PointADD);

    body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("robotIp", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().robotIp.toLocal8Bit()));
	body.jsonAppendElement("robotName", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().robotName.toLocal8Bit()));
	body.jsonAppendElement("hcUserName", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcUserName.toLocal8Bit()));
	body.jsonAppendElement("hcPassword", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcPassword.toLocal8Bit()));

	body.jsonAppendElement("hcIP", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcIP.toLocal8Bit()));
	body.jsonAppendElement("hcCtrlPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcCtrlPort);
	body.jsonAppendElement("hcRtspPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcRtspPort);

	body.jsonAppendElement("hcIPFront", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcIPFront.toLocal8Bit()));
	body.jsonAppendElement("hcFrontCtrlPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcFrontCtrlPort);
	body.jsonAppendElement("hcFrontRtspPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcFrontRtspPort);

	body.jsonAppendElement("hcIPBack", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcIPBack.toLocal8Bit()));
	body.jsonAppendElement("hcBackCtrlPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcBackCtrlPort);
	body.jsonAppendElement("hcBackRtspPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcBackRtspPort);

	body.jsonAppendElement("infraredNeed", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().infraredNeed);
	body.jsonAppendElement("infraredCameraIp", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().infraredCameraIp.toLocal8Bit()));
	body.jsonAppendElement("infraredCtrlPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().infraredCtrlPort);
	body.jsonAppendElement("infraredRtspPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().infraredRtspPort);
	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_auto_relevance_device_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_auto_relevance_device_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

// 	if (val.isNull())
// 	{
// 		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_auto_relevance_device_req = NULL");
// 		return;
// 	}
	WHEEL_VIRTUAL_DEVICE.autoRelevanceDevice();

	Remote_robot_auto_relevance_device_resp(true);
}

void LibDLWheelRobotCoreServer::Remote_robot_auto_relevance_device_resp(bool bRet)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_auto_relevance_device_req + WHEELROBOT_PointADD);

	QString retMsg = "三项设备关联成功！";
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", retMsg.toStdString());

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_disconnect(bool isConnect)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_disconnect_req);
    body.jsonAppendElement("isConnect", isConnect);

    m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_fast_audit_task_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::romote_robot_fast_audit_task_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::romote_robot_fast_audit_task_req = NULL");
		return;
	}

	QString taskUUid = QString::fromStdString(val["taskUUid"].asString());

	QString retMsg;
	bool bRet = WHEEL_ROBOT_DB.updateFastAuditTaskDB(taskUUid);

	Romote_robot_fast_audit_task_resp(bRet);
}

void LibDLWheelRobotCoreServer::Romote_robot_fast_audit_task_resp(bool bRet)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_fast_audit_task_req + WHEELROBOT_PointADD);

	QString errMsg = "";
	if (!bRet)
	{
		errMsg = "一键审核任务失败！";
	}
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("retMsg", errMsg.toStdString());

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_control_infrared_take_photo_resp(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_control_infrared_take_photo_resp==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	//	不是本台机器人不做转发
	if (msg->msgBody.getJsonVal()["robot_id"].asInt() != m_current_robotID) {
		return;
	}
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_control_infrared_take_photo_resp = NULL");
		return;
	}

	QStringList lisPath = QString::fromStdString(val["name"].asString()).split("/");
	QString strPath = "";
	QString strName = "";
	for (int i = 0; i < lisPath.size(); i++)
	{
		if (i == lisPath.size() - 1)
		{
			strName = lisPath[i];
			break;
		}

		if (i == 0)
		{
			strPath = strPath + lisPath[i];
		}
		else
		{
			strPath = strPath + "/" + lisPath[i];
		}
	}
    int type = val["type"].asInt();

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_Control_Infrared_Take_Photo + WHEELROBOT_PointADD);

	body.jsonAppendElement("strPath", strPath.toStdString());
    body.jsonAppendElement("strName", strName.toStdString());
    body.jsonAppendElement("type", type);

    ROS_INFO("Romote_robot_control_infrared_take_photo_resp  return!=! ==Str:%s", body.getJsonString().c_str());
	m_serverSocket->postMsg2AllConnection(body);
}

///////////////
void LibDLWheelRobotCoreServer::Romote_robot_add_new_interval_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_add_new_interval_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_add_new_interval_req = NULL");
		return;
	}
	QString retMsg = "";
	WheelRobortEquipmentIntervalStruct data;
	data.equipment_interval_uuid = QUuid::createUuid().toString().remove("{").remove("}").remove("-");
	data.voltage_level_id = QString::fromStdString(val["voltageLevelUUid"].asString());
	data.equipment_interval_name = QString::fromStdString(val["newIntervalName"].asString());
	bool bRet = false;
	bRet = WHEEL_ROBOT_DB.insertEquipmentIntervalDB(data, retMsg);

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_add_new_interval_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(data.voltage_level_id.toStdString());
	root.append(data.equipment_interval_uuid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);
	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_update_interval_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_update_interval_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_update_interval_req = NULL");
		return;
	}
	WheelRobortEquipmentIntervalStruct data;
	data.voltage_level_id = QString::fromStdString(val["voltageLevelUUid"].asString());
	data.equipment_interval_uuid = QString::fromStdString(val["intervalUUid"].asString());
	data.equipment_interval_name = QString::fromStdString(val["newIntervalName"].asString());
	bool bRet = WHEEL_ROBOT_DB.updateEquipmentIntervalDB(data);

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_update_interval_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(data.voltage_level_id.toStdString());
	root.append(data.equipment_interval_uuid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);
	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_copy_paste_interval_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_copy_paste_interval_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_copy_paste_interval_req = NULL");
		return;
	}
	
	QString originalVoltageLevelUUid = QString::fromStdString(val["originalVoltageLevelUUid"].asString());
	QString intervalUUid = QString::fromStdString(val["intervalUUid"].asString());
	QString newVoltageLevelUUid = QString::fromStdString(val["newVoltageLevelUUid"].asString());
	QString newIntervalName = QString::fromStdString(val["newIntervalName"].asString());
	WheelRobortEquipmentIntervalStruct data;
	data.equipment_interval_uuid = QUuid::createUuid().toString().remove("{").remove("}").remove("-");
	data.equipment_interval_name = newIntervalName;
	data.voltage_level_id = newVoltageLevelUUid;
	bool bRet = WHEEL_ROBOT_DB.insertEquipmentIntervalDB(data);
	QList<QStringList> _deviceData;
	bRet = WHEEL_ROBOT_DB.selectDeviceDataWithIntervalUUid(_deviceData, intervalUUid, originalVoltageLevelUUid);
	bRet = WHEEL_ROBOT_DB.insertDeviceWithCopyInterval(_deviceData, newVoltageLevelUUid, data.equipment_interval_uuid);
	

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_copy_paste_interval_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(newVoltageLevelUUid.toStdString());
	root.append(data.equipment_interval_uuid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);
	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_delete_interval_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_delete_interval_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_delete_interval_req = NULL");
		return;
	}
	QString voltageLevelUUid = QString::fromStdString(val["voltageLevelUUid"].asString());
	QString intervalUUid = QString::fromStdString(val["intervalUUid"].asString());
	bool bRet = WHEEL_ROBOT_DB.deleteEquipmentIntervalDB(intervalUUid);

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_interval_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(voltageLevelUUid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_add_deviceType_andDevices_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_add_deviceType_andDevices_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_add_deviceType_andDevices_req = NULL");
		return;
	}
	QStringList deviceTypeUUidList;
	for (int i = 0; i < val["deviceTypeUUidList"].size(); i++)
	{
		deviceTypeUUidList.append(QString::fromStdString(val["deviceTypeUUidList"][i].asString()));
	}
	QString voltageLevelUUid = QString::fromStdString(val["voltageLevelUUid"].asString());
	QString intervalUUid = QString::fromStdString(val["intervalUUid"].asString());
	
	QList<QStringList> deviceData;
	bool bRet = WHEEL_ROBOT_DB.selectPointUUidAndDeviceTypeUUid(deviceData, deviceTypeUUidList, voltageLevelUUid);
	bRet = WHEEL_ROBOT_DB.insertDeviceWithCopyInterval(deviceData, voltageLevelUUid, intervalUUid);

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_add_deviceType_andDevices_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(voltageLevelUUid.toStdString());
	root.append(intervalUUid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_delete_device_type_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_delete_device_type_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_delete_device_type_req = NULL");
		return;
	}
	QString voltageLevelUUid = QString::fromStdString(val["voltageLevelUUid"].asString());
	QString intervalUUid = QString::fromStdString(val["intervalUUid"].asString());
	QString deviceTypeUUid = QString::fromStdString(val["deviceTypeUUid"].asString());
	bool bRet = WHEEL_ROBOT_DB.deleteDeviceTypeForDevices(voltageLevelUUid, intervalUUid, deviceTypeUUid);

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_device_type_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(voltageLevelUUid.toStdString());
	root.append(intervalUUid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_delete_device_type_list_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_delete_device_type_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_delete_device_type_req = NULL");
		return;
	}
	QString voltageLevelUUid = QString::fromStdString(val["voltageLevelUUid"].asString());
	QString intervalUUid = QString::fromStdString(val["intervalUUid"].asString());
	bool bRet = false;
	for (int i = 0; i < val["deviceTypeUUidList"].size(); i++)
	{
		bRet = WHEEL_ROBOT_DB.deleteDeviceTypeForDevices(voltageLevelUUid, intervalUUid, QString::fromStdString(val["deviceTypeUUidList"][i].asString()));
	}

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_device_type_list_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(voltageLevelUUid.toStdString());
	root.append(intervalUUid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_add_devices_fromlist_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_add_devices_fromlist_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_add_devices_fromlist_req = NULL");
		return;
	}
	QStringList _data;
	for (int i = 0; i < val["pointNameList"].size(); i++)
	{
		_data.append(QString::fromStdString(val["pointNameList"][i].asString()));
	}
	QString voltageLevelUUid = QString::fromStdString(val["voltageLevelUUid"].asString());
	QString intervalUUid = QString::fromStdString(val["intervalUUid"].asString());
	QString deviceTypeUUid = QString::fromStdString(val["deviceTypeUUid"].asString());

	QList<QStringList> deviceData;
	bool bRet = WHEEL_ROBOT_DB.selectPointAllDataWithPointList(deviceData, _data, voltageLevelUUid);
	bRet = WHEEL_ROBOT_DB.insertDeviceWithCopyInterval(deviceData, voltageLevelUUid, intervalUUid);

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_add_devices_fromlist_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(voltageLevelUUid.toStdString());
	root.append(intervalUUid.toStdString());
	root.append(deviceTypeUUid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Romote_robot_delete_devices_fromlist_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Romote_robot_delete_devices_fromlist_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Romote_robot_delete_devices_fromlist_req = NULL");
		return;
	}
	QStringList devicesUUidList;
	for (int i = 0; i < val["devicesUUidList"].size(); i++)
	{
		devicesUUidList.append(QString::fromStdString(val["devicesUUidList"][i].asString()));
	}
	QString voltageLevelUUid = QString::fromStdString(val["voltageLevelUUid"].asString());
	QString intervalUUid = QString::fromStdString(val["intervalUUid"].asString());
	QString deviceTypeUUid = QString::fromStdString(val["deviceTypeUUid"].asString());
	bool bRet = WHEEL_ROBOT_DB.deleteDeviceFromDeviceUUidList(devicesUUidList);

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_devices_fromlist_req + WHEELROBOT_PointADD);
	Json::Value root;
	root.append(voltageLevelUUid.toStdString());
	root.append(intervalUUid.toStdString());
	root.append(deviceTypeUUid.toStdString());
	body.jsonAppendElement("bRet", bRet);
	body.jsonAppendElement("nodeUUidList", root);

	m_serverSocket->postMsg2AllConnection(body);
}

void LibDLWheelRobotCoreServer::Remote_robot_switch_robot_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_switch_robot_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_switch_robot_req = NULL");
		return;
	}

	m_current_robotID = val["robot_id"].asInt();
	ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_switch_robot_req, str = #%d# robot", val["robot_id"].asInt());

}

//////////////
//.///////////////////////////////////////////////////////////////////////.//
void LibDLWheelRobotCoreServer::registerBack2CoreHandles()
{
	m_serverSocket->registerMsgHandle(Request_robot_switch_current_robot, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_switch_robot_req, this, _1)); // 不需处理

    //  ctrl Msg req
    m_serverSocket->registerMsgHandle(Request_robot_control_stop, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_gyrocal, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_reloc, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_comfirmloc, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_ptz_motion, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_control_ptz_abs, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_ptz_relative, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_ptz_msg, this, _1));
	//红外窗口右键中心点调节
	m_serverSocket->registerMsgHandle(Request_robot_control_ptz_monodrome_infra, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_ptz_infradred_msg, this, _1));

    m_serverSocket->registerMsgHandle(Request_robot_control_ptz_monodrome, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_ptz_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_motion, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_gotarget, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_translate, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_turn, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_slam, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_endslam, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_loadmap, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_loadmapobj, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_Control_SqlFile, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_device_ctrl, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_control_back_to_charge, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    // task Msg
    m_serverSocket->registerMsgHandle(Request_robot_task_pause, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_task_resume, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_task_cancel, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_task_assign, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_assign_msg, this, _1));

	m_serverSocket->registerMsgHandle(Request_robot_Control_Infrared_Take_Photo, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_Control_Infrared_Auto_Focus, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_Control_Infrared_Record_Video, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_Control_Infrared_Set_focus, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_Setting_Out_Run_Point, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_run_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_Control_Remote_Upgrade, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_control_remote_upgrade_req, this, _1));
	

    //m_serverSocket->registerMsgHandle(Request_robot_task_query_curr_task, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 不用查,core收到了之后主动推送
    //m_serverSocket->registerMsgHandle(Request_robot_task_query_task_list, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 不用查,core收到了之后主动推送
    //m_serverSocket->registerMsgHandle(Request_robot_task_device_finish, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 不需处理
    //m_serverSocket->registerMsgHandle(Request_robot_task_finish, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 不需处理
    //m_serverSocket->registerMsgHandle(Request_robot_task_all_finish, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 不需处理


    //..///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////..//
    m_serverSocket->registerMsgHandle(Request_robot_task_edit_insert, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_edit_insert_req, this, _1)); // 需处理 已测试
    m_serverSocket->registerMsgHandle(Request_robot_task_edit_update, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_edit_update_req, this, _1)); // 需处理 已测试
    m_serverSocket->registerMsgHandle(Request_robot_task_edit_delete, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_edit_delete_req, this, _1)); // 需处理
    //m_serverSocket->registerMsgHandle(Request_robot_result_batch_verify, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 需处理
    m_serverSocket->registerMsgHandle(Request_robot_partrol_result_verify, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_partrol_result_verify_req, this, _1)); // 需处理 待测试
    //m_serverSocket->registerMsgHandle(Request_robot_threshold_insert, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 需处理
    //m_serverSocket->registerMsgHandle(Request_robot_threshold_update, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 需处理

    m_serverSocket->registerMsgHandle(Request_robot_device_insert, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_device_insert_req, this, _1)); // 需处理
    m_serverSocket->registerMsgHandle(Request_robot_device_update, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_device_update_req, this, _1)); // 需处理
    m_serverSocket->registerMsgHandle(Request_robot_device_delete, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_device_delete_single_device_req, this, _1)); // 需处理
    m_serverSocket->registerMsgHandle(Request_robot_device_delete_device_type, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_device_delete_device_type_deivce_req, this, _1)); // 需处理
    m_serverSocket->registerMsgHandle(Request_robot_device_delete_interval, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_device_delete_interval_deivce_req, this, _1)); // 需处理
    m_serverSocket->registerMsgHandle(Request_robot_device_delete_voltage_level, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_device_delete_voltage_deivce_req, this, _1)); // 需处理

    m_serverSocket->registerMsgHandle(Request_robot_task_template_insert, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_template_insert_req, this, _1)); // 需处理
    //m_serverSocket->registerMsgHandle(Request_robot_task_template_update, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 需处理
    //m_serverSocket->registerMsgHandle(Request_robot_task_template_delete, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_msg, this, _1)); // 需处理

	m_serverSocket->registerMsgHandle(Request_robot_db_task_delete, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_db_task_delete_req, this, _1)); // 需处理 已测试

	m_serverSocket->registerMsgHandle(Request_robot_patrol_result_audit, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_patrol_result_audit_req, this, _1)); // 需处理 已测试
	//m_serverSocket->registerMsgHandle(Request_robot_task_edit_update, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_edit_delete_req, this, _1)); // 已测试
	m_serverSocket->registerMsgHandle(Request_robot_excel_import_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_excel_import_req, this, _1)); // 需处理 已测试
	m_serverSocket->registerMsgHandle(Request_robot_updata_standard_spot_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_updata_standard_patrol_vindicate_req, this, _1)); // 需处理 已测试
	m_serverSocket->registerMsgHandle(Request_robot_delete_standard_spot_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_delete_standard_patrol_vindicate_req, this, _1)); // 需处理 已测试
	m_serverSocket->registerMsgHandle(Request_robot_insert_standard_spot_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_insert_standard_patrol_vindicate_req, this, _1)); // 需处理 已测试

	m_serverSocket->registerMsgHandle(Request_robot_task_edit_import_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_task_edit_import_req, this, _1)); // 需处理 已测试

	m_serverSocket->registerMsgHandle(Request_robot_user_add_config_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_user_config_add_req, this, _1)); // 需处理 已测试
	m_serverSocket->registerMsgHandle(Request_robot_user_delete_config_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_user_config_delete_req, this, _1)); // 需处理 已测试

	m_serverSocket->registerMsgHandle(Request_robot_delete_patrol_point_set_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_delete_patrol_point_set_req, this, _1)); // 需处理 已测试

	m_serverSocket->registerMsgHandle(Request_robot_patrol_point_add_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_patrol_point_add_req, this, _1)); // 需处理 已测试
	m_serverSocket->registerMsgHandle(Request_robot_patrol_point_updata_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_patrol_point_updata_req, this, _1)); // 需处理 已测试

	m_serverSocket->registerMsgHandle(Request_robot_start_using_status_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_start_using_status_req, this, _1)); // 需处理 已测试

	m_serverSocket->registerMsgHandle(Request_robot_update_task_status_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_update_task_status_req, this, _1)); // 需处理 已测试
	
	m_serverSocket->registerMsgHandle(Request_robot_insert_note_message_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_insert_note_message_req, this, _1)); // 需处理 已测试
	m_serverSocket->registerMsgHandle(Request_robot_delete_note_message_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_delete_note_message_req, this, _1)); // 需处理 已测试

	m_serverSocket->registerMsgHandle(Request_robot_reconnect_2_new_robot_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_connect_2_new_robot_req, this, _1));

	m_serverSocket->registerMsgHandle(Request_robot_auto_relevance_device_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_auto_relevance_device_req, this, _1));

	m_serverSocket->registerMsgHandle(Request_robot_fast_audit_task_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_fast_audit_task_req, this, _1));

	/*--树--*/
	m_serverSocket->registerMsgHandle(Request_robot_add_new_interval_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_add_new_interval_req, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_update_interval_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_update_interval_req, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_copy_paste_interval_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_copy_paste_interval_req, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_delete_interval_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_delete_interval_req, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_add_deviceType_andDevices_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_add_deviceType_andDevices_req, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_delete_device_type_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_delete_device_type_req, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_delete_device_type_list_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_delete_device_type_list_req, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_add_devices_fromlist_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_add_devices_fromlist_req, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_delete_devices_fromlist_req, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_delete_devices_fromlist_req, this, _1));
	
	/*----*/
	//..///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////..//

    // cfg Msg
    m_serverSocket->registerMsgHandle(Request_robot_config_mode, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_switch_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_config_uploadmap2Robot, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_config_downloadmap, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_config_removemap, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    //m_serverSocket->registerMsgHandle(Request_robot_config_download2d, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1)); // 不需要客户端发送
    //m_serverSocket->registerMsgHandle(Request_robot_config_2d_map_query, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1)); // 不需下发至机器人
    m_serverSocket->registerMsgHandle(Request_robot_config_warning_oper, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_config_disconnect_oper, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_config_ptz_init, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_config_setparams_run, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_config_setparams_debug, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_config_md5, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));  //需处理
    m_serverSocket->registerMsgHandle(Request_robot_config_uploadSql, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));  //需处理
    m_serverSocket->registerMsgHandle(Request_robot_config_listing_area, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));  //需处理
	m_serverSocket->registerMsgHandle(Request_robot_config_update_embedded_software_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
	m_serverSocket->registerMsgHandle(Request_robot_config_urgency_stop_req, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));

    m_serverSocket->registerMsgHandle(Request_robot_insert_voltage_level, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_config_insert_voltage_level_req, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_insert_area, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_config_insert_area_req, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_insert_interval, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_config_insert_interval_req, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_delete_voltage_level, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_config_delete_voltage_level_req, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_delete_area, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_config_delete_area_req, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_delete_interval, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_config_delete_interval_req, this, _1));

    // core Msg
    m_serverSocket->registerMsgHandle(Request_robot_core_shutdown, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_core_reboot, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));
    m_serverSocket->registerMsgHandle(Request_robot_core_resetdsp, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_msg, this, _1));

}

void LibDLWheelRobotCoreServer::registerRobot2CoreHandels()
{
	for (auto itor = m_robotSocketMap.begin(); itor != m_robotSocketMap.end(); ++itor)
	{
		// task Msg
		itor->second->registerMsgHandle(Request_robot_task_query_curr_task + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_task_Direct, this, _1)); // 不用查,core收到了之后主动推送
		itor->second->registerMsgHandle(Request_robot_task_query_task_list, boost::bind(&LibDLWheelRobotCoreServer::robot_task_query_task_list_resp, this, _1)); // 不用查,core收到了之后主动推送
		itor->second->registerMsgHandle(Request_robot_task_device_finish, boost::bind(&LibDLWheelRobotCoreServer::robot_task_device_finish_resp, this, _1)); // 不需处理
		itor->second->registerMsgHandle(Request_robot_task_finish, boost::bind(&LibDLWheelRobotCoreServer::robot_task_finish_resp, this, _1)); // 需处理
		itor->second->registerMsgHandle(Request_robot_task_all_finish, boost::bind(&LibDLWheelRobotCoreServer::robot_task_all_finish_resp, this, _1)); // 需处理
		itor->second->registerMsgHandle(Request_robot_task_assign + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_task_Direct, this, _1)); // 不需处理
		itor->second->registerMsgHandle(Request_robot_task_begin, boost::bind(&LibDLWheelRobotCoreServer::robot_task_Begin, this, _1));
		itor->second->registerMsgHandle(Request_robot_task_current_device, boost::bind(&LibDLWheelRobotCoreServer::robot_task_Current_Task_Device, this, _1));
		itor->second->registerMsgHandle(Request_robot_task_point_serial_num, boost::bind(&LibDLWheelRobotCoreServer::robot_task_Direct, this, _1));
		itor->second->registerMsgHandle(Request_robot_task_already_use_time, boost::bind(&LibDLWheelRobotCoreServer::robot_task_Direct, this, _1)); // 不需处理
		itor->second->registerMsgHandle(Request_robot_task_point_percent, boost::bind(&LibDLWheelRobotCoreServer::robot_task_Direct, this, _1)); // 不需处理
		itor->second->registerMsgHandle(9999, boost::bind(&LibDLWheelRobotCoreServer::robot_heart_beat_msg, this, _1)); // 不需处理

		// status Msg
		itor->second->registerMsgHandle(Request_robot_env_sensor_info + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_env_sensor_info, this, _1));
		itor->second->registerMsgHandle(Request_robot_task_traj_record + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_task_traj_record, this, _1));

		itor->second->registerMsgHandle(Request_robot_status_real_time + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_status_real_time, this, _1));
		itor->second->registerMsgHandle(Request_robot_status_none_real_time + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_status_none_real_time, this, _1));
		itor->second->registerMsgHandle(Request_robot_status_alarm_initiative + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_status_alarm_initiative, this, _1));
		itor->second->registerMsgHandle(9999, boost::bind(&LibDLWheelRobotCoreServer::robot_heart_beat_msg, this, _1)); // 不需处理

		// cfg Msg
		itor->second->registerMsgHandle(Request_robot_config_mode + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_config_Direct_resp, this, _1));
		itor->second->registerMsgHandle(Request_robot_config_uploadmap2Robot + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_config_Direct_resp, this, _1));
		//itor->second->registerMsgHandle(Request_robot_config_downloadmap, boost::bind(&LibDLWheelRobotCoreServer::robot_config_Direct_resp, this, _1));
		//itor->second->registerMsgHandle(Request_robot_config_removemap + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_config_Direct_resp, this, _1));
		itor->second->registerMsgHandle(Request_robot_config_download2d, boost::bind(&LibDLWheelRobotCoreServer::robot_config_Direct_resp, this, _1));
		itor->second->registerMsgHandle(Request_robot_config_upload2d, boost::bind(&LibDLWheelRobotCoreServer::robot_config_Direct_resp, this, _1));
		itor->second->registerMsgHandle(Request_robot_config_listing_area + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::robot_config_Direct_resp, this, _1));
		itor->second->registerMsgHandle(9999, boost::bind(&LibDLWheelRobotCoreServer::robot_heart_beat_msg, this, _1)); // 不需处理
		itor->second->registerMsgHandle(Request_robot_Control_Infrared_Take_Photo + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::Romote_robot_control_infrared_take_photo_resp, this, _1));
		itor->second->registerMsgHandle(Request_robot_Control_Remote_Upgrade + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotCoreServer::Remote_robot_control_remote_upgrade_resp, this, _1));
	}
    
}

uint16_t LibDLWheelRobotCoreServer::getMsgId()
{
    boost::mutex::scoped_lock lock(getMsgIdMutex);
    return wheelRobotCoreServerMsgId++;
}

void LibDLWheelRobotCoreServer::pointInsertDB(QString uuid, QStringList c_data, bool &bRet, QString &retMsg)
{
	//设备类型
	if (c_data.size() == 0)
	{
		bRet = false;
		retMsg = QString("导入失败，数据不存在！");
		return;
	}
	QString deviceTypeName = c_data[0];
	QString deviceTypeUUid = WHEEL_DEVICE_CONFIG.getWheelDeviceTypeUUidQString(deviceTypeName);
	if (deviceTypeUUid.isEmpty())
	{
		deviceTypeUUid = WHEEL_DEVICE_CONFIG.getUUid();
		WHEEL_ROBOT_DB.insertDeviceType(deviceTypeUUid, deviceTypeName);
		WHEEL_DEVICE_CONFIG.loadWheelDeviceTypeData();
	}

	//小设备类型
	int iNull;
	QString subDeviceTypeName = WHEEL_DEVICE_CONFIG.dealNameLetter(c_data[1], iNull);
	QString subDeviceTypeUUid = WHEEL_DEVICE_CONFIG.getWheelSubDeviceTypeUUidQString(subDeviceTypeName);
	if (subDeviceTypeUUid.isEmpty())
	{
		subDeviceTypeUUid = WHEEL_DEVICE_CONFIG.getUUid();
		WHEEL_ROBOT_DB.insertSubDeviceType(subDeviceTypeUUid, deviceTypeUUid, subDeviceTypeName);
		WHEEL_DEVICE_CONFIG.loadWheelSubDeviceTypeData();
	}

	//识别类型
	QString recognitionTypeName = c_data[3];
	int recognitionTypeId = WHEEL_DEVICE_CONFIG.getWheelRecognitionTypeIdInt(recognitionTypeName);
	if (recognitionTypeId == -1)
	{
	//	WHEEL_ROBOT_DB.insertRecognitionType(recognitionTypeName);
	//	WHEEL_DEVICE_CONFIG.loadWheelRecognitionTypeData();
	//	recognitionTypeId = WHEEL_DEVICE_CONFIG.getWheelRecognitionTypeIdInt(recognitionTypeName);
	}

	//表计类型
	QString meterTypeName = c_data[4];
	int meterTypeId = -2;
	if (!meterTypeName.isEmpty())
	{
		meterTypeId = WHEEL_DEVICE_CONFIG.getWheelMeterTypeIdInt(meterTypeName);
		if (meterTypeId == -1)
		{
		//	WHEEL_ROBOT_DB.insertMeterType(meterTypeName);
		//	WHEEL_DEVICE_CONFIG.loadWheelMeterTypeData();
		//	meterTypeId = WHEEL_DEVICE_CONFIG.getWheelMeterTypeIdInt(meterTypeName);
		}
	}

	//发热类型
	QString feverTypeName = c_data[5];
	int feverTypeId = -2;
	if (!feverTypeName.isEmpty())
	{
		feverTypeId = WHEEL_DEVICE_CONFIG.getWheelFeverTypeIdInt(feverTypeName);
		if (feverTypeId == -1)
		{
		//	WHEEL_ROBOT_DB.insertFeverType(feverTypeName);
		//	WHEEL_DEVICE_CONFIG.loadWheelFeverTypeData();
		//	feverTypeId = WHEEL_DEVICE_CONFIG.getWheelFeverTypeIdInt(feverTypeName);
		}
	}

	//保存类型
	QString saveTypeName = c_data[6];
	int saveTypeId = -2;
	if (!saveTypeName.isEmpty())
	{
		saveTypeId = WHEEL_DEVICE_CONFIG.getWheelSaveTypeIdInt(saveTypeName);
		if (saveTypeId == -1)
		{
		//	WHEEL_ROBOT_DB.insertSaveType(saveTypeName);
		//	WHEEL_DEVICE_CONFIG.loadWheelSaveTypeData();
		//	saveTypeId = WHEEL_DEVICE_CONFIG.getWheelSaveTypeIdInt(saveTypeName);
		}
	}

	//点位名
	iNull;
	QString devicePointName = WHEEL_DEVICE_CONFIG.dealNameLetter(c_data[2],iNull);
	QString devicePointUUid = WHEEL_DEVICE_CONFIG.getWheelDevicePointTypeUUidQString(devicePointName);
	if (devicePointUUid.isEmpty())
	{
		if (uuid.isEmpty())
		{
			devicePointUUid = WHEEL_DEVICE_CONFIG.getUUid();
		}
		else
		{
			devicePointUUid = uuid;
		}

		QStringList insertData;
		insertData.append(devicePointUUid);
		insertData.append(subDeviceTypeUUid);
		insertData.append(devicePointName);
		insertData.append(QString("%1").arg(recognitionTypeId));
		insertData.append(QString("%1").arg(meterTypeId));
		insertData.append(QString("%1").arg(feverTypeId));
		insertData.append(QString("%1").arg(saveTypeId));

		bRet = WHEEL_ROBOT_DB.insertDevicePointName(insertData, retMsg);
		WHEEL_DEVICE_CONFIG.loadWheelDevicePointNameData();
	}
	else
	{
		retMsg = QString("点位:%1 已存在!").arg(devicePointName);
	}
}

void LibDLWheelRobotCoreServer::robotRealtimeStatusFunc()
{
    while (bStatusRealtimeRunning)
    {
        Sleep(100);
		if (getCurrentRobotSocket() != NULL)
		{
			if (getCurrentRobotSocket()->isConnected())
			{
				robot_status_real_time_req();
			}
		}
    }
}

void LibDLWheelRobotCoreServer::robotNoneRealtimeStatusFunc()
{
    while (bStatusRealtimeRunning)
    {
        Sleep(1000);
		if (getCurrentRobotSocket() != NULL)
		{
			if (getCurrentRobotSocket()->isConnected())
			{
				robot_status_none_real_time_req();
				robot_status_alarm_req();
			}
		}
    }
}

void LibDLWheelRobotCoreServer::robotheartbeatFunc()
{
    while (bheartbeatRunning)
    {
        Sleep(10* 1000);

        jsonBody body;
        body.setMsgNumber(getMsgId());
        body.setMsgType(9999);

		for (auto itor = m_robotSocketMap.begin(); itor != m_robotSocketMap.end(); ++itor)
		{
			if (itor->second->isConnected())
			{
				itor->second->baseMsgPostMsg(body);
				Remote_robot_disconnect(true);
			}
			else
			{
				Remote_robot_disconnect(false);
			}
		}
    }
}

void LibDLWheelRobotCoreServer::robot_task_delete_req(QString strUuid)
{
    ROS_INFO("LibDLWheelRobotCoreServer::robot_task_delete_req, strUuid:%s", strUuid.toStdString().c_str());
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_task_delete);

    body.jsonAppendElement("uuid", strUuid.toStdString());

	if (!getCurrentRobotSocket())
		return;
	getCurrentRobotSocket()->baseMsgPostMsg(body);
}

bool LibDLWheelRobotCoreServer::get_is_connect_robot()
{
	return true;
    //return m_robotSocket->get_is_connect_robot();
}

