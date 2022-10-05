#include "LibDLHangRailRobotCoreSocket.h"
#include <QDebug>
#include <fstream>

static uint16_t hangRailRobotCoreSocketMsgId = 0;

static boost::asio::io_service io_service;
static boost::asio::io_service::work io_work(io_service);
static bool bIOServiceRunning = false;

LibDLHangRailRobotBackground::LibDLHangRailRobotBackground()
{
    motorStatusString.push_back("");
    motorStatusString.push_back("输入电压过高");
    motorStatusString.push_back("输出电流过高");
    motorStatusString.push_back("输入电压过低");
    motorStatusString.push_back("现场总线故障");
    motorStatusString.push_back("虽然锁闭功能已启用，电机仍向错误方向旋转");
    motorStatusString.push_back("仅CANopen : 在发送节点保护要求时，NMT主机需要的时间过长");
    motorStatusString.push_back("电气故障或硬件损坏造成编码器故障");
    motorStatusString.push_back("编码器故障；自动设置时未找到标识脉冲");
    motorStatusString.push_back("AB通道出错");
    motorStatusString.push_back("超过正向限位开关和公差范围");
    motorStatusString.push_back("超过负向限位开关和公差范围");
    motorStatusString.push_back("设备温度超过80度");
    motorStatusString.push_back("对象6065h(Following Error Window) 和 对 象 6066h(Following Error Time Out) 的值被超过，因此触发错误。该错误必须通过对象3202h中的位7进行激活。");
    motorStatusString.push_back("非易失性存储器已满，需要重启控制器进行清理工作。");
    motorStatusString.push_back("电机堵转");
    motorStatusString.push_back("非易失性存储器损坏，需要重启控制器进行清理工作。");
    motorStatusString.push_back("从机发送PDO消息所需时间过长。");
}

LibDLHangRailRobotBackground::~LibDLHangRailRobotBackground()
{
    disconnect();
}

hangRobotUserLoginRetVal LibDLHangRailRobotBackground::dologin(std::string host, int port, std::string username, std::string password)
{
	boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string(host), port);
//	boost::shared_ptr<SocketClient> m_socket = boost::make_shared<SocketClient>(endpoint, io_service);
	ROS_ERROR("dologin:host:%s,port:%d,username:%s,password:%s",host,port,username,password);
	m_clientSocket = boost::make_shared<SocketClient>(endpoint, io_service);
	registerHandles();
    hangRobotUserLoginRetVal ret = m_clientSocket->doLogin(username, password);
	boost::thread *m_thread = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
	Sleep(1000);
    return ret;
}

void LibDLHangRailRobotBackground::registerHandles()
{
    // 机器人状态查询
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_CURRENT_STATUS), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_robot_status, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_ENVIRONMENT_STATUS), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_environment_status, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_ENVIRONMENT_ALARM), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_environment_alarm_status, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_CURRENT_TASK_ALARM), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_current_task_alarm, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_CURRENT_TASK_STRING), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_current_task_string, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_CURRENT_TASK_PROCESS), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_current_task_process, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_CURRENT_TASK_DEVICE_SHOW), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_current_task_device_show, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_CURRENT_TASK_DEVICE_FIRST_RETURN), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_current_task_device_first_return, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_STATUS), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_robot_query_status, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_CURRENT_STATUS_LIGHT), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_current_status_light, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_SICK_STATUS_DEBUG), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_sick_debug_info, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_MOTOR_STATUS_DEBUG), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_motor_debug_info, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_NEXT_TIMED_TASK_TICK), boost::bind(&LibDLHangRailRobotBackground::query_resp_robot_next_task_tick, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_IEC_REMOTE_VALUE_MSG), boost::bind(&LibDLHangRailRobotBackground::query_robot_IEC_104_remote_value, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_IEC_REMOTE_SIGNAL_MSG), boost::bind(&LibDLHangRailRobotBackground::query_robot_IEC_104_remote_signal, this, _1));

    // 机器人控制消息回传
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_TO_POINT + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_to_point_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_MOVE_ABS + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_move_abs_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_MOVE + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_move_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_LIFT_ABS + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_lift_abs_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_LIFT + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_lift_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_CAM_PTZ_ABS + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_cam_ptz_abs_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_CAM_PTZ + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_cam_ptz_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_BODY_ABS + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_body_abs_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_BODY + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_body_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_PARTIALDISCHARGE + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_partialdischarge_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_MAN_PD + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_man_pd_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_MAN_TO_POINT + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_man_to_point_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_PD_PTZ + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_pd_ptz_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_PD_COLLECT + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_pd_collect_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_STATUS_LIGHT + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_status_light_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_ZERO_LIFT + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_zero_lift_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_EMERGENCY_STOP + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_emergency_stop_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_WARNING_LIGHT_FLASH + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_warning_light_flash_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_TOUCH_SCREEN_SWITCH + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::robot_ctrl_pd_reset_resp, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_INIT_STATUS + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::resp_robot_query_robot_init_status, this, _1));
    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_ROBOT_CURRENT_STATUS_EMERGENCY), boost::bind(&LibDLHangRailRobotBackground::resp_robot_query_robot_current_status_emergency, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_DEVELOP_MODE + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::resp_robot_query_robot_current_status_developer, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_CTRL_WALK_THRESHOLD + MSG_TYPE_RETURN_VAL), boost::bind(&LibDLHangRailRobotBackground::resp_robot_query_robot_walk_threshold, this, _1));
	

	/*--------------------数据库操作--------------------*/
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_UPDATE_STATION_CFG + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_update_station_cfg_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_INSERT_DEVICE), boost::bind(&LibDLHangRailRobotBackground::robot_insert_device_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_INSERT_MULTI_DEVICES + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_insert_multi_devices_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_UPDATE_DEVICE_BY_SSID), boost::bind(&LibDLHangRailRobotBackground::robot_update_device_by_ssid_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_UPDATE_MULTI_DEVICE_BY_SSID + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_update_multi_devices_by_ssid_resp, this, _1));
	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_DELETE_BY_SSID), boost::bind(&LibDLHangRailRobotBackground::robot_delete_device_by_ssid_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_DELETE_MULTI_DEVICE_BY_SSID + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_delete_multi_devices_by_ssid_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_INSERT_VIR_DEVICE), boost::bind(&LibDLHangRailRobotBackground::robot_insert_vir_device_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_INSERT_MULTI_VIR_DEVICE + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_insert_multi_vir_device_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_UPDATE_VIR_DEVICE + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_update_vir_device_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_UPDATE_MULTI_VIR_DEVICE + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_update_multi_vir_device_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_DELETE_VIR_DEVICE + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_delete_vir_device_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_DELETE_MULTI_VIR_DEVICE + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_delete_multi_vir_device_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_INSERT_THRESHOLD_ENVIRONMENT), boost::bind(&LibDLHangRailRobotBackground::robot_insert_threshol_environment_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_INSERT_THRESHOL_PATROL), boost::bind(&LibDLHangRailRobotBackground::robot_insert_threshol_patrol_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_INSERT_TASK + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_insert_task_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_INSERT_TASK_AT_BEGINNING + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_insert_task_at_beginning_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_UPDATE_TASK + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_update_task_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_DELETE_TASK_BY_SSID + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_insert_task_template_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_INSERT_TASK_TEMPLATE), boost::bind(&LibDLHangRailRobotBackground::robot_insert_task_template_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_DELETE_TASK_TEMPLATE), boost::bind(&LibDLHangRailRobotBackground::robot_delete_task_template_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_INSERT_INPECT_RESULT + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_delete_task_by_ssid_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_UPDATE_INSPECT_RESULT), boost::bind(&LibDLHangRailRobotBackground::robot_update_inspect_result_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_INSERT_MAP_DATA), boost::bind(&LibDLHangRailRobotBackground::robot_insert_map_Data_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_INSERT_ENVIRONMENT_RESULT + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_insert_environment_result_resp, this, _1));
// 	m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_DELETE_ENVIRONMENT_RESULT + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_delete_environment_result_resp, this, _1));
 	m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_INSERT_UPDATE_RESTORATION), boost::bind(&LibDLHangRailRobotBackground::robot_insert_or_update_restoration_value_resp, this, _1));
	//  m_clientSocket->registerMsgHandle((MSG_TYPE_DATA_UPDATE_DEVICES_FOR_RELATIVE_DEV + RailRobot_PointAdd), boost::bind(&LibDLHangRailRobotBackground::robot_update_devices_for_relative_dev_resp, this, _1));
//    m_clientSocket->registerMsgHandle((MSG_TYPE_QUERY_INSERT_WALK_THRESHOLD), boost::bind(&LibDLHangRailRobotBackground::robot_update_insert_walk_threshold_resp, this, _1));
}

void LibDLHangRailRobotBackground::disconnect()
{
    m_clientSocket->closeSocket();
}

void LibDLHangRailRobotBackground::robot_task_assign_all_station()
{
    boost::shared_ptr<commonMsg> msg(new commonMsg);

    msg->setMsgTypeId(MSG_TYPE_TASK_ASSIGN_ALL_STATION);

    m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_task_normal_task(QString task_name, QStringList task_list)
{
    boost::shared_ptr<commonMsg> msg(new commonMsg);

    msg->setMsgTypeId(MSG_TYPE_TASK_ASSIGN_TASK);

    msg->AppendMsgBodyElement("task_name", std::string(task_name.toLocal8Bit()));

    Json::Value taskListRoot;
    for (int i = 0; i < task_list.size(); i++)
    {
        taskListRoot[i] = std::string(task_list[i].toLocal8Bit());
    }

    msg->AppendMsgBodyElement("task_list", taskListRoot);

    m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_device_proceed(QString dealed_info, QVector<updateAlarmInfoStruct> infoList)
{
    boost::shared_ptr<commonMsg> msg(new commonMsg);

    msg->setMsgTypeId(MSG_TYPE_TASK_PROCEED_INSPECT_RESULT_ALARM);

    msg->AppendMsgBodyElement("dealed_info", std::string(dealed_info.toLocal8Bit()));

    Json::Value infoListRoot;
    for (int i = 0; i < infoList.size(); i++)
    {
        infoListRoot[i]["task_uuid"] = std::string(infoList[i].task_uuid.toLocal8Bit());
        infoListRoot[i]["device_uuid"] = std::string(infoList[i].device_uuid.toLocal8Bit());
    }

    msg->AppendMsgBodyElement("infoList", infoListRoot);

    m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::query_resp_robot_environment_status(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_environment_status==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_environment_status retVal = NULL");
        return;
    }

    float temperature = val["temperature"].asFloat();
    float humidity = val["humidity"].asFloat();
    float SF6 = val["SF6"].asFloat();
    float O3 = val["O3"].asFloat();

    signal_robot_environment_status(QString("%1").arg(temperature), QString("%1").arg(humidity), QString("%1").arg(SF6), QString("%1").arg(O3));
}

void LibDLHangRailRobotBackground::query_resp_robot_environment_alarm_status(boost::shared_ptr<commonMsg> msg)
{
	ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_environment_alarm_status==Str:%s", msg->getBodyJsonString().c_str());
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_environment_alarm_status retVal = NULL");
		return;
	}
	thresholdEnviAlarm thresholdEnvi;
	thresholdEnvi.envi_datae_time = QString::fromStdString(val["envi_datae_time"].asString());
	thresholdEnvi.envi_station_name = QString::fromStdString(val["envi_station_name"].asString());
	thresholdEnvi.envi_device_type_name = QString::fromStdString(val["envi_device_type_name"].asString());
	thresholdEnvi.envi_alarm_value = val["envi_alarm_value"].asFloat();
	thresholdEnvi.envi_alarm_up = val["envi_alarm_up"].asFloat();
	thresholdEnvi.envi_alarm_down = val["envi_alarm_down"].asFloat();
	signal_robot_environment_alarm_status(thresholdEnvi);
}

void LibDLHangRailRobotBackground::query_resp_robot_robot_status(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_robot_status==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_robot_status retVal = NULL");
        return;
    }

    robotSensorStruct tempStatus;
    tempStatus.bWalkBoxConnected = val["bWalkBoxConnected"].asBool();
    tempStatus.bSensorBoxConnected = val["bSensorBoxConnected"].asBool();
    tempStatus.bHcNetCameraConnected = val["bHcNetCameraConnected"].asBool();
    tempStatus.bMagnityCameraConnected = val["bMagnityCameraConnected"].asBool();

    tempStatus.bBlock[0] = val["left"].asInt();
    tempStatus.bBlock[1] = val["right"].asInt();
    tempStatus.bBlock[2] = val["bottom1"].asInt();
    tempStatus.bBlock[3] = val["bottom2"].asInt();

    Json::Value hardwareRoot = val["hardwareRoot"];
    tempStatus.hardwareStatus[ROBOT_HARDWARE_SICK] = hardwareRoot["ROBOT_HARDWARE_SICK"].asInt();
    tempStatus.hardwareStatus[ROBOT_HARDWARE_HORIZON_MOTOR] = hardwareRoot["ROBOT_HARDWARE_HORIZON_MOTOR"].asInt();
    tempStatus.hardwareStatus[ROBOT_HARDWARE_VERTICAL_MOTOR] = hardwareRoot["ROBOT_HARDWARE_VERTICAL_MOTOR"].asInt();
    tempStatus.hardwareStatus[ROBOT_HARDWARE_LIFT_SENSOR] = hardwareRoot["ROBOT_HARDWARE_LIFT_SENSOR"].asInt();
    tempStatus.hardwareStatus[ROBOT_HARDWARE_BODY_ROTATE_MOTOR] = hardwareRoot["ROBOT_HARDWARE_BODY_ROTATE_MOTOR"].asInt();
    tempStatus.hardwareStatus[ROBOT_HARDWARE_CAM_ROTATE_MOTOR] = hardwareRoot["ROBOT_HARDWARE_CAM_ROTATE_MOTOR"].asInt();
    tempStatus.hardwareStatus[ROBOT_HARDWARE_PD_ROTATE_MOTOR] = hardwareRoot["ROBOT_HARDWARE_PD_ROTATE_MOTOR"].asInt();
    tempStatus.hardwareStatus[ROBOT_HARDWARE_PD_STRETCH_MOTOR] = hardwareRoot["ROBOT_HARDWARE_PD_STRETCH_MOTOR"].asInt();

    signal_robot_current_status(tempStatus);

}

void LibDLHangRailRobotBackground::query_resp_robot_robot_query_status(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_robot_query_status==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_robot_query_status retVal = NULL");
        return;
    }

    robotQueryStatus tempStatus;
    
    tempStatus.armStatus = (RobotBodyPDArmStatus)val["armStatus"].asInt();

    tempStatus.bBlock[0] = val["left"].asInt();
    tempStatus.bBlock[1] = val["right"].asInt();
    tempStatus.bBlock[2] = val["bottom1"].asInt();
    tempStatus.bBlock[3] = val["bottom2"].asInt();

    tempStatus.dbNum = val["dbNum"].asInt();
    tempStatus.liftoffset = val["liftoffset"].asInt();
    tempStatus.OnNode = val["OnNode"].asInt();
    tempStatus.pdArmRotate = val["pdArmRotate"].asInt();
    tempStatus.ptzRotate = val["ptzRotate"].asInt();
    tempStatus.robotOffset = val["robotOffset"].asInt();
    tempStatus.robotRotate = val["robotRotate"].asInt();
    tempStatus.speed = val["speed"].asInt();
    tempStatus.stopBtn = val["stopBtn"].asInt();

    signal_robot_query_status(tempStatus);

}

void LibDLHangRailRobotBackground::query_resp_robot_current_task_alarm(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_current_task_alarm==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_current_task_alarm retVal = NULL");
        return;
    }

    int count = val["count"].asInt();

    signal_robot_task_alarm_count(count);
}

void LibDLHangRailRobotBackground::query_resp_robot_current_task_string(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_current_task_string==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_current_task_string retVal = NULL");
        return;
    }

    QString task_ssid = QString::fromLocal8Bit(val["task_ssid"].asString().c_str());
    QString task_name = QString::fromLocal8Bit(val["task_name"].asString().c_str());
    int dev_count = val["dev_count"].asInt();
    int err_dev_count = val["err_dev_count"].asInt();
    int total_dev = val["total_dev"].asInt();

    signal_robot_current_string(task_ssid, task_name, dev_count, err_dev_count, total_dev);
}

void LibDLHangRailRobotBackground::query_resp_robot_current_task_process(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_current_task_process==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_current_task_process retVal = NULL");
        return;
    }

    bool bStart = val["bStart"].asBool();
    QString task_ssid = QString::fromLocal8Bit(val["task_ssid"].asString().c_str());
    signal_robot_current_task_process(bStart, task_ssid);
}

void LibDLHangRailRobotBackground::query_resp_robot_current_task_device_show(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_current_task_device_show==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_current_task_device_show retVal = NULL");
        return;
    }

    inspectResultMsg tmpMsg;
 //   tmpMsg.task_ssid = QString::fromLocal8Bit(val["task_ssid"].asString().c_str());
 //   tmpMsg.task_name = QString::fromLocal8Bit(val["task_name"].asString().c_str());
 //   tmpMsg.station_name = QString::fromLocal8Bit(val["station_name"].asString().c_str());
    tmpMsg.device_ssid = QString::fromLocal8Bit(val["device_ssid"].asString().c_str());
 //   tmpMsg.device_area_name = QString::fromLocal8Bit(val["device_area_name"].asString().c_str());
 //   tmpMsg.device_name = QString::fromLocal8Bit(val["device_name"].asString().c_str());
 //   tmpMsg.device_type = QString::fromLocal8Bit(val["device_type"].asString().c_str());
    tmpMsg.inspect_result = QString::fromLocal8Bit(val["inspect_result"].asString().c_str());
    tmpMsg.inspect_status = QString::fromLocal8Bit(val["inspect_status"].asString().c_str());
    tmpMsg.inspect_time = QString::fromLocal8Bit(val["inspect_time"].asString().c_str());

    signal_robot_current_task_device_show(tmpMsg);
}

void LibDLHangRailRobotBackground::query_resp_robot_current_task_device_first_return(boost::shared_ptr<commonMsg> msg)
{
	ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_current_task_device_first_return==Str:%s", msg->getBodyJsonString().c_str());
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_current_task_device_first_return retVal = NULL");
		return;
	}

	QList<inspectResultMsg> data;
	inspectResultMsg dataStru;
	for (int i = 0; i < val["data"].size(); i++)
	{
		dataStru.device_ssid = QString::fromStdString(val["data"][i]["device_ssid"].asString());
		dataStru.station_name = QString::fromStdString(val["data"][i]["station_name"].asString());
		dataStru.task_name = QString::fromStdString(val["data"][i]["task_name"].asString());
		dataStru.device_area_name = QString::fromStdString(val["data"][i]["device_area"].asString());
		dataStru.device_name = QString::fromStdString(val["data"][i]["device_name"].asString());
		dataStru.device_type = QString::fromStdString(val["data"][i]["device_type"].asString());
		dataStru.inspect_status = QString::fromStdString(val["data"][i]["device_status"].asString());
		data.append(dataStru);
	}
	signal_robot_current_task_device_first_return(data);
}

void LibDLHangRailRobotBackground::query_resp_robot_motor_debug_info(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_robot_motor_status_debug==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_robot_motor_status_debug retVal = NULL");
        return;
    }

    int hStatus = val["horizontal"].asInt();
    int vStatus = val["vertical"].asInt();
    QString currTime = QDateTime::currentDateTime().toString("yyyy-MM-hh hh:mm:ss");
    QString hStatusString = "";
    QString vStatusString = "";
    if (hStatus)
    {
        hStatusString = currTime + " " + motorStatusString[hStatus];
    }
    
    if (vStatus)
    {
        vStatusString = currTime + " " + motorStatusString[vStatus];
    }

    signal_motorDebugInfo(hStatusString, vStatusString);
}

void LibDLHangRailRobotBackground::query_resp_robot_sick_debug_info(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_robot_sick_status_debug==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_robot_sick_status_debug retVal = NULL");
        return;
    }

    unsigned hStatus = val["status"].asUInt();

    signal_sickDebugInfo(hStatus);
}

void LibDLHangRailRobotBackground::query_resp_robot_current_status_light(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_current_status_light==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_current_status_light retVal = NULL");
        return;
    }

    RobotBodyWarnLight type = (RobotBodyWarnLight)val["type"].asInt();

    singnal_robot_current_status_light(type);
}

void LibDLHangRailRobotBackground::query_resp_robot_next_task_tick(boost::shared_ptr<commonMsg> msg)
{
    ROS_INFO("LibDLHangRailRobotBackground::query_resp_robot_next_task_tick==Str:%s", msg->getBodyJsonString().c_str());
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_resp_robot_next_task_tick retVal = NULL");
        return;
    }

    int tick = val["tick"].asInt();

    singnal_robot_next_timed_task_tick(tick);
}

/*-------------------------------------------------------*/
void LibDLHangRailRobotBackground::robot_ctrl_to_point_req(int pointId)
{
	ROS_INFO("LibDLHangRailRobotBackground::robot_ctrl_to_point_req,id:%d", pointId);
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_TO_POINT);

	msg->AppendMsgBodyElement("pointId", pointId);

	m_clientSocket->postMsg(msg);

}

void LibDLHangRailRobotBackground::robot_ctrl_move_abs_req(int offset, int speed)
{
	ROS_INFO("LibDLHangRailRobotBackground::robot_ctrl_move_abs_req,offset:%d speed:%d", offset, speed);
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_MOVE_ABS);

	msg->AppendMsgBodyElement("offset", offset);
	msg->AppendMsgBodyElement("speed", speed);

	m_clientSocket->postMsg(msg);

}

void LibDLHangRailRobotBackground::robot_ctrl_move_req(RobotMoveMode type, int speed)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_MOVE);

	msg->AppendMsgBodyElement("type", (int)type);
	msg->AppendMsgBodyElement("speed", speed);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_lift_abs_req(int length)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_LIFT_ABS);

	msg->AppendMsgBodyElement("length", length);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_lift_req(RobotLiftMode type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_LIFT);

	msg->AppendMsgBodyElement("type", (int)type);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_cam_ptz_abs_req(int rotate) 
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_CAM_PTZ_ABS);

	msg->AppendMsgBodyElement("rotate", rotate);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_cam_ptz_req(RobotCamPtzMode type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_CAM_PTZ);

	msg->AppendMsgBodyElement("type", (int)type);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_body_abs_req(int rotate)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_BODY_ABS);

	msg->AppendMsgBodyElement("rotate", rotate);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_body_req(RobotBodyRotateMode type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_BODY);

	msg->AppendMsgBodyElement("type", (int)type);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_partialdischarge_req(RobotPartialDischargeOper type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_PARTIALDISCHARGE);

	msg->AppendMsgBodyElement("type", (int)type);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_man_pd_req(RobotBodyPDMode type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_MAN_PD);

	msg->AppendMsgBodyElement("type", (int)type);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_man_to_point_req(int pointId, int speed)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_MAN_TO_POINT);

	msg->AppendMsgBodyElement("pointId", pointId);
	msg->AppendMsgBodyElement("speed", speed);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_pd_ptz_req(RobotBodyPDArm type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_PD_PTZ);

	msg->AppendMsgBodyElement("type", (int)type);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_pd_collect_req()
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_PD_COLLECT);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_status_light_req(RobotBodyWarnLight type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_STATUS_LIGHT);

	msg->AppendMsgBodyElement("type", (int)type);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_zero_lift_req()
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_ZERO_LIFT);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_emergency_stop_req(bool bEmergency)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_EMERGENCY_STOP);
    msg->AppendMsgBodyElement("emergency", bEmergency);
	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_warning_light_flash_req(RobotBodyWarnLight type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_WARNING_LIGHT_FLASH);

	msg->AppendMsgBodyElement("type", (int)type);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_touch_screen_req(int type)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_TOUCH_SCREEN_SWITCH);
    
    msg->AppendMsgBodyElement("type", type);

    m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_develop_mode(bool bOpen)
{
    boost::shared_ptr<commonMsg> msg(new commonMsg);

    msg->setMsgTypeId(MSG_TYPE_CTRL_DEVELOP_MODE);

    msg->AppendMsgBodyElement("bOpen", bOpen);

    m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_walk_threshold_req(int start_value, int terminus_value)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_CTRL_WALK_THRESHOLD);

	msg->AppendMsgBodyElement("start_value", start_value);
	msg->AppendMsgBodyElement("terminus_value", terminus_value);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_remote_control_req(int dataNo, int status)
{
    boost::shared_ptr<commonMsg> msg(new commonMsg);

    msg->setMsgTypeId(MSG_TYPE_CTRL_REMOTE_CONTROL);

    msg->AppendMsgBodyElement("dataNo", dataNo);
    msg->AppendMsgBodyElement("status", status);

    m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_ctrl_update_embedded_software_req(RobotBodyUpdateEmbeddedSoftwareType type)
{
    boost::shared_ptr<commonMsg> msg(new commonMsg);

    msg->setMsgTypeId(MSG_TYPE_CTRL_UPDATE_EMBEDDED_SOFTWARE);

    msg->AppendMsgBodyElement("type", (int)type);

    m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_task_automatic_take_photo_req()
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MAS_TYPE_TASK_AUTOMATIC_TAKE_PHOTO);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_task_stop_take_photo_req()
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MAS_TYPE_TASK_STOP_TAKE_PHOTO);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_task_recover_task_req()
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MAS_TYPE_TASK_RECOVER_TASK);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_task_pause_task_req()
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MAS_TYPE_TASK_PAUSE_TASK);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_task_stop_task()
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MAS_TYPE_TASK_STOP_TASK);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_query_robot_init_status()
{
    boost::shared_ptr<commonMsg> msg(new commonMsg);

    msg->setMsgTypeId(MSG_TYPE_QUERY_ROBOT_INIT_STATUS);

    m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::resp_robot_query_robot_init_status(boost::shared_ptr<commonMsg> msg)
{
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::resp_robot_query_robot_init_status retVal = NULL");
        return;
    }
    
    RobotBodyWarnLight type = (RobotBodyWarnLight)val["type"].asInt();
    bool bEmergency = val["emergency"].asBool();

    singnal_robot_current_status_light(type);
    singnal_robot_current_status_emergency(bEmergency);
}

void LibDLHangRailRobotBackground::resp_robot_query_robot_current_status_emergency(boost::shared_ptr<commonMsg> msg)
{
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::resp_robot_query_robot_current_status_emergency retVal = NULL");
        return;
    }

    bool bEmergency = val["emergency"].asBool();

    singnal_robot_current_status_emergency(bEmergency);
}

void LibDLHangRailRobotBackground::resp_robot_query_robot_current_status_developer(boost::shared_ptr<commonMsg> msg)
{
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::resp_robot_query_robot_current_status_emergency retVal = NULL");
        return;
    }

    bool bOpen = val["bOpen"].asBool();

    singnal_robot_current_status_developer(bOpen);
}

void LibDLHangRailRobotBackground::resp_robot_query_robot_walk_threshold(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::resp_robot_query_robot_current_status_emergency retVal = NULL");
		return;
	}

	bool error_status = val["error_status"].asBool();
	QString error_msg = QString::fromStdString(val["error_msg"].asString());

	singnal_robot__walk_threshold(error_status, error_msg);
}

void LibDLHangRailRobotBackground::query_robot_IEC_104_remote_value(boost::shared_ptr<commonMsg> msg)
{
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_robot_IEC_104_remote_value retVal = NULL");
        return;
    }

    QVector<float> remote_value;

    for (int i = 0; i < val["remote_value"].size(); i++)
    {
        remote_value.push_back(val["remote_value"][i].asFloat());
    }

    singnal_remote_value_callback(remote_value);

}

void LibDLHangRailRobotBackground::query_robot_IEC_104_remote_signal(boost::shared_ptr<commonMsg> msg)
{
    Json::Value val = msg->getBodyJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLHangRailRobotBackground::query_robot_IEC_104_remote_signal retVal = NULL");
        return;
    }

    QVector<int> remote_signal;

    for (int i = 0; i < val["remote_signal"].size(); i++)
    {
        remote_signal.push_back(val["remote_signal"][i].asInt());
    }
    singnal_remote_signal_callback(remote_signal);
}

/////
void LibDLHangRailRobotBackground::robot_update_station_cfg_req(stationCfg cfg)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_UPDATE_STATION_CFG);

	msg->AppendMsgBodyElement("station_ssid", cfg.station_ssid.toStdString());
	msg->AppendMsgBodyElement("station_name", cfg.station_name.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_device_req(deviceConfigType dev)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_DEVICE);

//	msg->AppendMsgBodyElement("device_ssid", dev.device_ssid.toStdString());
	msg->AppendMsgBodyElement("device_alternate_name_id", dev.device_alternate_name_id);
	msg->AppendMsgBodyElement("device_name_id", dev.device_name_id);
	msg->AppendMsgBodyElement("device_area_name", dev.device_area_name.toStdString());
	msg->AppendMsgBodyElement("device_area_id", dev.device_area_id);
	msg->AppendMsgBodyElement("device_robot_offset", dev.device_robot_offset);
	msg->AppendMsgBodyElement("device_lift_offset", dev.device_lift_offset);
	msg->AppendMsgBodyElement("device_body_rotate", dev.device_body_rotate);
	msg->AppendMsgBodyElement("device_cam_rotate", dev.device_cam_rotate);
	msg->AppendMsgBodyElement("device_hc_zoom", dev.device_hc_zoom);
	msg->AppendMsgBodyElement("device_hc_focus", dev.device_hc_focus);
	msg->AppendMsgBodyElement("device_thermo_focus", dev.device_thermo_focus);
	msg->AppendMsgBodyElement("station_ssid", dev.station_ssid.toStdString());
	msg->AppendMsgBodyElement("device_hc_zoom_two_stage", dev.device_hc_zoom_two_stage);
	msg->AppendMsgBodyElement("device_hc_focus_two_stage", dev.device_hc_focus_two_stage);
	msg->AppendMsgBodyElement("device_thermo_focus_two_stage", dev.device_thermo_focus_two_stage);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_multi_devices_req(QList<deviceConfigType> devs)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_MULTI_DEVICES);

	Json::Value dev;
	for (int i = 0; i < devs.size(); i++)
	{
		dev[i]["device_ssid"] = devs[i].device_ssid.toStdString();
		dev[i]["device_alternate_name_id"] = devs[i].device_alternate_name_id;
		dev[i]["device_name_id"] = devs[i].device_name_id;
		dev[i]["device_area_name"] = devs[i].device_area_name.toStdString();
		dev[i]["device_area_id"] = devs[i].device_area_id;
		dev[i]["device_robot_offset"] = devs[i].device_robot_offset;
		dev[i]["device_lift_offset"] = devs[i].device_lift_offset;
		dev[i]["device_body_rotate"] = devs[i].device_body_rotate;
		dev[i]["device_cam_rotate"] = devs[i].device_cam_rotate;
		dev[i]["device_hc_zoom"] = devs[i].device_hc_zoom;
		dev[i]["device_hc_focus"] = devs[i].device_hc_focus;
		dev[i]["device_thermo_focus"] = devs[i].device_thermo_focus;
		dev[i]["station_ssid"] = devs[i].station_ssid.toStdString();
		dev[i]["device_hc_zoom_two_stage"] = devs[i].device_hc_zoom_two_stage;
		dev[i]["device_hc_focus_two_stage"] = devs[i].device_hc_focus_two_stage;
		dev[i]["device_thermo_focus_two_stage"] = devs[i].device_thermo_focus_two_stage;
	}
	msg->AppendMsgBodyElement("devices", dev);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_update_device_by_ssid_req(deviceConfigType dev)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_UPDATE_DEVICE_BY_SSID);

	msg->AppendMsgBodyElement("device_alternate_name_id", dev.device_alternate_name_id);//
	msg->AppendMsgBodyElement("device_name_id", dev.device_name_id);//
	msg->AppendMsgBodyElement("device_area_name", dev.device_area_name.toStdString());//
	msg->AppendMsgBodyElement("device_area_id", dev.device_area_id);//
	msg->AppendMsgBodyElement("relative_dev", dev.relative_dev.toStdString());    //
	msg->AppendMsgBodyElement("device_robot_offset", dev.device_robot_offset);
	msg->AppendMsgBodyElement("device_lift_offset", dev.device_lift_offset);
	msg->AppendMsgBodyElement("device_body_rotate", dev.device_body_rotate);
	msg->AppendMsgBodyElement("device_cam_rotate", dev.device_cam_rotate);
	msg->AppendMsgBodyElement("device_hc_zoom", dev.device_hc_zoom);
	msg->AppendMsgBodyElement("device_hc_focus", dev.device_hc_focus);
	msg->AppendMsgBodyElement("device_thermo_focus", dev.device_thermo_focus);
	msg->AppendMsgBodyElement("station_ssid", dev.station_ssid.toStdString());
	msg->AppendMsgBodyElement("device_hc_zoom_two_stage", dev.device_hc_zoom_two_stage);
	msg->AppendMsgBodyElement("device_hc_focus_two_stage", dev.device_hc_focus_two_stage);
	msg->AppendMsgBodyElement("device_thermo_focus_two_stage", dev.device_thermo_focus_two_stage);
	msg->AppendMsgBodyElement("device_ssid", dev.device_ssid.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_update_multi_devices_by_ssid_req(QList<deviceConfigType> devs)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_UPDATE_MULTI_DEVICE_BY_SSID);

	Json::Value dev;
	for (int i = 0; i < devs.size(); i++)
	{
		dev[i]["device_ssid"] = devs[i].device_ssid.toStdString();
		dev[i]["device_alternate_name_id"] = devs[i].device_alternate_name_id;
		dev[i]["device_name_id"] = devs[i].device_name_id;
		dev[i]["device_area_name"] = devs[i].device_area_name.toStdString();
		dev[i]["device_area_id"] = devs[i].device_area_id;
		dev[i]["relative_dev"] = devs[i].relative_dev.toStdString();
		dev[i]["device_robot_offset"] = devs[i].device_robot_offset;
		dev[i]["device_lift_offset"] = devs[i].device_lift_offset;
		dev[i]["device_body_rotate"] = devs[i].device_body_rotate;
		dev[i]["device_cam_rotate"] = devs[i].device_cam_rotate;
		dev[i]["device_hc_zoom"] = devs[i].device_hc_zoom;
		dev[i]["device_hc_focus"] = devs[i].device_hc_focus;
		dev[i]["device_thermo_focus"] = devs[i].device_thermo_focus;
		dev[i]["station_ssid"] = devs[i].station_ssid.toStdString();
		dev[i]["device_hc_zoom_two_stage"] = devs[i].device_hc_zoom_two_stage;
		dev[i]["device_hc_focus_two_stage"] = devs[i].device_hc_focus_two_stage;
		dev[i]["device_thermo_focus_two_stage"] = devs[i].device_thermo_focus_two_stage;
	}
	msg->AppendMsgBodyElement("devices", dev);
}

void LibDLHangRailRobotBackground::robot_delete_device_by_ssid_req(QString ssid)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_DELETE_BY_SSID);

	msg->AppendMsgBodyElement("ssid", ssid.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_delete_multi_devices_by_ssid_req(QList<QString> ssids)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_DELETE_MULTI_DEVICE_BY_SSID);

	Json::Value devSsid;
	for (int i = 0; i < ssids.size(); i++)
	{
		devSsid[i]["ssid"] = ssids[i].toStdString();
	}

	msg->AppendMsgBodyElement("ssid", devSsid);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_vir_device_req(virtualDeviceType dev)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_VIR_DEVICE);

	msg->AppendMsgBodyElement("vir_dev_ssid", dev.vir_dev_ssid.toStdString());
	msg->AppendMsgBodyElement("vir_dev_name_id", dev.vir_dev_name_id);
	msg->AppendMsgBodyElement("vir_dev_area_name", dev.vir_dev_area_name.toStdString());
	msg->AppendMsgBodyElement("vir_dev_area_id", dev.vir_dev_area_id);
	msg->AppendMsgBodyElement("relative_dev", dev.relative_dev.toStdString());
	msg->AppendMsgBodyElement("station_ssid", dev.station_ssid.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_multi_vir_device_req(QList<virtualDeviceType> devs)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_MULTI_VIR_DEVICE);

	Json::Value virDev;
	for (int i = 0; i < devs.size(); i++)
	{
		virDev[i]["vir_dev_ssid"] = devs[i].vir_dev_ssid.toStdString();
		virDev[i]["vir_dev_name_id"] = devs[i].vir_dev_name_id;
		virDev[i]["vir_dev_area_name"] = devs[i].vir_dev_area_name.toStdString();
		virDev[i]["vir_dev_area_id"] = devs[i].vir_dev_area_id;
		virDev[i]["relative_dev"] = devs[i].relative_dev.toStdString();
		virDev[i]["station_ssid"] = devs[i].station_ssid.toStdString();
	}
	msg->AppendMsgBodyElement("virDev", virDev);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_update_vir_device_req(virtualDeviceType dev)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_UPDATE_VIR_DEVICE);

	msg->AppendMsgBodyElement("vir_dev_name_id", dev.vir_dev_name_id);
	msg->AppendMsgBodyElement("vir_dev_area_name", dev.vir_dev_area_name.toStdString());
	msg->AppendMsgBodyElement("vir_dev_area_id", dev.vir_dev_area_id);
	msg->AppendMsgBodyElement("relative_dev", dev.relative_dev.toStdString());
	msg->AppendMsgBodyElement("station_ssid", dev.station_ssid.toStdString());
	msg->AppendMsgBodyElement("vir_dev_ssid", dev.vir_dev_ssid.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_update_multi_vir_device_req(QList<virtualDeviceType> devs)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_UPDATE_MULTI_VIR_DEVICE);

	Json::Value virDev;
	for (int i = 0; i < devs.size(); i++)
	{
		virDev[i]["vir_dev_ssid"] = devs[i].vir_dev_ssid.toStdString();
		virDev[i]["vir_dev_name_id"] = devs[i].vir_dev_name_id;
		virDev[i]["vir_dev_area_name"] = devs[i].vir_dev_area_name.toStdString();
		virDev[i]["vir_dev_area_id"] = devs[i].vir_dev_area_id;
		virDev[i]["relative_dev"] = devs[i].relative_dev.toStdString();
		virDev[i]["station_ssid"] = devs[i].station_ssid.toStdString();
	}
	msg->AppendMsgBodyElement("virDev", virDev);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_delete_vir_device_req(QString ssid)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_DELETE_VIR_DEVICE);

	msg->AppendMsgBodyElement("ssid", ssid.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_delete_multi_vir_device_req(QList<QString> ssids)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_DELETE_MULTI_VIR_DEVICE);

	Json::Value devSsid;
	for (int i = 0; i < ssids.size(); i++)
	{
		devSsid[i]["ssid"] = ssids[i].toStdString();
	}

	msg->AppendMsgBodyElement("ssid", devSsid);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_threshol_environment_req(thresholdEnvi TEnvi)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_THRESHOL_ENVIRONMENT);

	msg->AppendMsgBodyElement("envi_station_name", TEnvi.envi_station_name.toStdString());
	msg->AppendMsgBodyElement("envi_alarm_up", TEnvi.envi_alarm_up);
	msg->AppendMsgBodyElement("envi_alarm_down", TEnvi.envi_alarm_down);
	msg->AppendMsgBodyElement("envi_device_type_name", TEnvi.envi_device_type_name.toStdString());

	m_clientSocket->postMsg(msg);

}

void LibDLHangRailRobotBackground::robot_insert_threshol_patrol_req(thresholdPatrol TPatrol)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_THRESHOL_PATROL);

	msg->AppendMsgBodyElement("patrol_station_name", TPatrol.patrol_station_name.toStdString());
	msg->AppendMsgBodyElement("patrol_alarm_up", TPatrol.patrol_alarm_up);
	msg->AppendMsgBodyElement("patrol_alarm_down", TPatrol.patrol_alarm_down);
	msg->AppendMsgBodyElement("patrol_device_type_name", TPatrol.patrol_device_type_name.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_task_req(taskType task)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_TASK);

	msg->AppendMsgBodyElement("task_ssid", task.task_ssid.toStdString());
	msg->AppendMsgBodyElement("task_name", task.task_name.toStdString());
	msg->AppendMsgBodyElement("task_template_ssid", task.task_template_ssid.toStdString());
	msg->AppendMsgBodyElement("station_ssid", task.station_ssid.toStdString());
	msg->AppendMsgBodyElement("task_start_time", task.task_start_time.toString("yyyy-MM-dd hh:mm:ss").toStdString());
	msg->AppendMsgBodyElement("task_end_time", task.task_end_time.toString("yyyy-MM-dd hh:mm:ss").toStdString());
	msg->AppendMsgBodyElement("task_duration", task.task_duration);
	msg->AppendMsgBodyElement("task_total_devices", task.task_total_devices);
	msg->AppendMsgBodyElement("task_total_bugs", task.task_total_bugs);
	msg->AppendMsgBodyElement("task_type_id", task.task_type_id);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_task_at_beginning_req(taskType task)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_TASK_AT_BEGINNING);

	msg->AppendMsgBodyElement("task_ssid", task.task_ssid.toStdString());
	msg->AppendMsgBodyElement("task_name", task.task_name.toStdString());
	msg->AppendMsgBodyElement("task_template_ssid", task.task_template_ssid.toStdString());
	msg->AppendMsgBodyElement("station_ssid", task.station_ssid.toStdString());
	msg->AppendMsgBodyElement("task_start_time", task.task_start_time.toString("yyyy-MM-dd hh:mm:ss").toStdString());
	msg->AppendMsgBodyElement("task_type_id", task.task_type_id);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_update_task_req(taskType task)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_UPDATE_TASK);

	msg->AppendMsgBodyElement("task_ssid", task.task_ssid.toStdString());
	msg->AppendMsgBodyElement("task_end_time", task.task_end_time.toString("yyyy-MM-dd hh:mm:ss").toStdString());
	msg->AppendMsgBodyElement("task_duration", task.task_duration);
	msg->AppendMsgBodyElement("task_total_devices", task.task_total_devices);
	msg->AppendMsgBodyElement("task_total_bugs", task.task_total_bugs);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_delete_task_by_ssid_req(QString ssid)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_DELETE_TASK_BY_SSID);

	msg->AppendMsgBodyElement("ssid", ssid.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_task_template_req(taskTemplateType taskTemplate)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_TASK_TEMPLATE);

	msg->AppendMsgBodyElement("task_template_ssid", taskTemplate.task_template_ssid.toStdString());
	msg->AppendMsgBodyElement("task_template_device_list", taskTemplate.task_template_device_list.join(" ").toStdString());
	msg->AppendMsgBodyElement("task_end_action_id", taskTemplate.task_end_action_id);
	msg->AppendMsgBodyElement("task_type_id", taskTemplate.task_type_id);
	msg->AppendMsgBodyElement("task_start_date", taskTemplate.task_start_date.toString("yyyy-MM-dd hh:mm:ss").toStdString());
	msg->AppendMsgBodyElement("task_repeat_duration", taskTemplate.task_repeat_duration);
	msg->AppendMsgBodyElement("task_template_name", taskTemplate.task_template_name.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_delete_task_template_req(QString taskTemplateSsid)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_DELETE_TASK_TEMPLATE);

	msg->AppendMsgBodyElement("taskTemplateSsid", taskTemplateSsid.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_inpect_result_req(inspectResultType inspectResult)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_INPECT_RESULT);

	msg->AppendMsgBodyElement("task_ssid", inspectResult.task_ssid.toStdString());
	msg->AppendMsgBodyElement("device_ssid", inspectResult.device_ssid.toStdString());
	msg->AppendMsgBodyElement("inspect_time", inspectResult.inspect_time.toString("yyyy-MM-dd hh:mm:ss").toStdString());
	msg->AppendMsgBodyElement("inspect_result", inspectResult.inspect_result.toStdString());
	msg->AppendMsgBodyElement("inspect_status", inspectResult.inspect_status);
	msg->AppendMsgBodyElement("visible_file_path", inspectResult.visible_file_path.toStdString());
	msg->AppendMsgBodyElement("thermo_file_path", inspectResult.thermo_file_path.toStdString());
	msg->AppendMsgBodyElement("is_dealed", inspectResult.is_dealed);
	msg->AppendMsgBodyElement("dealed_info", inspectResult.dealed_info.toStdString());
	msg->AppendMsgBodyElement("deal_user", inspectResult.deal_user.toStdString());
	msg->AppendMsgBodyElement("is_virtual", inspectResult.is_virtual);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_update_inspect_result_req(QList<DeviceTaskSsid> Struct, QString dealed_info)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_UPDATE_INSPECT_RESULT);

	Json::Value deviceTask;
	for (int i = 0; i < Struct.size(); i++)
	{
		deviceTask[i]["task_ssid"] = Struct[i].taskSsid.toStdString();
		deviceTask[i]["device_ssid"] = Struct[i].deviceSsid.toStdString();
	}
	msg->AppendMsgBodyElement("deviceTask", deviceTask);
	msg->AppendMsgBodyElement("dealed_info", dealed_info.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_map_Data_req(QList<MapItemData> MapDataStruct, QString station_id)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_MAP_DATA);

	Json::Value mapdata;
	for (int i = 0; i < MapDataStruct.size(); i++)
	{
		mapdata[i]["startPointX"] = MapDataStruct[i].startPointX;
		mapdata[i]["startPointY"] = MapDataStruct[i].startPointY;
		mapdata[i]["endPointX"] = MapDataStruct[i].endPointX;
		mapdata[i]["endPointY"] = MapDataStruct[i].endPointY;
		mapdata[i]["startOffset"] = MapDataStruct[i].startOffset;
		mapdata[i]["endOffset"] = MapDataStruct[i].endOffset;
		mapdata[i]["itemType"] = MapDataStruct[i].itemType;
		mapdata[i]["deviceName"] = MapDataStruct[i].deviceName.toStdString();
		mapdata[i]["staionId"] = MapDataStruct[i].staionId.toStdString();
	}
	msg->AppendMsgBodyElement("mapdata", mapdata);
	msg->AppendMsgBodyElement("station_id", station_id.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_environment_result_req(EnvironmentResult m_enviResult)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_ENVIRONMENT_RESULT);

	msg->AppendMsgBodyElement("dateTime", m_enviResult.dateTime.toStdString());
	msg->AppendMsgBodyElement("envi_result_temp", m_enviResult.envi_result_temp);
	msg->AppendMsgBodyElement("envi_result_humi", m_enviResult.envi_result_humi);
	msg->AppendMsgBodyElement("envi_result_sf6", m_enviResult.envi_result_sf6);
	msg->AppendMsgBodyElement("envi_result_o3", m_enviResult.envi_result_o3);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_delete_environment_result_req(QString DeletedateTime)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_DELETE_ENVIRONMENT_RESULT);

	msg->AppendMsgBodyElement("DeletedateTime", DeletedateTime.toStdString());

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_or_update_restoration_value_req(RestorationValue m_restorationValue)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_OR_UPDATE_RESTORATION_VALUE);

	msg->AppendMsgBodyElement("rest_station_id", m_restorationValue.rest_station_id);
	msg->AppendMsgBodyElement("rest_move_location", m_restorationValue.rest_move_location);
	msg->AppendMsgBodyElement("rest_move_speed", m_restorationValue.rest_move_speed);
	msg->AppendMsgBodyElement("rest_lift_location", m_restorationValue.rest_lift_location);
	msg->AppendMsgBodyElement("rest_bady_rotate", m_restorationValue.rest_bady_rotate);
	msg->AppendMsgBodyElement("rest_camera_rotate", m_restorationValue.rest_camera_rotate);
	msg->AppendMsgBodyElement("rest_camera_zoom", m_restorationValue.rest_camera_zoom);
	msg->AppendMsgBodyElement("rest_camera_focal", m_restorationValue.rest_camera_focal);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_update_devices_for_relative_dev_req(RelevanceDevice m_releDevice)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_UPDATE_DEVICES_FOR_RELATIVE_DEV);

	Json::Value devSsid;
	for (int i = 0; i < m_releDevice.m_device_id.size(); i++)
	{
		devSsid[i]["device_id"] = m_releDevice.m_device_id[i].toStdString();
	}
	msg->AppendMsgBodyElement("virtualDevice_id", m_releDevice.m_virtualDevice_id.toStdString());
	msg->AppendMsgBodyElement("ssid", devSsid);

	m_clientSocket->postMsg(msg);
}

void LibDLHangRailRobotBackground::robot_insert_walk_threshold_req(int startValue, int terminusValue)
{
	boost::shared_ptr<commonMsg> msg(new commonMsg);

	msg->setMsgTypeId(MSG_TYPE_DATA_INSERT_WALK_THRESHOLD);

	msg->AppendMsgBodyElement("start_value", startValue);
	msg->AppendMsgBodyElement("terminus_value", terminusValue);

	m_clientSocket->postMsg(msg);
}

/*-------------------------------------------------------*/
void LibDLHangRailRobotBackground::robot_ctrl_to_point_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
// 	bool bret = val["bret"].asBool();
// 	std::string result = val["result"].asString();
}

void LibDLHangRailRobotBackground::robot_ctrl_move_abs_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_move_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_lift_abs_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_lift_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_cam_ptz_abs_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_cam_ptz_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_body_abs_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_body_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_partialdischarge_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_man_pd_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_man_to_point_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_pd_ptz_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_pd_collect_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_status_light_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_zero_lift_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();
	singnal_robot_ctrl_core_zero_lift();
}

void LibDLHangRailRobotBackground::robot_ctrl_emergency_stop_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_warning_light_flash_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_ctrl_pd_reset_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

/*-------------------------------------------------------*/
void LibDLHangRailRobotBackground::robot_update_station_cfg_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_insert_device_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_insert_device_resp retVal = NULL");
		return;
	}
	InsertDeviceReturn rdev;
	bool bRet = val["bRet"].asBool();
	rdev.deviceSsid = QString::fromStdString(val["deviceSsid"].asString());
	rdev.areaName = QString::fromStdString(val["areaName"].asString());
	rdev.deviceName = QString::fromStdString(val["deviceName"].asString());
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_insert_device(bRet, rdev, errMsg);
}

void LibDLHangRailRobotBackground::robot_insert_multi_devices_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_update_device_by_ssid_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_update_device_by_ssid_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	deviceConfigType dev;
	dev.device_alternate_name_id = val["device_alternate_name_id"].asInt();
	dev.device_name_id = val["device_name_id"].asInt();
	dev.device_area_name = QString::fromStdString(val["device_area_name"].asString());
	dev.device_area_id = val["device_area_id"].asInt();
	dev.relative_dev = QString::fromStdString(val["relative_dev"].asString());
	dev.device_robot_offset = val["device_robot_offset"].asInt();
	dev.device_lift_offset = val["device_lift_offset"].asInt();
	dev.device_body_rotate = val["device_body_rotate"].asInt();
	dev.device_cam_rotate = val["device_cam_rotate"].asInt();
	dev.device_hc_zoom = val["device_hc_zoom"].asInt();
	dev.device_hc_focus = val["device_hc_focus"].asInt();
	dev.device_thermo_focus = val["device_thermo_focus"].asInt();
	dev.station_ssid = QString::fromStdString(val["station_ssid"].asString());
	dev.device_hc_zoom_two_stage = val["device_hc_zoom_two_stage"].asInt();
	dev.device_hc_focus_two_stage = val["device_hc_focus_two_stage"].asInt();
	dev.device_thermo_focus_two_stage = val["device_thermo_focus_two_stage"].asInt();
	dev.device_ssid = QString::fromStdString(val["device_ssid"].asString());
	singnal_database_update_device_by_ssid(bRet, dev, errMsg);
}

void LibDLHangRailRobotBackground::robot_update_multi_devices_by_ssid_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_delete_device_by_ssid_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_delete_device_by_ssid_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString device_ssid = QString::fromStdString(val["device_ssid"].asString());
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_delete_device_by_ssid(bRet, device_ssid, errMsg);
}

void LibDLHangRailRobotBackground::robot_delete_multi_devices_by_ssid_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_insert_vir_device_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_insert_vir_device_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	virtualDeviceType dev;
	dev.vir_dev_ssid = QString::fromStdString(val["vir_dev_ssid"].asString());
	dev.vir_dev_name_id = val["vir_dev_name_id"].asInt();
	dev.vir_dev_area_name = QString::fromStdString(val["vir_dev_area_name"].asString());
	dev.vir_dev_area_id = val["vir_dev_area_id"].asInt();
	dev.relative_dev = QString::fromStdString(val["relative_dev"].asString());
	dev.station_ssid = QString::fromStdString(val["station_ssid"].asString());
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_insert_vir_device(bRet, dev, errMsg);
}

void LibDLHangRailRobotBackground::robot_insert_multi_vir_device_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_update_vir_device_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_update_multi_vir_device_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_delete_vir_device_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_delete_multi_vir_device_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_insert_threshol_environment_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_insert_threshol_environment_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_insert_threshol_environment(bRet, errMsg);
}

void LibDLHangRailRobotBackground::robot_insert_threshol_patrol_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_insert_threshol_patrol_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_insert_threshol_patrol(bRet, errMsg);
}

void LibDLHangRailRobotBackground::robot_insert_task_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_insert_task_at_beginning_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_update_task_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_insert_task_template_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_insert_task_template_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_insert_task_template(bRet, errMsg);
}

void LibDLHangRailRobotBackground::robot_delete_task_template_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_delete_task_template_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_delete_task_template(bRet, errMsg);
}

void LibDLHangRailRobotBackground::robot_insert_inpect_result_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_delete_task_by_ssid_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_update_inspect_result_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_update_inspect_result_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_update_alarm_info(bRet, errMsg);
}

void LibDLHangRailRobotBackground::robot_insert_map_Data_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_insert_map_Data_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString errMsg = QString::fromStdString(val["errMsg"].asString());

	singnal_database_insert_map_Data(bRet, errMsg);
}

void LibDLHangRailRobotBackground::robot_insert_environment_result_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_delete_environment_result_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		return;
	}
}

void LibDLHangRailRobotBackground::robot_insert_or_update_restoration_value_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLHangRailRobotBackground::robot_insert_or_update_restoration_value_resp retVal = NULL");
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString errMsg = QString::fromStdString(val["errMsg"].asString());
	singnal_database_insert_or_update_restoration_value(bRet, errMsg);
}

void LibDLHangRailRobotBackground::robot_update_devices_for_relative_dev_resp(boost::shared_ptr<commonMsg> msg)
{
	Json::Value val = msg->getBodyJsonVal();
}