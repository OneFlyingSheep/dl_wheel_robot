#include <LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h>
#include "LibDLWheelRobotCoreSocket.h"
#include <QDebug>
#include <fstream>
#include "LibHCNetCamera/HikCameraPointData.h"
#include "LibDLWheelAlarmManager/AlarmMessageManager.h"

static uint16_t wheelRobotCoreSocketMsgId = 0;

static boost::asio::io_service io_service;

LibDLWheelRobotBackground::LibDLWheelRobotBackground() : voltage_(20.0)
{
    initMouseClickData();
//    initAlarmMap();

}

LibDLWheelRobotBackground::~LibDLWheelRobotBackground()
{
	if (nullptr != m_pSpeeker)
	{
		delete m_pSpeeker;
		m_pSpeeker = nullptr;
	}
}

void LibDLWheelRobotBackground::InitVoiceSpeak()
{
	if (nullptr == m_pSpeeker)
	{
		m_pSpeeker = new LibVoiceSpeak;
		m_pSpeeker->wheelRobotAlarmDisplay.connect(boost::bind(&LibDLWheelRobotBackground::robot_alarm_display_msg, this, _1));
	}
}

void LibDLWheelRobotBackground::initMouseClickData(QString fileName /*= ".\\DataCfg.txt"*/)
{
    std::fstream fin;
    fin.open(std::string(fileName.toLocal8Bit()));
    std::string line;
    int count = 1;
    getline(fin, line);
    while (getline(fin, line))
    {
        std::string pt = line.substr(line.find(" ") + 1, line.size());
        std::string xp = pt.substr(0, pt.find(" "));
        std::string yt = pt.substr(pt.find(" ") + 1, pt.size());
        tmpPtz ang;
        ang.dPan = atof(xp.c_str());
        ang.dTilt = atof(yt.c_str());
        angMap.insert(std::make_pair(count++, ang));
    }
}

void LibDLWheelRobotBackground::closeConnect()
{
    if (NULL != m_clientSocket)
        m_clientSocket->closeSocket();
}

bool LibDLWheelRobotBackground::isConnected()
{
    return m_clientSocket->IsConnected();
}

userLoginRetVal LibDLWheelRobotBackground::doLogin(std::string host, int port, std::string username, std::string password)
{
    ROS_INFO("runMessageLoop");
    userLoginRetVal retVal;
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string(host), port);

    m_clientSocket = boost::shared_ptr<CCoreClient>(new CCoreClient(endpoint, io_service, username, password));
    registerHandles();
    retVal = m_clientSocket->doLogin();
    new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
    return retVal;
}

void LibDLWheelRobotBackground::postMsg(jsonBody &msg)
{
    m_clientSocket->postMsg(msg);
}

void LibDLWheelRobotBackground::registerHandles()
{
	for (int i = Request_robot_control_stop + WHEELROBOT_PointADD; i <= Request_robot_control_ptz_abs + WHEELROBOT_PointADD; i++)
	{
		m_clientSocket->registerMsgHandle((i),boost::bind(&LibDLWheelRobotBackground::robot_ctrl_All_Resp, this, _1));
	}
	for (int i = Request_robot_control_motion + WHEELROBOT_PointADD; i <= Request_robot_control_turn + WHEELROBOT_PointADD; i++)
	{
		m_clientSocket->registerMsgHandle((i),boost::bind(&LibDLWheelRobotBackground::robot_ctrl_All_Resp, this, _1));
	}
	for (int i = Request_robot_control_slam + WHEELROBOT_PointADD; i <= Request_robot_Control_SqlFile + WHEELROBOT_PointADD; i++)
	{
		m_clientSocket->registerMsgHandle((i),boost::bind(&LibDLWheelRobotBackground::robot_ctrl_All_Resp, this, _1));
	}
	for (int i = Request_robot_task_pause + WHEELROBOT_PointADD; i <= Request_robot_task_cancel + WHEELROBOT_PointADD; i++)
	{
		m_clientSocket->registerMsgHandle((i),boost::bind(&LibDLWheelRobotBackground::robot_ctrl_All_Resp, this, _1));
	}
	m_clientSocket->registerMsgHandle((Request_robot_task_assign + WHEELROBOT_PointADD), boost::bind(&LibDLWheelRobotBackground::robot_task_assign_resp, this, _1));

	m_clientSocket->registerMsgHandle((Request_robot_config_mode + WHEELROBOT_PointADD), boost::bind(&LibDLWheelRobotBackground::robot_ctrl_All_Resp, this, _1));
	m_clientSocket->registerMsgHandle((Request_robot_config_setparams_run + WHEELROBOT_PointADD), boost::bind(&LibDLWheelRobotBackground::robot_ctrl_All_Resp, this, _1));
	m_clientSocket->registerMsgHandle((Request_robot_config_setparams_debug + WHEELROBOT_PointADD), boost::bind(&LibDLWheelRobotBackground::robot_ctrl_All_Resp, this, _1));
	m_clientSocket->registerMsgHandle((Request_robot_config_md5 + WHEELROBOT_PointADD), boost::bind(&LibDLWheelRobotBackground::robot_ctrl_All_Resp, this, _1));
    m_clientSocket->registerMsgHandle((Request_robot_config_2d_map_query + WHEELROBOT_PointADD), boost::bind(&LibDLWheelRobotBackground::robot_config_2d_map_query_resp, this, _1));
    m_clientSocket->registerMsgHandle((Request_robot_config_smap_query + WHEELROBOT_PointADD), boost::bind(&LibDLWheelRobotBackground::robot_config_smap_query_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_config_listing_area + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_config_listing_are_resp, this, _1));

    m_clientSocket->registerMsgHandle(Request_robot_status_real_time + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_status_real_time_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_status_none_real_time + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_status_none_real_time_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_status_alarm_initiative + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_status_alarm_initiative_resp, this, _1));
    

    m_clientSocket->registerMsgHandle(Request_robot_device_insert + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_device_insert_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_device_update + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_device_update_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_device_delete + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_device_delete_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_device_delete_device_type + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_device_delete_device_type_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_device_delete_interval + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_device_delete_interval_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_device_delete_voltage_level + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_device_delete_voltage_level_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_config_download2d, boost::bind(&LibDLWheelRobotBackground::robot_config_download2d_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_task_edit_insert + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_task_edit_insert_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_task_edit_update + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_task_edit_updata_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_task_edit_delete + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_task_edit_delete_resp, this, _1));

    m_clientSocket->registerMsgHandle(Request_robot_task_device_finish, boost::bind(&LibDLWheelRobotBackground::robot_task_device_finish_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_task_finish, boost::bind(&LibDLWheelRobotBackground::robot_task_finish_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_task_all_finish, boost::bind(&LibDLWheelRobotBackground::robot_task_all_finish_resp, this, _1));

    m_clientSocket->registerMsgHandle(Request_robot_current_task_infomation, boost::bind(&LibDLWheelRobotBackground::robot_task_current_task_info_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_task_point_percent, boost::bind(&LibDLWheelRobotBackground::robot_task_point_percent_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_inspect_result_res, boost::bind(&LibDLWheelRobotBackground::robot_inspect_result_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_send_compare_device_inspect_req, boost::bind(&LibDLWheelRobotBackground::robot_compare_inspect_resule_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_task_begin, boost::bind(&LibDLWheelRobotBackground::robot_task_begin, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_task_point_serial_num, boost::bind(&LibDLWheelRobotBackground::robot_task_serial, this, _1));

    
    m_clientSocket->registerMsgHandle(Request_robot_db_task_delete + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_db_task_delete_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_partrol_result_verify + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_partrol_result_verify_resp, this, _1));
	
    m_clientSocket->registerMsgHandle(Request_robot_insert_voltage_level + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_config_insert_area_map_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_insert_area + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_config_insert_area_map_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_insert_interval + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_config_insert_area_map_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_delete_voltage_level + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_config_delete_area_map_resp, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_delete_area + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_config_delete_area_map_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_delete_interval + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_config_delete_area_map_resp, this, _1));


	m_clientSocket->registerMsgHandle(Request_robot_patrol_result_audit + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_patrol_result_audit_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_fast_audit_task_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_ctrl_fast_audit_task_resp, this, _1));
	
	m_clientSocket->registerMsgHandle(Request_robot_excel_import_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_excel_import_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_updata_standard_spot_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_update_standard_patrol_vindicate_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_delete_standard_spot_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_delete_standard_patrol_vindicate_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_insert_standard_spot_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_insert_standard_patrol_vindicate_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_task_edit_import_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_task_edit_import_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_user_add_config_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_user_config_add_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_user_delete_config_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_user_config_delete_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_create_report_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_create_report_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_examine_report_isexist_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_examine_report_isexist_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_delete_patrol_point_set_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_delete_patrol_point_set_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_start_using_status_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_start_using_status_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_patrol_point_add_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_patrol_point_add_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_patrol_point_updata_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_patrol_point_updata_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_task_edit_insert_from_map_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_task_edit_insert_from_map_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_update_task_status_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_update_task_status_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_insert_note_message_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_insert_note_message_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_delete_note_message_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_delete_note_message_resp, this, _1));
	
    m_clientSocket->registerMsgHandle(Request_robot_threshold_set_by_device_uuid_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::resp_robot_insert_threshold, this, _1));
    m_clientSocket->registerMsgHandle(Request_robot_threshold_set_by_meter_type_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::resp_robot_insert_threshold, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_reconnect_2_new_robot_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_connect_2_new_robot_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_auto_relevance_device_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_auto_relevance_device_resp, this, _1));

    m_clientSocket->registerMsgHandle(Request_robot_config_uploadmap2Robot + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_config_uploadmap2Robot_resp, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_disconnect_req, boost::bind(&LibDLWheelRobotBackground::robot_connect_status, this, _1));

	m_clientSocket->registerMsgHandle(Request_robot_task_face_recognition + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_face_recog_note_message_resp, this, _1));

	
	//采集设备树回传消息
	m_clientSocket->registerMsgHandle(Request_robot_add_new_interval_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_update_interval_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_copy_paste_interval_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_delete_interval_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_add_deviceType_andDevices_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_delete_device_type_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_delete_device_type_list_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_add_devices_fromlist_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));
	m_clientSocket->registerMsgHandle(Request_robot_delete_devices_fromlist_req + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_collect_device_tree_resp, this, _1));

    m_clientSocket->registerMsgHandle(Request_robot_Control_Infrared_Take_Photo + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_control_infrared_take_photo_resp, this, _1));

    m_clientSocket->registerMsgHandle(Request_robot_update_task_table_signal_req, boost::bind(&LibDLWheelRobotBackground::robot_update_task_table_signal, this, _1));

    m_clientSocket->registerMsgHandle(Request_robot_Control_Remote_Upgrade + WHEELROBOT_PointADD, boost::bind(&LibDLWheelRobotBackground::robot_control_remote_upgrade_resp, this, _1));
    
}

void LibDLWheelRobotBackground::switch_robot_req(int robot_id)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_switch_current_robot);
	body.jsonAppendElement("robot_id", robot_id);
	postMsg(body);
}


void LibDLWheelRobotBackground::robot_face_recognition_req(std::string input_name)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_face_recognition);
	body.jsonAppendElement("input_path", input_name);
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_ctrl_All_Resp(boost::shared_ptr<roboKitMsg> msg)
{
	int ret_code;
	std::string err_msg;
	uint16_t type = msg->msgHeader.getType();
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
	}
	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		ret_code = val["ret_code"].asInt();
		err_msg = val["err_msg"].asString();
		qDebug() << "receive(true):" << type << ret_code << QString::fromStdString(err_msg.c_str());
	//	updateSendMessageStatus(msg->msgHeader.getNumber(), msg->msgHeader.getType());
	}
	else
	{
		ret_code = val["ret_code"].asInt();
		err_msg = val["err_msg"].asString();
		qDebug() << "receive(false):" << type << ret_code << QString::fromStdString(err_msg.c_str());
	//	updateSendMessageStatus(msg->msgHeader.getNumber(), msg->msgHeader.getType());
	}
}

uint16_t LibDLWheelRobotBackground::getMsgId()
{
    boost::mutex::scoped_lock lock(getMsgIdMutex);
    return wheelRobotCoreSocketMsgId++;
}

void LibDLWheelRobotBackground::robot_status_real_time_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("LibDLWheelRobotBackground::robot_status_real_time_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_status_real_time_resp retVal = NULL");
        return;
    }

    WheelRobotRealtimeStatus status;

    status.locStatus.x = val["x"].asFloat();
    status.locStatus.y = val["y"].asFloat();
    status.locStatus.angle = val["angle"].asFloat();
    status.locStatus.confidence = val["confidence"].asFloat();

    wheelRobotCurrentLoc2CollectMapTable(status.locStatus.x, -status.locStatus.y, -status.locStatus.angle);
//    wheelRobotCurrentLoc2CollectMapTable(status.locStatus.x, status.locStatus.y, -status.locStatus.angle);

    status.moveStatus.vx = val["vx"].asFloat();
    status.moveStatus.vy = val["vy"].asFloat();
    status.moveStatus.w = val["w"].asFloat();

    /*status.ptzStatus.pan = val["pan"].asFloat();
    status.ptzStatus.tilt = val["tilt"].asFloat();

    m_currPtz.pan = status.ptzStatus.pan;
    m_currPtz.tilt = status.ptzStatus.tilt;*/

    status.blockStatus.blocked = val["blocked"].asBool();
    if (status.blockStatus.blocked)
    {
        status.blockStatus.block_reason = val["block_reason"].asFloat();
        status.blockStatus.block_x = val["block_x"].asFloat();
        status.blockStatus.block_y = val["block_y"].asFloat();
    }

    status.batteryStatus.battery_level = val["battery_level"].asFloat();
    status.batteryStatus.battery_temp = val["battery_temp"].asFloat();
    status.batteryStatus.charging = val["charging"].asBool();
    status.batteryStatus.voltage = val["voltage"].asFloat();
    status.batteryStatus.current = val["current"].asFloat();

    status.wheelVel.vLeft = val["vLeft"].asFloat();
    status.wheelVel.vRight = val["vRight"].asFloat();

    status.point_id = val["point_id"].asInt();
    status.brake = val["brake"].asBool();
    status.emergency = val["emergency"].asBool();

    status.charge_voltage = val["charge_voltage"].asFloat();
	
	status.urgency_type = val["urgency_type"].asInt();
    status.infrared_focus = val["infrared_focus"].asInt();

	//cdk_robot
	status.firePtzStatus.pan	= val["fire_platform_pan"].asFloat();
	status.firePtzStatus.tilt	= val["fire_platform_tilt"].asFloat();

	status.doubleLightPtzStatus.pan = val["doulbe_light_platform_pan"].asFloat();
	status.doubleLightPtzStatus.tilt = val["doulbe_light_platform_tilt"].asFloat();

	m_currPtz.pan = status.doubleLightPtzStatus.pan;
	m_currPtz.tilt = status.doubleLightPtzStatus.tilt;
	
	status.visibleCameraStatus.focus = val["visible_camera_focus"].asInt();
	status.visibleCameraStatus.zoom = val["visible_camera_zoom"].asInt();

	status.infraredCameraStatus.focus = val["infrared_camera_focus"].asInt();
	status.infraredCameraStatus.zoom = val["infrared_camera_zoom"].asInt();

	status.fireExtinguisherStatusLeft.dev_switch = val["fire_extinguisher_switch_left"].asInt();
	status.fireExtinguisherStatusLeft.pressure = val["fire_extinguisher_pressure_left"].asInt();

	status.fireExtinguisherStatusRight.dev_switch = val["fire_extinguisher_switch_right"].asInt();
	status.fireExtinguisherStatusRight.pressure = val["fire_extinguisher_pressure_right"].asInt();

	status.robotDiskInfo.usage = val["disk_usage"].asFloat();
	status.robotDiskInfo.total = val["disk_total"].asFloat();

    wheelRobotRealtimeStatus(status);

    wheelRobotInfraredFocusSignal(status.infrared_focus);
    if (status.locStatus.confidence < 0.3)
    {
        if (nullptr != m_pSpeeker)
        {
            ROS_INFO("status.locStatus.confidence: %f", status.locStatus.confidence);
            AlarmMesgErrorCode stAlarmMesgErrorCode;
            stAlarmMesgErrorCode.errorId = CLIENT_ALARM_LOCATION_RELIABILITY;
            //stAlarmMesgErrorCode.errorCode = QString::number(status.locStatus.confidence, 'f', 2);
            m_pSpeeker->addSpeakErrorCode(stAlarmMesgErrorCode);
        }
    }
    else
    {
        if (nullptr != m_pSpeeker)
        {
            ROS_INFO("status.locStatus.confidence: %f", status.locStatus.confidence);
            AlarmMesgErrorCode stAlarmMesgErrorCode;
            stAlarmMesgErrorCode.errorId = CLIENT_ALARM_LOCATION_RELIABILITY;
            //stAlarmMesgErrorCode.errorCode = QString::number(status.locStatus.confidence, 'f', 2);
            m_pSpeeker->removeSpeakErrorCode(stAlarmMesgErrorCode);
        }
    }

	if (status.batteryStatus.battery_level * 100 < voltage_)
    {
		if (nullptr != m_pSpeeker)
		{
			AlarmMesgErrorCode stAlarmMesgErrorCode;
			stAlarmMesgErrorCode.errorId = CLIENT_ALARM_BATTERY_POWER;
			stAlarmMesgErrorCode.errorCode = QString::number(status.batteryStatus.battery_level * 100);
			m_pSpeeker->addSpeakErrorCode(stAlarmMesgErrorCode);
		}
    }
    else
    {
        if (nullptr != m_pSpeeker)
        {
            AlarmMesgErrorCode stAlarmMesgErrorCode;
            stAlarmMesgErrorCode.errorId = CLIENT_ALARM_BATTERY_POWER;
            //stAlarmMesgErrorCode.errorCode = QString::number(status.batteryStatus.battery_level * 100);
            m_pSpeeker->removeSpeakErrorCode(stAlarmMesgErrorCode);
        }
    }

    int ret_code;
    std::string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        ROS_INFO("LibDLWheelRobotBackground::robot_status_real_time_resp");
        return;
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_status_real_time_resp error : code:%d, msg:%s", ret_code, err_msg.c_str());
    }
}

void LibDLWheelRobotBackground::robot_status_none_real_time_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("LibDLWheelRobotBackground::robot_status_none_real_time_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_status_none_real_time_resp retVal = NULL");
        return;
    }

    WheelRobotNoneRealtimeStatus status;

    status.odo = val["odo"].asFloat();
    status.time = val["time"].asFloat();
    status.total_time = val["total_time"].asFloat();
    status.controller_temp = val["controller_temp"].asFloat();
    status.controller_humi = val["controller_humi"].asFloat();
    status.controller_voltage = val["controller_voltage"].asFloat();
    status.status = (WheelRobotCurrentStatus)val["status"].asInt();

    Json::Value::iterator areaIdItr = val["area_ids"].begin();

    for (; areaIdItr != val["area_ids"].end(); areaIdItr++)
    {
        status.area_ids.push_back(QString::fromStdString(areaIdItr->asString()));
    }


    Json::Value laser_array = val["laser_beams"];
    std::vector<QPointF> laser_beams;
    laser_beams.clear();

    for (int i = 0; i < laser_array.size(); ++i)
    {
        double x = laser_array[i][0].asDouble();
        double y = laser_array[i][1].asDouble();
        QPointF pos;
        pos.setX(x);
        pos.setY(y);
        laser_beams.push_back(pos);
    }

    status.cpu_temp = val["cpu_temp"].asFloat();

    WheelRobotSwitchRunningStatus robotMode = (WheelRobotSwitchRunningStatus)val["mode"].asInt();
    wheelRobotSwitchRunningStatusSignal(robotMode);
    
    wheelRobotNoneRealtimeStatus(status);

    wheelRobotLaserData2CollectMapTable(laser_beams);

    if (status.controller_humi > 100.0 || status.controller_humi < -100.0)
    {
        QString retMsg = QString("控制器湿度异常：%1").arg(QString::number(status.controller_humi, 'f', 2));

		if (nullptr != m_pSpeeker)
		{
			AlarmMesgErrorCode stAlarmMesgErrorCode;
			stAlarmMesgErrorCode.errorId = CLIENT_ALARM_CONTROL_HUMIDTY_ANOMALY;
			stAlarmMesgErrorCode.errorCode = QString::number(status.controller_humi, 'f', 2);
			m_pSpeeker->addSpeakErrorCode(stAlarmMesgErrorCode);
		}
        //speeker_.add2SpeakList("控制器 湿度 异常");
       // wheelRobotSystemWarningCallback(retMsg);
    }
    else
    {
        if (nullptr != m_pSpeeker)
        {
            AlarmMesgErrorCode stAlarmMesgErrorCode;
            stAlarmMesgErrorCode.errorId = CLIENT_ALARM_CONTROL_HUMIDTY_ANOMALY;
            //stAlarmMesgErrorCode.errorCode = QString::number(status.controller_humi, 'f', 2);
            m_pSpeeker->removeSpeakErrorCode(stAlarmMesgErrorCode);
        }
    }

    if (status.controller_temp > 60.0 || status.controller_temp < -30.0)
    {
        QString retMsg = QString("控制器温度异常：%1").arg(QString::number(status.controller_temp, 'f', 2));
        //speeker_.add2SpeakList("控制器 温度 异常");
		if (nullptr != m_pSpeeker)
		{
			AlarmMesgErrorCode stAlarmMesgErrorCode;
			stAlarmMesgErrorCode.errorId = CLIENT_ALARM_CONTROL_T_ANOMALY;
			stAlarmMesgErrorCode.errorCode = QString::number(status.controller_temp, 'f', 2);
			m_pSpeeker->addSpeakErrorCode(stAlarmMesgErrorCode);
		}

        //wheelRobotSystemWarningCallback(retMsg);
    }
    else
    {
        if (nullptr != m_pSpeeker)
        {
            AlarmMesgErrorCode stAlarmMesgErrorCode;
            stAlarmMesgErrorCode.errorId = CLIENT_ALARM_CONTROL_T_ANOMALY;
            m_pSpeeker->removeSpeakErrorCode(stAlarmMesgErrorCode);
        }
    }

//     if (laser_beams.size() == 0)
//     {
//         QString msg = QString("激光数据异常");
//         wheelRobotSystemWarningCallback(msg);
//         speeker_.add2SpeakList("激光数据 异常");
//     }
/*
	Json::Value alarmRoot = val["alarm_code"];
	QVector<int> alarmData;
	for (int i = 0; i < alarmRoot.size(); i++)
	{
		int alarmCode = alarmRoot[i].asInt();
		if (alarmData.indexOf(alarmCode) == -1)
		{
			alarmData.push_back(alarmCode);
			//            std::string errMsg = getAlarmCode(alarmCode);
			//             wheelRobotSystemWarningCallback(QString::fromStdString(errMsg));
			//             speeker_.add2SpeakList(QString::fromStdString(errMsg));
		}
	}

	for (int i = 0; i < alarmData.size(); i++)
	{
		// 		if (m_alarmData.indexOf(alarmData[i]) == -1)
		// 		{
		//			m_alarmData.append(alarmData[i]);
		std::string errMsg = getAlarmCode(alarmData[i]);
		wheelRobotSystemWarningCallback(QString::fromStdString(errMsg));
		//添加语音
		if (nullptr != m_pSpeeker)
		{
			AlarmMesgErrorCode stAlarmMesgErrorCode;
			stAlarmMesgErrorCode.errorId = alarmData[i];
			stAlarmMesgErrorCode.errorCode = "";
			m_pSpeeker->addSpeakErrorCode(stAlarmMesgErrorCode);

		}
		//speeker_.add2SpeakList(QString::fromStdString(errMsg));
		//		}
	}

	for (int i = 0; i < m_alarmData.size(); i++)
	{
		if (alarmData.indexOf(m_alarmData[i]) == -1)
		{
			m_alarmData.removeAt(i);
		}
	}
    */
	//     if (alarmData.size() > 0)
	//     {
	//         wheelRobotHardwareAlarmCode(alarmData);
	//         speeker_.add2SpeakList("硬件 异常");
	//     }

	int ret_code;
	std::string err_msg;

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		ROS_INFO("no error");
		return;
	}

	ret_code = val["ret_code"].asInt();
	err_msg = val["err_msg"].asString();

	if (0 == ret_code)
	{
		ROS_INFO("LibDLWheelRobotBackground::robot_status_none_real_time_resp");
		return;
	}
	else
	{
		ROS_ERROR("LibDLWheelRobotBackground::robot_status_none_real_time_resp error : code:%d, msg:%s", ret_code, err_msg.c_str());
	}
}

void LibDLWheelRobotBackground::robot_status_alarm_initiative_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("LibDLWheelRobotBackground::robot_status_alarm_initiative_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_status_alarm_initiative_resp retVal = NULL");
        return;
    }
    QVector<AlarmMesgErrorCode> errorVec;
    
    Json::Value errVal = val["warnMsgs"];
    for (int i = 0; i < errVal.size(); i++)
    {
		AlarmMesgErrorCode error;
        error.errorId = errVal[i]["errorId"].asInt();
        error.errorCode = QString::fromStdString(errVal[i]["errorCode"].asString());
        if (error.errorCode == "0")
        {
            error.errorCode = "";
        }
        errorVec.append(error);


		/*AlarmMessage stAlarmMessage;
		AlarmMessageManager::GetInstance()->GetAlarmMessage(error.errorId, stAlarmMessage);
		QString strDisplayMsg = QString("%1%2").arg(stAlarmMessage.strDisplayAlarmMessage.c_str()).arg(error.errorCode);

		if (!strDisplayMsg.isEmpty())
		{
			wheelRobotSystemWarningCallback(strDisplayMsg);
		}*/

    }

	if (nullptr != m_pSpeeker)
	{
		m_pSpeeker->addSpeakErrorCode(errorVec);
	}

//    wheelRobotAlarmStatusSignal(errorVec);
}

// void LibDLWheelRobotBackground::robot_printf_error_msg()
// {
// 
// }

void LibDLWheelRobotBackground::robot_control_stop_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_stop);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_gyrocal_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_gyrocal);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_reloc_req(float x, float y, float angle)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_reloc);

	body.jsonAppendElement("x", x);
	body.jsonAppendElement("y", -y);
	body.jsonAppendElement("angle", -angle);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_comfirmloc_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_comfirmloc);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_motion_req(float vx, float vy, float w)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_motion);

	body.jsonAppendElement("vx", vx);
	body.jsonAppendElement("vy", vy);
	body.jsonAppendElement("w", w);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_gotarget_req(std::string id, float angle)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_gotarget);

	body.jsonAppendElement("id", atoi(id.c_str()));
	//body.jsonAppendElement("angle", angle);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_translate_req(float dist, float vx, int mode)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_translate);

	body.jsonAppendElement("dist", dist);
	body.jsonAppendElement("vx", vx);
	body.jsonAppendElement("mode", mode);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_turn_req(float angle, float vw, int mode)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_turn);

	body.jsonAppendElement("angle", angle);
	body.jsonAppendElement("vw", vw);
	body.jsonAppendElement("mode", mode);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_slam_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_slam);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_endslam_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_endslam);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_loadmap_req(std::string map_name)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_loadmap);

	body.jsonAppendElement("map_name", map_name);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_loadmapobj_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_loadmapobj);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_update_device_req(QString deviceFileName)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_uploadSql);

    body.jsonAppendElement("sql_name", "device_parameter.txt");

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_back_to_charge()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_control_back_to_charge);

    postMsg(body);
}

//./////////////////////////////////////////////////////////////////////
void LibDLWheelRobotBackground::robot_control_ptz_motion_req(WheelRobotPtzMoveType type, int ptz_id, float speed)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_ptz_motion);

    body.jsonAppendElement("type", (int)type);
    body.jsonAppendElement("speed", speed);
	body.jsonAppendElement("ptz_id", ptz_id);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_ptz_abs_req(int pan, int tilt)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_ptz_abs);

    body.jsonAppendElement("pan", pan);
    body.jsonAppendElement("tilt", tilt);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_ptz_relative_req(int offset_pan, int offset_tilt)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_ptz_relative);

	//body.jsonAppendElement("type", (int)type);
	body.jsonAppendElement("offset_pan", offset_pan);
	body.jsonAppendElement("offset_tilt", offset_tilt);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_ptz_monodrome_req(int pt_value, int pt_type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_control_ptz_monodrome);

    body.jsonAppendElement("pt_value", pt_value);
    body.jsonAppendElement("pt_type", pt_type);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_device_ctrl_req(bool ultraSonic, bool bWiper, bool bAutoDoor, bool bChargerArm, bool bLedLamp, bool bWarnLamp, bool bFireExtinguisher)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_control_device_ctrl);

    body.jsonAppendElement("ultra_sonic", ultraSonic);
    body.jsonAppendElement("wiper", bWiper);
    body.jsonAppendElement("auto_door", bAutoDoor);
    body.jsonAppendElement("charger_arm", bChargerArm);

	body.jsonAppendElement("led_lamp", bLedLamp);
	body.jsonAppendElement("warn_lamp", bWarnLamp);
	body.jsonAppendElement("fire_extinguisher", bFireExtinguisher);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_ptz_light_req(bool bPtzLight)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_control_device_ctrl);

    body.jsonAppendElement("light", bPtzLight);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_ptz_wiper_req(bool bPtzWiper)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_control_device_ctrl);

    body.jsonAppendElement("wiper", bPtzWiper);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_utral_sonic_req(bool bUtralSonic)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_control_device_ctrl);

    body.jsonAppendElement("utral_sonic", bUtralSonic);

    postMsg(body);
}
///////////////////////////////////////////////////////////////////////
void LibDLWheelRobotBackground::robot_task_pause_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_pause);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_resume_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_resume);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_cancel_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_cancel);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_assign_req(WheelRobotAssignTask task)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_assign);

	Json::Value dev;

	for (int i = 0; i < task.devices.size(); i++)
	{
		dev[i] = task.devices[i];
	}

	body.jsonAppendElement("task_uuid", task.task_uuid.toStdString().c_str());
    body.jsonAppendElement("task_name", task.task_name.toStdString().c_str());
    body.jsonAppendElement("task_edit_uuid", task.task_edit_uuid.toStdString().c_str());
    body.jsonAppendElement("task_template_uuid", task.task_template_uuid.toStdString().c_str());
	body.jsonAppendElement("priority", task.priority);
	body.jsonAppendElement("devices", dev);
	body.jsonAppendElement("dev_optimize", task.dev_optimize);
	body.jsonAppendElement("task_end_action", (int)task.task_end_action);
	body.jsonAppendElement("breakTask", task.breakTask);
	body.jsonAppendElement("return", 1);
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_delete_req(std::string uuid)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_delete);

	body.jsonAppendElement("uuid", uuid);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_query_curr_task_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_query_curr_task);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_pause_resp(boost::shared_ptr<roboKitMsg> msg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_task_pause);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_resume_resp(boost::shared_ptr<roboKitMsg> msg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_task_resume);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_cancel_resp(boost::shared_ptr<roboKitMsg> msg)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_task_cancel);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_assign_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    std::vector<std::string> devices;
    Json::Value::iterator devItr = val["devices"].begin();
    for (; devItr != val["devices"].end(); devItr++)
    {
        devices.push_back(devItr->asString());
    }

    std::string err_msg;
    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        return;
    }
}

void LibDLWheelRobotBackground::robot_task_delete_resp(boost::shared_ptr<roboKitMsg> msg)
{

}

void LibDLWheelRobotBackground::robot_task_query_curr_task_resp(boost::shared_ptr<roboKitMsg> msg)
{

}

//void LibDLWheelRobotBackground::robot_status_alarm_resp(boost::shared_ptr<roboKitMsg> msg)
//{
//	Json::Value val = msg->msgBody.getJsonVal();
//	if (val.isNull())
//	{
//		return;
//	}
//
//	QString strList = val["ids"].asString().c_str();
//
//	QStringList lstIDs = strList.split(",");
//
//}

void LibDLWheelRobotBackground::robot_task_current_task_info_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    WheelRobotCurrentTaskInfoShow task;
    task.task_uuid = QString::fromLocal8Bit(val["task_uuid"].asString().c_str());
    task.task_name = QString::fromLocal8Bit(val["task_name"].asString().c_str());
    task.predict_duration = val["predict_duration"].asInt();
    task.current_device_uuid = QString::fromLocal8Bit(val["current_device_uuid"].asString().c_str());
    task.current_device_name = QString::fromLocal8Bit(val["current_device_name"].asString().c_str());
    task.total_devices = val["total_devices"].asInt();
    task.checked_devices = val["checked_devices"].asInt();
    task.percent = val["percent"].asFloat();
    task.alarmDeviceCount = val["alarmDeviceCount"].asInt();
    task.task_property = QString::fromLocal8Bit(val["task_property"].asString().c_str());

    // func
    wheelRobotCurrentTaskInfoCallback(task);
}

void LibDLWheelRobotBackground::robot_task_point_percent_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }
    WheelRobotTaskCurrentPointStatus status;
    status.start_point = val["start_point"].asInt();
    status.end_point = val["end_point"].asInt();

    status.percent = val["percent"].asFloat();

    wheelRobotCurrentPointStatusCallback(status);
}

void LibDLWheelRobotBackground::robot_task_begin(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    WheelRobotTaskBegin taskBegin;

    taskBegin.task_uuid = QString::fromStdString(val["task_uuid"].asString());
    taskBegin.start_time = QString::fromStdString(val["start_time"].asString());
    taskBegin.predict_duration = val["predict_duration"].asInt();

    Json::Value::iterator pointItr = val["points"].begin();
    for (; pointItr != val["points"].end(); pointItr++)
    {
        taskBegin.points.push_back(QString::fromStdString(pointItr->asString()));
    }

    signal_wheelRobotTaskBegin(taskBegin);
}

void LibDLWheelRobotBackground::robot_task_serial(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    int serial_num = val["serial_num"].asInt();

    wheelRobotUpdateTaskSerialNum(serial_num);
}

////////////////////////////////////////////////////////////////////////
void LibDLWheelRobotBackground::robot_config_mode_req(WheelRobotSwitchRunningStatus type)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_config_mode);

	body.jsonAppendElement("type", type);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_uploadmap_req(QString map_name)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_uploadmap2Robot);

    body.jsonAppendElement("map_name", std::string(map_name.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_downloadmap_req(QString map_name)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_downloadmap);

    body.jsonAppendElement("map_name", std::string(map_name.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_removemap_req(QString map_name)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_removemap);

    body.jsonAppendElement("map_name", std::string(map_name.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_download2d_req()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_download2d);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_2d_map_query_req()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_2d_map_query);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_smap_query_req()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_smap_query);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_setparams_run_req(WheelRobotRunningParameter p)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_config_setparams_run);

	body.jsonAppendElement("max_vel_x", p.max_vel_x);
	body.jsonAppendElement("min_vel_x", p.min_vel_x);
	body.jsonAppendElement("max_vel_y", p.max_vel_y);
	body.jsonAppendElement("min_vel_y", p.min_vel_y);
	body.jsonAppendElement("max_rot_vel", p.max_rot_vel);
	body.jsonAppendElement("min_rot_vel", p.min_rot_vel);
	body.jsonAppendElement("acc_lim_x", p.acc_lim_x);
	body.jsonAppendElement("acc_lim_y", p.acc_lim_y);
	body.jsonAppendElement("acc_lim_theta", p.acc_lim_theta);
    body.jsonAppendElement("stop_dist", p.warn_dist);
    body.jsonAppendElement("battery_low", p.stop_dist);
	body.jsonAppendElement("stop_dist", p.battery_high);
	body.jsonAppendElement("battery_low", p.battery_low);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_setparams_debug_req(WheelRobotDebugParameter p)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_config_setparams_debug);

	body.jsonAppendElement("xy_goal_tolerance", p.xy_goal_tolerance);
	body.jsonAppendElement("yaw_goal_tolerance", p.yaw_goal_tolerance);
	body.jsonAppendElement("sim_time", p.sim_time);
	body.jsonAppendElement("trans_stopped_vel", p.trans_stopped_vel);
	body.jsonAppendElement("rot_stopped_vel", p.rot_stopped_vel);
	body.jsonAppendElement("forward_point_distance", p.forward_point_distance);
	body.jsonAppendElement("path_distance_bias", p.path_distance_bias);
	body.jsonAppendElement("goal_distance_bias", p.goal_distance_bias);
	body.jsonAppendElement("vx_sample", p.vx_sample);
	body.jsonAppendElement("vy_sample", p.vy_sample);
	body.jsonAppendElement("vth_sample", p.vth_sample);
	body.jsonAppendElement("battery_high", p.battery_high);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_md5_req(std::string md5)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_config_md5);

	body.jsonAppendElement("md5", md5);

	postMsg(body);
}

bool LibDLWheelRobotBackground::robot_config_battery_threshold_req(float top, float buttom)
{
    if ((top - buttom) < 0.0001)
    {
        ROS_ERROR("阈值上下限设置值不符合要求：top < buttom");
        return false;
    }

    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_setparams_run);

    if (top > 0.0001)
    {
        body.jsonAppendElement("battery_high", top);
        
    }

    if (buttom > 0.0001)
    {
        body.jsonAppendElement("battery_low", buttom);
    }

    return true;
}

void LibDLWheelRobotBackground::robot_config_warning_oper_req(RobotOperationAfterWarning type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_warning_oper);

    body.jsonAppendElement("type", (int)type);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_disconnect_oper_req(RobotOperationAfterDisconnect type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_disconnect_oper);

    body.jsonAppendElement("type", (int)type);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_ptz_init_req(int x, int y, int x_offset, int y_offset)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_ptz_init);

    body.jsonAppendElement("x", x);
    body.jsonAppendElement("y", y);
    body.jsonAppendElement("x_offset", x_offset);
    body.jsonAppendElement("y_offset", y_offset);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_listing_area_req(QVector<WheelRobotPoint> listPoints)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_listing_area);

    Json::Value root;
    for (int i = 0; i < listPoints.size(); i++)
    {
        Json::Value tmpPoint;
        tmpPoint.append(listPoints[i].x);
        tmpPoint.append(listPoints[i].y);
        root.append(tmpPoint);
    }

    body.jsonAppendElement("blocked_segment", root);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_download2d_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("LibDLWheelRobotBackground::robot_config_download2d_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_config_download2d_resp retVal = NULL");
        return;
    }

    std::string map_path = val["map_path"].asString();

    int ret_code;
    std::string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        // rcf get robot map
        ROS_INFO("LibDLWheelRobotBackground::robot_config_download2d_resp, map_path:%s", map_path.c_str());
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_config_download2d_resp error : code:%d, msg:%s", ret_code, err_msg.c_str());
    }
}

void LibDLWheelRobotBackground::robot_config_upload2d_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("LibDLWheelRobotBackground::robot_config_upload2d_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_config_upload2d_resp retVal = NULL");
        return;
    }

    std::string map_name = val["map_name"].asString();

    int ret_code;
    std::string err_msg;

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        wheelRobotSlam2DMapFinish(QString::fromStdString(map_name));
        //wheelRobotSlam2DMapFinish(QString::fromStdString(map_name));
        ROS_INFO("LibDLWheelRobotBackground::robot_config_upload2d_resp, map_path:%s", map_name.c_str());
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_config_upload2d_resp error : code:%d, msg:%s", ret_code, err_msg.c_str());
    }
}

void LibDLWheelRobotBackground::robot_config_2d_map_query_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotBackground::robot_config_2d_map_query_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_config_2d_map_query_resp retVal = NULL");
        return;
    }

    QStringList mapList;

    Json::Value::iterator mapItr = val["map_name"].begin();

    for (; mapItr != val["map_name"].end(); mapItr++)
    {
        mapList.push_back(QString::fromStdString(mapItr->asString()));
    }
    
    int ret_code;
    std::string err_msg;

//     if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
//     {
//         ROS_INFO("no error");
//         return;
//     }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        wheelRobotQueryMapList(mapList);
        ROS_INFO("LibDLWheelRobotBackground::robot_config_2d_map_query_resp, map_size:%d", mapList.size());
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_config_2d_map_query_resp error : code:%d, msg:%s", ret_code, err_msg.c_str());
    }
}

void LibDLWheelRobotBackground::robot_config_smap_query_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotBackground::robot_config_smap_query_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_config_smap_query_resp retVal = NULL");
        return;
    }

    QStringList mapList;

    Json::Value::iterator mapItr = val["smap_name"].begin();

    for (; mapItr != val["smap_name"].end(); mapItr++)
    {
        mapList.push_back(QString::fromStdString(mapItr->asString()));
    }

    int ret_code;
    std::string err_msg;

    //     if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    //     {
    //         ROS_INFO("no error");
    //         return;
    //     }

    ret_code = val["ret_code"].asInt();
    err_msg = val["err_msg"].asString();


    if (0 == ret_code)
    {
        wheelRobotQuerySMapList(mapList);
        ROS_INFO("LibDLWheelRobotBackground::robot_config_smap_query_resp, map_size:%d", mapList.size());
    }
    else
    {
        ROS_ERROR("LibDLWheelRobotBackground::robot_config_smap_query_resp error : code:%d, msg:%s", ret_code, err_msg.c_str());
    }
}

void LibDLWheelRobotBackground::robot_config_listing_are_resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotBackground::robot_config_listing_are_resp==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    WheelRobotTaskBegin taskBegin;

    taskBegin.task_uuid = QString::fromStdString(val["task_uuid"].asString());
    taskBegin.start_time = QString::fromStdString(val["update_time"].asString());
    taskBegin.predict_duration = val["update_duration"].asInt();

    Json::Value::iterator pointItr = val["update_points"].begin();
    for (; pointItr != val["update_points"].end(); pointItr++)
    {
        taskBegin.points.push_back(QString::fromStdString(pointItr->asString()));
    }

    signal_wheelRobotTaskBegin(taskBegin);
}

void LibDLWheelRobotBackground::robot_task_edit_insert_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_edit_insert);

	body.jsonAppendElement("task_edit_uuid", m_wheelTaskEditStru.task_edit_uuid.toStdString());
	body.jsonAppendElement("task_edit_name", m_wheelTaskEditStru.task_edit_name.toStdString());
	body.jsonAppendElement("task_edit_date", m_wheelTaskEditStru.task_edit_date.toStdString());
	body.jsonAppendElement("task_edit_type_id", (int)m_wheelTaskEditStru.task_edit_type_id);

	Json::Value dev;
	for (int i = 0; i < device_uuid.size(); i++)
	{
		dev[i] = device_uuid[i].toStdString();
	}
	body.jsonAppendElement("device_uuid", dev);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_edit_updata_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_edit_update);

	body.jsonAppendElement("task_edit_uuid", m_wheelTaskEditStru.task_edit_uuid.toStdString());
	body.jsonAppendElement("task_edit_name", m_wheelTaskEditStru.task_edit_name.toStdString());
	body.jsonAppendElement("task_edit_date", m_wheelTaskEditStru.task_edit_date.toStdString());
	body.jsonAppendElement("task_edit_type_id", (int)m_wheelTaskEditStru.task_edit_type_id);

	Json::Value dev;
	for (int i = 0; i < device_uuid.size(); i++)
	{
		dev[i] = device_uuid[i].toStdString();
	}
	body.jsonAppendElement("device_uuid", dev);
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_edit_delete_req(std::string m_task_edit_uuid, WheelTaskAdminType edit_task_type_id)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_edit_delete);

	body.jsonAppendElement("task_edit_uuid", m_task_edit_uuid);
	body.jsonAppendElement("edit_task_type_id", (int)edit_task_type_id);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_partrol_result_verify_req(DeviceAlarmSearchStruct m_result, int choose)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_partrol_result_verify);

	body.jsonAppendElement("task_uuid", m_result.task_uuid.toStdString());
	body.jsonAppendElement("device_uuid", m_result.device_uuid.toStdString());
	body.jsonAppendElement("inspect_status_id", (int)m_result.inspect_status_id);
	body.jsonAppendElement("is_dealed", m_result.is_dealed);
	body.jsonAppendElement("alarm_level_id", WHEEL_DEVICE_CONFIG.getWheelAlarmLevelId(m_result.alarm_level_name));
	body.jsonAppendElement("deal_info_uuid", m_result.deal_uuid.toStdString());
	body.jsonAppendElement("dealed_info", m_result.deal_cause.toStdString());
	body.jsonAppendElement("dealed_result", m_result.deal_result.toStdString());
	body.jsonAppendElement("dealed_status_id", (int)m_result.deal_status_id);

	body.jsonAppendElement("choose", choose);
    postMsg(body);
}
// void LibDLWheelRobotBackground::robot_result_batch_verify_req(WheelRobortResultbatchVerifyStruct m_resultBatchVerify)
// {
// 	jsonBody body;
// 	body.setMsgNumber(getMsgId());
// 	body.setMsgType((int)Request_robot_result_batch_verify);
// 
// 	body.jsonAppendElement("task_uuid", m_resultBatchVerify.task_uuid.toStdString());
// 	body.jsonAppendElement("device_uuid", m_resultBatchVerify.device_uuid.toStdString());
// 	body.jsonAppendElement("is_dealed", m_resultBatchVerify.is_dealed);
// 	body.jsonAppendElement("deal_info_uuid", m_resultBatchVerify.wheelRobortDealInfoStru.deal_info_uuid.toStdString());
// 	body.jsonAppendElement("dealed_info", m_resultBatchVerify.wheelRobortDealInfoStru.dealed_info.toStdString());
// 	body.jsonAppendElement("dealed_result", m_resultBatchVerify.wheelRobortDealInfoStru.dealed_result.toStdString());
// 	body.jsonAppendElement("dealed_status_id", m_resultBatchVerify.wheelRobortDealInfoStru.dealed_status_id);
// 	body.jsonAppendElement("dealed_user", m_resultBatchVerify.wheelRobortDealInfoStru.dealed_user.toStdString());
// 	body.jsonAppendElement("dealed_finish", m_resultBatchVerify.dealed_finish);
// 
// 	postMsg(body);
// }

// void LibDLWheelRobotBackground::robot_threshold_insert_req(WheelThresholdStruct m_wheelThresholdInsert)
// {
// 	jsonBody body;
// 	body.setMsgNumber(getMsgId());
// 	body.setMsgType((int)Request_robot_threshold_insert);
// 
// 	body.jsonAppendElement("threshold_filename", m_wheelThresholdInsert.threshold_filename.toStdString());
// 	body.jsonAppendElement("device_uuid", m_wheelThresholdInsert.device_uuid.toStdString());
// 	body.jsonAppendElement("thre_earlywarn_bottom", m_wheelThresholdInsert.thre_earlywarn_bottom);
// 	body.jsonAppendElement("thre_earlywarn_top", m_wheelThresholdInsert.thre_earlywarn_top);
// 	body.jsonAppendElement("thre_generalwarn_bottom", m_wheelThresholdInsert.thre_generalwarn_bottom);
// 	body.jsonAppendElement("thre_generalwarn_top", m_wheelThresholdInsert.thre_generalwarn_top);
// 	body.jsonAppendElement("thre_severitywarn_bottom", m_wheelThresholdInsert.thre_severitywarn_bottom);
// 	body.jsonAppendElement("thre_severitywarn_top", m_wheelThresholdInsert.thre_severitywarn_top);
// 	body.jsonAppendElement("thre_dangerwarn_bottom", m_wheelThresholdInsert.thre_dangerwarn_bottom);
// 	body.jsonAppendElement("thre_dangerwarn_top", m_wheelThresholdInsert.thre_dangerwarn_top);
// 
// 	postMsg(body);
// }
// 
// void LibDLWheelRobotBackground::robot_threshold_updata_req(WheelThresholdStruct m_wheelThresholdInsert)
// {
// 	jsonBody body;
// 	body.setMsgNumber(getMsgId());
// 	body.setMsgType((int)Request_robot_threshold_update);
// 
// 	body.jsonAppendElement("threshold_filename", m_wheelThresholdInsert.threshold_filename.toStdString());
// 	body.jsonAppendElement("device_uuid", m_wheelThresholdInsert.device_uuid.toStdString());
// 	body.jsonAppendElement("thre_earlywarn_bottom", m_wheelThresholdInsert.thre_earlywarn_bottom);
// 	body.jsonAppendElement("thre_earlywarn_top", m_wheelThresholdInsert.thre_earlywarn_top);
// 	body.jsonAppendElement("thre_generalwarn_bottom", m_wheelThresholdInsert.thre_generalwarn_bottom);
// 	body.jsonAppendElement("thre_generalwarn_top", m_wheelThresholdInsert.thre_generalwarn_top);
// 	body.jsonAppendElement("thre_severitywarn_bottom", m_wheelThresholdInsert.thre_severitywarn_bottom);
// 	body.jsonAppendElement("thre_severitywarn_top", m_wheelThresholdInsert.thre_severitywarn_top);
// 	body.jsonAppendElement("thre_dangerwarn_bottom", m_wheelThresholdInsert.thre_dangerwarn_bottom);
// 	body.jsonAppendElement("thre_dangerwarn_top", m_wheelThresholdInsert.thre_dangerwarn_top);
// 
// 	postMsg(body);
// }

void LibDLWheelRobotBackground::robot_device_insert_req(WheelRobotInsertDeviceStruct dev)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_insert);

    body.jsonAppendElement("device_uuid", dev.device.device_uuid.toStdString());
    body.jsonAppendElement("voltage_level_id", dev.device.voltage_level_id.toStdString());
    body.jsonAppendElement("equipment_interval_uuid", dev.device.equipment_interval_uuid.toStdString());
    body.jsonAppendElement("device_area_uuid", dev.device.device_area_uuid.toStdString());
    body.jsonAppendElement("device_type_uuid", dev.device.device_type_uuid.toStdString());
    body.jsonAppendElement("sub_device_type_uuid", dev.device.sub_device_type_uuid.toStdString());
    body.jsonAppendElement("device_point_type_uuid", dev.device.device_point_type_uuid.toStdString());
    body.jsonAppendElement("unit_type_uuid", dev.device.unit_type_uuid.toStdString());
    body.jsonAppendElement("recognition_type_id", dev.device.recognition_type_id);
    body.jsonAppendElement("save_type_id", dev.device.save_type_id);
    body.jsonAppendElement("meter_type_id", dev.device.meter_type_id);
    body.jsonAppendElement("fever_type_id", dev.device.fever_type_id);
    body.jsonAppendElement("threshold_filename", dev.device.threshold_filename.toStdString());
	body.jsonAppendElement("device_phase_id", dev.device.device_phase_id);
	body.jsonAppendElement("alarm_level_id", (int)dev.device.alarm_level_id);

    body.jsonAppendElement("point_id", dev.paratmeter.point_id);
    body.jsonAppendElement("ptz_pan", dev.paratmeter.ptz_pan);
    body.jsonAppendElement("ptz_tilt", dev.paratmeter.ptz_tilt);
    body.jsonAppendElement("hc_zoom_near", dev.paratmeter.hc_zoom_near);
    body.jsonAppendElement("hc_focus_near", dev.paratmeter.hc_focus_near);
    body.jsonAppendElement("hc_zoom_far", dev.paratmeter.hc_zoom_far);
    body.jsonAppendElement("hc_focus_far", dev.paratmeter.hc_focus_far);
    body.jsonAppendElement("mag_focus", dev.paratmeter.mag_focus);
    body.jsonAppendElement("video_length", dev.paratmeter.video_length);
    body.jsonAppendElement("audio_length", dev.paratmeter.audio_length);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_device_update_req(WheelRobortDeviceParameterStruct dev)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_update);

    body.jsonAppendElement("device_uuid", dev.device_uuid.toStdString());
    body.jsonAppendElement("point_id", dev.point_id);
    body.jsonAppendElement("ptz_pan", dev.ptz_pan);
    body.jsonAppendElement("ptz_tilt", dev.ptz_tilt);
    body.jsonAppendElement("hc_zoom_near", dev.hc_zoom_near);
    body.jsonAppendElement("hc_focus_near", dev.hc_focus_near);
    body.jsonAppendElement("hc_zoom_far", dev.hc_zoom_far);
    body.jsonAppendElement("hc_focus_far", dev.hc_focus_far);
    body.jsonAppendElement("mag_focus", dev.mag_focus);
    body.jsonAppendElement("video_length", dev.video_length);
    body.jsonAppendElement("audio_length", dev.audio_length);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_device_insert_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    std::string device_uuid = val["device_uuid"].asString();
    bool bInserted = val["bInserted"].asBool();
    std::string insertMsg = val["insertMsg"].asString();
     wheelRobotDeviceInsertStatus(QString::fromStdString(device_uuid), bInserted, QString::fromLocal8Bit(insertMsg.c_str()));

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        return;
    }
}

void LibDLWheelRobotBackground::robot_device_update_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    std::string device_uuid = val["device_uuid"].asString();
    bool bUpdated = val["bUpdated"].asBool();
    std::string updateMsg = val["updateMsg"].asString();
    wheelRobotDeviceUpdateStatus(QString::fromStdString(device_uuid), bUpdated, QString::fromStdString(updateMsg));

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        return;
    }

}

void LibDLWheelRobotBackground::robot_device_delete_single_dev_req(QString device_uuid)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_delete);

    body.jsonAppendElement("device_uuid", device_uuid.toStdString());

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_template_insert_req(WheelTaskTemplateStruct temp)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_template_insert);

	body.jsonAppendElement("task_template_uuid", temp.task_template_uuid.toStdString());
	body.jsonAppendElement("task_edit_uuid", temp.task_edit_uuid.toStdString());
	body.jsonAppendElement("task_type_id", (int)temp.task_type_id);
	body.jsonAppendElement("task_end_action_id", (int)temp.task_end_action_id);
	body.jsonAppendElement("task_template_name", temp.task_template_name.toStdString());
	body.jsonAppendElement("task_start_date", temp.task_start_date.toStdString());
	body.jsonAppendElement("task_repeat_duration", temp.task_repeat_duration);
	body.jsonAppendElement("task_status_id", (int)temp.task_status_id);
	body.jsonAppendElement("task_loop_type_id", (int)temp.task_loop_type_id);
	body.jsonAppendElement("task_start_time", temp.task_start_time.toString("hh:mm:ss").toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_device_delete_device_type_req(QString interval_uuid, QString device_uuid)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_delete_device_type);

    body.jsonAppendElement("interval_uuid", interval_uuid.toStdString());
    body.jsonAppendElement("device_uuid", device_uuid.toStdString());

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_device_delete_interval_req(QString interval_uuid)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_delete_interval);

    body.jsonAppendElement("interval_uuid", interval_uuid.toStdString());

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_device_delete_voltage_level_req(QString voltage_level)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_device_delete_voltage_level);

    body.jsonAppendElement("voltage_id", voltage_level.toStdString());

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_db_task_delete_req(QList<WheelDeleteTaskChoose> choo)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_db_task_delete);

	Json::Value taskchose;
	for (int i = 0; i < choo.size(); i++)
	{
		taskchose[i]["task_uuid"] = choo[i].task_delete_uuid.toStdString();
		taskchose[i]["taskChoose"] = (int)choo[i].taskChoose;
	}
	body.jsonAppendElement("task_choose", taskchose);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_patrol_result_audit_req(WheelPartrolResultAudit audit)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_patrol_result_audit);

	body.jsonAppendElement("task_uuid", audit.task_uuid.toStdString());
	body.jsonAppendElement("dealed_time", audit.dealed_time.toStdString());
	body.jsonAppendElement("dealed_user", audit.dealed_user.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_excel_import_req(QStringList excelData)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_excel_import_req);

	Json::Value c_excelData;
	for (int i = 0; i < excelData.size(); i++)
	{
		c_excelData[i] = excelData[i].toStdString();
	}
	body.jsonAppendElement("excelData", c_excelData);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_update_standard_patrol_vindicate_req(WheelStandardPatrolVindicateStruct data)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_updata_standard_spot_req);

	body.jsonAppendElement("device_point_uuid", data.m_device_point_uuid.toStdString());
	body.jsonAppendElement("device_type_name", data.m_device_type_name.toStdString());
	body.jsonAppendElement("sub_device_name", data.m_sub_device_name.toStdString());
	body.jsonAppendElement("device_point_name", data.m_device_point_name.toStdString());
	body.jsonAppendElement("recognition_type_name", data.m_recognition_type_name.toStdString());
	body.jsonAppendElement("meter_type_name", data.m_meter_type_name.toStdString());
	body.jsonAppendElement("fever_type_name", data.m_fever_type_name.toStdString());
	body.jsonAppendElement("save_type_name", data.m_save_type_name.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_delete_standard_patrol_vindicate_req(QString uuid)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_standard_spot_req);

	body.jsonAppendElement("device_point_uuid", uuid.toStdString());
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_insert_standard_patrol_vindicate_req(WheelStandardPatrolVindicateStruct data)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_insert_standard_spot_req);
	QStringList c_data;
	c_data.append(data.m_device_type_name);
	c_data.append(data.m_sub_device_name);
	c_data.append(data.m_device_point_name);
	c_data.append(data.m_recognition_type_name);
	c_data.append(data.m_meter_type_name);
	c_data.append(data.m_fever_type_name);
	c_data.append(data.m_save_type_name);

	Json::Value c_excelData;
	for (int i = 0; i < c_data.size(); i++)
	{
		c_excelData[i] = c_data[i].toStdString();
	}
	body.jsonAppendElement("excelData", c_excelData);
	body.jsonAppendElement("uuid", data.m_device_point_uuid.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_edit_import_req(WheelTaskEditStruct stru)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_edit_import_req);

	body.jsonAppendElement("task_edit_uuid_old", stru.task_edit_uuid.toStdString());
	body.jsonAppendElement("task_edit_name", stru.task_edit_name.toStdString());
	body.jsonAppendElement("task_edit_date", stru.task_edit_date.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_user_config_add_req(WheelUserConfig data)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_user_add_config_req);

	body.jsonAppendElement("user_uuid", data.user_uuid.toStdString());
	body.jsonAppendElement("user_name", data.user_name.toStdString());
	body.jsonAppendElement("user_role", (int)data.user_role);
	body.jsonAppendElement("user_authority", data.user_authority);
	body.jsonAppendElement("user_password", data.user_password.toStdString());
	body.jsonAppendElement("user_telephone", data.user_telephone.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_user_config_delete_req(QString user_uuid)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_user_delete_config_req);

	body.jsonAppendElement("user_uuid", user_uuid.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_create_report_req(WheelTaskShow task)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_create_report_req);

	body.jsonAppendElement("task_uuid", task.task_uuid.toStdString());
	body.jsonAppendElement("task_name", task.task_name.toStdString());
	body.jsonAppendElement("task_time", task.task_time.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_examine_report_isexist_req(QString reportName, WheelTaskShow task)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_examine_report_isexist_req);

	body.jsonAppendElement("reportName", reportName.toStdString());
	body.jsonAppendElement("task_uuid", task.task_uuid.toStdString());
	body.jsonAppendElement("task_name", task.task_name.toStdString());
	body.jsonAppendElement("task_time", task.task_time.toStdString());


	postMsg(body);
}

void LibDLWheelRobotBackground::robot_delete_patrol_point_set_req(QStringList device_uuid)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_patrol_point_set_req);
	Json::Value c_device_uuid;
	for (int i = 0; i < device_uuid.size(); i++)
	{
		c_device_uuid[i] = device_uuid[i].toStdString();
	}
	body.jsonAppendElement("device_uuid", c_device_uuid);
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_start_using_status_req(QStringList device_uuid,WheelRootStartUsing start_using)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_start_using_status_req);

	Json::Value c_device_uuid;
	for (int i = 0; i < device_uuid.size(); i++)
	{
		c_device_uuid[i] = device_uuid[i].toStdString();
	}
	body.jsonAppendElement("device_uuid", c_device_uuid);
	body.jsonAppendElement("start_using", (int)start_using);
	postMsg(body);
}
void LibDLWheelRobotBackground::robot_patrol_point_add_req(WheelPatrolPointSet data)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_patrol_point_add_req);

	body.jsonAppendElement("device_uuid", data.device_uuid.toStdString());
	body.jsonAppendElement("device_point_type_uuid", data.device_point_type_uuid.toStdString());
	body.jsonAppendElement("station_name", data.station_name.toStdString());
	body.jsonAppendElement("voltahe_level_name", data.voltahe_level_name.toStdString());
	body.jsonAppendElement("equipment_interval_name", data.equipment_interval_name.toStdString());
	body.jsonAppendElement("device_area_name", data.device_area_name.toStdString());
	body.jsonAppendElement("device_type_name", data.device_type_name.toStdString());
	body.jsonAppendElement("sub_device_type", data.sub_device_type.toStdString());
	body.jsonAppendElement("device_point_type_name", data.device_point_type_name.toStdString());
	body.jsonAppendElement("recognition_type_name", data.recognition_type_name.toStdString());
	body.jsonAppendElement("meter_type_name", data.meter_type_name.toStdString());
	postMsg(body);
}
void LibDLWheelRobotBackground::robot_patrol_point_updata_req(WheelPatrolPointSet data)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_patrol_point_updata_req);

	body.jsonAppendElement("device_uuid", data.device_uuid.toStdString());
	body.jsonAppendElement("device_point_type_uuid", data.device_point_type_uuid.toStdString());
	body.jsonAppendElement("station_name", data.station_name.toStdString());
	body.jsonAppendElement("voltahe_level_name", data.voltahe_level_name.toStdString());
	body.jsonAppendElement("equipment_interval_name", data.equipment_interval_name.toStdString());
	body.jsonAppendElement("device_area_name", data.device_area_name.toStdString());
	body.jsonAppendElement("device_type_name", data.device_type_name.toStdString());
	body.jsonAppendElement("sub_device_type", data.sub_device_type.toStdString());
	body.jsonAppendElement("device_point_type_name", data.device_point_type_name.toStdString());
	body.jsonAppendElement("recognition_type_name", data.recognition_type_name.toStdString());
	body.jsonAppendElement("meter_type_name", data.meter_type_name.toStdString());
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_task_edit_insert_from_map_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_edit_insert_from_map_req);

//	body.jsonAppendElement("task_edit_uuid", m_wheelTaskEditStru.task_edit_uuid.toStdString());
	body.jsonAppendElement("task_edit_name", m_wheelTaskEditStru.task_edit_name.toStdString());
	body.jsonAppendElement("task_edit_date", m_wheelTaskEditStru.task_edit_date.toStdString());
//	body.jsonAppendElement("task_edit_type_id", (int)m_wheelTaskEditStru.task_edit_type_id);

	Json::Value dev;
	for (int i = 0; i < device_uuid.size(); i++)
	{
		dev[i] = device_uuid[i].toStdString();
	}
	body.jsonAppendElement("device_uuid", dev);

	postMsg(body);
}
void LibDLWheelRobotBackground::robot_update_task_status_req(QString task_uuid, WheelRobotTaskStatusType status)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_update_task_status_req);

	body.jsonAppendElement("task_uuid", task_uuid.toStdString());
	body.jsonAppendElement("status", (int)status);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_insert_note_message_req(WheelSubMsgInsert msg)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_insert_note_message_req);

	body.jsonAppendElement("user_receive_uuid", msg.user_receive_uuid.toStdString());
	body.jsonAppendElement("alarm_level_id", msg.alarm_level_id.toStdString());
	body.jsonAppendElement("send_time", msg.send_time.toStdString());
	body.jsonAppendElement("send_freq_id", msg.send_freq_id.toStdString());
	body.jsonAppendElement("fault_name_uuid", msg.fault_name_uuid.toStdString());
	body.jsonAppendElement("device_tree_path", (int)msg.type);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_delete_note_message_req(QString noteUUid, QString fault_name_uuid)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_note_message_req);

	body.jsonAppendElement("noteUUid", noteUUid.toStdString());
	body.jsonAppendElement("fault_name_uuid", noteUUid.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_auto_relevance_device_req()
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_auto_relevance_device_req);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_fast_audit_task_req(QString taskUUid)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_fast_audit_task_req);
	body.jsonAppendElement("taskUUid", taskUUid.toStdString());
	
	postMsg(body);
}
/*--采集新设备树设计--*/
void LibDLWheelRobotBackground::robot_add_new_interval_req(QString voltageLevelUUid, QString newIntervalName)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_add_new_interval_req);
	body.jsonAppendElement("voltageLevelUUid", voltageLevelUUid.toStdString());
	body.jsonAppendElement("newIntervalName", newIntervalName.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_add_new_interval_resp()
{

}

void LibDLWheelRobotBackground::robot_update_interval_req(QString voltageLevelUUid, QString intervalUUid, QString newIntervalName)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_update_interval_req);
	body.jsonAppendElement("voltageLevelUUid", voltageLevelUUid.toStdString());
	body.jsonAppendElement("intervalUUid", intervalUUid.toStdString());
	body.jsonAppendElement("newIntervalName", newIntervalName.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_update_interval_resp()
{

}

void LibDLWheelRobotBackground::robot_copy_paste_interval_req(QString originalVoltageLevelUUid, QString intervalUUid, QString newVoltageLevelUUid, QString newIntervalName)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_copy_paste_interval_req);
	body.jsonAppendElement("originalVoltageLevelUUid", originalVoltageLevelUUid.toStdString());
	body.jsonAppendElement("intervalUUid", intervalUUid.toStdString());
	body.jsonAppendElement("newVoltageLevelUUid", newVoltageLevelUUid.toStdString());
	body.jsonAppendElement("newIntervalName", newIntervalName.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_copy_paste_interval_resp()
{

}

void LibDLWheelRobotBackground::robot_delete_interval_req(QString voltageLevelUUid, QString intervalUUid)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_interval_req);
	body.jsonAppendElement("voltageLevelUUid", voltageLevelUUid.toStdString());
	body.jsonAppendElement("intervalUUid", intervalUUid.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_delete_interval_resp()
{

}

void LibDLWheelRobotBackground::robot_add_deviceType_andDevices_req(QString voltageLevelUUid, QString intervalUUid, QStringList deviceTypeUUidList)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_add_deviceType_andDevices_req);
	Json::Value root;
	for (int i = 0; i < deviceTypeUUidList.size(); i++)
	{
		root.append(deviceTypeUUidList[i].toStdString());
	}

	body.jsonAppendElement("deviceTypeUUidList", root);
	body.jsonAppendElement("voltageLevelUUid", voltageLevelUUid.toStdString());
	body.jsonAppendElement("intervalUUid", intervalUUid.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_add_deviceType_andDevices_resp()
{

}

void LibDLWheelRobotBackground::robot_delete_device_type_req(QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_device_type_req);

	body.jsonAppendElement("voltageLevelUUid", voltageLevelUUid.toStdString());
	body.jsonAppendElement("intervalUUid", intervalUUid.toStdString());
	body.jsonAppendElement("deviceTypeUUid", deviceTypeUUid.toStdString());

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_delete_device_type_req(QString voltageLevelUUid, QString intervalUUid, QStringList deviceTypeUUidList)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_device_type_list_req);

	Json::Value root;
	for (int i = 0; i < deviceTypeUUidList.size(); i++)
	{
		root.append(deviceTypeUUidList[i].toStdString());
	}
	body.jsonAppendElement("voltageLevelUUid", voltageLevelUUid.toStdString());
	body.jsonAppendElement("intervalUUid", intervalUUid.toStdString());
	body.jsonAppendElement("deviceTypeUUidList", root);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_delete_device_type_resp()
{

}

void LibDLWheelRobotBackground::robot_add_devices_fromlist_req(QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid, QStringList pointNameUUid)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_add_devices_fromlist_req);
	Json::Value root;
	for (int i = 0; i < pointNameUUid.size(); i++)
	{
		root.append(pointNameUUid[i].toStdString());
	}
	body.jsonAppendElement("voltageLevelUUid", voltageLevelUUid.toStdString());
	body.jsonAppendElement("intervalUUid", intervalUUid.toStdString());
	body.jsonAppendElement("deviceTypeUUid", deviceTypeUUid.toStdString());
	body.jsonAppendElement("pointNameList", root);
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_add_devices_fromlist_resp()
{

}

void LibDLWheelRobotBackground::robot_delete_devices_fromlist_req(QStringList devicesUUidList, QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid)
{
	jsonBody body;

	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_delete_devices_fromlist_req);
	Json::Value root;
	for (int i = 0; i < devicesUUidList.size(); i++)
	{
		root.append(devicesUUidList[i].toStdString());
	}
	body.jsonAppendElement("devicesUUidList", root);
	body.jsonAppendElement("voltageLevelUUid", voltageLevelUUid.toStdString());
	body.jsonAppendElement("intervalUUid", intervalUUid.toStdString());
	body.jsonAppendElement("deviceTypeUUid", deviceTypeUUid.toStdString());
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_delete_devices_fromlist_resp()
{

}

void LibDLWheelRobotBackground::robot_collect_device_tree_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	QStringList nodeUUidList;
	RootNodeType nodeType;
	int count = 0;
	bool bRet = val["bRet"].asBool();
	for (int i = 0; i < val["nodeUUidList"].size(); i++)
	{
		nodeUUidList.append(QString::fromStdString(val["nodeUUidList"][i].asString()));
		count++;
	}
	if (count == 4)
	{
		count = count - 1;
		nodeUUidList.removeAt(3);
	}
	nodeType = RootNodeType(count + 2);
	
	wheelRobotCollectDeviceTreeSignal(bRet, nodeUUidList, nodeType);
}

void LibDLWheelRobotBackground::robot_control_infrared_take_photo_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	QString strPath = QString::fromStdString(val["strPath"].asString());
    QString strName = QString::fromStdString(val["strName"].asString());
    int type = val["type"].asInt();

    wheelRobotInfraredTakePhotoSignal(strPath, strName, type);
}

void LibDLWheelRobotBackground::robot_update_task_table_signal(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("robot_update_task_table_signal:update table");

    wheelRobotUpdateTaskTableSignal();
}

/*end*/
//.//////////////////////////////////////////////////////////////////////////////////////////.//

void LibDLWheelRobotBackground::robot_device_delete_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bDeleted = val["bDeleted"].asBool();
    std::string deleteMsg = val["deleteMsg"].asString();
    wheelRobotDeviceDeleteStatus(bDeleted, QString::fromStdString(deleteMsg));

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        return;
    }
}

void LibDLWheelRobotBackground::robot_inspect_result_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    WheelInspectResultStruct res;

    res.task_uuid = QString::fromLocal8Bit(val["task_uuid"].asString().c_str());
    res.device_uuid = QString::fromLocal8Bit(val["device_uuid"].asString().c_str());
    res.inspect_time = QString::fromLocal8Bit(val["inspect_time"].asString().c_str());
    res.inspect_result = QString::fromLocal8Bit(val["inspect_result"].asString().c_str());
    res.inspect_status_id = (WheelRobotInspectResultStatusType)val["inspect_status_id"].asInt();
    res.is_dealed = val["is_dealed"].asInt();
    res.alarm_level_id = (DeviceAlarmLevel)val["alarm_level_id"].asInt();
	res.deal_info_uuid = QString::fromLocal8Bit(val["deal_info_uuid"].asString().c_str());
    res.virtual_name = QString::fromLocal8Bit(val["virtual_name"].asString().c_str());

    res.isHangOut = val["is_hangout"].asBool();
    res.alarm_count = val["alarm_count"].asInt();

    WheelRobotInspectResultCallback(res);
	WheelRobotAlarmLevelStatus(res.device_uuid, res.alarm_level_id);
}

void LibDLWheelRobotBackground::robot_compare_inspect_resule_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

//	CompareDeviceInspectResult data;
	WheelInspectResultStruct data;
	data.task_uuid = QString::fromStdString(val["task_uuid"].asString());
	data.device_uuid = QString::fromStdString(val["device_uuid"].asString());
	data.inspect_time = QString::fromStdString(val["inspect_time"].asString());
	data.virtual_name = QString::fromStdString(val["device_point_name"].asString());
	data.inspect_result = QString::fromStdString(val["inspect_result"].asString());
	data.alarm_level_id = DeviceAlarmLevel(val["alarm_level"].asInt());

	WheelRobotCompareInspectResultCallback(data);
}

void LibDLWheelRobotBackground::robot_task_template_insert_resq(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	std::string task_edit_uuid = val["task_template_uuid"].asString();
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
    wheelRobotTaskTemplateInsertStatus(QString::fromStdString(task_edit_uuid), bInserted, QString::fromStdString(insertMsg));

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		return;
	}
}

void LibDLWheelRobotBackground::robot_device_delete_device_type_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bDeleted = val["bDeleted"].asBool();
    std::string deleteMsg = val["deleteMsg"].asString();
    wheelRobotDeviceDeleteStatus(bDeleted, QString::fromStdString(deleteMsg));

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        return;
    }
}

void LibDLWheelRobotBackground::robot_device_delete_interval_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bDeleted = val["bDeleted"].asBool();
    std::string deleteMsg = val["deleteMsg"].asString();
    wheelRobotDeviceDeleteStatus(bDeleted, QString::fromStdString(deleteMsg));

    //wheelRobotDeviceDeleteStatus(bDeleted, QString::fromStdString(deleteMsg));

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        return;
    }
}

void LibDLWheelRobotBackground::robot_device_delete_voltage_level_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bDeleted = val["bDeleted"].asBool();
    std::string deleteMsg = val["deleteMsg"].asString();
    wheelRobotDeviceDeleteStatus(bDeleted, QString::fromStdString(deleteMsg));

    //wheelRobotDeviceDeleteStatus(bDeleted, QString::fromStdString(deleteMsg));

    if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
    {
        return;
    }
}

void LibDLWheelRobotBackground::robot_task_edit_insert_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	int task_edit_type_id = val["task_edit_type_id"].asInt();
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
    wheelRobotTaskEditInsertStatus(task_edit_type_id, bInserted, QString::fromStdString(insertMsg));

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		return;
	}
}

void LibDLWheelRobotBackground::robot_task_edit_updata_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	int task_edit_type_id = val["task_edit_type_id"].asInt();
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
    wheelRobotTaskEditUpdataStatus(task_edit_type_id, bInserted, QString::fromStdString(insertMsg));

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		return;
	}
}

void LibDLWheelRobotBackground::robot_task_edit_delete_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	int task_edit_type_id = val["task_edit_type_id"].asInt();
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
    wheelRobotTaskEditDeleteStatus(task_edit_type_id, bInserted, QString::fromStdString(insertMsg));


	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		return;
	}
}

void LibDLWheelRobotBackground::robot_task_device_finish_resp(boost::shared_ptr<roboKitMsg> msg)
{
//     Json::Value val = msg->msgBody.getJsonVal();
//     if (val.isNull())
//     {
//         return;
//     }
// 
//     std::string task_edit_uuid = val["task_edit_uuid"].asString();
//     bool bInserted = val["bInserted"].asBool();
//     std::string insertMsg = val["insertMsg"].asString();
//     if (NULL != wheelRobotTaskEditUpdataStatus)
//     {
//         wheelRobotTaskEditUpdataStatus(QString::fromStdString(task_edit_uuid), bInserted, QString::fromStdString(insertMsg));
//     }
//     //wheelRobotTaskEditUpdataStatus(QString::fromStdString(task_edit_uuid), bInserted, QString::fromStdString(insertMsg));
// 
//     int ret_code;
//     std::string err_msg;
//     if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
//     {
//         return;
//     }
//     ret_code = val["ret_code"].asInt();
//     err_msg = val["err_msg"].asString();
}

void LibDLWheelRobotBackground::robot_task_finish_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    QString task_uuid = QString::fromLatin1(val["task_uuid"].asString().c_str());
    QString date_time = QString::fromLatin1(val["date_time"].asString().c_str());
    int status = val["status"].asInt();

    wheelRobotEndTaskMsg(task_uuid, date_time, status);

}

void LibDLWheelRobotBackground::robot_task_all_finish_resp(boost::shared_ptr<roboKitMsg> msg)
{
//     Json::Value val = msg->msgBody.getJsonVal();
//     if (val.isNull())
//     {
//         return;
//     }
// 
//     std::string task_edit_uuid = val["task_edit_uuid"].asString();
//     bool bInserted = val["bInserted"].asBool();
//     std::string insertMsg = val["insertMsg"].asString();
//     if (NULL != wheelRobotTaskEditUpdataStatus)
//     {
//         wheelRobotTaskEditUpdataStatus(QString::fromStdString(task_edit_uuid), bInserted, QString::fromStdString(insertMsg));
//     }
//     //wheelRobotTaskEditUpdataStatus(QString::fromStdString(task_edit_uuid), bInserted, QString::fromStdString(insertMsg));
// 
//     int ret_code;
//     std::string err_msg;
//     if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
//     {
//         return;
//     }
//     ret_code = val["ret_code"].asInt();
//     err_msg = val["err_msg"].asString();
}

void LibDLWheelRobotBackground::robot_db_task_delete_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	std::string task_delete_uuid = val["task_edit_uuid"].asString();
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
    wheelRobotTaskChooseDeleteStatus(QString::fromStdString(task_delete_uuid), bInserted, QString::fromStdString(insertMsg));

    //wheelRobotTaskChooseDeleteStatus(QString::fromStdString(task_delete_uuid), bInserted, QString::fromStdString(insertMsg));

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		return;
	}
}

void LibDLWheelRobotBackground::robot_partrol_result_verify_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	int choose = val["choose"].asInt();

	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
    wheelRobotPartrolResultVerifyStatus(choose, bInserted, QString::fromStdString(insertMsg));

	if (val["ret_code"].isNull() || val["ret_code"].asInt() == 0)
	{
		return;
	}
}

void LibDLWheelRobotBackground::robot_ctrl_on_mouse_click_on_In_window(int width, int height, int x, int y, int z, bool isInfrared)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_control_ptz_monodrome_infra);//Request_robot_control_ptz_monodrome_infra===>>>2008

	float scale_x = (float)x / width;
	float scale_y = (float)y / height;
	body.jsonAppendElement("scale_x", scale_x);	//机器人接收到数据计算云台偏差值需要乘以红外图的width，得出的值就是前端画面点击的坐标，也就是云台调节的目标点
	body.jsonAppendElement("scale_y", scale_y);	//机器人接收到数据计算云台偏差值需要乘以红外图的height
	body.jsonAppendElement("is_infrared", isInfrared);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_ctrl_on_mouse_click_on_hc_window(int width, int height, int x, int y, int z, bool isInfrared)
{
	int zoomValue = z;
	if (zoomValue < 0)
	{
		return;
	}

    zoomInfo info_;
    if (isInfrared)
    {
        info_.levelAangle = 37.09;
        info_.verticalAngle = 21.399;
    }
    else
    {
        info_ = HikPointData::getInitance()->getZoomInfoData(HikPointData::getInitance()->getZoomValue(zoomValue));
    }

	float horizontalAngle;
	float verticalAngle;
	int rotateAngleValue;
	int pitchAngleValue;
	if (HikPointData::getInitance()->getIsUseDefaultZoomMap())
	{
		horizontalAngle = 0 - (x - width / 2.0) / width * info_.levelAangle;
		verticalAngle = 0 - (y - height / 2.0) / height * info_.verticalAngle;
		rotateAngleValue = horizontalAngle * 100;
		pitchAngleValue = verticalAngle * 100;
	}
	else
	{
		horizontalAngle = ((x - width / 2.0) / width * 1920) * (info_.levelAangle);
		verticalAngle = 0 - ((y - height / 2.0) / height * 1080) * (info_.verticalAngle);
		rotateAngleValue = horizontalAngle;
		pitchAngleValue = verticalAngle;
	}
	/*
	WheelRobotPtzStatus final_pos;
	final_pos.pan = m_currPtz.pan + rotateAngleValue;
	if (final_pos.pan > 36000)
	{
		final_pos.pan = final_pos.pan - 36000;
	}
	if (final_pos.pan < 0)
	{
		final_pos.pan = final_pos.pan + 36000;
	}

	final_pos.tilt = m_currPtz.tilt - pitchAngleValue;
	if (final_pos.tilt < 27090 && m_currPtz.tilt > 27090)
	{
		final_pos.tilt = 27090;
	}
	if (final_pos.tilt > 3065 && m_currPtz.tilt < 3100)
	{
		final_pos.tilt = 3065;
	}

	if (final_pos.tilt > 36000)
	{
		final_pos.tilt = final_pos.tilt - 36000;
	}

	if (final_pos.tilt < 0)
	{
		final_pos.tilt = final_pos.tilt + 36000;
	}
	*/
    //robot_control_ptz_abs_req(final_pos.pan, final_pos.tilt);
	robot_control_ptz_relative_req(rotateAngleValue, pitchAngleValue);
	//jsonBody body;
	//body.setMsgNumber(getMsgId());
	//body.setMsgType((int)Request_robot_control_ptz_monodrome_infra);//Request_robot_control_ptz_monodrome_infra===>>>2008

	//body.jsonAppendElement("scale_x", rotateAngleValue);	//机器人接收到数据计算云台偏差值需要乘以红外图的width，得出的值就是前端画面点击的坐标，也就是云台调节的目标点
	//body.jsonAppendElement("scale_y", pitchAngleValue);	//机器人接收到数据计算云台偏差值需要乘以红外图的height
	//body.jsonAppendElement("is_infrared", isInfrared);

	//postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_insert_voltage_level(QString voltage_level_name)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_insert_voltage_level);

    body.jsonAppendElement("voltage_level_name", std::string(voltage_level_name.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_insert_area(QString device_area_name)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_insert_area);

    body.jsonAppendElement("device_area_name", std::string(device_area_name.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_insert_interval(QString voltage_level_id, QString equipment_interval_name)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_insert_interval);

    body.jsonAppendElement("voltage_level_id", std::string(voltage_level_id.toLocal8Bit()));
    body.jsonAppendElement("equipment_interval_name", std::string(equipment_interval_name.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_delete_voltage_level(QString voltage_level_id)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_delete_voltage_level);

    body.jsonAppendElement("voltage_level_id", std::string(voltage_level_id.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_delete_area(QString device_area_uuid)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_delete_area);

    body.jsonAppendElement("device_area_uuid", std::string(device_area_uuid.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_config_delete_interval(QString equipment_interval_uuid)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_delete_interval);

    body.jsonAppendElement("equipment_interval_uuid", std::string(equipment_interval_uuid.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_threshold_set_by_device(QVector<THRESHOLD_ELEMENT> alarmNormalList, 
    QVector<THRESHOLD_ELEMENT> alarmWarningList, QVector<THRESHOLD_ELEMENT> alarmCommonList, QVector<THRESHOLD_ELEMENT> alarmSerialList, 
    QVector<THRESHOLD_ELEMENT> alarmDangerList, QVector<QString> device_uuid)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_threshold_set_by_device_uuid_req);

    Json::Value alarmNormal;
    for (int i = 0; i < alarmNormalList.size(); i++)
    {
        alarmNormal[i]["type"] = alarmNormalList[i].type;
        for (int j = 0; j < alarmNormalList[i].val.size(); j++)
        {
            alarmNormal[i]["val"][j] = alarmNormalList[i].val[j].toStdString();
        }        
    }

    Json::Value alarmWarning;
    for (int i = 0; i < alarmWarningList.size(); i++)
    {
        alarmWarning[i]["type"] = alarmWarningList[i].type;
        for (int j = 0; j < alarmWarningList[i].val.size(); j++)
        {
            alarmWarning[i]["val"][j] = alarmWarningList[i].val[j].toStdString();
        }
    }

    Json::Value alarmCommon;
    for (int i = 0; i < alarmCommonList.size(); i++)
    {
        alarmCommon[i]["type"] = alarmCommonList[i].type;
        for (int j = 0; j < alarmCommonList[i].val.size(); j++)
        {
            alarmCommon[i]["val"][j] = alarmCommonList[i].val[j].toStdString();
        }
    }

    Json::Value alarmSerial;
    for (int i = 0; i < alarmSerialList.size(); i++)
    {
        alarmSerial[i]["type"] = alarmSerialList[i].type;
        for (int j = 0; j < alarmSerialList[i].val.size(); j++)
        {
            alarmSerial[i]["val"][j] = alarmSerialList[i].val[j].toStdString();
        }
    }

    Json::Value alarmDanger;
    for (int i = 0; i < alarmDangerList.size(); i++)
    {
        alarmDanger[i]["type"] = alarmDangerList[i].type;
        for (int j = 0; j < alarmDangerList[i].val.size(); j++)
        {
            alarmDanger[i]["val"][j] = alarmDangerList[i].val[j].toStdString();
        }
    }

    body.jsonAppendElement("alarmNormal", alarmNormal);
    body.jsonAppendElement("alarmWarning", alarmWarning);
    body.jsonAppendElement("alarmCommon", alarmCommon);
    body.jsonAppendElement("alarmSerial", alarmSerial);
    body.jsonAppendElement("alarmDanger", alarmDanger);

    Json::Value devRoot;
    for (int i = 0; i < device_uuid.size(); i++)
    {
        devRoot[i] = device_uuid[i].toStdString();
    }

    body.jsonAppendElement("device_uuid", devRoot);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_threshold_set_by_meter_type(QVector<THRESHOLD_ELEMENT> alarmNormalList, 
    QVector<THRESHOLD_ELEMENT> alarmWarningList, QVector<THRESHOLD_ELEMENT> alarmCommonList, QVector<THRESHOLD_ELEMENT> alarmSerialList, 
    QVector<THRESHOLD_ELEMENT> alarmDangerList, WheelRobotMeterType meter_type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_threshold_set_by_meter_type_req);

    Json::Value alarmNormal;
    for (int i = 0; i < alarmNormalList.size(); i++)
    {
        alarmNormal[i]["type"] = alarmNormalList[i].type;
        for (int j = 0; j < alarmNormalList[i].val.size(); j++)
        {
            alarmNormal[i]["val"][j] = alarmNormalList[i].val[j].toStdString();
        }
    }

    Json::Value alarmWarning;
    for (int i = 0; i < alarmWarningList.size(); i++)
    {
        alarmWarning[i]["type"] = alarmWarningList[i].type;
        for (int j = 0; j < alarmWarningList[i].val.size(); j++)
        {
            alarmWarning[i]["val"][j] = alarmWarningList[i].val[j].toStdString();
        }
    }

    Json::Value alarmCommon;
    for (int i = 0; i < alarmCommonList.size(); i++)
    {
        alarmCommon[i]["type"] = alarmCommonList[i].type;
        for (int j = 0; j < alarmCommonList[i].val.size(); j++)
        {
            alarmCommon[i]["val"][j] = alarmCommonList[i].val[j].toStdString();
        }
    }

    Json::Value alarmSerial;
    for (int i = 0; i < alarmSerialList.size(); i++)
    {
        alarmSerial[i]["type"] = alarmSerialList[i].type;
        for (int j = 0; j < alarmSerialList[i].val.size(); j++)
        {
            alarmSerial[i]["val"][j] = alarmSerialList[i].val[j].toStdString();
        }
    }

    Json::Value alarmDanger;
    for (int i = 0; i < alarmDangerList.size(); i++)
    {
        alarmDanger[i]["type"] = alarmDangerList[i].type;
        for (int j = 0; j < alarmDangerList[i].val.size(); j++)
        {
            alarmDanger[i]["val"][j] = alarmDangerList[i].val[j].toStdString();
        }
    }

    body.jsonAppendElement("alarmNormal", alarmNormal);
    body.jsonAppendElement("alarmWarning", alarmWarning);
    body.jsonAppendElement("alarmCommon", alarmCommon);
    body.jsonAppendElement("alarmSerial", alarmSerial);
    body.jsonAppendElement("alarmDanger", alarmDanger);

    body.jsonAppendElement("meter_type", meter_type);

    postMsg(body);
}

void LibDLWheelRobotBackground::resp_robot_insert_threshold(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bSucceed = val["bSucceed"].asBool();
    QString retMsg = QString::fromLatin1(val["retMsg"].asString().c_str());

    wheelRobotInsertThresholdStatus(bSucceed, retMsg);
}

void LibDLWheelRobotBackground::robot_config_insert_area_map_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bInserted = val["status"].asBool();
    std::string insertMsg = val["errMsg"].asString();
    wheelRobotInsertAreaStatus(bInserted, QString::fromLocal8Bit(insertMsg.c_str()));
}

void LibDLWheelRobotBackground::robot_config_delete_area_map_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bInserted = val["status"].asBool();
    std::string deleteMsg = val["errMsg"].asString();
    wheelRobotDeleteAreaStatus(bInserted, QString::fromLocal8Bit(deleteMsg.c_str()));
}

void LibDLWheelRobotBackground::robot_patrol_result_audit_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool bInserted = val["bInserted"].asBool();
	QString task_uuid = QString::fromStdString(val["task_uuid"].asString());
	std::string insertMsg = val["insertMsg"].asString();
    wheelRobotPatrolResultAuditStatus(bInserted, task_uuid, QString::fromStdString(insertMsg));
}

void LibDLWheelRobotBackground::robot_excel_import_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
	wheelRobotExcelImportStatus(bInserted, QString::fromStdString(insertMsg));
}

void LibDLWheelRobotBackground::robot_update_standard_patrol_vindicate_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
	wheelRobotUpdateStandardStatus(bInserted, QString::fromStdString(insertMsg));
}

void LibDLWheelRobotBackground::robot_delete_standard_patrol_vindicate_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
	std::string uuid = val["uuid"].asString();
	wheelRobotDeleteStandardStatus(QString::fromStdString(uuid), bInserted, QString::fromLocal8Bit(insertMsg.c_str()));
}

void LibDLWheelRobotBackground::robot_insert_standard_patrol_vindicate_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool bInserted = val["bInserted"].asBool();
	std::string insertMsg = val["insertMsg"].asString();
	wheelRobotInsertStandardStatus(bInserted, QString::fromLocal8Bit(insertMsg.c_str()));
}

void LibDLWheelRobotBackground::robot_task_edit_import_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool bImport = val["bImport"].asBool();
	std::string importMsg = val["importMsg"].asString();
	wheelRobotTaskEditImportStatus(bImport, QString::fromLocal8Bit(importMsg.c_str()));
}

void LibDLWheelRobotBackground::robot_user_config_add_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool bAdd = val["bAdd"].asBool();
	std::string addMsg = val["addMsg"].asString();
	wheelRobotAddUserConfigStatus(bAdd, QString::fromLocal8Bit(addMsg.c_str()));
}

void LibDLWheelRobotBackground::robot_user_config_delete_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool bDelete = val["bDelete"].asBool();
	std::string deleteMsg = val["deleteMsg"].asString();
	wheelRobotDeleteUserConfigStatus(bDelete, QString::fromLocal8Bit(deleteMsg.c_str()));
}

void LibDLWheelRobotBackground::robot_create_report_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	bool report_bool = val["report_bool"].asBool();
	std::string report_name = val["report_name"].asString();

	wheelRobotCreateReportStatus(report_bool, QString::fromStdString(report_name));
}

void LibDLWheelRobotBackground::robot_examine_report_isexist_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}
	WheelTaskShow task;
	bool is_exist = val["is_exist"].asBool();
	QString report_name = QString::fromStdString(val["report_name"].asString());
	task.task_uuid = QString::fromStdString(val["task_uuid"].asString());
	task.task_name = QString::fromStdString(val["task_name"].asString());
	task.task_time = QString::fromStdString(val["task_time"].asString());

	wheelRobotExamineReportIsExistStatus(is_exist, report_name, task);
}

void LibDLWheelRobotBackground::robot_delete_patrol_point_set_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	wheelRobotDeletePatrolPointSetStatus(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_start_using_status_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	wheelRobotStartUsingStatus(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_patrol_point_add_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	wheelRobotPatrolPointAddStatus(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_patrol_point_updata_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	wheelRobotPatrolPointUpdataStatus(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_task_edit_insert_from_map_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_task_assign);

	Json::Value dev;
	int i = 0;
	Json::Value::iterator devItr = val["devices"].begin();
	for (; devItr != val["devices"].end(); devItr++)
	{
		dev[i] = devItr->asString();
		i++;
	}
	body.jsonAppendElement("task_uuid", val["task_uuid"].asString());
	body.jsonAppendElement("task_name", val["task_name"].asString());
	body.jsonAppendElement("task_edit_uuid", val["task_edit_uuid"].asString());
	body.jsonAppendElement("task_template_uuid", "");
	body.jsonAppendElement("priority", (int)WHEEL_ROBOT_TASK_1ST);
	body.jsonAppendElement("devices", dev);
	body.jsonAppendElement("dev_optimize", true);
	body.jsonAppendElement("task_end_action", (int)WHEEL_ROBOT_TASK_END_TYPE_STANDBY);
	body.jsonAppendElement("breakTask", true);
	body.jsonAppendElement("return", 2);
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_update_task_status_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	wheelRobotUpdateTaskStatusStatus(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_insert_note_message_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	wheelRobotInsertNoteMessageStatus(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_delete_note_message_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	wheelRobotDeleteNoteMessageStatus(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_face_recog_note_message_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	if (bRet)
	{
		if (nullptr != m_pSpeeker)
		{
			AlarmMesgErrorCode stAlarmMesgErrorCode;
			stAlarmMesgErrorCode.errorId = CLIENT_CAP_STRANGER;
			stAlarmMesgErrorCode.errorCode = "";
			m_pSpeeker->addSpeakErrorCode(stAlarmMesgErrorCode);
		}
	}
	else
	{
		if (nullptr != m_pSpeeker)
		{
			AlarmMesgErrorCode stAlarmMesgErrorCode;
			stAlarmMesgErrorCode.errorId = CLIENT_CAP_STRANGER;
			m_pSpeeker->removeSpeakErrorCode(stAlarmMesgErrorCode);
		}
	}
}

void LibDLWheelRobotBackground::robot_connect_2_new_robot_req(QString robotName)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_reconnect_2_new_robot_req);

    body.jsonAppendElement("robot_name", std::string(robotName.toLocal8Bit()));

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_connect_2_new_robot_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bRet = val["bRet"].asBool();
	WheelRobotCoreRobotConfig tmpConfig;

	tmpConfig.robotIp = QString::fromLocal8Bit(val["robotIp"].asString().c_str());
	tmpConfig.robotName = QString::fromLocal8Bit(val["robotName"].asString().c_str());
	tmpConfig.hcUserName = QString::fromLocal8Bit(val["hcUserName"].asString().c_str());
	tmpConfig.hcPassword = QString::fromLocal8Bit(val["hcPassword"].asString().c_str());

	tmpConfig.hcIP = QString::fromLocal8Bit(val["hcIP"].asString().c_str());
	tmpConfig.hcCtrlPort = val["hcCtrlPort"].asInt();
	tmpConfig.hcRtspPort = val["hcRtspPort"].asInt();

	tmpConfig.hcIPFront = QString::fromLocal8Bit(val["hcIPFront"].asString().c_str());
	tmpConfig.hcFrontCtrlPort = val["hcFrontCtrlPort"].asInt();
	tmpConfig.hcFrontRtspPort = val["hcFrontRtspPort"].asInt();

	tmpConfig.hcIPBack = QString::fromLocal8Bit(val["hcIPBack"].asString().c_str());
	tmpConfig.hcBackCtrlPort = val["hcBackCtrlPort"].asInt();
	tmpConfig.hcBackRtspPort = val["hcBackRtspPort"].asInt();

	tmpConfig.infraredNeed = val["infraredNeed"].asInt();
	tmpConfig.infraredCameraIp = QString::fromLocal8Bit(val["infraredCameraIp"].asString().c_str());
	tmpConfig.infraredCtrlPort = val["infraredCtrlPort"].asInt();
	tmpConfig.infraredRtspPort = val["infraredRtspPort"].asInt();
    wheelRobotConnect2NewRobot(bRet, tmpConfig);
}

void LibDLWheelRobotBackground::robot_auto_relevance_device_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	if (val.isNull())
	{
		return;
	}

	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());

	wheelAutoRelevanceDevice(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_config_uploadmap2Robot_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bRet = val["bRet"].asBool();
    QString retMsg = QString::fromStdString(val["retMsg"].asString());

    wheelUploadMap2Robot(bRet, retMsg);
}

void LibDLWheelRobotBackground::set_battery_low_voltage(float vol)
{
    voltage_ = vol;
}

void LibDLWheelRobotBackground::robot_connect_status(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    bool bRet = val["isConnect"].asBool();
    if (!bRet)
    {
        if (nullptr != m_pSpeeker)
        {
            AlarmMesgErrorCode stAlarmMesgErrorCode;
            stAlarmMesgErrorCode.errorId = CLIENT_ALARM_ROBOT_COMMUNICATION;
            stAlarmMesgErrorCode.errorCode = "";
            m_pSpeeker->addSpeakErrorCode(stAlarmMesgErrorCode);
        }
    }
    else
    {
        if (nullptr != m_pSpeeker)
        {
            AlarmMesgErrorCode stAlarmMesgErrorCode;
            stAlarmMesgErrorCode.errorId = CLIENT_ALARM_ROBOT_COMMUNICATION;
            m_pSpeeker->removeSpeakErrorCode(stAlarmMesgErrorCode);
        }
    }
}

void LibDLWheelRobotBackground::robot_update_embedded_software()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_config_update_embedded_software_req);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_cloud_infrared_take_photo_req(QString uploadPath, QString deviceUUid, int type)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_Control_Infrared_Take_Photo);

	body.jsonAppendElement("upload_Path", std::string(uploadPath.toLocal8Bit()));
    body.jsonAppendElement("name", std::string(deviceUUid.toLocal8Bit()));
    body.jsonAppendElement("type", type);
	
	postMsg(body);
}

void LibDLWheelRobotBackground::robot_cloud_infrared_auto_focus_req(InfraredFocusMode modeType)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_Control_Infrared_Auto_Focus);

    body.jsonAppendElement("type", int(modeType));

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_cloud_infrared_record_video_req(QString uploadPath, int status)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_Control_Infrared_Record_Video);

	body.jsonAppendElement("upload_Path", std::string(uploadPath.toLocal8Bit()));
	body.jsonAppendElement("status", status);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_cloud_infrared_set_focus_req(int focus)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_Control_Infrared_Set_focus);

    body.jsonAppendElement("focus_value", focus);

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_setting_out_run_point_req(QList<MapSettingOutCoordinates> data)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_Setting_Out_Run_Point);

    Json::Value root;
	for (int i = 0; i < data.size(); i++)
	{
		Json::Value Point;
		Point.append(data[i].pointType);
		Point.append(data[i].pointID);
		Point.append(data[i].pointX);
		Point.append(data[i].pointY);
		root.append(Point);
	}
	body.jsonAppendElement("point_coord", root);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_alarm_display_msg(QString strAlarmDisplaymsg)
{
	wheelRobotSystemWarningCallback(strAlarmDisplaymsg);
}

void LibDLWheelRobotBackground::robot_ctrl_fast_audit_task_resp(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		return;
	}
	bool bRet = val["bRet"].asBool();
	QString retMsg = QString::fromStdString(val["retMsg"].asString());
	wheelRobotFastAuditTaskSignal(bRet, retMsg);
}

void LibDLWheelRobotBackground::robot_config_urgency_stop_req(int type)
{
	jsonBody body;
	body.setMsgNumber(getMsgId());
	body.setMsgType((int)Request_robot_config_urgency_stop_req);

	body.jsonAppendElement("urgency_type", type);

	postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_remote_upgrade_req(WheelRemoteUpgradeType type, QString strFileName)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_Control_Remote_Upgrade);

    body.jsonAppendElement("type", int(type));
    body.jsonAppendElement("fileName", strFileName.toStdString());

    postMsg(body);
}

void LibDLWheelRobotBackground::robot_control_remote_upgrade_resp(boost::shared_ptr<roboKitMsg> msg)
{
    Json::Value val = msg->msgBody.getJsonVal();
    if (val.isNull())
    {
        return;
    }

    int ret_code = val["ret_code"].asBool();
    QString err_msg = QString::fromStdString(val["err_msg"].asString());
    wheelRobotControlUpgradeSignal(ret_code, err_msg);
}


void LibDLWheelRobotBackground::robot_temporary_door_req(bool type)
{
    int openStates = 0;
    if (type)
    {
        openStates = 1;
    }
    else
    {
        openStates = 0;
    }
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType((int)Request_robot_Control_Door);

    body.jsonAppendElement("store_status", int(openStates));

    postMsg(body);
}