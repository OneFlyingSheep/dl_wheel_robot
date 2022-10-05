#include "LibDLIntelligentSocket.h"
#include <time.h>
#include <qdir.h>
#include <QApplication>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreServer.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include <QXmlStreamWriter>
#include <QFile>
#include <QUuid>

static boost::asio::io_service io_service;
static boost::asio::io_service::work io_work(io_service);
static uint64_t static_sequence_id_ = 0;

LibDLIntelligentSocket::LibDLIntelligentSocket(std::string localCode, std::string remoteCode)// : local_code_(localCode), remote_code_(remoteCode)
{
    heart_beat_timer_ = NULL;
    heart_beat_interval_ = 60;
    robot_run_timer_ = NULL;
    robot_run_interval_ = 60;
    weather_timer_ = NULL;
    weather_interval_ = 60;
    root_path_ = QApplication::applicationDirPath();
    root_core_path_ = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath;
    m_HCNetCamera = new BJCameraCtrlSDK;
    m_IntelligentData = new BJIntelligentData;

    //机器人坐标变化时触发信号
    WHEEL_CORE_SERVER.bjRobotCoordAlterSignal.connect(boost::bind(&LibDLIntelligentSocket::slot_robot_coordinate_msg, this, _1));
    WHEEL_CORE_SERVER.bjRobotPatrolLineSignal.connect(boost::bind(&LibDLIntelligentSocket::slot_robot_patrol_line_msg, this, _1));
    WHEEL_CORE_SERVER.bjRobotTaskStatusDataSignal.connect(boost::bind(&LibDLIntelligentSocket::slot_robot_task_data_msg, this, _1));
    WHEEL_CORE_SERVER.bjRobotDeviceResultSendSignal.connect(boost::bind(&LibDLIntelligentSocket::slot_robot_task_device_send_msg, this, _1, _2));
}

void LibDLIntelligentSocket::connect(std::string ip_addr /*= "192.168.4.200"*/, int port /*= 10011*/, int ftpsPort /*= 10012*/, std::string ftps_username /*= "admin"*/, std::string ftps_passwd /*= "123456"*/)
{
    ROS_ERROR("doLogin");
//     server_ip_addr_ = ip_addr;
//     server_port_ = port;
//     ftps_server_port_ = ftpsPort;
    server_ip_addr_ = WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().socket_connect_ip.toStdString();
    server_port_ = WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().socket_connect_port.toInt();

    local_code_ = WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().send_code.toStdString();
    remote_code_ = WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().receive_code.toStdString();
    robot_name_ = WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().robot_name.toStdString();
    robot_code_ = WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().robot_code.toStdString();
    robot_main_code_ = WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().robot_main_code.toStdString();
    station_code_ = WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().station_code.toStdString();

    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string(server_ip_addr_), server_port_);
    m_socket = boost::shared_ptr<ProtocolSocketClient>(new ProtocolSocketClient(endpoint, io_service));
    registerHandles();

    ftps_client_ = new FTPSClient();
    //ftps_client_->initClient(QString::fromStdString(ip_addr), ftpsPort, QString::fromStdString(ftps_username), QString::fromStdString(ftps_passwd));
    //ftps_client_->initClient(QString::fromStdString("47.114.91.22"), ftpsPort, QString::fromStdString(ftps_username), QString::fromStdString(ftps_passwd));
    ftps_client_->initClient(
        WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().ftps_connect_ip, 
        WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().ftps_connect_port.toInt(), 
        WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().ftps_username,
        WHEEL_ROBOT_CORE_CONFIG.getBJSocketCfg().ftps_passwd);
    
    m_socket->initSocket();
    boost::thread * t1 = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));  

    while (1)
    {
        if (bConnected())
        {
            send_register_msg_req();
            break;
        }
    }

    if (robot_state_timer_ == NULL)
    {
        robot_state_timer_ = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(1)));
        robot_state_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::robot_state_data_func, shared_from_this(), boost::asio::placeholders::error));
    }
}

void LibDLIntelligentSocket::registerHandles()
{
    m_socket->registerMsgHandle(boost::bind(&LibDLIntelligentSocket::recv_handle_code, this, _1));

    m_msgHandleMap[MSG_TYPE_ROBOT_BODY][1] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_body_remote_reset_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BODY][2] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_body_system_check_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BODY][3] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_body_one_key_return_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BODY][4] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_body_manual_charge_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BODY][5] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_body_switch_ctrl_mode_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BODY][6] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_body_access_to_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BODY][7] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_body_release_control_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_ROBOT_BASE][1] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_base_forward_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BASE][2] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_base_backward_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BASE][3] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_base_turn_left_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BASE][4] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_base_turn_right_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BASE][5] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_base_rotate_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_BASE][6] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_base_stop_control_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][1] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_up_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][2] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_down_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][3] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_left_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][4] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_right_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][5] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_rise_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][6] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_fall_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][7] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_preset_bit_call_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][8] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_stop_control_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_PTZ][9] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_ptz_reset_control_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_ROBOT_SUPPORT_DEV][1] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_auxiliary_equipment_infrared_power_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_SUPPORT_DEV][2] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_auxiliary_equipment_wiper_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_SUPPORT_DEV][3] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_auxiliary_equipment_snoise_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_SUPPORT_DEV][4] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_auxiliary_equipment_infrared_light_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][1] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_zoom_in_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][2] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_zoom_out_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][3] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_focus_stop_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][4] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_focus_in_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][5] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_focus_out_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][6] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_auto_focus_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][7] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_capture_img_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][8] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_restart_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][9] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_start_rec_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][10] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_stop_rec_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][11] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_zoom_set_rec_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_CAMERA][12] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_focus_set_rec_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][0] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_undefined_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][1] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_undefined_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][2] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_undefined_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][3] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_undefined_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][4] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_undefined_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][5] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_set_focus_val_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][6] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_auto_focus_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][7] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_capture_img_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][8] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_restart_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][9] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_undefined_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_ROBOT_INFRARED][10] = boost::bind(&LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_undefined_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_TASK_CTRL][1] = boost::bind(&LibDLIntelligentSocket::recv_task_ctrl_start_msg_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_TASK_CTRL][2] = boost::bind(&LibDLIntelligentSocket::recv_task_ctrl_pause_msg_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_TASK_CTRL][3] = boost::bind(&LibDLIntelligentSocket::recv_task_ctrl_resume_msg_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_TASK_CTRL][4] = boost::bind(&LibDLIntelligentSocket::recv_task_ctrl_stop_msg_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_MODEL_SYNC][1] = boost::bind(&LibDLIntelligentSocket::recv_model_sync_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_MODEL_SEND][1] = boost::bind(&LibDLIntelligentSocket::recv_model_assign_dev_model_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_MODEL_SEND][2] = boost::bind(&LibDLIntelligentSocket::recv_model_assign_robot_cfg_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_MODEL_SEND][3] = boost::bind(&LibDLIntelligentSocket::recv_model_assign_threshold_cfg_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_MODEL_SEND][4] = boost::bind(&LibDLIntelligentSocket::recv_model_assign_overhaul_area_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_TASK_ARRANGE][1] = boost::bind(&LibDLIntelligentSocket::recv_task_assign_resp, this, _1); 
    m_msgHandleMap[MSG_TYPE_TASK_ATONCE][1] = boost::bind(&LibDLIntelligentSocket::recv_task_assign_atonce_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_DATA_QUERY][1] = boost::bind(&LibDLIntelligentSocket::recv_data_query_visible_img_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_DATA_QUERY][2] = boost::bind(&LibDLIntelligentSocket::recv_data_query_infrared_img_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_DATA_QUERY][3] = boost::bind(&LibDLIntelligentSocket::recv_data_query_audio_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_DATA_QUERY][4] = boost::bind(&LibDLIntelligentSocket::recv_data_query_video_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_DATA_QUERY][21] = boost::bind(&LibDLIntelligentSocket::recv_data_query_patrol_res_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_DATA_QUERY][22] = boost::bind(&LibDLIntelligentSocket::recv_data_query_alarm_data_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_DATA_QUERY][23] = boost::bind(&LibDLIntelligentSocket::recv_data_query_patrol_report_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_DATA_QUERY][41] = boost::bind(&LibDLIntelligentSocket::recv_data_query_robot_status_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_DATA_QUERY][42] = boost::bind(&LibDLIntelligentSocket::recv_data_query_robot_alarm_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_DATA_QUERY][61] = boost::bind(&LibDLIntelligentSocket::recv_data_task_map_point_resp, this, _1);

    m_msgHandleMap[MSG_TYPE_SYSTEM_MSG][4] = boost::bind(&LibDLIntelligentSocket::recv_register_msg_resp, this, _1);
    m_msgHandleMap[MSG_TYPE_SYSTEM_MSG][3] = boost::bind(&LibDLIntelligentSocket::recv_heartbeat_msg_resp, this, _1);
}

bool LibDLIntelligentSocket::bConnected()
{
    return m_socket->IsConnected();
}

void LibDLIntelligentSocket::send_common_response_no_item(std::string remote_code, int seq, std::string code /*= std::to_string(MSG_CODE_SUCCESS)*/)
{
    if (!bConnected())
    {
        ROS_ERROR("send_common_response_no_item: false! socket not connect!");
        return;
    }
    ROS_ERROR("send_common_response_no_item");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code);
    msg->setType(MSG_TYPE_SYSTEM_MSG);
    msg->setCmd(3);
    msg->setCode(code);
    msg->setTime(getFormatDateTime());
    msg->generiateMsg(seq);

    ROS_ERROR("F: send_common_response_no_item \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_common_response_no_item(std::string code /*= std::to_string(MSG_CODE_SUCCESS)*/)
{
    if (!bConnected())
    {
        ROS_ERROR("send_common_response_no_item: false! socket not connect!");
        return;
    }
    ROS_ERROR("send_common_response_no_item nothing");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setType(MSG_TYPE_SYSTEM_MSG);
    msg->setCmd(3);
    msg->setCode(code);
    msg->setTime(getFormatDateTime());
    msg->generiateMsg(1);

    ROS_ERROR("F: send_common_response_no_item \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_common_response_with_item(std::string remote_code, int seq, std::string code /*= std::to_string(MSG_CODE_SUCCESS)*/, TiXmlElement *item /*= NULL*/)
{
    if (!bConnected())
    {
        ROS_ERROR("send_common_response_with_item: false! socket not connect!");
        return;
    }
    ROS_ERROR("send_common_response_with_item");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code);
    msg->setType(MSG_TYPE_SYSTEM_MSG);
    msg->setCmd(4);
    msg->setCode(code);
    msg->setItem(item);
    msg->setTime(getFormatDateTime());
    msg->generiateMsg(seq);

    ROS_ERROR("F: send_common_response_with_item \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_register_msg_req()
{
    if (!bConnected())
    {
        ROS_ERROR("send_register_msg_req: false! socket not connect!");
        return;
    }
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setType(MSG_TYPE_SYSTEM_MSG);
    msg->setCmd(1);// register
    msg->setTime(getFormatDateTime());
    msg->generiateMsg(static_sequence_id_++);

    ROS_ERROR("F/I.4.1.1: send_register_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::recv_register_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("注册指令返回");
    ROS_ERROR("F/I.4.1.2: recv_register_msg_resp \n#xml:\n%s", msg->getString().c_str());
//    return;
    TiXmlElement *item = msg->getItem();

    if (item->FirstAttribute() != NULL)
    {
        //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        TiXmlAttribute *attri = item->FirstAttribute();    
        //若属性不为空则输出
        while (attri != NULL)         
        {
            if (attri->NameTStr().compare("heart_beat_interval") == 0)
            {
                heart_beat_interval_ = std::stoi(attri->Value());
                if (heart_beat_timer_ == NULL)
                {
                    heart_beat_timer_ = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(heart_beat_interval_)));
                    heart_beat_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::heart_beat_timer_func, shared_from_this(), boost::asio::placeholders::error));
                }
            }
            else if (attri->NameTStr().compare("robot_run_interval") == 0)
            {
                robot_run_interval_ = std::stoi(attri->Value());
                if (robot_run_timer_ == NULL)
                {
                    robot_run_timer_ = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(robot_run_interval_)));
                    robot_run_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::robot_run_timer_func, shared_from_this(), boost::asio::placeholders::error));
                }
            }
            else if (attri->NameTStr().compare("weather_interval") == 0)
            {
                weather_interval_ = std::stoi(attri->Value());
                if (weather_timer_ == NULL)
                {
                    weather_timer_ = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(weather_interval_)));
                    weather_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::weather_timer_func, shared_from_this(), boost::asio::placeholders::error));
                }
            }
            else
            {
                ROS_ERROR("unknown attr:%s", attri->NameTStr().c_str());
            }
            attri = attri->Next();
        }
    }
    else
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = i->FirstAttribute();
            while (attri != NULL)
            {
                if (attri->NameTStr().compare("heart_beat_interval") == 0)
                {
                    heart_beat_interval_ = std::stoi(attri->Value());
                    if (heart_beat_timer_ == NULL)
                    {
                        heart_beat_timer_ = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(heart_beat_interval_)));
                        heart_beat_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::heart_beat_timer_func, shared_from_this(), boost::asio::placeholders::error));
                    }
                }
                else if (attri->NameTStr().compare("robot_run_interval") == 0)
                {
                    robot_run_interval_ = std::stoi(attri->Value());
                    if (robot_run_timer_ == NULL)
                    {
                        robot_run_timer_ = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(robot_run_interval_)));
                        robot_run_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::robot_run_timer_func, shared_from_this(), boost::asio::placeholders::error));
                    }
                }
                else if (attri->NameTStr().compare("weather_interval") == 0)
                {
                    weather_interval_ = std::stoi(attri->Value());
                    if (weather_timer_ == NULL)
                    {
                        weather_timer_ = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(weather_interval_)));
                        weather_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::weather_timer_func, shared_from_this(), boost::asio::placeholders::error));
                    }
                }
                else
                {
                    ROS_ERROR("unknown attr:%s", attri->NameTStr().c_str());
                }
                attri = attri->Next();
            }
        }
    }
}

void LibDLIntelligentSocket::send_heart_beat_msg_req()
{
    if (!bConnected())
    {
        ROS_ERROR("send_heart_beat_msg_req: false! socket not connect!");
        return;
    }
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setType(MSG_TYPE_SYSTEM_MSG);
    msg->setCmd(2);
    msg->setTime(getFormatDateTime());
    msg->generiateMsg(static_sequence_id_++);

    ROS_ERROR("F/I.4.2.1: send_heart_beat_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::recv_heartbeat_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("心跳回应");
    ROS_ERROR("F/I.4.2.2: recv_heartbeat_msg_resp \n#xml:\n%s", msg->getString().c_str());
}

void LibDLIntelligentSocket::recv_ctrl_robot_body_remote_reset_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("远方复位");
    ROS_ERROR("F/I.5.1.1 #Type:1 #Command:1 :recv_ctrl_robot_body_remote_reset_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_body_system_check_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("系统自检");
    ROS_ERROR("F/I.5.1.1 #Type:1 #Command:2 :recv_ctrl_robot_body_system_check_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_body_one_key_return_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("一键返航");
    ROS_ERROR("F/I.5.1.1 #Type:1 #Command:3 :recv_ctrl_robot_body_one_key_return_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_body_manual_charge_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("手动充电");
    ROS_ERROR("F/I.5.1.1 #Type:1 #Command:4 :recv_ctrl_robot_body_manual_charge_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_body_switch_ctrl_mode_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("recv_ctrl_robot_body_switch_ctrl_mode_resp");
    ROS_ERROR("F/I.5.1.1 #Type:1 #Command:5 :recv_ctrl_robot_body_switch_ctrl_mode_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *item = msg->getItem();
    if (item->FirstAttribute() != NULL)
    {
        TiXmlAttribute *attri = item->FirstAttribute();
        while (attri != NULL)
        {
            if (attri->NameTStr().compare("value") == 0)
            {
                ROBOT_CTRL_MODE mode = (ROBOT_CTRL_MODE)std::stoi(attri->Value());
                switch (mode)
                {
                case ROBOT_MODE_UNKNOWN:
                {
                //    ROS_ERROR("未识别模式");
                    ROS_ERROR("Type:1 Command 5 Value:0 Status:ROBOT_MODE_UNKNOWN");
                    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(MSG_CODE_ERROR));
                    break;
                }
                case ROBOT_MODE_TASK:
                {
                //    ROS_ERROR("任务模式");
                    ROS_ERROR("Type:1 Command 5 Value:1 Status:ROBOT_MODE_TASK");
                    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
                    break;
                }
                case ROBOT_MODE_EMERGENCY_LOC:
                {
                //    ROS_ERROR("紧急定位模式");
                    ROS_ERROR("Type:1 Command 5 Value:2 Status:ROBOT_MODE_EMERGENCY_LOC");
                    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
                    break;
                }
                case ROBOT_MODE_BACKGROUND_CTRL:
                {
                //    ROS_ERROR("后台遥控模式");
                    ROS_ERROR("Type:1 Command 5 Value:3 Status:ROBOT_MODE_BACKGROUND_CTRL");
                    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
                    break;
                }
                case ROBOT_MODE_JOY_CTRL:
                {
                //    ROS_ERROR("手持遥控模式");
                    ROS_ERROR("Type:1 Command 5 Value:4 Status:ROBOT_MODE_JOY_CTRL");
                    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
                    break;
                }
                default:
                {
                //    ROS_ERROR("未识别模式 default");
                    ROS_ERROR("Type:1 Command 5 Value:0 Status:ROBOT_MODE_UNKNOWN");
                    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(MSG_CODE_ERROR));
                    break;
                }
                }
            }
            else
            {
                ROS_ERROR("unknown attr:%s", attri->NameTStr().c_str());
            }
            attri = attri->Next();
        }
    }
    else
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = i->FirstAttribute();
            while (attri != NULL)
            {
                if (attri->NameTStr().compare("value") == 0)
                {
                    ROBOT_CTRL_MODE mode = (ROBOT_CTRL_MODE)std::stoi(attri->Value());
                    switch (mode)
                    {
                    case ROBOT_MODE_UNKNOWN:
                    {
                    //    ROS_ERROR("未识别模式");
                        ROS_ERROR("Type:1 Command 5 Value:0 Status:ROBOT_MODE_UNKNOWN");
                        send_common_response_no_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(MSG_CODE_ERROR));
                        break;
                    }
                    case ROBOT_MODE_TASK:
                    {
                    //    ROS_ERROR("任务模式");
                        ROS_ERROR("Type:1 Command 5 Value:1 Status:ROBOT_MODE_TASK");
                        send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
                        break;
                    }
                    case ROBOT_MODE_EMERGENCY_LOC:
                    {
                    //    ROS_ERROR("紧急定位模式");
                        ROS_ERROR("Type:1 Command 5 Value:2 Status:ROBOT_MODE_EMERGENCY_LOC");
                        send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
                        break;
                    }
                    case ROBOT_MODE_BACKGROUND_CTRL:
                    {
                    //    ROS_ERROR("后台遥控模式");
                        ROS_ERROR("Type:1 Command 5 Value:3 Status:ROBOT_MODE_BACKGROUND_CTRL");
                        send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
                        break;
                    }
                    case ROBOT_MODE_JOY_CTRL:
                    {
                    //    ROS_ERROR("手持遥控模式");
                        ROS_ERROR("Type:1 Command 5 Value:4 Status:ROBOT_MODE_JOY_CTRL");
                        send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
                        break;
                    }
                    default:
                    {
                    //    ROS_ERROR("未识别模式 default");
                        ROS_ERROR("Type:1 Command 5 Value:0 Status:ROBOT_MODE_UNKNOWN");
                        send_common_response_no_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(MSG_CODE_ERROR));
                        break;
                    }
                    }
                }
                else
                {
                    ROS_ERROR("unknown attr:%s", attri->NameTStr().c_str());
                }
                attri = attri->Next();
            }
        }
    }
}

void LibDLIntelligentSocket::recv_ctrl_robot_body_access_to_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("控制权获得");
    ROS_ERROR("F/I.5.1.1 #Type:1 #Command:6 :recv_ctrl_robot_body_access_to_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_body_release_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("控制权释放");
    ROS_ERROR("F/I.5.1.1 #Type:1 #Command:7 :recv_ctrl_robot_body_release_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_base_forward_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人车体-前进");
    ROS_ERROR("F/I.5.1.1 #Type:2 #Command:1 :recv_ctrl_robot_base_forward_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_base_backward_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人车体-后退");
    ROS_ERROR("F/I.5.1.1 #Type:2 #Command:2 :recv_ctrl_robot_base_backward_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_base_turn_left_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人车体-左转");
    ROS_ERROR("F/I.5.1.1 #Type:2 #Command:3 :recv_ctrl_robot_base_turn_left_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_base_turn_right_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人车体-右转");
    ROS_ERROR("F/I.5.1.1 #Type:2 #Command:4 :recv_ctrl_robot_base_turn_right_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_base_rotate_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人车体-转弯");
    ROS_ERROR("F/I.5.1.1 #Type:2 #Command:5 :recv_ctrl_robot_base_rotate_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_base_stop_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人车体-停止");
    ROS_ERROR("F/I.5.1.1 #Type:2 #Command:6 :recv_ctrl_robot_base_stop_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_up_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-上仰");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:1 :recv_ctrl_robot_ptz_up_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_down_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-下俯");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:2 :recv_ctrl_robot_ptz_down_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_left_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-左转");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:3 :recv_ctrl_robot_ptz_left_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_right_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-右转");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:4 :recv_ctrl_robot_ptz_right_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_rise_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-上升");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:5 :recv_ctrl_robot_ptz_rise_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_fall_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-下降");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:6 :recv_ctrl_robot_ptz_fall_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_preset_bit_call_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-预置位调用");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:7 :recv_ctrl_robot_ptz_preset_bit_call_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_stop_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-停止");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:8 :recv_ctrl_robot_ptz_stop_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_ptz_reset_control_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人云台-复位");
    ROS_ERROR("F/I.5.1.1 #Type:3 #Command:9 :recv_ctrl_robot_ptz_reset_control_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_auxiliary_equipment_infrared_power_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人辅助设备-红外电源");
    ROS_ERROR("F/I.5.1.1 #Type:4 #Command:1 :recv_ctrl_robot_auxiliary_equipment_infrared_power_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_auxiliary_equipment_wiper_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人辅助设备-雨刷");
    ROS_ERROR("F/I.5.1.1 #Type:4 #Command:2 :recv_ctrl_robot_auxiliary_equipment_wiper_resp \n#xml:\n%s", msg->getString().c_str());
    if (bWiper)
    {
        bWiper = false;
    }
    else
    {
        bWiper = true;
    }
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_auxiliary_equipment_snoise_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人辅助设备-超声");
    ROS_ERROR("F/I.5.1.1 #Type:4 #Command:3 :recv_ctrl_robot_auxiliary_equipment_snoise_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_auxiliary_equipment_infrared_light_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("机器人辅助设备-红外射灯");
    ROS_ERROR("F/I.5.1.1 #Type:4 #Command:4 :recv_ctrl_robot_auxiliary_equipment_infrared_light_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_zoom_in_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机 - 镜头拉近");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:1 :recv_ctrl_robot_visible_camera_zoom_in_resp \n#xml:\n%s", msg->getString().c_str());
    m_HCNetCamera->CameraZoomIn();
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_zoom_out_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-镜头拉远");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:2 :recv_ctrl_robot_visible_camera_zoom_out_resp \n#xml:\n%s", msg->getString().c_str());
    m_HCNetCamera->CameraZoomOut();
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_focus_stop_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-镜头拉焦停止");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:3 :recv_ctrl_robot_visible_camera_focus_stop_resp \n#xml:\n%s", msg->getString().c_str());
    m_HCNetCamera->CameraZoomFocusStop();
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_focus_in_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-焦距增加");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:4 :recv_ctrl_robot_visible_camera_focus_in_resp \n#xml:\n%s", msg->getString().c_str());
    m_HCNetCamera->CameraFocusFar();
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_focus_out_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-焦距减少");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:5 :recv_ctrl_robot_visible_camera_focus_out_resp \n#xml:\n%s", msg->getString().c_str());
    m_HCNetCamera->CameraFocusNear();
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_auto_focus_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-自动聚焦");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:6 :recv_ctrl_robot_visible_camera_auto_focus_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_capture_img_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-抓图");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:7 :recv_ctrl_robot_visible_camera_capture_img_resp \n#xml:\n%s", msg->getString().c_str());
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");
    QString filePath = QString("D:/RCF_Server_Root/VisibleLightCapture/") + current_date;
    m_HCNetCamera->getImage(filePath.toStdString());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_restart_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-重启");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:8 :recv_ctrl_robot_visible_camera_restart_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_start_rec_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-启动录像");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:9 :recv_ctrl_robot_visible_camera_start_rec_resp \n#xml:\n%s", msg->getString().c_str());
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");
    QString filePath = QString("D:/RCF_Server_Root/VisibleLightVideo/") + current_date + QString(".mp4");
    m_HCNetCamera->startRecord(filePath.toStdString());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_stop_rec_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("可见光相机-停止录像");
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:10 :recv_ctrl_robot_visible_camera_stop_rec_resp \n#xml:\n%s", msg->getString().c_str());
    m_HCNetCamera->stopRecord();
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_zoom_set_rec_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:11 :recv_ctrl_robot_visible_camera_zoom_set_rec_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *item = msg->getItem()->FirstChildElement();
    TiXmlAttribute *attri = item->FirstAttribute();
    int zoom = 0;
    while (attri != NULL)
    {
        if (attri->NameTStr().compare("value") == 0)
        {
            zoom = (atoi(attri->Value()) - 1) * 564;
            if (zoom > 16384)
            {
                zoom = 16384;
            }
            if (zoom < 0)
            {
                zoom = 0;
            }
        }
        else
        {
            ROS_ERROR("recv_ctrl_robot_visible_camera_zoom_set_rec_resp: alay value error!");
        }
        attri = attri->Next();
    }
    m_HCNetCamera->setZoom(zoom);
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_visible_camera_focus_set_rec_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("F/I.5.1.1 #Type:21 #Command:12 :recv_ctrl_robot_visible_camera_focus_set_rec_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *item = msg->getItem()->FirstChildElement();
    TiXmlAttribute *attri = item->FirstAttribute();
    int focus = 0;
    while (attri != NULL)
    {
        if (attri->NameTStr().compare("value") == 0)
        {
            focus = atoi(attri->Value()) + 31874;
            if (focus > 33128)
            {
                focus = 33128;
            }
            if (focus < 31874)
            {
                focus = 31874;
            }
        }
        else
        {
            ROS_ERROR("recv_ctrl_robot_visible_camera_focus_set_rec_resp: alay value error!");
        }
        attri = attri->Next();
    }
    m_HCNetCamera->setFocus(focus);
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_undefined_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("红外热成像仪-未定义);
    ROS_ERROR("F/I.5.1.1 #Type:22 #Command:0-4 :recv_ctrl_robot_infrared_camera_undefined_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_set_focus_val_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("红外热成像仪-设定焦距值");
    ROS_ERROR("F/I.5.1.1 #Type:22 #Command:5 :recv_ctrl_robot_infrared_camera_set_focus_val_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *item = msg->getItem()->FirstChildElement();
    TiXmlAttribute *attri = item->FirstAttribute();
    taskAssignAtOnce taskCfg;
    int focus;
    while (attri != NULL)
    {
        if (attri->NameTStr().compare("value") == 0)
        {
            focus = atoi(attri->Value());
        }
        else
        {
            ROS_ERROR("recv_ctrl_robot_infrared_camera_set_focus_val_resp: alay value error!");
        }
        attri = attri->Next();
    }
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_auto_focus_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("红外热成像仪-自动聚焦");
    ROS_ERROR("F/I.5.1.1 #Type:22 #Command:6 :recv_ctrl_robot_infrared_camera_auto_focus_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_capture_img_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("红外热成像仪-抓图");
    ROS_ERROR("F/I.5.1.1 #Type:22 #Command:7 :recv_ctrl_robot_infrared_camera_capture_img_resp \n#xml:\n%s", msg->getString().c_str());
    QString date = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_ctrl_robot_infrared_camera_restart_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("红外热成像仪-重启");
    ROS_ERROR("F/I.5.1.1 #Type:22 #Command:8 :recv_ctrl_robot_infrared_camera_restart_resp \n#xml:\n%s", msg->getString().c_str());
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_task_ctrl_start_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("任务控制-任务启动");
    ROS_ERROR("F/I.5.2.1 #Type:41 #Command:1 :recv_task_ctrl_start_msg_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("task_patrolled_id", getSessionId());

    send_common_response_with_item(msg->getReceiveCode(), msg->getSequenceId(), std::to_string(MSG_CODE_SUCCESS), xmlItems);
}

void LibDLIntelligentSocket::recv_task_ctrl_pause_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("任务控制-任务暂停");
    ROS_ERROR("F/I.5.2.1 #Type:41 #Command:2 :recv_task_ctrl_pause_msg_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
//    xmlItem->SetAttribute("task_patrolled_id", getSessionId());


    send_common_response_with_item(msg->getReceiveCode(), msg->getSequenceId(), std::to_string(MSG_CODE_SUCCESS), xmlItems);
}

void LibDLIntelligentSocket::recv_task_ctrl_resume_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("任务控制-任务继续");
    ROS_ERROR("F/I.5.2.1 #Type:41 #Command:3 :recv_task_ctrl_resume_msg_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
//    xmlItem->SetAttribute("task_patrolled_id", getSessionId());

    send_common_response_with_item(msg->getReceiveCode(), msg->getSequenceId(), std::to_string(MSG_CODE_SUCCESS), xmlItems);
}

void LibDLIntelligentSocket::recv_task_ctrl_stop_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("任务控制-任务停止");
    ROS_ERROR("F/I.5.2.1 #Type:41 #Command:4 :recv_task_ctrl_stop_msg_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
//    xmlItem->SetAttribute("task_patrolled_id", getSessionId());
    

    send_common_response_with_item(msg->getReceiveCode(), msg->getSequenceId(), std::to_string(MSG_CODE_SUCCESS), xmlItems);
}

void LibDLIntelligentSocket::recv_model_sync_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("模型同步");
    ROS_ERROR("F/I.5.3.1 #Type:61 #Command:1 :recv_model_sync_resp \n#xml:\n%s", msg->getString().c_str());

    QString code = QString::fromStdString(msg->getCode());
//    QString code = "15";

    std::string dateModel = QDateTime::currentDateTime().toString("yyyyMMddhhmmss").toStdString();
    std::string deviceModel = code.toStdString() + "/Model/device_model_" + robot_main_code_ + ".xml";
    std::string robotModel = code.toStdString() + "/Model/robot_model_" + robot_main_code_ + ".xml";
    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("device_file_path", deviceModel);
    xmlItem->SetAttribute("robot_file_path", robotModel);

    model_file_create();

    ftps_client_->upLoadFile(root_core_path_ + "/model/device_model.xml", code + "/Model", "device_model_" + QString::fromStdString(robot_main_code_) + ".xml");
    ftps_client_->upLoadFile(root_core_path_ + "/model/robot_model.xml", code + "/Model", "robot_model_" + QString::fromStdString(robot_main_code_) + ".xml");
    ftps_client_->upLoadFile(root_core_path_ + "/map/map.jpg", code + "/Map", QString::fromStdString(robot_main_code_) + "_1.jpg");

    send_common_response_with_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(MSG_CODE_SUCCESS), xmlItems);
}

void LibDLIntelligentSocket::recv_task_assign_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("任务下发-任务配置");
    ROS_ERROR("F/I.5.4.1 #Type:101 #Command:1 :recv_task_assign_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *item = msg->getItem()->FirstChildElement();
    TiXmlAttribute *attri = item->FirstAttribute();
    taskAssignWhole taskCfg;
    while (attri != NULL)
    {
        if (attri->NameTStr().compare("type") == 0)
        {
            taskCfg.type = attri->Value();
        }
        else if (attri->NameTStr().compare("task_code") == 0)
        {
            taskCfg.task_code = attri->Value();
        }
        else if (attri->NameTStr().compare("task_name") == 0)
        {
            taskCfg.task_name = attri->Value();
        }
        else if (attri->NameTStr().compare("priority") == 0)
        {
            taskCfg.priority = attri->Value();
        }
        else if (attri->NameTStr().compare("device_level") == 0)
        {
            taskCfg.device_level = attri->Value();
        }
        else if (attri->NameTStr().compare("device_list") == 0)
        {
            taskCfg.device_list = attri->Value();
        }
        else if (attri->NameTStr().compare("fixed_start_time") == 0)
        {
            taskCfg.fixed_start_time = attri->Value();
        }
        else if (attri->NameTStr().compare("cycle_month") == 0)
        {
            taskCfg.cycle_month = attri->Value();
        }
        else if (attri->NameTStr().compare("cycle_week") == 0)
        {
            taskCfg.cycle_week = attri->Value();
        }
        else if (attri->NameTStr().compare("cycle_execute_time") == 0)
        {
            taskCfg.cycle_execute_time = attri->Value();
        }
        else if (attri->NameTStr().compare("cycle_start_time") == 0)
        {
            taskCfg.cycle_start_time = attri->Value();
        }
        else if (attri->NameTStr().compare("cycle_end_time") == 0)
        {
            taskCfg.cycle_end_time = attri->Value();
        }
        else if (attri->NameTStr().compare("interval_number") == 0)
        {
            taskCfg.interval_number = attri->Value();
        }
        else if (attri->NameTStr().compare("interval_type") == 0)
        {
            taskCfg.interval_type = attri->Value();
        }
        else if (attri->NameTStr().compare("interval_execute_time") == 0)
        {
            taskCfg.interval_execute_time = attri->Value();
        }
        else if (attri->NameTStr().compare("interval_start_time") == 0)
        {
            taskCfg.interval_start_time = attri->Value();
        }
        else if (attri->NameTStr().compare("interval_end_time") == 0)
        {
            taskCfg.interval_end_time = attri->Value();
        }
        else if (attri->NameTStr().compare("invalid_start_time") == 0)
        {
            taskCfg.invalid_start_time = attri->Value();
        }
        else if (attri->NameTStr().compare("invalid_end_time") == 0)
        {
            taskCfg.invalid_end_time = attri->Value();
        }
        else if (attri->NameTStr().compare("isenable") == 0)
        {
            taskCfg.isenable = attri->Value();
        }
        else if (attri->NameTStr().compare("creator") == 0)
        {
            taskCfg.creator = attri->Value();
        }
        else if (attri->NameTStr().compare("create_time") == 0)
        {
            taskCfg.create_time = attri->Value();
        }
        else
        {
        //    ROS_ERROR("    模型下发-机器人配置 未识别属性");
            ROS_ERROR("recv_task_assign_atonce_resp:task_assign error!");
        }
        attri = attri->Next();
    }

    WheelTaskEditStruct stru;
    stru.task_edit_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
    stru.task_edit_name = QString::fromStdString(taskCfg.task_name);
    
    if (taskCfg.type == "1")
    {
        stru.task_edit_type_id = WHEEL_TOTAL_PATROL;
    }
    if (taskCfg.type == "2")
    {
        stru.task_edit_type_id = WHEEL_ROUTINE_PATROL;
    }
    if (taskCfg.type == "3")
    {
        stru.task_edit_type_id = WHEEL_USER_DEFINED_TASK;
    }
    if (taskCfg.type == "4")
    {
        stru.task_edit_type_id = WHEEL_USER_DEFINED_TASK;
    }

    
    WheelTaskTemplateStruct temp;
    if (taskCfg.fixed_start_time != "")
    {
        stru.task_edit_date = QString::fromStdString(taskCfg.fixed_start_time);

        temp.task_template_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
        temp.task_edit_uuid = stru.task_edit_uuid;
        temp.task_type_id = WHEEL_ROBOT_TASK_TIMED_TASK;
        temp.task_end_action_id = WHEEL_ROBOT_TASK_END_TYPE_TO_CHARGE;
        temp.task_template_name = stru.task_edit_name;
        temp.task_status_id = WHEEL_ROBOT_TASK_STATUS_WAIT;
        temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_NONE;
        temp.task_repeat_duration = 0;
        QStringList startList = QString::fromStdString(taskCfg.fixed_start_time).split(" ");
        temp.task_start_date = startList[0];
        temp.task_start_time = QTime::fromString(startList[1], "hh:mm:ss");
    //    WHEEL_CORE_SERVER.robot_bj_task_template_insert_req(temp);
    }
    else
    {
    //    WheelTaskTemplateStruct temp;
        temp.task_template_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
        temp.task_edit_uuid = stru.task_edit_uuid;
        temp.task_type_id = WHEEL_ROBOT_TASK_LOOP_TASK;
        temp.task_end_action_id = WHEEL_ROBOT_TASK_END_TYPE_TO_CHARGE;
        temp.task_template_name = stru.task_edit_name;
        temp.task_status_id = WHEEL_ROBOT_TASK_STATUS_WAIT;
        temp.task_repeat_duration = 0;

        if (taskCfg.interval_type == "2" && taskCfg.interval_number == "1")
        {
            stru.task_edit_date = QString::fromStdString(taskCfg.cycle_start_time);
            temp.task_start_time = QTime::fromString(QString::fromStdString(taskCfg.interval_execute_time), "hh:mm:ss");
            temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_DAY;
        //    WHEEL_CORE_SERVER.robot_bj_task_template_insert_req(temp);
        }
        if (taskCfg.cycle_week != "")
        {
            stru.task_edit_date = QString::fromStdString(taskCfg.cycle_start_time);
            temp.task_start_date = QString::fromStdString(taskCfg.cycle_week);
            temp.task_start_time = QTime::fromString(QString::fromStdString(taskCfg.cycle_execute_time), "hh:mm:ss");
            temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_WEEK;
        //    WHEEL_CORE_SERVER.robot_bj_task_template_insert_req(temp);
        }
        if (taskCfg.cycle_month != "")
        {
            stru.task_edit_date = QString::fromStdString(taskCfg.cycle_start_time);
            temp.task_start_date = QString::fromStdString(taskCfg.cycle_month);
            temp.task_start_time = QTime::fromString(QString::fromStdString(taskCfg.cycle_execute_time), "hh:mm:ss");
            temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_MONTH;
        //    WHEEL_CORE_SERVER.robot_bj_task_template_insert_req(temp);
        }
        if (taskCfg.interval_type == "2" && taskCfg.interval_number != "1"&&taskCfg.interval_number != "0")
        {
            stru.task_edit_date = QString::fromStdString(taskCfg.interval_start_time);
            temp.task_repeat_duration = QString::fromStdString(taskCfg.interval_number).toInt();
            temp.task_start_date = QString::fromStdString(taskCfg.interval_start_time).split(" ")[0];
            temp.task_start_time = QTime::fromString(QString::fromStdString(taskCfg.interval_execute_time), "hh:mm:ss");
            temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_FIXED_INTERVAL_DAYS;
        //    WHEEL_CORE_SERVER.robot_bj_task_template_insert_req(temp);
        }
    }

    QString retMsg;
    QStringList deviceIdList = QString::fromStdString(taskCfg.device_list).split(",");
    bool bRet = WHEEL_ROBOT_DB.getBJDeviceUUidWithDeviceId(deviceIdList);
    bRet = WHEEL_ROBOT_DB.insertTaskEditAndInsertTaskDevicesDB(stru, deviceIdList, retMsg);
    WHEEL_CORE_SERVER.Remote_robot_task_edit_insert_resp(int(stru.task_edit_type_id), bRet, std::string(retMsg.toLocal8Bit()));


    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_task_assign_atonce_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
//    ROS_ERROR("任务下发-任务配置-立即任务");
    ROS_ERROR("F/I.5.4.3 #Type:103 #Command:1 :recv_task_assign_atonce_resp \n#xml:\n%s", msg->getString().c_str());
    TiXmlElement *item = msg->getItem()->FirstChildElement();
    TiXmlAttribute *attri = item->FirstAttribute();
    taskAssignAtOnce taskCfg;
    while (attri != NULL)
    {
        if (attri->NameTStr().compare("task_code") == 0)
        {
            taskCfg.task_code = attri->Value();
        }
        else if (attri->NameTStr().compare("task_name") == 0)
        {
            taskCfg.task_name = attri->Value();
        }
        else if (attri->NameTStr().compare("priority") == 0)
        {
            taskCfg.priority = attri->Value();
        }
        else if (attri->NameTStr().compare("device_level") == 0)
        {
            taskCfg.device_level = attri->Value();
        }
        else if (attri->NameTStr().compare("device_list") == 0)
        {
            taskCfg.device_list = attri->Value();
        }
        else
        {
        //    ROS_ERROR("    模型下发-机器人配置 未识别属性");
            ROS_ERROR("recv_task_assign_atonce_resp:task_assign_atonce error!");
        }
        attri = attri->Next();
    }
    WheelRobotAssignTask task;
    task.task_code = QString::fromStdString(taskCfg.task_code);
    task.task_name = QString::fromStdString(taskCfg.task_name);
    task.priority = WheelRobotTaskpriority(atoi(taskCfg.priority.c_str()));
    task.task_edit_uuid = "";
    task.task_template_uuid = "";
    task.dev_optimize = true;
    task.task_end_action = WHEEL_ROBOT_TASK_END_TYPE_TO_CHARGE;
    task.breakTask = true;
    task.task_uuid = QUuid::createUuid().toString().remove("{").remove("}").remove("-");
//         QUuid id = QUuid::createUuid();
//     QString strId = id.toString();

    QStringList devList;
    switch (atoi(taskCfg.device_level.c_str()))
    {
    case 1:
        WHEEL_ROBOT_DB.getBJDeviceUUidWithEquipment(devList, QString::fromStdString(taskCfg.device_level));
        break;
    case 2:
        WHEEL_ROBOT_DB.getBJDeviceUUidWithDevicetype(devList, QString::fromStdString(taskCfg.device_level));
        break;
    case 3:
        devList = QString::fromStdString(taskCfg.device_list).split(",");
        break;
    default:
        break;
    }

    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::slot_robot_state_data_msg()
{
    if (!bConnected())
    {
        ROS_ERROR("slot_robot_state_data_msg: false! socket not connect!");
        return;
    }

    robotStatus status;
    status.robot_name = robot_name_;
    status.robot_code = robot_code_;
    status.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
  
}

void LibDLIntelligentSocket::send_robot_status_msg_req(robotStatus val)
{
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());

    msg->setType(1);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    for (int i = 0; i < val.data.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_name", val.robot_name);
        xmlItem->SetAttribute("robot_code", val.robot_code);
        xmlItem->SetAttribute("time", val.time);
        xmlItem->SetAttribute("type", val.data[i].type);
        xmlItem->SetAttribute("value", val.data[i].value);
        xmlItem->SetAttribute("value_unit", val.data[i].value_unit);
        xmlItem->SetAttribute("unit", val.data[i].unit);
    }
    msg->setItem(xmlItems);
    msg->generiateMsg(static_sequence_id_++);

    ROS_ERROR("F/I.6.1.1 #Type:1 :send_robot_status_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_status_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{

}

void LibDLIntelligentSocket::send_robot_running_status_msg_req(robotRunningStatus val)
{
    if (!bConnected())
    {
        ROS_ERROR("send_robot_running_status_msg_req: false! socket not connect!");
        return;
    }
    ROS_ERROR("send_robot_running_status_msg_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());

    msg->setType(2);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    for (int i = 0; i < val.data.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_name", val.robot_name);
        xmlItem->SetAttribute("robot_code", val.robot_code);
        xmlItem->SetAttribute("time", val.time);
        xmlItem->SetAttribute("type", val.data[i].type);
        xmlItem->SetAttribute("value", val.data[i].value);
        xmlItem->SetAttribute("value_unit", val.data[i].value_unit);
        xmlItem->SetAttribute("unit", val.data[i].unit);
    }
    msg->setItem(xmlItems);

    msg->generiateMsg(static_sequence_id_++);
    ROS_ERROR("F/I.6.2.1 #Type:1 :send_robot_running_status_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_running_status_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{

}

void LibDLIntelligentSocket::slot_robot_coordinate_msg(std::string coord)
{
    if (!bConnected())
    {
        ROS_ERROR("slot_robot_coordinate_msg: false! socket not connect!");
        return;
    }
    robotCoordinate data;
    data.robot_code = robot_code_;
    data.robot_name = robot_name_;
    data.file_path = "map.jpg";
    data.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
    data.coordinate_pixel = coord;
//    data.coordinate_geography = "5,7,8,9";
    send_robot_coordinate_msg_req(data);
}

void LibDLIntelligentSocket::send_robot_coordinate_msg_req(robotCoordinate val)
{
    ROS_ERROR("send_robot_coordinate_msg_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());

    msg->setType(3);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("robot_name", val.robot_name);
    xmlItem->SetAttribute("file_path", val.file_path);
    xmlItem->SetAttribute("robot_code", val.robot_code);
    xmlItem->SetAttribute("time", val.time);
    xmlItem->SetAttribute("coordinate_pixel", val.coordinate_pixel);
    xmlItem->SetAttribute("coordinate_geography", val.coordinate_geography);
    msg->setItem(xmlItems);

    msg->generiateMsg(static_sequence_id_++);
    ROS_ERROR("F/I.6.3.1 #Type:1 :send_robot_coordinate_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_coordinate_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
}

void LibDLIntelligentSocket::slot_robot_patrol_line_msg(std::vector<std::string> pointIdVec)
{
    if (!bConnected())
    {
        ROS_ERROR("slot_robot_patrol_line_msg: false! socket not connect!");
        return;
    }
    robotCoordinate val;
    val.robot_code = "Robot1";
    val.robot_name = "testRobot1";
    //val.file_path = "D:\\Test\\File.jpg";
    val.file_path = "";
    val.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
    send_robot_patrol_line_msg_req(val, pointIdVec);
}

void LibDLIntelligentSocket::send_robot_patrol_line_msg_req(robotCoordinate val, std::vector<std::string> pointIdVec)
{
    ROS_ERROR("send_robot_patrol_line_msg_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    std::string now_time = getFormatDateTime();
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(now_time);
    msg->setType(4);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    for (int i = 0; i < pointIdVec.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_name", robot_name_);
        xmlItem->SetAttribute("file_path", "");
        xmlItem->SetAttribute("robot_code", robot_code_);
        xmlItem->SetAttribute("time", now_time);
        xmlItem->SetAttribute("coordinate_pixel", pointIdVec[i]);
        xmlItem->SetAttribute("coordinate_geography", "");
    }
    msg->setItem(xmlItems);
    msg->generiateMsg(static_sequence_id_++);
    ROS_ERROR("F/I.6.4.1 #Type:1 :send_robot_patrol_line_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_patrol_line_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{

}

void LibDLIntelligentSocket::send_robot_micro_weather_req(robotMicroWeatherData val)
{
    if (!bConnected())
    {
        ROS_ERROR("send_robot_micro_weather_req: false! socket not connect!");
        return;
    }

    ROS_ERROR("send_robot_micro_weather_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());
    msg->setType(21);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    for (int i = 0; i < val.data.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_name", val.robot_name);
        xmlItem->SetAttribute("robot_code", val.robot_code);
        xmlItem->SetAttribute("time", val.time);
        xmlItem->SetAttribute("type", val.data[i].type);
        xmlItem->SetAttribute("value", val.data[i].value);
        xmlItem->SetAttribute("value_unit", val.data[i].value_unit);
        xmlItem->SetAttribute("unit", val.data[i].unit);
    }
    msg->setItem(xmlItems);

    msg->generiateMsg(static_sequence_id_++);
    ROS_ERROR("F/I.6.6.1 #Type:1 :send_robot_micro_weather_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_micro_weather_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{

}

void LibDLIntelligentSocket::slot_robot_task_data_msg(WheelRobotCurrentTaskInfoShow taskData)
{
    if (!bConnected())
    {
        ROS_ERROR("slot_robot_task_data_msg: false! socket not connect!");
        return;
    }

    robotTaskData val;
    BJRobotTaskStatusData data;
    WHEEL_ROBOT_DB.getBJTaskData(taskData.task_uuid, data);
    switch (WheelRobotTaskStatusType(data.task_state.toInt()))
    {
    case WHEEL_ROBOT_TASK_STATUS_WAIT:
        val.task_state = "3";
        break;
    case WHEEL_ROBOT_TASK_STATUS_FINISH:
        val.task_state = "1";
        break;
    case WHEEL_ROBOT_TASK_STATUS_EXEC:
        val.task_state = "2";
        break;
    case WHEEL_ROBOT_TASK_STATUS_ABORT:
        val.task_state = "4";
        break;
    case WHEEL_ROBOT_TASK_STATUS_OVERTIME:
        val.task_state = "6";
        break;
    case WHEEL_ROBOT_TASK_STATUS_HANGUP:
        val.task_state = "4";
        break;
    case WHEEL_ROBOT_TASK_STATUS_HANGDOWN:
        val.task_state = "3";
        break;
    default:
        val.task_state = "5";
        break;
    }
    val.task_patrolled_id = taskData.task_uuid.toStdString();
    val.task_name = taskData.task_name.toStdString();
    val.task_code = data.task_code.toStdString();
    val.plan_start_time = data.task_start_time.toStdString();
    val.start_time = data.task_start_time.toStdString();
    val.task_progress = std::to_string(int(taskData.percent * 100)) + "%";
    val.task_estimated_time = std::to_string(int(taskData.predict_duration / 60)) + "min";
    val.description = "this a task!";

    send_robot_task_data_msg_req(val);
}

void LibDLIntelligentSocket::send_robot_task_data_msg_req(robotTaskData val)
{
    ROS_ERROR("send_robot_task_data_msg_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());

    msg->setType(41);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("task_patrolled_id", val.task_patrolled_id);
    xmlItem->SetAttribute("task_name", val.task_name);
    xmlItem->SetAttribute("task_code", val.task_code);
    xmlItem->SetAttribute("task_state", val.task_state);
    xmlItem->SetAttribute("plan_start_time", val.plan_start_time);
    xmlItem->SetAttribute("start_time", val.start_time);
    xmlItem->SetAttribute("task_progress", val.task_progress);
    xmlItem->SetAttribute("task_estimated_time", val.task_estimated_time);
    xmlItem->SetAttribute("description", val.description);
    msg->setItem(xmlItems);

    msg->generiateMsg(static_sequence_id_++);
    ROS_ERROR("F/I.6.7.1 #Type:1 :send_robot_task_data_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_task_data_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    
}

void LibDLIntelligentSocket::send_robot_alarm_msg_req(robotAlarmData val)
{
    if (!bConnected())
    {
        ROS_ERROR("send_robot_alarm_msg_req: false! socket not connect!");
        return;
    }

    ROS_ERROR("ftp send_robot_alarm_msg_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());

    msg->setType(5);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("robot_name", val.robot_name);
    xmlItem->SetAttribute("robot_code", val.robot_code);
    xmlItem->SetAttribute("time", val.time);
    xmlItem->SetAttribute("content", val.content);
    msg->setItem(xmlItems);

    msg->generiateMsg(static_sequence_id_++);
    ROS_ERROR("F/I.6.5.1 #Type:1 :send_robot_alarm_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_alarm_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{

}

void LibDLIntelligentSocket::slot_robot_task_device_send_msg(QString taskUUid, QString deviceUUid)
{
    if (!bConnected())
    {
        ROS_ERROR("send_robot_patrol_result_msg_req: false! socket not connect!");
        return;
    }
    BJDeviceResultSendData data;
    WHEEL_ROBOT_DB.getBJDeviceResultFromDB(taskUUid, deviceUUid, data);
    data.value_unit = data.value + data.unit;
    data.recognition_type = m_IntelligentData->contrast_recognition_type_with_key(data.recognition_type);
    data.file_type = m_IntelligentData->contrast_collect_type_with_recognition_type(data.recognition_type);

    robotPatrolResult val;
    val.robot_code = robot_code_;
    val.task_name = data.task_name.toStdString();
    val.task_code = data.task_code.toStdString();
    val.task_patrolled_id = data.task_patrolled_id.toStdString();

    robotDeviceUploadType val_data;
    val_data.device_name = data.device_name.toStdString();
    val_data.device_id = data.device_id.toStdString();
    val_data.value = data.value.toStdString();
    val_data.unit = data.unit.toStdString();
    val_data.value_unit = val_data.value + val_data.unit;
    val_data.time = data.time.toStdString();
    val_data.recognition_type = data.recognition_type.toStdString();
    val_data.file_type = data.file_type.toStdString();
    val_data.file_path = data.file_path.toStdString();
    val_data.rectangle = data.rectangle.toStdString();

    val.data.push_back(val_data);
    send_robot_patrol_result_msg_req(val);
}

void LibDLIntelligentSocket::send_robot_patrol_result_msg_req(robotPatrolResult val)
{
    if (!bConnected())
    {
        ROS_ERROR("send_robot_patrol_result_msg_req: false! socket not connect!");
        return;
    }

    ROS_ERROR("send_robot_patrol_result_msg_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());

    msg->setType(61);

    TiXmlElement *xmlItems = new TiXmlElement("Items");

    for (int i = 0; i < val.data.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_code", val.robot_code);
        xmlItem->SetAttribute("task_name", val.task_name);
        xmlItem->SetAttribute("task_code", val.task_code);
        xmlItem->SetAttribute("device_name", val.data[i].device_name);
        xmlItem->SetAttribute("device_id", val.data[i].device_id);
        xmlItem->SetAttribute("value", val.data[i].value);
        xmlItem->SetAttribute("value_unit", val.data[i].value_unit);
        xmlItem->SetAttribute("unit", val.data[i].unit);
        xmlItem->SetAttribute("time", val.data[i].time);
        xmlItem->SetAttribute("recognition_type", val.data[i].recognition_type);
        xmlItem->SetAttribute("file_type", val.data[i].file_type);
        xmlItem->SetAttribute("file_path", val.data[i].file_path);
        xmlItem->SetAttribute("rectangle", val.data[i].rectangle);
        xmlItem->SetAttribute("task_patrolled_id", val.task_patrolled_id);
    }
    msg->setItem(xmlItems);

    msg->generiateMsg(static_sequence_id_++);
    ROS_ERROR("F/I.6.8.1 #Type:1 :send_robot_patrol_result_msg_req \n#xml:\n%s", msg->getString().c_str());
    m_socket->postMsg(msg);

    QString rootPath = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/task/" + QString::fromStdString(val.task_patrolled_id) + "/";
    for (int i = 0; i < val.data.size(); i++)
    {
        //ftps
        QString devicePath = rootPath + QString::fromStdString(val.data[i].device_id) + "/";
        QDateTime dateTime = QDateTime::fromString(QString::fromStdString(val.data[i].time), "yyyy-MM-dd hh:mm:ss");
        
        int iyear = dateTime.date().year();
        int imonth = dateTime.date().month();
        int iday = dateTime.date().day();

        QString sendPath = QString::fromStdString(station_code_) + QString("/%1/%2/%3/").arg(iyear).arg(imonth).arg(iday) + QString::fromStdString(val.task_code);
        QString sendFile = QString::fromStdString(val.data[i].device_id) + "_" + QString::fromStdString(robot_code_) + QString::fromStdString(val.data[i].time).remove("-").remove(" ").remove(":");

        if (val.data[i].recognition_type == "1" || val.data[i].recognition_type == "2")
        {
            sendPath = sendPath + "/CCD";
            ftps_client_->upLoadFile(devicePath + QString::fromStdString(val.data[i].device_id) + "_result.jpg", sendPath, sendFile + ".jpg");
            Sleep(100);
            ftps_client_->upLoadFile(devicePath + QString::fromStdString(val.data[i].device_id) + ".jpg", sendPath, sendFile + "_raw.jpg");
        }
        else if (val.data[i].recognition_type == "3")
        {
            sendPath = sendPath + "/CCD";
            ftps_client_->upLoadFile(devicePath + QString::fromStdString(val.data[i].device_id) + ".jpg", sendPath, sendFile + "_raw.jpg");
        }
        else if (val.data[i].recognition_type == "4")
        {
            sendPath = sendPath + "/FIR";
            ftps_client_->upLoadFile(devicePath + QString::fromStdString(val.data[i].device_id) + ".jpg", sendPath, sendFile + ".jpg");
            Sleep(100);
            ftps_client_->upLoadFile(devicePath + QString::fromStdString(val.data[i].device_id) + ".fir", sendPath, sendFile + ".fir");
        }
        else if (val.data[i].recognition_type == "3,4")
        {
            ftps_client_->upLoadFile(devicePath + QString::fromStdString(val.data[i].device_id) + ".jpg", sendPath + "/CCD", sendFile + ".jpg");
            Sleep(100);
            ftps_client_->upLoadFile(devicePath + QString::fromStdString(val.data[i].device_id) + ".fir", sendPath + "/FIR", sendFile + ".fir");
        }
        else
        {
        }
    }
}

void LibDLIntelligentSocket::send_robot_patrol_result_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{

}

void LibDLIntelligentSocket::send_robot_dev_alarm_data_msg_req(robotDevAlarmData val)
{
    if (!bConnected())
    {
        ROS_ERROR("send_robot_dev_alarm_data_msg_req: false! socket not connect!");
        return;
    }

    ROS_ERROR("send_robot_dev_alarm_data_msg_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());

    msg->setType(62);

    TiXmlElement *xmlItems = new TiXmlElement("Items");

    for (int i = 0; i < val.data.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_code", val.robot_code);
        xmlItem->SetAttribute("task_name", val.task_name);
        xmlItem->SetAttribute("task_code", val.task_code);
        xmlItem->SetAttribute("device_name", val.data[i].device_name);
        xmlItem->SetAttribute("device_id", val.data[i].device_id);
        xmlItem->SetAttribute("alarm_level", val.data[i].alarm_level);
        xmlItem->SetAttribute("alarm_type", val.data[i].alarm_type);
        xmlItem->SetAttribute("recognition_type", val.data[i].recognition_type);
        xmlItem->SetAttribute("value", val.data[i].value);
        xmlItem->SetAttribute("value_unit", val.data[i].value_unit);
        xmlItem->SetAttribute("unit", val.data[i].unit);
        xmlItem->SetAttribute("time", val.data[i].time);
        xmlItem->SetAttribute("task_patrolled_id", val.task_patrolled_id);
        xmlItem->SetAttribute("content", val.data[i].content);
    }
    msg->setItem(xmlItems);

    msg->generiateMsg(static_sequence_id_++);

    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_dev_alarm_data_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{

}

void LibDLIntelligentSocket::send_robot_patrol_report_msg_req(robotPatrolReportData val)
{
    if (!bConnected())
    {
        ROS_ERROR("send_robot_patrol_report_msg_req: false! socket not connect!");
        return;
    }

    ROS_ERROR("send_robot_patrol_report_msg_req");
    boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
    msg->setSendCode(local_code_);
    msg->setReceiveCode(remote_code_);
    msg->setTime(getFormatDateTime());

    msg->setType(63);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("task_patrolled_id", val.task_patrolled_id);
    xmlItem->SetAttribute("task_name", val.task_name);
    xmlItem->SetAttribute("task_code", val.task_code);
    xmlItem->SetAttribute("plan_start_time", val.plan_start_time);
    xmlItem->SetAttribute("start_time", val.start_time);
    xmlItem->SetAttribute("end_time", val.end_time);
    xmlItem->SetAttribute("all_count", val.all_count);
    xmlItem->SetAttribute("normal_count", val.normal_count);
    xmlItem->SetAttribute("alarm_count", val.alarm_count);
    xmlItem->SetAttribute("recognition_error_count", val.recognition_error_count);
    xmlItem->SetAttribute("miss_count", val.miss_count);
    xmlItem->SetAttribute("temperature", val.temperature);
    xmlItem->SetAttribute("humidity", val.humidity);
    xmlItem->SetAttribute("wind_speed", val.wind_speed);
    xmlItem->SetAttribute("description", val.description);
    xmlItem->SetAttribute("road_file_path", val.road_file_path);
    xmlItem->SetAttribute("task_finish_state", val.task_finish_state);

    msg->setItem(xmlItems);

    msg->generiateMsg(static_sequence_id_++);

    m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::send_robot_patrol_report_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{

}

void LibDLIntelligentSocket::test_send_msg()
{
	boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
	msg->setSendCode(local_code_);
	msg->setReceiveCode(remote_code_);
	msg->setTime(getFormatDateTime());
	msg->setCode("10");
	msg->setType(21);
	msg->setCmd(1);

// 	TiXmlElement *xmlItems = new TiXmlElement("Items");
// 	TiXmlElement *xmlItem = new TiXmlElement("Item");
// 	xmlItems->LinkEndChild(xmlItem);
// 	xmlItem->SetAttribute("SendCode", "11111111");
// 	xmlItem->SetAttribute("ReceiveCode", "22222222");
// 	xmlItem->SetAttribute("Code", "10");
// 	xmlItem->SetAttribute("Type", "1");
// 	xmlItem->SetAttribute("Command ", "3");
// 	xmlItem->SetAttribute("Item ", "11");
// 	xmlItem->SetAttribute("Time ", "3");
//	msg->setItem(xmlItems);

	msg->generiateMsg(static_sequence_id_++);

	m_socket->postMsg(msg);
}

void LibDLIntelligentSocket::recv_handle_code(boost::shared_ptr<XmlProtocolMsg> msg)
{
	ROS_ERROR("recv_handle_code:\n  {type:%d, cmd:%d, code:%s}", msg->getType(), msg->getCmd(), msg->getCode().c_str());
//    ROS_ERROR("recv_handle_code:\n%s", msg->getString().c_str());
    if (m_msgHandleMap.find(msg->getType()) == m_msgHandleMap.end())
    {
        ROS_ERROR("error msg type");
    }
    else if (m_msgHandleMap[msg->getType()].find(msg->getCmd()) == m_msgHandleMap[msg->getType()].end())
    {
        ROS_ERROR("error msg cmd");
    }
    else
    {
        m_msgHandleMap[msg->getType()][msg->getCmd()](msg);
    }    
}

































void LibDLIntelligentSocket::recv_model_assign_dev_model_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    // todo;
    ROS_ERROR("模型下发-设备模型");

    TiXmlElement *item = msg->getItem();

    std::vector<xml_deviceType> devList;
    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            xml_deviceType dev;
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("device_level") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:device_level:%d", attri->IntValue());
                    dev.device_level = attri->IntValue();
                }
                else if (attri->NameTStr().compare("device_id") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:device_id:%s", attri->Value());
                    dev.device_id = attri->Value();
                }
                else if (attri->NameTStr().compare("device_name") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:device_name:%s", attri->Value());
                    dev.device_name = attri->Value();
                }
                else if (attri->NameTStr().compare("voltage_level") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:voltage_level:%s", attri->Value());
                    dev.voltage_level = attri->Value();
                }
                else if (attri->NameTStr().compare("bay_id") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:bay_id:%s", attri->Value());
                    dev.bay_id = attri->Value();
                }
                else if (attri->NameTStr().compare("bay_name") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:bay_name:%s", attri->Value());
                    dev.bay_name = attri->Value();
                }
                else if (attri->NameTStr().compare("main_device_id") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:main_device_id:%s", attri->Value());
                    dev.main_device_id = attri->Value();
                }
                else if (attri->NameTStr().compare("main_device_name") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:main_device_name:%s", attri->Value());
                    dev.main_device_name = attri->Value();
                }
                else if (attri->NameTStr().compare("device_type") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:device_type:%s", attri->Value());
                    dev.device_type = attri->Value();
                }
                else if (attri->NameTStr().compare("meter_type") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:meter_type:%s", attri->Value());
                    dev.meter_type = attri->Value();
                }
                else if (attri->NameTStr().compare("appearance_type") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:appearance_type:%s", attri->Value());
                    dev.appearance_type = attri->Value();
                }
                else if (attri->NameTStr().compare("fever_type") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:fever_type:%s", attri->Value());
                    dev.fever_type = attri->Value();
                }
                else if (attri->NameTStr().compare("save_type_list") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:save_type_list:%s", attri->Value());
                    dev.save_type_list = attri->Value();
                }
                else if (attri->NameTStr().compare("recognition_type_list") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:recognition_type_list:%s", attri->Value());
                    dev.recognition_type_list = attri->Value();
                }
                else if (attri->NameTStr().compare("phase") == 0)
                {
                    ROS_ERROR("    模型下发-设备模型:phase:%s", attri->Value());
                    dev.phase = attri->Value();
                }
                else
                {
                    ROS_ERROR("unknowd device cfg attr : %s", attri->NameTStr());
                }
                attri = attri->Next();
            }
            devList.push_back(dev);
        }
    }
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_model_assign_robot_cfg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    // todo;
    ROS_ERROR("模型下发-机器人配置");
    TiXmlElement *item = msg->getItem()->FirstChildElement();

    TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
    robotCfg robot;
    while (attri != NULL)          //若属性不为空则输出
    {
        if (attri->NameTStr().compare("robot_name") == 0)
        {
            ROS_ERROR("    模型下发-机器人配置:robot_name:%s", attri->Value());
            robot.robot_name = attri->Value();
        }
        else if (attri->NameTStr().compare("robot_code") == 0)
        {
            ROS_ERROR("    模型下发-机器人配置:robot_code:%d", attri->Value());
            robot.robot_code = attri->Value();
        }
        else if (attri->NameTStr().compare("manufacturer") == 0)
        {
            ROS_ERROR("    模型下发-机器人配置:manufacturer:%s", attri->Value());
            robot.manufacturer = attri->Value();
        }
        else if (attri->NameTStr().compare("istransport") == 0)
        {
            ROS_ERROR("    模型下发-机器人配置:istransport:%s", attri->Value());
            robot.istransport = attri->Value();
        }
        else if (attri->NameTStr().compare("type") == 0)
        {
            ROS_ERROR("    模型下发-机器人配置:type:%s", attri->Value());
            robot.type = attri->Value();
        }
        else
        {
            ROS_ERROR("    模型下发-机器人配置 未识别属性");
        }
        // todo
        attri = attri->Next();
    }

    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_model_assign_threshold_cfg_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    // todo;
    ROS_ERROR("模型下发-告警阈值配置");

    TiXmlElement *item = msg->getItem();

    std::vector<alarmThresholdCfg> cfgList;
    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            alarmThresholdCfg cfg;
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("device_id") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:file_path:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("device_name") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:device_level:%d", attri->Value());
                }
                else if (attri->NameTStr().compare("decision_rules") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_pixel:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("warning_upper_limit") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("warning_lower_limit") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("warning_state") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("common_upper_limit") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("common_lower_limit") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("common_state") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("serious_upper_limit") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("serious_lower_limit") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("serious_state") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("critical_upper_limit") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("critical_lower_limit") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("critical_state") == 0)
                {
                    ROS_ERROR("任务下发-告警阈值配置:coordinate_geography:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("任务下发-告警阈值配置 未识别属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_model_assign_overhaul_area_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    // todo;
    ROS_ERROR("模型下发-检修区域配置");
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}



void LibDLIntelligentSocket::recv_data_query_visible_img_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-可见光照片");
    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-可见光照片-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-可见光照片-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }

        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-可见光照片-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-可见光照片-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }

//    ftps_client_->upLoadFile(root_path_ + "/relayFiles/CCD/100_10_20161116121514.jpg", "/1/2016/11/21/55/CCD/100_10_20161116121514.jpg");
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_data_query_infrared_img_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-红外图谱数据");

    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-红外图谱数据-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-红外图谱数据-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-红外图谱数据-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-红外图谱数据-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }

//    ftps_client_->upLoadFile(root_path_ + "/relayFiles/Fir/100_10_20161116121514.fir", "/1/2016/11/21/55/Fir/100_10_20161116121514.fir");
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_data_query_audio_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-音频数据");
    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-音频数据-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-音频数据-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-音频数据-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-音频数据-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }

//    ftps_client_->upLoadFile(root_path_ + "/relayFiles/Audio/100_10_20161116121514.wav", "/1/2016/11/21/55/Audio/100_10_20161116121514.wav");
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_data_query_video_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-视频数据");
    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-视频数据-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-视频数据-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-视频数据-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-视频数据-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }
//    ftps_client_->upLoadFile(root_path_ + "/relayFiles/Video/100_10_20161116121514.mp4", "/1/2016/11/21/55/Video/100_10_20161116121514.mp4");
    send_common_response_no_item(msg->getSendCode(), msg->getSequenceId());
}

void LibDLIntelligentSocket::recv_data_query_patrol_res_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-巡检结果");
    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-巡检结果-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-巡检结果-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-巡检结果-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-巡检结果-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }

    robotPatrolResult task;
    robotDeviceUploadType devRes1, devRes2, devRes3;
    task.robot_code = "Robot1";
    task.task_name = "test Robot task";
    task.task_code = "task1";
    task.task_patrolled_id = "task01";

    devRes1.device_name = "test dev 1 infrared";
    devRes1.device_id = "test id dev 1";
    devRes1.value = "29.0";
    devRes1.value_unit = "29.0℃";
    devRes1.unit = "℃";
    devRes1.time = getFormatDateTime();
    devRes1.recognition_type = "4";
    devRes1.file_type = "1";
    devRes1.file_path = "D:\\test\\test1";
    devRes1.rectangle = "1,2,3,4";

    devRes2.device_name = "test dev 2 visible";
    devRes2.device_id = "testiddev2";
    devRes2.value = "21.1";
    devRes2.value_unit = "29.0V";
    devRes2.unit = "V";
    devRes2.time = getFormatDateTime();
    devRes2.recognition_type = "1";
    devRes2.file_type = "2";
    devRes2.file_path = "D:\\test\\test2";
    devRes2.rectangle = "1,2,3,4";

    devRes3.device_name = "test dev 3 visible";
    devRes3.device_id = "test id dev 3";
    devRes3.value = "18.9";
    devRes3.value_unit = "18.9V";
    devRes3.unit = "V";
    devRes3.time = getFormatDateTime();
    devRes3.recognition_type = "2";
    devRes3.file_type = "1";
    devRes3.file_path = "D:\\test\\test4";
    devRes3.rectangle = "1,2,3,4";

    task.data.push_back(devRes1);
    task.data.push_back(devRes2);
    task.data.push_back(devRes3);

    TiXmlElement *xmlItems = new TiXmlElement("Items");

    for (int i = 0; i < task.data.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_code", task.robot_code);
        xmlItem->SetAttribute("task_name", task.task_name);
        xmlItem->SetAttribute("task_code", task.task_code);
        xmlItem->SetAttribute("device_name", task.data[i].device_name);
        xmlItem->SetAttribute("device_id", task.data[i].device_id);
        xmlItem->SetAttribute("value", task.data[i].value);
        xmlItem->SetAttribute("value_unit", task.data[i].value_unit);
        xmlItem->SetAttribute("unit", task.data[i].unit);
        xmlItem->SetAttribute("time", task.data[i].time);
        xmlItem->SetAttribute("recognition_type", task.data[i].recognition_type);
        xmlItem->SetAttribute("file_type", task.data[i].file_type);
        xmlItem->SetAttribute("file_path", task.data[i].file_path);
        xmlItem->SetAttribute("rectangle", task.data[i].rectangle);
        xmlItem->SetAttribute("task_patrolled_id", task.task_patrolled_id);
    }

    send_common_response_with_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(200), xmlItems);
}

void LibDLIntelligentSocket::recv_data_query_alarm_data_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-告警数据");
    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-告警数据-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-告警数据-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-告警数据-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-告警数据-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }

    robotDevAlarmData data;
    robotDeviceAlarmType alarmData1, alarmData2;
    data.robot_code = "Robot1";
    data.task_name = "test Robot task";
    data.task_code = "task1";
    data.task_patrolled_id = "task01";

    alarmData1.device_name = "test dev 1 visible";
    alarmData1.device_id = "taskdev1";
    alarmData1.alarm_level = "3";
    alarmData1.alarm_type = "6";
    alarmData1.recognition_type = "3";
    alarmData1.value = "";
    alarmData1.value_unit = "";
    alarmData1.unit = "";
    alarmData1.time = getFormatDateTime();
    alarmData1.content = "visible test alarm";

    alarmData2.device_name = "test dev 2 visible";
    alarmData2.device_id = "taskdev2";
    alarmData2.alarm_level = "1";
    alarmData2.alarm_type = "1";
    alarmData2.recognition_type = "4";
    alarmData2.value = "158.0";
    alarmData2.value_unit = "℃";
    alarmData2.unit = "℃";
    alarmData2.time = getFormatDateTime();
    alarmData2.content = "infrared test alarm";

    data.data.push_back(alarmData1);
    data.data.push_back(alarmData2);

    TiXmlElement *xmlItems = new TiXmlElement("Items");

    for (int i = 0; i < data.data.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_code", data.robot_code);
        xmlItem->SetAttribute("task_name", data.task_name);
        xmlItem->SetAttribute("task_code", data.task_code);
        xmlItem->SetAttribute("device_name", data.data[i].device_name);
        xmlItem->SetAttribute("device_id", data.data[i].device_id);
        xmlItem->SetAttribute("alarm_level", data.data[i].alarm_level);
        xmlItem->SetAttribute("alarm_type", data.data[i].alarm_type);
        xmlItem->SetAttribute("recognition_type", data.data[i].recognition_type);
        xmlItem->SetAttribute("value", data.data[i].value);
        xmlItem->SetAttribute("value_unit", data.data[i].value_unit);
        xmlItem->SetAttribute("unit", data.data[i].unit);
        xmlItem->SetAttribute("time", data.data[i].time);
        xmlItem->SetAttribute("task_patrolled_id", data.task_patrolled_id);
        xmlItem->SetAttribute("content", data.data[i].content);
    }

    send_common_response_with_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(200), xmlItems);
}

void LibDLIntelligentSocket::recv_data_query_patrol_report_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-巡检报告");
    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-巡检报告-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-巡检报告-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-巡检报告-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-巡检报告-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }

    robotPatrolReportData res;
    res.task_patrolled_id = "task01";
    res.task_name = "test Robot task";
    res.task_code = "task1";
    res.plan_start_time = "2019-05-08 08:20:11";
    res.start_time = "2019-05-08 08:20:20";
    res.end_time = "2019-05-08 08:40:25";
    res.all_count = 100;
    res.normal_count = 80;
    res.all_count = 10;
    res.recognition_error_count = 5;
    res.miss_count = 5;
    res.temperature = "28.5℃";
    res.humidity = "58%";
    res.wind_speed = "0.6m/3";
    res.description = "task report test";
    res.road_file_path = "D:\\test1.map";
    res.task_finish_state = "1";

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("task_patrolled_id", res.task_patrolled_id);
    xmlItem->SetAttribute("task_name", res.task_name);
    xmlItem->SetAttribute("task_code", res.task_code);
    xmlItem->SetAttribute("plan_start_time", res.plan_start_time);
    xmlItem->SetAttribute("start_time", res.start_time);
    xmlItem->SetAttribute("end_time", res.end_time);
    xmlItem->SetAttribute("all_count", res.all_count);
    xmlItem->SetAttribute("normal_count", res.normal_count);
    xmlItem->SetAttribute("alarm_count", res.alarm_count);
    xmlItem->SetAttribute("recognition_error_count", res.recognition_error_count);
    xmlItem->SetAttribute("miss_count", res.miss_count);
    xmlItem->SetAttribute("temperature", res.temperature);
    xmlItem->SetAttribute("humidity", res.humidity);
    xmlItem->SetAttribute("wind_speed", res.wind_speed);
    xmlItem->SetAttribute("description", res.description);
    xmlItem->SetAttribute("road_file_path", res.road_file_path);
    xmlItem->SetAttribute("task_finish_state", res.task_finish_state);

    send_common_response_with_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(200), xmlItems);

}

void LibDLIntelligentSocket::recv_data_query_robot_status_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-机器人状态数据");
    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-机器人状态数据-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-机器人状态数据-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-机器人状态数据-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-机器人状态数据-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }

    robotStatus status;
    status.robot_name = "testRobot1";
    status.robot_code = "Robot1";
    status.time = getFormatDateTime();
    tvu temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9;
    temp1.type = std::to_string(1);
    temp1.value = "22.8";
    temp1.value_unit = "22.8V";
    temp1.unit = "V";

    temp2.type = std::to_string(2);
    temp2.value = "1";
    temp2.value_unit = "1";
    temp2.unit = "";

    temp3.type = std::to_string(3);
    temp3.value = "1";
    temp3.value_unit = "1";
    temp3.unit = "";

    temp4.type = std::to_string(4);
    temp4.value = "0";
    temp4.value_unit = "0";
    temp4.unit = "";

    temp5.type = std::to_string(21);
    temp5.value = "0";
    temp5.value_unit = "0";
    temp5.unit = "0";

    temp6.type = std::to_string(41);
    temp6.value = "2";
    temp6.value_unit = "2";
    temp6.unit = "";

    temp7.type = std::to_string(61);
    temp7.value = "4";
    temp7.value_unit = "4";
    temp7.unit = "";

    temp8.type = std::to_string(81);
    temp8.value = "0";
    temp8.value_unit = "0";
    temp8.unit = "";

    temp9.type = std::to_string(101);
    temp9.value = "1";
    temp9.value_unit = "1";
    temp9.unit = "";

    status.data.push_back(temp1);
    status.data.push_back(temp2);
    status.data.push_back(temp3);
    status.data.push_back(temp4);
    status.data.push_back(temp5);
    status.data.push_back(temp6);
    status.data.push_back(temp7);
    status.data.push_back(temp8);
    status.data.push_back(temp9);

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    for (int i = 0; i < status.data.size(); i++)
    {
        TiXmlElement *xmlItem = new TiXmlElement("Item");
        xmlItems->LinkEndChild(xmlItem);
        xmlItem->SetAttribute("robot_name", status.robot_name);
        xmlItem->SetAttribute("robot_code", status.robot_code);
        xmlItem->SetAttribute("time", status.time);
        xmlItem->SetAttribute("type", status.data[i].type);
        xmlItem->SetAttribute("value", status.data[i].value);
        xmlItem->SetAttribute("value_unit", status.data[i].value_unit);
        xmlItem->SetAttribute("unit", status.data[i].unit);
    }

    send_common_response_with_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(200), xmlItems);
}

void LibDLIntelligentSocket::recv_data_query_robot_alarm_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("数据查询-机器人异常告警");
    TiXmlElement *item = msg->getItem();

    if (item->FirstChildElement() != NULL)
    {
        for (TiXmlElement* i = item->FirstChildElement(); i != NULL; i = (TiXmlElement*)item->IterateChildren(i))
        {
            TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
            while (attri != NULL)          //若属性不为空则输出
            {
                if (attri->NameTStr().compare("start_time") == 0)
                {
                    ROS_ERROR("数据查询-机器人异常告警-start_time:%s", attri->Value());
                }
                else if (attri->NameTStr().compare("end_time") == 0)
                {
                    ROS_ERROR("数据查询-机器人异常告警-end_time:%s", attri->Value());
                }
                else
                {
                    ROS_ERROR("错误的属性");
                }
                // todo
                attri = attri->Next();
            }
        }
    }
    else
    {
        TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
        while (attri != NULL)          //若属性不为空则输出
        {
            if (attri->NameTStr().compare("start_time") == 0)
            {
                ROS_ERROR("数据查询-机器人异常告警-start_time:%s", attri->Value());
            }
            else if (attri->NameTStr().compare("end_time") == 0)
            {
                ROS_ERROR("数据查询-机器人异常告警-end_time:%s", attri->Value());
            }
            else
            {
                ROS_ERROR("错误的属性");
            }
            attri = attri->Next();
        }
    }

    robotAlarmData data;
    data.robot_code = "Robot1";
    data.robot_name = "testRobot1";
    data.time = getFormatDateTime();
    data.content = "robot test alarm";

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("robot_name", data.robot_name);
    xmlItem->SetAttribute("robot_code", data.robot_code);
    xmlItem->SetAttribute("time", data.time);
    xmlItem->SetAttribute("content", data.content);
    msg->setItem(xmlItems);

    send_common_response_with_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(200), xmlItems);
}

void LibDLIntelligentSocket::recv_data_task_map_point_resp(boost::shared_ptr<XmlProtocolMsg> msg)
{
    ROS_ERROR("任务下发-地图选点");
    TiXmlElement *item = msg->getItem()->FirstChildElement();

    TiXmlAttribute *attri = item->FirstAttribute();   //获取节点的第一个属性，可以用LastAttribute()获取最后一个属性节点
    while (attri != NULL)          //若属性不为空则输出
    {
        if (attri->NameTStr().compare("file_path") == 0)
        {
            ROS_ERROR("任务下发-地图选点:file_path:%s", attri->Value());
        }
        else if (attri->NameTStr().compare("device_level") == 0)
        {
            ROS_ERROR("任务下发-地图选点:device_level:%d", attri->IntValue());
        }
        else if (attri->NameTStr().compare("coordinate_pixel") == 0)
        {
            ROS_ERROR("任务下发-地图选点:coordinate_pixel:%s", attri->Value());
        }
        else if (attri->NameTStr().compare("coordinate_geography") == 0)
        {
            ROS_ERROR("任务下发-地图选点:coordinate_geography:%s", attri->Value());
        }
        else
        {
            ROS_ERROR("任务下发-地图选点 未识别属性");
        }
        // todo
        attri = attri->Next();
    }

    robotMapSelectPointResp data;
    data.device_level = 1;
    data.device_list = "1,2,3,4,5,6,7";

    TiXmlElement *xmlItems = new TiXmlElement("Items");
    TiXmlElement *xmlItem = new TiXmlElement("Item");
    xmlItems->LinkEndChild(xmlItem);
    xmlItem->SetAttribute("devel_level", data.device_level);
    xmlItem->SetAttribute("coordinate_pixel", data.device_list);
    msg->setItem(xmlItems);

    send_common_response_with_item(msg->getSendCode(), msg->getSequenceId(), std::to_string(200), xmlItems);
}





void LibDLIntelligentSocket::ftp_send_test()
{
//     ftps_client_->upLoadFile(root_path_ + "/relayFiles/Road/100_10_20161116121514.jpg", "/1/2016/11/21/55/Road/100_10_20161116121514.jpg");
//     Sleep(100);
//     ftps_client_->upLoadFile(root_path_ + "/relayFiles/Map/100_10_20161116121514.jpg", "/1/2016/11/21/55/Map/100_10_20161116121514.jpg");
//     Sleep(100);
//     ftps_client_->upLoadFile(root_path_ + "/relayFiles/CCD/100_10_20161116121514.jpg", "/1/2016/11/21/55/CCD/100_10_20161116121514.jpg");
//     Sleep(100);
//     ftps_client_->upLoadFile(root_path_ + "/relayFiles/Fir/100_10_20161116121514.fir", "/1/2016/11/21/55/Fir/100_10_20161116121514.fir");
//     Sleep(100);
//     ftps_client_->upLoadFile(root_path_ + "/relayFiles/Audio/100_10_20161116121514.wav", "/1/2016/11/21/55/Audio/100_10_20161116121514.wav");
//     Sleep(100);
//     ftps_client_->upLoadFile(root_path_ + "/relayFiles/Video/100_10_20161116121514.mp4", "/1/2016/11/21/55/Video/100_10_20161116121514.mp4");
//     Sleep(100);
}

std::string LibDLIntelligentSocket::getSessionId()
{
    boost::uuids::random_generator rgen;//随机生成器  
    boost::uuids::uuid ssid = rgen();//生成一个随机的UUID
    std::string tmp = boost::lexical_cast<std::string>(ssid);
    boost::erase_all(tmp, "-"); 
    
    return tmp;
}

std::string LibDLIntelligentSocket::getFormatDateTime()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
    return tmp;
}

void LibDLIntelligentSocket::heart_beat_timer_func(boost::system::error_code err)
{
    if (err)
    {
        ROS_ERROR("heart_beat_timer is canceled");
        return;
    }

    send_heart_beat_msg_req();
    heart_beat_timer_->expires_from_now(boost::posix_time::seconds(heart_beat_interval_));
    heart_beat_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::heart_beat_timer_func, shared_from_this(), boost::asio::placeholders::error));
}

void LibDLIntelligentSocket::robot_run_timer_func(boost::system::error_code err)
{
    if (err)
    {
        ROS_ERROR("robot_run_timer is canceled");
        return;
    }
}

void LibDLIntelligentSocket::weather_timer_func(boost::system::error_code err)
{
    if (err)
    {
        ROS_ERROR("weather_timer is canceled");
        return;
    }
}

void LibDLIntelligentSocket::robot_state_data_func(boost::system::error_code err)
{
    if (err)
    {
        ROS_ERROR("robot_run_timer is canceled");
        return;
    }
    slot_robot_state_data_msg();
    robot_state_timer_->expires_from_now(boost::posix_time::seconds(3));
    robot_state_timer_->async_wait(boost::bind(&LibDLIntelligentSocket::robot_state_data_func, shared_from_this(), boost::asio::placeholders::error));
}

bool LibDLIntelligentSocket::model_file_create()
{
    QString filePath = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/model/";
    QList<BJDeviceModel> deviceModelList;
    WHEEL_ROBOT_DB.getBJDeviceModelData(deviceModelList);
    for (int i = 0; i < deviceModelList.size(); i++)
    {
        deviceModelList[i].deviceType = m_IntelligentData->contrast_device_type_with_key(deviceModelList[i].mainDeviceName);
        deviceModelList[i].meterType = m_IntelligentData->contrast_meter_type_with_key(deviceModelList[i].meterType);
    //    deviceModelList[i].saveTypeList = deviceModelList[i].saveTypeList.replace("+", ","); contrast_collect_type_with_recognition_type;
        
    //    deviceModelList[i].recognitionTypeList = deviceModelList[i].recognitionTypeList.replace("+", ","); contrast_recognition_type_with_key;
        deviceModelList[i].recognitionTypeList = m_IntelligentData->contrast_recognition_type_with_key(deviceModelList[i].recognitionTypeList);
        deviceModelList[i].saveTypeList = m_IntelligentData->contrast_collect_type_with_recognition_type(deviceModelList[i].recognitionTypeList);
        deviceModelList[i].phase = m_IntelligentData->contrast_phase_with_string(deviceModelList[i].phase);
    }
    QFile file(filePath + "device_model.xml");
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QXmlStreamWriter writer(&file);
        writer.setAutoFormatting(true);
        writer.writeStartDocument();
        writer.writeStartElement("device_model");
        for (int i = 0; i < deviceModelList.size(); i++)
        {
            writer.writeStartElement("model");
            writer.writeAttribute("device_id", deviceModelList[i].deviceId);
            writer.writeAttribute("device_name", deviceModelList[i].deviceName);
            writer.writeAttribute("bay_id", deviceModelList[i].bayId);
            writer.writeAttribute("bay_name", deviceModelList[i].bayName);
            writer.writeAttribute("main_device_id", deviceModelList[i].mainDeviceId);
            writer.writeAttribute("main_device_name", deviceModelList[i].mainDeviceName);
            writer.writeAttribute("device_type", deviceModelList[i].deviceType);
            writer.writeAttribute("meter_type", deviceModelList[i].meterType);
            writer.writeAttribute("appearance_type", deviceModelList[i].appearanceType);
            writer.writeAttribute("save_type_list", deviceModelList[i].saveTypeList);
            writer.writeAttribute("recognition_type_list", deviceModelList[i].recognitionTypeList);
            writer.writeAttribute("phase", deviceModelList[i].phase);
            writer.writeAttribute("device_info", deviceModelList[i].deviceInfo);
            writer.writeAttribute("device_type_item_name", deviceModelList[i].device_type_item_name);
            writer.writeEndElement();
        }
        writer.writeEndElement();
        writer.writeEndDocument();
        file.close();
    }

    QFile robotFile(filePath + "robot_model.xml");
    if (robotFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QXmlStreamWriter writer(&robotFile);
        writer.setAutoFormatting(true);
        writer.writeStartDocument();
        writer.writeStartElement("robot_model");
        
        writer.writeStartElement("model");
        writer.writeAttribute("robot_name", QString::fromStdString(robot_name_));
        writer.writeAttribute("robot_code", QString::fromStdString(robot_code_));
        writer.writeAttribute("manufacturer", "中信重工机械股份有限公司");
        writer.writeAttribute("istransport", "0");
        writer.writeAttribute("type", "1");
        writer.writeAttribute("mappath", "");
        writer.writeAttribute("robot_info", "这是一个电力自动巡检机器人！");
        writer.writeEndElement();
        
        writer.writeEndElement();
        writer.writeEndDocument();
        robotFile.close();
    }
    return true;
}
