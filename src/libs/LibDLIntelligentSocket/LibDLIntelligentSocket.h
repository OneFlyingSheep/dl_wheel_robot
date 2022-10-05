#include "ProtocolSocketClient.h"
#include <boost/uuid/uuid_generators.hpp>  
#include <boost/uuid/uuid_io.hpp>  
#include <boost/uuid/uuid.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <common/DLRobotCommonDef.h>
#include "FTPSClient/FTPSClient.h"
#include <LibHCNetCamera/BJCameraCtrlSDK.h>
#include "BJIntelligentData.hpp"
#include "common/DLWheelRobotGlobalDef.hpp"

#if _MSC_VER >= 1600  
#pragma execution_character_set("utf-8")  
#endif 

typedef boost::function<void(boost::shared_ptr<XmlProtocolMsg>) > protocolMsgCallback;
typedef std::map<int, protocolMsgCallback>                        protocolMsgCoreMap;

class LibDLIntelligentSocket : public boost::enable_shared_from_this<LibDLIntelligentSocket>
{

public:
    LibDLIntelligentSocket(std::string localCode, std::string remoteCode);

    void connect(std::string ip_addr = "192.168.4.200", int port = 10011, int ftpsPort = 10012, std::string ftps_username = "admin", std::string ftps_passwd = "123456");
    void registerHandles();

    bool bConnected();
public:
    //回消息
    void send_common_response_no_item(std::string code = std::to_string(MSG_CODE_SUCCESS));
    void send_common_response_no_item(std::string remote_code, int seq, std::string code = std::to_string(MSG_CODE_SUCCESS));
    void send_common_response_with_item(std::string remote_code, int seq, std::string code = std::to_string(MSG_CODE_SUCCESS), TiXmlElement *item = NULL);

    //F/I.4.1
    //注册指令 主动发
    void send_register_msg_req();
    void recv_register_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //F/I.4.2
    //心跳指令 根据注册指令 心跳间隔 定时上报
    void send_heart_beat_msg_req();
    void recv_heartbeat_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //F/I.5.1
    //机器人本体     - 远方复位     - 未处理 -//重启
    void recv_ctrl_robot_body_remote_reset_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人本体     - 系统自检     - 未处理 -
    void recv_ctrl_robot_body_system_check_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人本体     - 一键返航     - 已处理 -
    void recv_ctrl_robot_body_one_key_return_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人本体     - 手动充电     - 已处理 -//一键返航
    void recv_ctrl_robot_body_manual_charge_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人本体     - 控制模式切换 - 已处理 -
    void recv_ctrl_robot_body_switch_ctrl_mode_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人本体     - 控制权获得   - 已处理 -//已获得
    void recv_ctrl_robot_body_access_to_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人本体     - 控制权释放   - 已处理 -
    void recv_ctrl_robot_body_release_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //机器人车体     - 前进         - 已处理 -
    void recv_ctrl_robot_base_forward_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人车体     - 后退         - 已处理 -
    void recv_ctrl_robot_base_backward_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人车体     - 左转         - 已处理 -
    void recv_ctrl_robot_base_turn_left_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人车体     - 右转         - 已处理 -
    void recv_ctrl_robot_base_turn_right_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人车体     - 转弯         - 已处理 -
    void recv_ctrl_robot_base_rotate_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人车体     - 停止         - 已处理 -
    void recv_ctrl_robot_base_stop_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //机器人云台     - 上仰         - 已处理 -
    void recv_ctrl_robot_ptz_up_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人云台     - 下俯         - 已处理 -
    void recv_ctrl_robot_ptz_down_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人云台     - 左转         - 已处理 -
    void recv_ctrl_robot_ptz_left_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人云台     - 右转         - 已处理 -
    void recv_ctrl_robot_ptz_right_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人云台     - 上升         - 已处理 -//返回
    void recv_ctrl_robot_ptz_rise_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人云台     - 下降         - 已处理 -
    void recv_ctrl_robot_ptz_fall_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人云台     - 预置位调用   - 未处理 -//内存里写一份
    void recv_ctrl_robot_ptz_preset_bit_call_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人云台     - 停止         - 已处理 -
    void recv_ctrl_robot_ptz_stop_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人云台     - 复位         - 已处理 -
    void recv_ctrl_robot_ptz_reset_control_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //机器人辅助设备 - 红外电源     - 未处理-
    void recv_ctrl_robot_auxiliary_equipment_infrared_power_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人辅助设备 - 雨刷         - 已处理-
    void recv_ctrl_robot_auxiliary_equipment_wiper_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人辅助设备 - 超声         - 未处理-
    void recv_ctrl_robot_auxiliary_equipment_snoise_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人辅助设备 - 红外射灯     - 未处理-
    void recv_ctrl_robot_auxiliary_equipment_infrared_light_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    
    //可见光摄像机   - 镜头拉近     - 已处理
    void recv_ctrl_robot_visible_camera_zoom_in_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 镜头拉远     - 已处理
    void recv_ctrl_robot_visible_camera_zoom_out_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 镜头拉焦停止 - 已处理
    void recv_ctrl_robot_visible_camera_focus_stop_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 焦距增加     - 已处理
    void recv_ctrl_robot_visible_camera_focus_in_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 焦距减少     - 已处理
    void recv_ctrl_robot_visible_camera_focus_out_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 自动聚焦     - 未处理//返回true
    void recv_ctrl_robot_visible_camera_auto_focus_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 抓图         - 已处理
    void recv_ctrl_robot_visible_camera_capture_img_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 重启         - 未处理//返回true
    void recv_ctrl_robot_visible_camera_restart_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 启动录像     - 已处理
    void recv_ctrl_robot_visible_camera_start_rec_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光摄像机   - 停止录像     - 已处理
    void recv_ctrl_robot_visible_camera_stop_rec_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可见光倍率值设定
    void recv_ctrl_robot_visible_camera_zoom_set_rec_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //可将光焦距值设定
    void recv_ctrl_robot_visible_camera_focus_set_rec_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //红外热像仪     - 未定义       - 已处理
    void recv_ctrl_robot_infrared_camera_undefined_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //红外热像仪     - 设定焦距值   - 已处理
    void recv_ctrl_robot_infrared_camera_set_focus_val_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //红外热像仪     - 自动聚焦     - 已处理
    void recv_ctrl_robot_infrared_camera_auto_focus_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //红外热像仪     - 抓图         - 已处理
    void recv_ctrl_robot_infrared_camera_capture_img_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //红外热像仪     - 重启         - 未处理
    void recv_ctrl_robot_infrared_camera_restart_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //任务控制       - 任务启动     - 已处理//返回true
    void recv_task_ctrl_start_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //任务控制       - 任务暂停     - 已处理
    void recv_task_ctrl_pause_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //任务控制       - 任务继续     - 已处理
    void recv_task_ctrl_resume_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //任务控制       - 任务停止     - 已处理 - 多任务下需上报当前任务
    void recv_task_ctrl_stop_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //模型同步指令   - 模型同步     - 已处理
    void recv_model_sync_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    
    //任务下发指令   - 任务下发     - 未处理 - 周期任务
    void recv_task_assign_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //任务下发指令   - 联动任务下发 - 已处理
    void recv_task_assign_atonce_resp(boost::shared_ptr<XmlProtocolMsg> msg);


    // 机器人状态数据  - 定时器 注册指令 - 已处理 运行状态需处理
    void slot_robot_state_data_msg();
    void send_robot_status_msg_req(robotStatus val);
    void send_robot_status_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    // 机器人运行数据  - 定时器 注册指令 - 已处理
    void send_robot_running_status_msg_req(robotRunningStatus val);
    void send_robot_running_status_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人坐标       - 变化时触发      - 未处理完全
    void slot_robot_coordinate_msg(std::string coord);
    void send_robot_coordinate_msg_req(robotCoordinate val);
    void send_robot_coordinate_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //机器人巡视路线   - 生成时触发      - 需做坐标转换
    void slot_robot_patrol_line_msg(std::vector<std::string> pointIdVec);
    void send_robot_patrol_line_msg_req(robotCoordinate val, std::vector<std::string> pointIdVec);
    void send_robot_patrol_line_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //微气象数据                         - 已处理  风速未添加
    void send_robot_micro_weather_req(robotMicroWeatherData val);
    void send_robot_micro_weather_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    
    //任务状态数据                       - 已处理，多任务下需要重复发送上一任务
    void slot_robot_task_data_msg(WheelRobotCurrentTaskInfoShow taskData);
    void send_robot_task_data_msg_req(robotTaskData val);
    void send_robot_task_data_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //机器人异常告警数据                 - 未处理
    void send_robot_alarm_msg_req(robotAlarmData val);
    void send_robot_alarm_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //巡视结果                           - 已处理
    void slot_robot_task_device_send_msg(QString taskUUid, QString deviceUUid);
    void send_robot_patrol_result_msg_req(robotPatrolResult val);
    void send_robot_patrol_result_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //未定义
    void send_robot_dev_alarm_data_msg_req(robotDevAlarmData val);
    void send_robot_dev_alarm_data_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //未定义
    void send_robot_patrol_report_msg_req(robotPatrolReportData val);
    void send_robot_patrol_report_msg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //未定义
    void recv_model_assign_dev_model_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_model_assign_robot_cfg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_model_assign_threshold_cfg_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_model_assign_overhaul_area_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    //未定义
    void recv_data_query_visible_img_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_query_infrared_img_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_query_audio_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_query_video_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_query_patrol_res_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_query_alarm_data_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_query_patrol_report_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_query_robot_status_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_query_robot_alarm_resp(boost::shared_ptr<XmlProtocolMsg> msg);
    void recv_data_task_map_point_resp(boost::shared_ptr<XmlProtocolMsg> msg);

    //测试函数
	void test_send_msg();
public: // register callback func

    void recv_handle_code(boost::shared_ptr<XmlProtocolMsg> msg);
    void ftp_send_test();
    bool model_file_create();
private:
    std::string getSessionId();
    std::string getFormatDateTime();

    //心跳定时发送
    void heart_beat_timer_func(boost::system::error_code err);
    boost::shared_ptr<boost::asio::deadline_timer> heart_beat_timer_;
    int heart_beat_interval_;
    //机器人运行数据定时发送
    void robot_run_timer_func(boost::system::error_code err);
    boost::shared_ptr<boost::asio::deadline_timer> robot_run_timer_;
    int robot_run_interval_;
    //机器人微气象数据定时发送
    void weather_timer_func(boost::system::error_code err);
    boost::shared_ptr<boost::asio::deadline_timer> weather_timer_;
    int weather_interval_;

    void robot_state_data_func(boost::system::error_code err);
    boost::shared_ptr<boost::asio::deadline_timer> robot_state_timer_;
    //boost::asio::deadline_timer *robot_state_timer_;

private:
    boost::shared_ptr<ProtocolSocketClient> m_socket;

    std::map<int, protocolMsgCoreMap> m_msgHandleMap;

    std::string local_code_;
    std::string remote_code_;
    std::string robot_name_ = "轮式电力巡检机器人";
    std::string robot_code_ = "10";
    std::string robot_main_code_ = "11";
    std::string station_code_ = "12";

    std::string server_ip_addr_;
    int server_port_;
    int ftps_server_port_;

    FTPSClient *ftps_client_;

    QString root_path_;
    QString root_core_path_;

	bool bWiper = false;
    BJCameraCtrlSDK *m_HCNetCamera;
    BJIntelligentData *m_IntelligentData;

    BJRobotBodyStatusData m_robotBodyStatus;
};
