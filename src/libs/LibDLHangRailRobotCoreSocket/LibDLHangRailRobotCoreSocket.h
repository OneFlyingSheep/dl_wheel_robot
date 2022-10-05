#ifndef __DL_WHEEL_ROBOT_BACKGROUND_MSG_H__
#define __DL_WHEEL_ROBOT_BACKGROUND_MSG_H__

#define TEN_HZ_MICRO_SECOND 80
#define ONE_HZ_MICRO_SECOND 800

//#include "LibSocketLogin/CCoreClient.h"
#include "LibCommonSocket/SocketClient.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <common/Singleton.hpp>
//#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibSocket/generaldef.h"
#include "common/DLHangRailRobotGlobalDef.hpp"
#include <boost/bind.hpp>
#include <vector>
#include <windows.h>
#include <boost/signals2.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

class LibDLHangRailRobotBackground : public Singleton<LibDLHangRailRobotBackground>
{
public:
    LibDLHangRailRobotBackground();
    ~LibDLHangRailRobotBackground();

public:
  
    hangRobotUserLoginRetVal dologin(std::string host, int port, std::string username, std::string password);
    void registerHandles();
    void disconnect();

    /*--------------------信号定义--------------------*/
    boost::signals2::signal<void(robotSensorStruct)> signal_robot_current_status;
    boost::signals2::signal<void(robotQueryStatus)> signal_robot_query_status;
	boost::signals2::signal<void(QString, QString, QString, QString)> signal_robot_environment_status;
	boost::signals2::signal<void(thresholdEnviAlarm)> signal_robot_environment_alarm_status;
    boost::signals2::signal<void(int)> signal_robot_task_alarm_count;// 红色气泡告警
    boost::signals2::signal<void(QString, QString, int, int, int)> signal_robot_current_string; // 当前任务进度字符串
    boost::signals2::signal<void(QString, QString)> signal_motorDebugInfo;
    boost::signals2::signal<void(unsigned char)> signal_sickDebugInfo;
    boost::signals2::signal<void(bool, QString)> signal_robot_current_task_process; // 当前处理状态 （ 任务结束/任务开始）
	boost::signals2::signal<void(inspectResultMsg)> signal_robot_current_task_device_show; // 当前设备巡检记录
	boost::signals2::signal<void(QList<inspectResultMsg>)> signal_robot_current_task_device_first_return; // 当前设备巡检记录


	boost::signals2::signal<void(bool, QString)> singnal_database_update_alarm_info;//告警设备处理
	boost::signals2::signal<void(bool, QString)> singnal_database_insert_task_template;//插入任务模板
	boost::signals2::signal<void(bool, QString)> singnal_database_delete_task_template;//删除任务模板
	boost::signals2::signal<void(bool, QString)> singnal_database_insert_threshol_environment;//环境阈值
	boost::signals2::signal<void(bool, QString)> singnal_database_insert_threshol_patrol;
	boost::signals2::signal<void(bool, QString, QString)> singnal_database_delete_device_by_ssid;//删除设备
	boost::signals2::signal<void(bool, InsertDeviceReturn ,QString)> singnal_database_insert_device;//新增设备
	boost::signals2::signal<void(bool, virtualDeviceType ,QString)> singnal_database_insert_vir_device;//新增虚拟设备
	boost::signals2::signal<void(bool, QString)> singnal_database_insert_map_Data;//新增地图
	boost::signals2::signal<void(bool, QString)> singnal_database_insert_or_update_restoration_value;//标零
	boost::signals2::signal<void(bool, deviceConfigType, QString)> singnal_database_update_device_by_ssid;//保存
    boost::signals2::signal<void(RobotBodyWarnLight)> singnal_robot_current_status_light;//机器人信号灯
    boost::signals2::signal<void(bool)> singnal_robot_current_status_emergency;//机器人急停状态
    boost::signals2::signal<void(bool)> singnal_robot_current_status_developer;//机器人开发者模式状态
    boost::signals2::signal<void(int)> singnal_robot_next_timed_task_tick;//机器人开发者模式状态

	boost::signals2::signal<void()> singnal_robot_ctrl_core_zero_lift;//机器人开发者模式状态
	boost::signals2::signal<void(bool, QString)> singnal_robot__walk_threshold;

    boost::signals2::signal<void(QVector<int>)> singnal_remote_signal_callback;
    boost::signals2::signal<void(QVector<float>)> singnal_remote_value_callback;

	

    /*--------------------指令下发--------------------*/
    void robot_task_assign_all_station();
    void robot_task_normal_task(QString task_name, QStringList task_list);
    void robot_device_proceed(QString dealed_info, QVector<updateAlarmInfoStruct> infoList);
	void robot_task_automatic_take_photo_req();//开始抓图
	void robot_task_stop_take_photo_req();//停止抓图
	void robot_task_recover_task_req();//恢复任务
	void robot_task_pause_task_req();//暂停任务
	void robot_task_stop_task();//停止任务

    void robot_query_robot_init_status();// 获取机器人当前状态
    void resp_robot_query_robot_init_status(boost::shared_ptr<commonMsg> msg);// 当前状态返回
    void resp_robot_query_robot_current_status_emergency(boost::shared_ptr<commonMsg> msg);// 返回
    void resp_robot_query_robot_current_status_developer(boost::shared_ptr<commonMsg> msg);// 返回

	void resp_robot_query_robot_walk_threshold(boost::shared_ptr<commonMsg> msg);
    void query_robot_IEC_104_remote_value(boost::shared_ptr<commonMsg> msg);
    void query_robot_IEC_104_remote_signal(boost::shared_ptr<commonMsg> msg);

	/*--------------------机器人状态反馈--------------------*/
    void query_resp_robot_environment_status(boost::shared_ptr<commonMsg> msg);
	void query_resp_robot_environment_alarm_status(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_robot_status(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_robot_query_status(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_current_task_alarm(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_current_task_string(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_current_task_process(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_current_task_device_show(boost::shared_ptr<commonMsg> msg);
	void query_resp_robot_current_task_device_first_return(boost::shared_ptr<commonMsg> msg);
	void query_resp_robot_motor_debug_info(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_sick_debug_info(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_current_status_light(boost::shared_ptr<commonMsg> msg);
    void query_resp_robot_next_task_tick(boost::shared_ptr<commonMsg> msg);

	/*--------------------机器人控制--------------------*/
	void robot_ctrl_to_point_req(int pointId);
	void robot_ctrl_move_abs_req(int offset, int speed = 1);
	void robot_ctrl_move_req(RobotMoveMode type, int speed);
	void robot_ctrl_lift_abs_req(int length);
	void robot_ctrl_lift_req(RobotLiftMode type);
	void robot_ctrl_cam_ptz_abs_req(int rotate);
	void robot_ctrl_cam_ptz_req(RobotCamPtzMode type);
	void robot_ctrl_body_abs_req(int rotate);
	void robot_ctrl_body_req(RobotBodyRotateMode type);
	void robot_ctrl_partialdischarge_req(RobotPartialDischargeOper type);
	void robot_ctrl_man_pd_req(RobotBodyPDMode type);
	void robot_ctrl_man_to_point_req(int pointId, int speed);
	void robot_ctrl_pd_ptz_req(RobotBodyPDArm type);
	void robot_ctrl_pd_collect_req();
	void robot_ctrl_status_light_req(RobotBodyWarnLight type);
	void robot_ctrl_zero_lift_req();
	void robot_ctrl_emergency_stop_req(bool bEmergency);
	void robot_ctrl_warning_light_flash_req(RobotBodyWarnLight type);
	void robot_ctrl_touch_screen_req(int type);
    void robot_ctrl_develop_mode(bool bOpen);
	void robot_ctrl_walk_threshold_req(int start_value,int terminus_value);
    void robot_ctrl_remote_control_req(int dataNo, int status);
    void robot_ctrl_update_embedded_software_req(RobotBodyUpdateEmbeddedSoftwareType type);

	/*--------------------数据库操作--------------------*/
	//更新表station_cfg
	void robot_update_station_cfg_req(stationCfg cfg);
	//devices表新增
	void robot_insert_device_req(deviceConfigType dev);
	//devices表批量新增
	void robot_insert_multi_devices_req(QList<deviceConfigType> devs);
	//devices表根据ssid修改数据
	void robot_update_device_by_ssid_req(deviceConfigType dev);
	//devices表根据批量修改数据
	void robot_update_multi_devices_by_ssid_req(QList<deviceConfigType> devs);
	//devices表根据ssid删除数据
	void robot_delete_device_by_ssid_req(QString ssid);
	//devices表根据批量删除数据
	void robot_delete_multi_devices_by_ssid_req(QList<QString> ssids);
	//虚拟设备表新增数据
	void robot_insert_vir_device_req(virtualDeviceType dev);
	//虚拟设备表批量新增数据
	void robot_insert_multi_vir_device_req(QList<virtualDeviceType> devs);
	//虚拟设备表根据ssid修改数据
	void robot_update_vir_device_req(virtualDeviceType dev);
	//虚拟设备表批量修改数据
	void robot_update_multi_vir_device_req(QList<virtualDeviceType> devs);
	//虚拟设备表删除数据
	void robot_delete_vir_device_req(QString ssid);
	//虚拟设备表批量删除数据
	void robot_delete_multi_vir_device_req(QList<QString> ssids);
	//环境阈值插入
	void robot_insert_threshol_environment_req(thresholdEnvi TEnvi);
	//巡检设备阈值插入
	void robot_insert_threshol_patrol_req(thresholdPatrol TPatrol);
	//task表新增任务
	void robot_insert_task_req(taskType task);
	//task任务开始时插入一条数据
	void robot_insert_task_at_beginning_req(taskType task);
	//task更新任务数据
	void robot_update_task_req(taskType task);
	//task表根据ssid删除任务
	void robot_delete_task_by_ssid_req(QString ssid);
	//task_template表新增数据
	void robot_insert_task_template_req(taskTemplateType taskTemplate);
	//task_template表删除数据
	void robot_delete_task_template_req(QString taskTemplateSsid);
	//inpect_result表新增数据
	void robot_insert_inpect_result_req(inspectResultType inspectResult);
	//inpect_result表更新数据-DeviceAlarmUpDateViewData报警信息处理
	void robot_update_inspect_result_req(QList<DeviceTaskSsid> Struct, QString dealed_info);
	//地图相关数据先删除在插入
	void robot_insert_map_Data_req(QList<MapItemData> MapDataStruct, QString station_id);
	//environment_result环境结果插入
	void robot_insert_environment_result_req(EnvironmentResult m_enviResult);
	//environment_result环境记录根据时间删除
	void robot_delete_environment_result_req(QString DeletedateTime);
	//restoration_value表数据，存在则更新，不存在则新增
	void robot_insert_or_update_restoration_value_req(RestorationValue m_restorationValue);
	//更新devices表relative_dev字段
	void robot_update_devices_for_relative_dev_req(RelevanceDevice m_releDevice);
	//插入行走阈值
	void robot_insert_walk_threshold_req(int startValue,int terminusValue);
	
	/*-------------------------------------------------*/
	//回传
	/*-------------------------------------------------*/
 	void robot_ctrl_to_point_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_move_abs_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_move_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_lift_abs_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_lift_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_cam_ptz_abs_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_cam_ptz_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_body_abs_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_body_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_partialdischarge_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_man_pd_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_man_to_point_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_pd_ptz_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_pd_collect_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_status_light_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_zero_lift_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_emergency_stop_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_warning_light_flash_resp(boost::shared_ptr<commonMsg> msg);
	void robot_ctrl_pd_reset_resp(boost::shared_ptr<commonMsg> msg);
	
	/*--------------------数据库操作--------------------*/
	void robot_update_station_cfg_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_device_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_multi_devices_resp(boost::shared_ptr<commonMsg> msg);
	void robot_update_device_by_ssid_resp(boost::shared_ptr<commonMsg> msg);
	void robot_update_multi_devices_by_ssid_resp(boost::shared_ptr<commonMsg> msg);
	void robot_delete_device_by_ssid_resp(boost::shared_ptr<commonMsg> msg);
	void robot_delete_multi_devices_by_ssid_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_vir_device_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_multi_vir_device_resp(boost::shared_ptr<commonMsg> msg);
	void robot_update_vir_device_resp(boost::shared_ptr<commonMsg> msg);
	void robot_update_multi_vir_device_resp(boost::shared_ptr<commonMsg> msg);
	void robot_delete_vir_device_resp(boost::shared_ptr<commonMsg> msg);
	void robot_delete_multi_vir_device_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_threshol_environment_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_threshol_patrol_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_task_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_task_at_beginning_resp(boost::shared_ptr<commonMsg> msg);
	void robot_update_task_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_task_template_resp(boost::shared_ptr<commonMsg> msg);
	void robot_delete_task_template_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_inpect_result_resp(boost::shared_ptr<commonMsg> msg);
	void robot_delete_task_by_ssid_resp(boost::shared_ptr<commonMsg> msg);
	void robot_update_inspect_result_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_map_Data_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_environment_result_resp(boost::shared_ptr<commonMsg> msg);
	void robot_delete_environment_result_resp(boost::shared_ptr<commonMsg> msg);
	void robot_insert_or_update_restoration_value_resp(boost::shared_ptr<commonMsg> msg);
	void robot_update_devices_for_relative_dev_resp(boost::shared_ptr<commonMsg> msg);
	
private:

    boost::shared_ptr<SocketClient> m_clientSocket;
    QStringList motorStatusString;


};

#define HangRail_BACK_TO_CORE_SOCKET LibDLHangRailRobotBackground::GetSingleton()

#endif