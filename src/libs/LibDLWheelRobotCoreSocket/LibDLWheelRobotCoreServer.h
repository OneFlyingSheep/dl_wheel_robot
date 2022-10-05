#ifndef __DL_WHEEL_Remote_robot_BACKGROUND_MSG_H__
#define __DL_WHEEL_Remote_robot_BACKGROUND_MSG_H__

#include "LibSocketLogin/CCoreServer.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <common/Singleton.hpp>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLWheelRobotMsg.h"
#include <boost/bind.hpp>
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include <boost/signals2.hpp>
#include <LibConvertSymbolsToLua/LibConvertSymbolsToLua.h>

class LibDLWheelRobotCoreServer : public Singleton<LibDLWheelRobotCoreServer>
{
public:
    LibDLWheelRobotCoreServer();
    ~LibDLWheelRobotCoreServer();

    boost::signals2::signal<void(WheelRobotDeviceCollectStatus)> signal_taskCollectFinish;
    boost::signals2::signal<void()> signal_updateTimedTask;
    boost::signals2::signal<void(WheelRobotDeviceCollectData)> signal_deviceUploadFinish;

    boost::signals2::signal<void(WheelRobotTaskBegin)> wheelRobotTaskCallback; // 任务开始回调
    boost::signals2::signal<void(QString, QString, int)> wheelRobotCurrentDeviceCallback; // 当前设备回调

    boost::signals2::signal<void()> countTodyTaskCallback; // 重新计算今日周期任务

    boost::signals2::signal<void(std::string)> bjRobotCoordAlterSignal;                       // 机器人坐标变化
    boost::signals2::signal<void(std::vector<std::string>)> bjRobotPatrolLineSignal;          // 机器人生成巡视路线
    boost::signals2::signal<void(WheelRobotCurrentTaskInfoShow)> bjRobotTaskStatusDataSignal; // 

    boost::signals2::signal<void(QString, QString)> bjRobotDeviceResultSendSignal;            // 任务识别设备上报

	boost::signals2::signal<void(QString)> bAssignFaceRecogSignal;            // 下发人脸识别任务

	//	new add
	boost::signals2::signal<void(int, std::string)> robotBodyInfoSignal;	//机器人本体信息
	boost::signals2::signal<void(int, std::string)> robotEnvInfoSignal;		//机器人环境采集信息
	boost::signals2::signal<void(int, std::string)> robotTaskTrajSignal;	//机器人任务结构信息
	boost::signals2::signal<void(int, std::string)> robotPosSignal;			//机器人任务结构信息
	boost::signals2::signal<void(int, std::string)> robotPatrolStatusSignal;//机器人巡检任务状态

    void reconnectToRobot();

    void disconnect();
	LibDLWheelRobotMsg* getCurrentRobotSocket();

public:

    void startServer();
    // background => core
    // 可直接转发至机器人的消息
	void Remote_robot_msg(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_run_msg(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_switch_msg(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_ptz_msg(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_ptz_infradred_msg(boost::shared_ptr<roboKitMsg> msg);	///<红外右键调节中心点位置

	// 需在最上层处理的消息
	void Remote_robot_switch_robot_req(boost::shared_ptr<roboKitMsg> msg); 

    void Remote_robot_task_assign_msg(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_device_insert_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_device_insert_resp(std::string device_uuid, bool bInserted, std::string insertMsg);
    void Remote_robot_device_update_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_device_update_resp(std::string device_uuid, bool bUpdated, std::string updateMsg);
    void Remote_robot_device_delete_single_device_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_device_delete_operation_resp(bool bDeleted, std::string deleteMsg);
    void Remote_robot_device_delete_device_type_deivce_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_device_delete_interval_deivce_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_device_delete_voltage_deivce_req(boost::shared_ptr<roboKitMsg> msg);

    void Remote_robot_control_remote_upgrade_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_control_remote_upgrade_resp(boost::shared_ptr<roboKitMsg> msg);

    void Remote_robot_task_edit_insert_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_task_edit_insert_resp(int task_edit_type_id, bool bInserted, std::string insertMsg);
	void Remote_robot_task_edit_update_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_task_edit_update_resp(int task_edit_type_id, bool bInserted, std::string deleteMsg);
	void Remote_robot_task_edit_delete_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_task_edit_delete_resp(int task_edit_type_id, bool bInserted, std::string deleteMsg);

	void Remote_robot_db_task_delete_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_db_task_delete_resp(std::string task_delete_uuid, bool bInserted, std::string insertMsg);

	void Remote_robot_partrol_result_verify_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_partrol_result_verify_resp(int choose, bool bInserted, std::string insertMsg);

	void Remote_robot_task_template_insert_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_task_template_insert_resq(std::string task_template_uuid, bool bInserted, std::string insertMsg);

    void Remote_robot_config_insert_voltage_level_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_config_insert_area_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_config_insert_interval_req(boost::shared_ptr<roboKitMsg> msg);

    void Remote_robot_config_delete_voltage_level_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_config_delete_area_req(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_config_delete_interval_req(boost::shared_ptr<roboKitMsg> msg);

    void Remote_robot_inspect_result(WheelInspectResultStruct res);

	void Remote_robot_compare_inspect_result(CompareDeviceInspectResult data);

    void Remote_robot_current_task_status(WheelRobotCurrentTaskInfoShow task);

	void Remote_robot_patrol_result_audit_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_patrol_result_audit_resp(bool bInserted,QString task_uuid, std::string insertMsg);

	void Remote_robot_excel_import_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_excel_import_resp(bool bInserted, std::string insertMsg);

	void Remote_robot_insert_standard_patrol_vindicate_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_insert_standard_patrol_vindicate_resp(bool bInserted, std::string insertMsg);

	void Remote_robot_updata_standard_patrol_vindicate_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_updata_standard_patrol_vindicate_resp(bool bInserted, std::string insertMsg);

	void Remote_robot_delete_standard_patrol_vindicate_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_delete_standard_patrol_vindicate_resp(std::string uuid, bool bInserted, std::string insertMsg);

	void Remote_robot_task_edit_import_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_task_edit_import_resp(bool bImport, std::string importMsg);

	void Remote_robot_user_config_add_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_user_config_add_resp(bool bAdd, std::string addMsg);

	void Remote_robot_user_config_delete_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_user_config_delete_resp(bool bAdd, std::string addMsg);

	void Remote_robot_delete_patrol_point_set_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_delete_patrol_point_set_resp(bool bRet, std::string retMsg);

	void Remote_robot_start_using_status_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_start_using_status_resp(bool bRet, std::string retMsg);

	void Remote_robot_patrol_point_add_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_patrol_point_add_resp(bool bRet, std::string retMsg);

	void Remote_robot_patrol_point_updata_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_patrol_point_updata_resp(bool bRet, std::string retMsg);

	void Remote_robot_update_task_status_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_update_task_status_resp(bool bRet, std::string retMsg);

	void Remote_robot_insert_note_message_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_insert_note_message_resp(bool bRet, std::string retMsg);

	void Remote_robot_delete_note_message_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_delete_note_message_resp(bool bRet, std::string retMsg);

    void Remote_robot_connect_2_new_robot_req(boost::shared_ptr<roboKitMsg> msg);

	void Remote_robot_auto_relevance_device_req(boost::shared_ptr<roboKitMsg> msg);
	void Remote_robot_auto_relevance_device_resp(bool bRet);

    void Remote_robot_disconnect(bool isConnect);

	void Romote_robot_fast_audit_task_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_fast_audit_task_resp(bool bRet);

	void Romote_robot_control_infrared_take_photo_resp(boost::shared_ptr<roboKitMsg> msg);
	/*--处理采集新设备树--*/
	void Romote_robot_add_new_interval_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_update_interval_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_copy_paste_interval_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_delete_interval_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_add_deviceType_andDevices_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_delete_device_type_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_delete_device_type_list_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_add_devices_fromlist_req(boost::shared_ptr<roboKitMsg> msg);
	void Romote_robot_delete_devices_fromlist_req(boost::shared_ptr<roboKitMsg> msg);
	/*end*/


    // robot <=> core
	//	new add Bim
	void robot_env_sensor_info(boost::shared_ptr<roboKitMsg> msg);
	void robot_task_traj_record(boost::shared_ptr<roboKitMsg> msg);

    void robot_task_Direct(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_Begin(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_Current_Task_Device(boost::shared_ptr<roboKitMsg> msg);

    void robot_heart_beat_msg(boost::shared_ptr<roboKitMsg> msg);

    bool robot_task_assign(WheelTaskTemplateStruct assignTask);

    void robot_task_query_task_list_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_device_finish_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_finish_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_all_finish_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_status_real_time_req();
    void robot_status_none_real_time_req();
    void robot_status_alarm_req();
    
    void robot_status_Direct_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_config_Direct_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_status_real_time(boost::shared_ptr<roboKitMsg> msg);
    void robot_status_none_real_time(boost::shared_ptr<roboKitMsg> msg);
    void robot_status_alarm_initiative(boost::shared_ptr<roboKitMsg> msg);


    void robot_config_time_sync();

    void robot_task_delete_req(QString strUuid);

    //获取机器人连接状态
    bool get_is_connect_robot();


private:
    void registerBack2CoreHandles();
    void registerRobot2CoreHandels();

private:
    boost::shared_ptr<CCoreServer> m_serverSocket;

    uint16_t getMsgId();
	void pointInsertDB(QString uuid, QStringList c_data, bool &bRet, QString &retMsg);
    boost::mutex getMsgIdMutex;

	boost::atomic_int m_current_robotID;

	std::map<int, LibDLWheelRobotMsg*> m_robotSocketMap;

    boost::thread *m_statusRealtimeThread;
    bool bStatusRealtimeRunning;
    void robotRealtimeStatusFunc();

    boost::thread *m_statusNoneRealtimeThread;
    bool bStatusNoneRealtimeRunning;
    void robotNoneRealtimeStatusFunc();

    boost::thread *m_heartbeatThread;
    bool bheartbeatRunning;
    void robotheartbeatFunc();
    
    conventSymbols2Lua m_convert2Lua;
	
    QString m_nowRunningTaskUuid;

};

#define WHEEL_CORE_SERVER LibDLWheelRobotCoreServer::GetSingleton()

#endif