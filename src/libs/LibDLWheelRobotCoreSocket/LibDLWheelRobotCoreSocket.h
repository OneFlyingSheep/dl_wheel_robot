#ifndef __DL_WHEEL_ROBOT_CORE_SOCKET_MSG_H__
#define __DL_WHEEL_ROBOT_CORE_SOCKET_MSG_H__

#define TEN_HZ_MICRO_SECOND 80
#define ONE_HZ_MICRO_SECOND 800

#include "LibSocketLogin/CCoreClient.h"
#include <boost/thread/mutex.hpp>
#include <common/Singleton.hpp>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <vector>
#include <boost/signals2.hpp>
#include "LibVoiceSpeak/LibVoiceSpeak.h"

struct tmpPtz
{
    double dPan;
    double dTilt;
};

class LibDLWheelRobotBackground : public Singleton<LibDLWheelRobotBackground>
{
public:
    LibDLWheelRobotBackground();
    ~LibDLWheelRobotBackground();

	void InitVoiceSpeak();

    void initMouseClickData(QString fileName = ".\\DataCfg.txt");
public:
    boost::signals2::signal<void(WheelRobotRealtimeStatus)> wheelRobotRealtimeStatus; // 机器人实时数据更新
    boost::signals2::signal<void(WheelRobotNoneRealtimeStatus)> wheelRobotNoneRealtimeStatus; // 机器人非实时数据更新
    boost::signals2::signal<void(QString, bool, QString)> wheelRobotDeviceInsertStatus; // 插入设备
    boost::signals2::signal<void(QString, bool, QString)> wheelRobotDeviceUpdateStatus; // 更新设备
    boost::signals2::signal<void(bool, QString)> wheelRobotDeviceDeleteStatus; // 删除设备
    boost::signals2::signal<void(QString)> wheelRobotSlam2DMapFinish; // 机器人上传地图成功
    boost::signals2::signal<void(QStringList)> wheelRobotQueryMapList; // 查询core端所有2d地图
    boost::signals2::signal<void(QStringList)> wheelRobotQuerySMapList; // 查询core端所有smap地图
    boost::signals2::signal<void(std::vector<QPointF>)> wheelRobotLaserData2CollectMapTable; // 机器人激光数据发送至采集地图tab
    boost::signals2::signal<void(double, double, double)> wheelRobotCurrentLoc2CollectMapTable; // 机器人激光数据发送至采集地图tab

	boost::signals2::signal<void(int, bool, QString)> wheelRobotTaskEditInsertStatus; // 插入任务编制
	boost::signals2::signal<void(int, bool, QString)> wheelRobotTaskEditUpdataStatus; // 修改任务编制
	boost::signals2::signal<void(int, bool, QString)> wheelRobotTaskEditDeleteStatus; // 删除任务编制
	boost::signals2::signal<void(bool, QString)> wheelRobotTaskEditImportStatus; // 导入自定义任务编制

	boost::signals2::signal<void(QString, bool, QString)> wheelRobotTaskChooseDeleteStatus; // 删除任务

	boost::signals2::signal<void(int, bool, QString)> wheelRobotPartrolResultVerifyStatus; // 巡检记录确认

	boost::signals2::signal<void(QString, bool, QString)> wheelRobotTaskTemplateInsertStatus; // 任务模板插入

    boost::signals2::signal<void(bool, QString)> wheelRobotInsertAreaStatus; // 地图编辑插入

    boost::signals2::signal<void(bool, QString)> wheelRobotDeleteAreaStatus; // 地图编辑删除

    boost::signals2::signal<void(WheelRobotCurrentTaskInfoShow)> wheelRobotCurrentTaskInfoCallback; // 当前任务回调

    boost::signals2::signal<void(WheelRobotTaskCurrentPointStatus)> wheelRobotCurrentPointStatusCallback; // 当前设备回调

    boost::signals2::signal<void(QString)> wheelRobotSystemWarningCallback; // ?????????
	boost::signals2::signal<void(WheelInspectResultStruct)> WheelRobotInspectResultCallback; // 巡检结果回调
	boost::signals2::signal<void(QString, DeviceAlarmLevel)> WheelRobotAlarmLevelStatus; // 巡检结果传递告警等级

	boost::signals2::signal<void(WheelInspectResultStruct)> WheelRobotCompareInspectResultCallback; // 巡检三项对比回调

	boost::signals2::signal<void(bool, QString, QString)> wheelRobotPatrolResultAuditStatus; //巡检结果审核回调 

	boost::signals2::signal<void(bool, QString)> wheelRobotExcelImportStatus; //excel数据导入回调 

	boost::signals2::signal<void(bool, QString)> wheelRobotUpdateStandardStatus; //标准点位库修改回调 

	boost::signals2::signal<void(QString, bool, QString)> wheelRobotDeleteStandardStatus; //标准点位库删除回调 

	boost::signals2::signal<void(bool, QString)> wheelRobotInsertStandardStatus; //标准点位库插入回调 

	boost::signals2::signal<void(bool, QString)> wheelRobotAddUserConfigStatus; //用户角色插入回调 

    boost::signals2::signal<void(bool, QString)> wheelRobotDeleteUserConfigStatus; //用户角色删除回调 

	boost::signals2::signal<void(bool, QString)> wheelRobotCreateReportStatus; //巡检报告生成

	boost::signals2::signal<void(bool, QString, WheelTaskShow)> wheelRobotExamineReportIsExistStatus; //检查报告是否已生成

	boost::signals2::signal<void(bool, QString)> wheelRobotDeletePatrolPointSetStatus; //巡检点位设置删除

	boost::signals2::signal<void(bool, QString)> wheelRobotStartUsingStatus; //巡检点位启用设置

	boost::signals2::signal<void(bool, QString)> wheelRobotPatrolPointAddStatus; //巡检点位添加回调
	boost::signals2::signal<void(bool, QString)> wheelRobotPatrolPointUpdataStatus; //巡检点位修改回调

	boost::signals2::signal<void(bool, QString)> wheelRobotUpdateTaskStatusStatus; //任务状态修改

    boost::signals2::signal<void(WheelRobotTaskBegin)> signal_wheelRobotTaskBegin; //任务开始
    boost::signals2::signal<void(int)> wheelRobotUpdateTaskSerialNum; //任务当前路径点序列号
    boost::signals2::signal<void(QString, QString, int)> wheelRobotEndTaskMsg; //任务结束

	boost::signals2::signal<void(bool, QString)> wheelRobotInsertNoteMessageStatus; //插入消息订阅回调
	boost::signals2::signal<void(bool, QString)> wheelRobotDeleteNoteMessageStatus; //删除消息订阅回调

	////////////////////人脸
	//boost::signals2::signal<void(bool, QString)> wheelRobotFaceRecogNoteMessageStatus; 


	boost::signals2::signal<void(bool, QString)> wheelRobotInsertThresholdStatus; // 阈值配置返回消息
	boost::signals2::signal<void(bool, WheelRobotCoreRobotConfig)> wheelRobotConnect2NewRobot;


    boost::signals2::signal<void(QVector<int>)> wheelRobotHardwareAlarmCode; // 机器人硬件异常代码
	boost::signals2::signal<void(bool, QString)> wheelAutoRelevanceDevice; // 自动关联回传

    boost::signals2::signal<void(bool, QString)> wheelUploadMap2Robot; // 机器人上传地图回调

    boost::signals2::signal<void(WheelRobotSwitchRunningStatus)> wheelRobotSwitchRunningStatusSignal;// 机器人当前模式

	boost::signals2::signal<void(bool, QString)> wheelRobotFastAuditTaskSignal;//一键审核信号

	boost::signals2::signal<void(bool, QStringList, RootNodeType)> wheelRobotCollectDeviceTreeSignal;//7600-7603设备树消息回传信号

    boost::signals2::signal<void(QString, QString, int)> wheelRobotInfraredTakePhotoSignal;

    boost::signals2::signal<void(int)> wheelRobotInfraredFocusSignal;

    boost::signals2::signal<void()> wheelRobotUpdateTaskTableSignal;

    boost::signals2::signal<void(QVector<AlarmMesgErrorCode>)> wheelRobotAlarmStatusSignal;
    //远程升级返回
    boost::signals2::signal<void(int, QString)> wheelRobotControlUpgradeSignal;

    void closeConnect();
    bool isConnected();
    userLoginRetVal doLogin(std::string host, int  port, std::string username, std::string password);
    void registerHandles();

	//切换机器人
	void switch_robot_req(int robot_id);

	void robot_face_recognition_req(std::string input_name);

	void robot_control_stop_req();
	void robot_control_gyrocal_req();
	void robot_control_reloc_req(float x, float y, float angle);
	void robot_control_comfirmloc_req();

	void robot_control_motion_req(float vx, float vy, float w);
	void robot_control_gotarget_req(std::string id, float angle);
	void robot_control_translate_req(float dist, float vx, int mode);
	void robot_control_turn_req(float angle, float vw, int mode);
	void robot_control_slam_req();
	void robot_control_endslam_req();
	void robot_control_loadmap_req(std::string map_name);
	void robot_control_loadmapobj_req();
    void robot_control_update_device_req(QString deviceFileName);
    void robot_control_back_to_charge();

	void robot_control_ptz_motion_req(WheelRobotPtzMoveType type, int ptz_id, float speed);
	void robot_control_ptz_abs_req(int pan, int tilt);
	void robot_control_ptz_relative_req(int offset_pan, int offset_tilt);
    void robot_control_ptz_monodrome_req(int pt_value, int pt_type);
	void robot_control_device_ctrl_req(bool ultraSonic, bool bWiper, bool bAutoDoor, bool bChargerArm, bool bLedLamp, bool bWarnLamp, bool bFireExtinguisher);
    void robot_control_ptz_light_req(bool bPtzLight);
    void robot_control_ptz_wiper_req(bool bPtzWiper);
    void robot_control_utral_sonic_req(bool bUtralSonic);

    void robot_task_pause_req();
	void robot_task_resume_req();     
    void robot_task_cancel_req();
    void robot_task_assign_req(WheelRobotAssignTask task);
	void robot_task_delete_req(std::string uuid);  
	void robot_task_query_curr_task_req();

	void robot_config_mode_req(WheelRobotSwitchRunningStatus type);
    void robot_config_uploadmap_req(QString map_name);
    void robot_config_downloadmap_req(QString map_name);
    void robot_config_removemap_req(QString map_name);
    void robot_config_download2d_req();
    void robot_config_2d_map_query_req();
    void robot_config_smap_query_req();
	void robot_config_setparams_run_req(WheelRobotRunningParameter p);
	void robot_config_setparams_debug_req(WheelRobotDebugParameter p);
	void robot_config_md5_req(std::string md5);
    bool robot_config_battery_threshold_req(float top, float buttom);
    void robot_config_warning_oper_req(RobotOperationAfterWarning type);
    void robot_config_disconnect_oper_req(RobotOperationAfterDisconnect type);
    void robot_config_ptz_init_req(int x, int y, int x_offset, int y_offset);
    void robot_config_listing_area_req(QVector<WheelRobotPoint> listPoints);
	//急停，0恢复  1停止
	void robot_config_urgency_stop_req(int type);
    //远程升级
    void robot_control_remote_upgrade_req(WheelRemoteUpgradeType type, QString strFileName);
    void robot_control_remote_upgrade_resp(boost::shared_ptr<roboKitMsg> msg);

    
    //req socket->server
	//7500 任务编制插入
	void robot_task_edit_insert_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid);
	//7501 任务编制修改
	void robot_task_edit_updata_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid);
	//  任务编制删除
	void robot_task_edit_delete_req(std::string m_task_edit_uuid, WheelTaskAdminType edit_task_type_id);
	// 任务确认
	void robot_partrol_result_verify_req(DeviceAlarmSearchStruct m_result, int choose);
	//void robot_result_batch_verify_req(WheelRobortResultbatchVerifyStruct m_resultBatchVerify);

// 	void robot_threshold_insert_req(WheelThresholdStruct m_wheelThresholdInsert);
// 	void robot_threshold_updata_req(WheelThresholdStruct m_wheelThresholdInsert);

	//7506 设备插入
    void robot_device_insert_req(WheelRobotInsertDeviceStruct dev);
    void robot_device_update_req(WheelRobortDeviceParameterStruct dev);
    void robot_device_delete_single_dev_req(QString device_uuid);

	//7509 任务模板插入
	void robot_task_template_insert_req(WheelTaskTemplateStruct temp);
	//void robot_task_template_delete_req();

    void robot_device_delete_device_type_req(QString interval_uuid, QString device_uuid);
    void robot_device_delete_interval_req(QString interval_uuid);
    void robot_device_delete_voltage_level_req(QString voltage_level);

	//7515 任务删除
	void robot_db_task_delete_req(QList<WheelDeleteTaskChoose> choo);
	//7519 巡检记录浏览审核
	void robot_patrol_result_audit_req(WheelPartrolResultAudit audit);
	//7520 excel数据导入
	void robot_excel_import_req(QStringList excelData);
	//7521 标准点位库修改
	void robot_update_standard_patrol_vindicate_req(WheelStandardPatrolVindicateStruct data);
	//7522 标准点位库删除
	void robot_delete_standard_patrol_vindicate_req(QString uuid);
	//7523 标准点位库插入
	void robot_insert_standard_patrol_vindicate_req(WheelStandardPatrolVindicateStruct data);
	//7524 任务编制导入
	void robot_task_edit_import_req(WheelTaskEditStruct stru);
	//7525 角色账号增加
	void robot_user_config_add_req(WheelUserConfig data);
	//7526 角色账号删除
	void robot_user_config_delete_req(QString user_uuid);
	//7527 巡检报告生成
	void robot_create_report_req(WheelTaskShow task);
	//7528 巡检报告是否存在
	void robot_examine_report_isexist_req(QString reportName, WheelTaskShow task);
	//7529 删除点位设备
	void robot_delete_patrol_point_set_req(QStringList device_uuid);
	//7530 巡检点位启用状态设置
	void robot_start_using_status_req(QStringList device_uuid, WheelRootStartUsing start_using);
	//7531 巡检点位增加
	void robot_patrol_point_add_req(WheelPatrolPointSet data);
	//7532 巡检点位修改
	void robot_patrol_point_updata_req(WheelPatrolPointSet data);
	//7533 地图选择巡检点下发立即任务
	void robot_task_edit_insert_from_map_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid);
	//7534 更新任务状态
	void robot_update_task_status_req(QString task_uuid, WheelRobotTaskStatusType status);
	//7535 插入消息订阅
	void robot_insert_note_message_req(WheelSubMsgInsert msg);
	//7536 删除消息订阅
	void robot_delete_note_message_req(QString noteUUid, QString fault_name_uuid);
	//7541自动关联按钮
	void robot_auto_relevance_device_req();
	//7543一键审核
	void robot_fast_audit_task_req(QString taskUUid);

	/*--采集构建新设备树/数据库操作--*/

	//7600增加新间隔，返回层级uuid list
	//参数：上一节点电压等级uuid，新间隔名字
	void robot_add_new_interval_req(QString voltageLevelUUid, QString newIntervalName);
	void robot_add_new_interval_resp();

	//7601更新修改间隔名称，返回层级uuid list
	//参数：上一节点电压等级uuid，目标间隔uuid，新间隔名字
	void robot_update_interval_req(QString voltageLevelUUid, QString intervalUUid, QString newIntervalName);
	void robot_update_interval_resp();

	//7602复制粘贴间隔，返回层级uuid list
	//参数：原电压等级uuid，原间隔uuid，新电压等级uuid，新间隔名字
	void robot_copy_paste_interval_req(QString originalVoltageLevelUUid, QString intervalUUid, QString newVoltageLevelUUid, QString newIntervalName);
	void robot_copy_paste_interval_resp();

	//7603删除间隔，返回层级uuid list
	//参数：目标电压等级uuid，目标间隔uuid
	void robot_delete_interval_req(QString voltageLevelUUid, QString intervalUUid);
	void robot_delete_interval_resp();

	//7604添加新设备类型及全设备，返回层级uuid list
	//参数：目标电压等级uuid，目标间隔uuid，目标设备类型list
	void robot_add_deviceType_andDevices_req(QString voltageLevelUUid, QString intervalUUid, QStringList deviceTypeUUidList);
	void robot_add_deviceType_andDevices_resp();

	//7605删除设备类型，返回层级uuid list
	//参数：目标电压等级uuid，目标间隔uuid，目标设备类型uuid
	void robot_delete_device_type_req(QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid);
	void robot_delete_device_type_resp();

	//7606删除设备类型，返回层级uuid list
	//参数：目标电压等级uuid，目标间隔uuid，目标设备类型uuid list
	void robot_delete_device_type_req(QString voltageLevelUUid, QString intervalUUid, QStringList deviceTypeUUidList);

	//7607新增具体设备，返回层级uuid list
	//参数：目标电压等级uuid，目标间隔uuid，目标设备类型uuid，点位uuid list
	void robot_add_devices_fromlist_req(QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid, QStringList pointNameUUid);
	void robot_add_devices_fromlist_resp();

	//7608删除具体设备，返回层级uuid list
	//参数：目标电压等级uuid，目标间隔uuid，目标设备类型uuid，设备uuid list
	void robot_delete_devices_fromlist_req(QStringList devicesUUidList, QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid);
	void robot_delete_devices_fromlist_resp();

	//7600-7608协议回传统一接口
	void robot_collect_device_tree_resp(boost::shared_ptr<roboKitMsg> msg);
	/*--end--*/

	void robot_control_infrared_take_photo_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_update_task_table_signal(boost::shared_ptr<roboKitMsg> msg);

	// resp server->socket
    void robot_ctrl_All_Resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_status_real_time_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_status_none_real_time_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_status_alarm_initiative_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_task_pause_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_resume_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_cancel_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_assign_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_delete_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_query_curr_task_resp(boost::shared_ptr<roboKitMsg> msg);

	//void robot_status_alarm_resp(boost::shared_ptr<roboKitMsg> msg);  //机器人状态告警

    void robot_task_current_task_info_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_point_percent_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_task_begin(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_serial(boost::shared_ptr<roboKitMsg> msg);

    void robot_config_download2d_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_config_upload2d_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_config_2d_map_query_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_config_smap_query_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_config_listing_are_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_device_insert_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_device_update_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_device_delete_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_inspect_result_resp(boost::shared_ptr<roboKitMsg> msg);
	void robot_compare_inspect_resule_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_task_template_insert_resq(boost::shared_ptr<roboKitMsg> msg);

    void robot_device_delete_device_type_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_device_delete_interval_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_device_delete_voltage_level_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_task_edit_insert_resp(boost::shared_ptr<roboKitMsg> msg);
	void robot_task_edit_updata_resp(boost::shared_ptr<roboKitMsg> msg);
	void robot_task_edit_delete_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_task_device_finish_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_finish_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_task_all_finish_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_db_task_delete_resp(boost::shared_ptr<roboKitMsg> msg);
	void robot_partrol_result_verify_resp(boost::shared_ptr<roboKitMsg> msg);
	//一键审核回传
	void robot_ctrl_fast_audit_task_resp(boost::shared_ptr<roboKitMsg> msg);

	///
	void robot_ctrl_on_mouse_click_on_In_window(int width, int height, int x, int y, int z, bool isInfrared = false);
	/// 
    void robot_ctrl_on_mouse_click_on_hc_window(int width, int height, int x, int y, int z, bool isInfrared = false);

    void robot_config_insert_voltage_level(QString voltage_level_name);
    void robot_config_insert_area(QString device_area_name);
    void robot_config_insert_interval(QString voltage_level_id, QString equipment_interval_name);

    void robot_config_delete_voltage_level(QString voltage_level_id);
    void robot_config_delete_area(QString device_area_uuid);
    void robot_config_delete_interval(QString equipment_interval_uuid);

    void robot_threshold_set_by_device(QVector<THRESHOLD_ELEMENT> alarmNormalList, QVector<THRESHOLD_ELEMENT> alarmWarningList, QVector<THRESHOLD_ELEMENT> alarmCommonList, QVector<THRESHOLD_ELEMENT> alarmSerialList, QVector<THRESHOLD_ELEMENT> alarmDangerList, QVector<QString> device_uuid);
    void robot_threshold_set_by_meter_type(QVector<THRESHOLD_ELEMENT> alarmNormalList, QVector<THRESHOLD_ELEMENT> alarmWarningList, QVector<THRESHOLD_ELEMENT> alarmCommonList, QVector<THRESHOLD_ELEMENT> alarmSerialList, QVector<THRESHOLD_ELEMENT> alarmDangerList, WheelRobotMeterType meter_type);
    void resp_robot_insert_threshold(boost::shared_ptr<roboKitMsg> msg);

    void robot_config_insert_area_map_resp(boost::shared_ptr<roboKitMsg> msg);
    void robot_config_delete_area_map_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_patrol_result_audit_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_excel_import_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_update_standard_patrol_vindicate_resp(boost::shared_ptr<roboKitMsg> msg);
	void robot_delete_standard_patrol_vindicate_resp(boost::shared_ptr<roboKitMsg> msg);
	void robot_insert_standard_patrol_vindicate_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_task_edit_import_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_user_config_add_resp(boost::shared_ptr<roboKitMsg> msg);
	void robot_user_config_delete_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_create_report_resp(boost::shared_ptr<roboKitMsg> msg);
	void robot_examine_report_isexist_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_delete_patrol_point_set_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_start_using_status_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_patrol_point_add_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_patrol_point_updata_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_task_edit_insert_from_map_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_update_task_status_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_insert_note_message_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_delete_note_message_resp(boost::shared_ptr<roboKitMsg> msg);

	//	人脸
	void robot_face_recog_note_message_resp(boost::shared_ptr<roboKitMsg> msg);


    void robot_connect_2_new_robot_req(QString robotName);
    void robot_connect_2_new_robot_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_auto_relevance_device_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_config_uploadmap2Robot_resp(boost::shared_ptr<roboKitMsg> msg);

    void set_battery_low_voltage(float vol);

    void robot_connect_status(boost::shared_ptr<roboKitMsg> msg);

    void robot_update_embedded_software();

	//红外拍照
    void robot_cloud_infrared_take_photo_req(QString uploadPath, QString deviceUUid, int type);
    //红外对焦
    void robot_cloud_infrared_auto_focus_req(InfraredFocusMode modeType);
    //红外录像
    void robot_cloud_infrared_record_video_req(QString uploadPath, int status);

    void robot_cloud_infrared_set_focus_req(int focus);

	//地图画线巡点
	void robot_setting_out_run_point_req(QList<MapSettingOutCoordinates> data);
	
	//speak显示的消息
	void robot_alarm_display_msg(QString strAlarmDisplaymsg);

    void robot_temporary_door_req(bool type);
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Function:       void postMsg(jsonBody &msg);
    ///
    /// Brief:
    ///         Posts a message.
    ///
    /// Author:
    ///         Berry
    ///
    /// Date:
    ///         2018/5/15
    ///
    /// Param:
    ///     msg -   [in,out] The message. 
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void postMsg(jsonBody &msg);
// 
//     void initAlarmMap();
// 
//     std::string getAlarmCode(int id);

    boost::shared_ptr<CCoreClient> m_clientSocket;

    uint16_t getMsgId();
    boost::mutex getMsgIdMutex;

    WheelRobotPtzStatus m_currPtz;
    std::map<int, tmpPtz> angMap;

    std::map<int, std::string> alarm_map_;
    
    float voltage_;

	LibVoiceSpeak *m_pSpeeker{nullptr};

	QList<int> m_alarmData;
};

#define WHEEL_BACK_TO_CORE_SOCKET LibDLWheelRobotBackground::GetSingleton()

#endif