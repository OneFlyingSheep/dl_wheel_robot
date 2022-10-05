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
    boost::signals2::signal<void(WheelRobotRealtimeStatus)> wheelRobotRealtimeStatus; // ������ʵʱ���ݸ���
    boost::signals2::signal<void(WheelRobotNoneRealtimeStatus)> wheelRobotNoneRealtimeStatus; // �����˷�ʵʱ���ݸ���
    boost::signals2::signal<void(QString, bool, QString)> wheelRobotDeviceInsertStatus; // �����豸
    boost::signals2::signal<void(QString, bool, QString)> wheelRobotDeviceUpdateStatus; // �����豸
    boost::signals2::signal<void(bool, QString)> wheelRobotDeviceDeleteStatus; // ɾ���豸
    boost::signals2::signal<void(QString)> wheelRobotSlam2DMapFinish; // �������ϴ���ͼ�ɹ�
    boost::signals2::signal<void(QStringList)> wheelRobotQueryMapList; // ��ѯcore������2d��ͼ
    boost::signals2::signal<void(QStringList)> wheelRobotQuerySMapList; // ��ѯcore������smap��ͼ
    boost::signals2::signal<void(std::vector<QPointF>)> wheelRobotLaserData2CollectMapTable; // �����˼������ݷ������ɼ���ͼtab
    boost::signals2::signal<void(double, double, double)> wheelRobotCurrentLoc2CollectMapTable; // �����˼������ݷ������ɼ���ͼtab

	boost::signals2::signal<void(int, bool, QString)> wheelRobotTaskEditInsertStatus; // �����������
	boost::signals2::signal<void(int, bool, QString)> wheelRobotTaskEditUpdataStatus; // �޸��������
	boost::signals2::signal<void(int, bool, QString)> wheelRobotTaskEditDeleteStatus; // ɾ���������
	boost::signals2::signal<void(bool, QString)> wheelRobotTaskEditImportStatus; // �����Զ����������

	boost::signals2::signal<void(QString, bool, QString)> wheelRobotTaskChooseDeleteStatus; // ɾ������

	boost::signals2::signal<void(int, bool, QString)> wheelRobotPartrolResultVerifyStatus; // Ѳ���¼ȷ��

	boost::signals2::signal<void(QString, bool, QString)> wheelRobotTaskTemplateInsertStatus; // ����ģ�����

    boost::signals2::signal<void(bool, QString)> wheelRobotInsertAreaStatus; // ��ͼ�༭����

    boost::signals2::signal<void(bool, QString)> wheelRobotDeleteAreaStatus; // ��ͼ�༭ɾ��

    boost::signals2::signal<void(WheelRobotCurrentTaskInfoShow)> wheelRobotCurrentTaskInfoCallback; // ��ǰ����ص�

    boost::signals2::signal<void(WheelRobotTaskCurrentPointStatus)> wheelRobotCurrentPointStatusCallback; // ��ǰ�豸�ص�

    boost::signals2::signal<void(QString)> wheelRobotSystemWarningCallback; // ?????????
	boost::signals2::signal<void(WheelInspectResultStruct)> WheelRobotInspectResultCallback; // Ѳ�����ص�
	boost::signals2::signal<void(QString, DeviceAlarmLevel)> WheelRobotAlarmLevelStatus; // Ѳ�������ݸ澯�ȼ�

	boost::signals2::signal<void(WheelInspectResultStruct)> WheelRobotCompareInspectResultCallback; // Ѳ������ԱȻص�

	boost::signals2::signal<void(bool, QString, QString)> wheelRobotPatrolResultAuditStatus; //Ѳ������˻ص� 

	boost::signals2::signal<void(bool, QString)> wheelRobotExcelImportStatus; //excel���ݵ���ص� 

	boost::signals2::signal<void(bool, QString)> wheelRobotUpdateStandardStatus; //��׼��λ���޸Ļص� 

	boost::signals2::signal<void(QString, bool, QString)> wheelRobotDeleteStandardStatus; //��׼��λ��ɾ���ص� 

	boost::signals2::signal<void(bool, QString)> wheelRobotInsertStandardStatus; //��׼��λ�����ص� 

	boost::signals2::signal<void(bool, QString)> wheelRobotAddUserConfigStatus; //�û���ɫ����ص� 

    boost::signals2::signal<void(bool, QString)> wheelRobotDeleteUserConfigStatus; //�û���ɫɾ���ص� 

	boost::signals2::signal<void(bool, QString)> wheelRobotCreateReportStatus; //Ѳ�챨������

	boost::signals2::signal<void(bool, QString, WheelTaskShow)> wheelRobotExamineReportIsExistStatus; //��鱨���Ƿ�������

	boost::signals2::signal<void(bool, QString)> wheelRobotDeletePatrolPointSetStatus; //Ѳ���λ����ɾ��

	boost::signals2::signal<void(bool, QString)> wheelRobotStartUsingStatus; //Ѳ���λ��������

	boost::signals2::signal<void(bool, QString)> wheelRobotPatrolPointAddStatus; //Ѳ���λ��ӻص�
	boost::signals2::signal<void(bool, QString)> wheelRobotPatrolPointUpdataStatus; //Ѳ���λ�޸Ļص�

	boost::signals2::signal<void(bool, QString)> wheelRobotUpdateTaskStatusStatus; //����״̬�޸�

    boost::signals2::signal<void(WheelRobotTaskBegin)> signal_wheelRobotTaskBegin; //����ʼ
    boost::signals2::signal<void(int)> wheelRobotUpdateTaskSerialNum; //����ǰ·�������к�
    boost::signals2::signal<void(QString, QString, int)> wheelRobotEndTaskMsg; //�������

	boost::signals2::signal<void(bool, QString)> wheelRobotInsertNoteMessageStatus; //������Ϣ���Ļص�
	boost::signals2::signal<void(bool, QString)> wheelRobotDeleteNoteMessageStatus; //ɾ����Ϣ���Ļص�

	////////////////////����
	//boost::signals2::signal<void(bool, QString)> wheelRobotFaceRecogNoteMessageStatus; 


	boost::signals2::signal<void(bool, QString)> wheelRobotInsertThresholdStatus; // ��ֵ���÷�����Ϣ
	boost::signals2::signal<void(bool, WheelRobotCoreRobotConfig)> wheelRobotConnect2NewRobot;


    boost::signals2::signal<void(QVector<int>)> wheelRobotHardwareAlarmCode; // ������Ӳ���쳣����
	boost::signals2::signal<void(bool, QString)> wheelAutoRelevanceDevice; // �Զ������ش�

    boost::signals2::signal<void(bool, QString)> wheelUploadMap2Robot; // �������ϴ���ͼ�ص�

    boost::signals2::signal<void(WheelRobotSwitchRunningStatus)> wheelRobotSwitchRunningStatusSignal;// �����˵�ǰģʽ

	boost::signals2::signal<void(bool, QString)> wheelRobotFastAuditTaskSignal;//һ������ź�

	boost::signals2::signal<void(bool, QStringList, RootNodeType)> wheelRobotCollectDeviceTreeSignal;//7600-7603�豸����Ϣ�ش��ź�

    boost::signals2::signal<void(QString, QString, int)> wheelRobotInfraredTakePhotoSignal;

    boost::signals2::signal<void(int)> wheelRobotInfraredFocusSignal;

    boost::signals2::signal<void()> wheelRobotUpdateTaskTableSignal;

    boost::signals2::signal<void(QVector<AlarmMesgErrorCode>)> wheelRobotAlarmStatusSignal;
    //Զ����������
    boost::signals2::signal<void(int, QString)> wheelRobotControlUpgradeSignal;

    void closeConnect();
    bool isConnected();
    userLoginRetVal doLogin(std::string host, int  port, std::string username, std::string password);
    void registerHandles();

	//�л�������
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
	//��ͣ��0�ָ�  1ֹͣ
	void robot_config_urgency_stop_req(int type);
    //Զ������
    void robot_control_remote_upgrade_req(WheelRemoteUpgradeType type, QString strFileName);
    void robot_control_remote_upgrade_resp(boost::shared_ptr<roboKitMsg> msg);

    
    //req socket->server
	//7500 ������Ʋ���
	void robot_task_edit_insert_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid);
	//7501 ��������޸�
	void robot_task_edit_updata_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid);
	//  �������ɾ��
	void robot_task_edit_delete_req(std::string m_task_edit_uuid, WheelTaskAdminType edit_task_type_id);
	// ����ȷ��
	void robot_partrol_result_verify_req(DeviceAlarmSearchStruct m_result, int choose);
	//void robot_result_batch_verify_req(WheelRobortResultbatchVerifyStruct m_resultBatchVerify);

// 	void robot_threshold_insert_req(WheelThresholdStruct m_wheelThresholdInsert);
// 	void robot_threshold_updata_req(WheelThresholdStruct m_wheelThresholdInsert);

	//7506 �豸����
    void robot_device_insert_req(WheelRobotInsertDeviceStruct dev);
    void robot_device_update_req(WheelRobortDeviceParameterStruct dev);
    void robot_device_delete_single_dev_req(QString device_uuid);

	//7509 ����ģ�����
	void robot_task_template_insert_req(WheelTaskTemplateStruct temp);
	//void robot_task_template_delete_req();

    void robot_device_delete_device_type_req(QString interval_uuid, QString device_uuid);
    void robot_device_delete_interval_req(QString interval_uuid);
    void robot_device_delete_voltage_level_req(QString voltage_level);

	//7515 ����ɾ��
	void robot_db_task_delete_req(QList<WheelDeleteTaskChoose> choo);
	//7519 Ѳ���¼������
	void robot_patrol_result_audit_req(WheelPartrolResultAudit audit);
	//7520 excel���ݵ���
	void robot_excel_import_req(QStringList excelData);
	//7521 ��׼��λ���޸�
	void robot_update_standard_patrol_vindicate_req(WheelStandardPatrolVindicateStruct data);
	//7522 ��׼��λ��ɾ��
	void robot_delete_standard_patrol_vindicate_req(QString uuid);
	//7523 ��׼��λ�����
	void robot_insert_standard_patrol_vindicate_req(WheelStandardPatrolVindicateStruct data);
	//7524 ������Ƶ���
	void robot_task_edit_import_req(WheelTaskEditStruct stru);
	//7525 ��ɫ�˺�����
	void robot_user_config_add_req(WheelUserConfig data);
	//7526 ��ɫ�˺�ɾ��
	void robot_user_config_delete_req(QString user_uuid);
	//7527 Ѳ�챨������
	void robot_create_report_req(WheelTaskShow task);
	//7528 Ѳ�챨���Ƿ����
	void robot_examine_report_isexist_req(QString reportName, WheelTaskShow task);
	//7529 ɾ����λ�豸
	void robot_delete_patrol_point_set_req(QStringList device_uuid);
	//7530 Ѳ���λ����״̬����
	void robot_start_using_status_req(QStringList device_uuid, WheelRootStartUsing start_using);
	//7531 Ѳ���λ����
	void robot_patrol_point_add_req(WheelPatrolPointSet data);
	//7532 Ѳ���λ�޸�
	void robot_patrol_point_updata_req(WheelPatrolPointSet data);
	//7533 ��ͼѡ��Ѳ����·���������
	void robot_task_edit_insert_from_map_req(WheelTaskEditStruct m_wheelTaskEditStru, QStringList device_uuid);
	//7534 ��������״̬
	void robot_update_task_status_req(QString task_uuid, WheelRobotTaskStatusType status);
	//7535 ������Ϣ����
	void robot_insert_note_message_req(WheelSubMsgInsert msg);
	//7536 ɾ����Ϣ����
	void robot_delete_note_message_req(QString noteUUid, QString fault_name_uuid);
	//7541�Զ�������ť
	void robot_auto_relevance_device_req();
	//7543һ�����
	void robot_fast_audit_task_req(QString taskUUid);

	/*--�ɼ��������豸��/���ݿ����--*/

	//7600�����¼�������ز㼶uuid list
	//��������һ�ڵ��ѹ�ȼ�uuid���¼������
	void robot_add_new_interval_req(QString voltageLevelUUid, QString newIntervalName);
	void robot_add_new_interval_resp();

	//7601�����޸ļ�����ƣ����ز㼶uuid list
	//��������һ�ڵ��ѹ�ȼ�uuid��Ŀ����uuid���¼������
	void robot_update_interval_req(QString voltageLevelUUid, QString intervalUUid, QString newIntervalName);
	void robot_update_interval_resp();

	//7602����ճ����������ز㼶uuid list
	//������ԭ��ѹ�ȼ�uuid��ԭ���uuid���µ�ѹ�ȼ�uuid���¼������
	void robot_copy_paste_interval_req(QString originalVoltageLevelUUid, QString intervalUUid, QString newVoltageLevelUUid, QString newIntervalName);
	void robot_copy_paste_interval_resp();

	//7603ɾ����������ز㼶uuid list
	//������Ŀ���ѹ�ȼ�uuid��Ŀ����uuid
	void robot_delete_interval_req(QString voltageLevelUUid, QString intervalUUid);
	void robot_delete_interval_resp();

	//7604������豸���ͼ�ȫ�豸�����ز㼶uuid list
	//������Ŀ���ѹ�ȼ�uuid��Ŀ����uuid��Ŀ���豸����list
	void robot_add_deviceType_andDevices_req(QString voltageLevelUUid, QString intervalUUid, QStringList deviceTypeUUidList);
	void robot_add_deviceType_andDevices_resp();

	//7605ɾ���豸���ͣ����ز㼶uuid list
	//������Ŀ���ѹ�ȼ�uuid��Ŀ����uuid��Ŀ���豸����uuid
	void robot_delete_device_type_req(QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid);
	void robot_delete_device_type_resp();

	//7606ɾ���豸���ͣ����ز㼶uuid list
	//������Ŀ���ѹ�ȼ�uuid��Ŀ����uuid��Ŀ���豸����uuid list
	void robot_delete_device_type_req(QString voltageLevelUUid, QString intervalUUid, QStringList deviceTypeUUidList);

	//7607���������豸�����ز㼶uuid list
	//������Ŀ���ѹ�ȼ�uuid��Ŀ����uuid��Ŀ���豸����uuid����λuuid list
	void robot_add_devices_fromlist_req(QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid, QStringList pointNameUUid);
	void robot_add_devices_fromlist_resp();

	//7608ɾ�������豸�����ز㼶uuid list
	//������Ŀ���ѹ�ȼ�uuid��Ŀ����uuid��Ŀ���豸����uuid���豸uuid list
	void robot_delete_devices_fromlist_req(QStringList devicesUUidList, QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid);
	void robot_delete_devices_fromlist_resp();

	//7600-7608Э��ش�ͳһ�ӿ�
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

	//void robot_status_alarm_resp(boost::shared_ptr<roboKitMsg> msg);  //������״̬�澯

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
	//һ����˻ش�
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

	//	����
	void robot_face_recog_note_message_resp(boost::shared_ptr<roboKitMsg> msg);


    void robot_connect_2_new_robot_req(QString robotName);
    void robot_connect_2_new_robot_resp(boost::shared_ptr<roboKitMsg> msg);

	void robot_auto_relevance_device_resp(boost::shared_ptr<roboKitMsg> msg);

    void robot_config_uploadmap2Robot_resp(boost::shared_ptr<roboKitMsg> msg);

    void set_battery_low_voltage(float vol);

    void robot_connect_status(boost::shared_ptr<roboKitMsg> msg);

    void robot_update_embedded_software();

	//��������
    void robot_cloud_infrared_take_photo_req(QString uploadPath, QString deviceUUid, int type);
    //����Խ�
    void robot_cloud_infrared_auto_focus_req(InfraredFocusMode modeType);
    //����¼��
    void robot_cloud_infrared_record_video_req(QString uploadPath, int status);

    void robot_cloud_infrared_set_focus_req(int focus);

	//��ͼ����Ѳ��
	void robot_setting_out_run_point_req(QList<MapSettingOutCoordinates> data);
	
	//speak��ʾ����Ϣ
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