#ifndef __DL_COMMON_DL_WHEEL_ROBOT_OPER_H__
#define __DL_COMMON_DL_WHEEL_ROBOT_OPER_H__

#include <map>
#include <QtSql/QSql>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlRecord>
#include <QtSql/QSqlTableModel>
#include "common/Singleton.hpp"
#include "common/DLWheelRobotGlobalDef.hpp"
#include <boost/serialization/singleton.hpp>
#include <boost/thread/mutex.hpp>
#include <QStandardItemModel>
#include <QTableView>
#include <QTreeView>
class LibDLWheelRobotDBOperation : public Singleton<LibDLWheelRobotDBOperation>
{

public:
    LibDLWheelRobotDBOperation();
    ~LibDLWheelRobotDBOperation();

	bool openDb(QString hostName, QString dbName, QString userName, QString passwd, int port = 3306);

private:
    void closeDb();
    bool execSqlString(QSqlQuery &qry);
    bool querySqlString(QString sqlString, QSqlQuery &qry);

public:
	bool insertDeviceSn(QString device_sn, QString device_name);
	bool getDeviceSn(std::map<QString, QString> &bim_device_map);

public:
    bool insertDevice(WheelRobotInsertDeviceStruct dev, QString &errMsg);
    bool updateDeviceParameter(WheelRobortDeviceParameterStruct dev, QString &errMsg);
    bool updateDeviceAlarmLevel(DeviceAlarmLevel level, QString device_uuid);
    bool deleteDeviceFromDeviceUUid(QString uuid, QString &errMsg);
    bool deleteDeviceTypeByIntervalAndDeviceType(QString interval_uuid, QString device_uuid, QString &errMsg);
    bool deleteIntervalByIntervalUuid(QString interval_uuid, QString &errMsg);
	bool deleteVoltageDevicesByVoltageId(QString voltage_id, QString &errMsg);
	bool deleteDeviceFromDeviceUUidList(QStringList devicesUUidList);
	bool getDeviceFullNameByDeviceUuid(QString device_uuid, QString &fullName);
	bool selectDeviceNodeUUidForDeviceUUid(QString device_uuid, QStringList &nodeUUid);;

    bool deleteDeviceFromDeviceList(QStringList deviceList);
    bool deleteDeviceParameterFromDeviceList(QStringList deviceList);

    bool queryDeviceByDeviceUuid(QString uuid, WheelRobotDeviceStruct &dev);

	bool getWheelStationConfigDataDB(std::map<int, WheelStationConfigStruct> &m_WheelStationConfigData);
	bool getWheelDeviceAreaDataDB(std::map<QString, QString> &m_WheelDeviceAreaData);
	bool getWheelDevicePointNameDB(std::map<QString, WheelDevicePointNameStruct> &m_WheelDevicePointNameData);
	bool getWheelSubDeviceTypeDB(std::map<QString, WheelSubDeviceTypeStruct> &m_WheelSubDeviceTypeData);
	bool getWheelDeviceTypeDB(std::map<QString, QString> &m_WheelDeviceTypeData);
	bool getWheelTaskEndActionDB(std::map<int, QString> &m_WheelTaskEndActionData);
	bool getWheelTaskEndTypeDB(std::map<int, QString> &m_WheelTaskEndTypeData);
	bool getWheelTaskTypeDB(std::map<int, QString> &m_WheelTaskTypeData);
	bool getWheelVoltageLevelDB(std::map<QString, QString> &m_WheelVoltageLevelData);
	bool getWheelFeverTypeDB(std::map<int, QString>& m_WheelFeverTypeData);
	bool getWheelRecognitionTypeDB(std::map<int, QString> &m_WheelRecognitionTypeData);
	bool getWheelMeterTypeDB(std::map<int, WheelRobortMeterTypeStruct> &m_WheelMeterTypeData);
	bool getWheelPointTreeDataDB(QList<WheelPointTreeDataStruct> &m_wheelPointTreeData);

	bool getWheelEquipmentTreeDataDB(QList<QStringList> &lstStrings);
	bool getWheelLevelDataDB(QList<QStringList> &lstStrings);
	bool getWheelEquipmentTypeDataDB(QList<QStringList> &lstStrings);
	bool getWheelEquipmentDataDB(QList<QStringList> &lstStrings, QString strTypeID);
	

	bool getWheelPointTreeDataDB(QList<WheelPointTreeDataStruct> &m_wheelPointTreeData, WheelPatrolParameter m_wheelPatrolPara);
	bool getWheelAlarmLevelDB(std::map<int, QString> &m_WheelAlarmLevelData);
	bool getWheelSaveTypeDB(std::map<int, QString> &m_WheelSaveTypeData);
	bool getWheelEquipmentIntervalDB(std::map<QString, WheelRobortEquipmentIntervalStruct> &m_WheelEquipmentIntervalData);
	bool getWheelDeviceAlarmSearchDB(int &m_count, int m_showCount, WheelPatrolParameter m_wheelPatrolPara, QList<DeviceAlarmSearchStruct> &m_deviceAlarmSearchStru);

	bool getWheelPatrolResultDB(int m_page, int m_showCount, QList<WheelPatrolResultStruct>& m_wheelPatrolResultStru, QString m_device_uuid, QString m_start_time, QString m_stop_time);
	bool getWheelPatrolResultCountDB(int & m_count, QString m_device_uuid, QString m_start_time, QString m_stop_time);
	bool getWheelPatrolResultDB(QList<WheelPatrolResultStruct>& m_wheelPatrolResultStru, QString m_start_time, QString m_stop_time);


	bool getWheelPatrolResultCompareDB(int m_count, int m_showCount, QList<WheelPatrolResultCompareStruct>& m_wheelPatrolResultCompareStru, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time);
	bool getWheelPatrolResultCompareCountDB(int &m_count, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time);

	bool getTaskListShowDB(QList<WheelTaskListShowStruct> &m_wheelTaskListShowStru, WheelTaskListSearchIndex m_index, QList<WheelTaskTemplateList> &m_taskTemplateList);

	bool getWheelDeviceAlarmColorDataDB(WheelRobortDevicesAlarmColorStruct &m_devicesAlarmColorStru);

	bool getWheelIntervalShowDB(QList<WheelRobortIntervalShowStruct> &m_wheelRobortIntervalShowStru);
	bool getWheelDeviceFromIntervalShowDB(QString m_equipment_interval_uuid, QList<WheelRobortDeviceFromIntervalStruct> &m_wrDeviceFromIntervalStru);

	bool getDeviceUUidForCheckBoxDB(QString m_choose, QString checkBoxTypeUUid, QList<QString> &m_devices);

	bool getWheelDeviceAlarmSearchVerifyDB(int m_count, int m_showCount, QList<WheelRobortTaskSearchStruct> &m_wheelRobortTaskSearchStru);
	bool getWheelDeviceAlarmSearchVerifyCountDB(int &m_count);

	bool getWheelDeviceAlarmForTaskUUidDB(int m_count, int m_showCount, QString m_task_uuid, QString m_start_time, QString m_stop_time, QList<DeviceAlarmSearchStruct>& m_deviceAlarmSearchStru);
	bool getWheelDeviceAlarmForTaskUUidCountDB(int & m_count, QString m_task_uuid, QString m_start_time, QString m_stop_time);

	bool getWheelAlarmDataForDeviceTaskUUidDB(QString task_uuid, QString device_uuid, DeviceAlarmSearchStruct &stru);

	bool getWheelTaskDataForDeviceUUidDB(QString deviceUUid, QList<WheelTaskShow> &taskData);

	bool insertTaskDB(WheelTaskStruct m_wheelTaskStruct);
    bool insertTaskDB(WheelTaskStruct m_wheelTaskStruct, QString &errMsg);
	bool insertImmediateTaskDB(WheelTaskStruct m_wheelTaskStruct, QString &retMsg);
	bool updataTaskDB(WheelTaskStruct m_wheelTaskStruct);
    bool updateTaskDB(QString taskUUid, QStringList taskQList);
    bool updateTaskDB(QString task_uuid, QString start_time);
    bool updateTaskEndStatus(WheelTaskStruct wheelTaskStruct);
	bool updataTaskForAuditIdDB(QString taskUUid);
    bool updataTaskDBWhenCollectFinished(QString task_uuid);
	bool deleteTaskDB(QString m_task_uuid, QString &retMsg);
    bool getTaskNameByTaskUuid(QString task_uuid, QString &task_name);
    bool getTaskPropertyByTaskUuid(QString task_uuid, QString &task_property);

	bool insertTaskTemplateDB(WheelTaskTemplateStruct m_wheelTaskTemplateStruct,QString &retMsg);
	bool deleteTaskTemplateDB(QString m_task_template_uuid, QString &retMsg);
    bool updateTaskTemplateDB(QString task_template_uuid, int type, QString &retMsg);
    bool getWheelTaskTemplateAllTimedTasksSortByStartTimeAfterCurrent(QList<WheelTaskTemplateStruct> &tasks);

	bool insertTaskEditDB(WheelTaskEditStruct m_wheelTaskEditStruct, QString &retMsg);
	bool deleteTaskEditDB(QString m_task_edit_uuid, QString &retMsg);
	bool updataTaskEditDB(WheelTaskEditStruct m_wheelTaskEditStruct, QString &retMsg);
	bool importTaskEditDB(QString c_task_edit_uuid_old, WheelTaskEditStruct stru, QString &Msg);

	bool getTaskEditListDB(int m_count, int m_showCount, WheelTaskAdminType m_task_edit_type_id, QList<WheelTaskEditStruct>& m_wheelTaskEditStruct);
	bool getTaskEditPageDB(int &m_count, WheelTaskAdminType m_task_edit_type_id);
	bool getTaskEditDB(WheelTaskEditStruct &m_taskEditStruct, QString m_edit_uuid);

	bool insertTaskDevicesDB(QString m_task_edit_uuid, QList<QString> m_device_uuid, QString &rectMsg);
	bool deleteTaskDevicesDB(QString m_task_edit_uuid, QString &rectMsg);
	bool updataTaskDevicesDB(QString m_task_edit_uuid, QList<QString> m_add_device_uuid, QList<QString> m_delete_device_uuid);
	bool getTaskDeviceDataDB(QString m_task_edit_uuid, QList<QString> &m_device_uuid);

	bool insertTaskEditAndInsertTaskDevicesDB(WheelTaskEditStruct edit, QList<QString> m_device_uuid, QString &retMsg);
	bool updataTaskEditAndUpdataTaskDevicesDB(WheelTaskEditStruct edit, QList<QString> m_device_uuid, QString &retMsg);

	bool insertInspectResultDB(WheelInspectResultStruct m_wheelInspectResultStruct);
	bool updataInspectResultForDealedDB(WheelRobotPartrolReaultVerify m_wheelInspectResultStruct, QString &retMsg);

	bool updataInspectResultAndInsertDealInfoDB(WheelPartolResult resultVerify, QString & rectMsg);
	bool updataInspectResultAndUpdataDealInfoDB(WheelPartolResult resultVerify, QString & rectMsg);

// 	bool insertThresholdDB(WheelThresholdStruct m_wheelThresholdStruct);
// 	bool updataThresholdDB(WheelThresholdStruct m_wheelThresholdStruct);
    bool insertThresholdByMeterType(QString normalString, QString warningString, QString commonString, QString serialString, QString dangerString, WheelRobotMeterType type, QString threshold_uuid);
    bool insertThresholdByDeviceUuid(QString normalString, QString warningString, QString commonString, QString serialString, QString dangerString, QVector<QString> device_uuid, QString threshold_uuid);
	bool deleteThresholdDB(QString m_threshold_id);
    bool getThresholdeByMeterType(QString &normalString, QString &warningString, QString &commonString, QString &serialString, QString &dangerString, WheelRobotMeterType type, QString &threshold_uuid);
    bool getThresholdeByDeviceUuid(QString &normalString, QString &warningString, QString &commonString, QString &serialString, QString &dangerString, QString device_uuid, QString &threshold_uuid);

	bool insertDealInfoDB(WheelRobortDealInfoStruct m_dealInfoStru, QString &rectMsg);
	bool updataDealInfoDB(WheelRobortDealInfoStruct m_dealInfoStru, QString &rectMsg);
	bool updataDealInfoForUserDB(QString deal_task_uuid, QString deal_time, QString deal_info, QString & rectMsg);

	bool insertDeviceParameterDB(WheelRobortDeviceParameterStruct m_deviceParamterStru);
	bool deleteDeviceParameterDB(QString m_device_uuid);
	bool updataDeviceParameterDB(WheelRobortDeviceParameterStruct m_deviceParamterStru);

	bool insertVoltageLevelDB(QString m_voltage_level_name);
    bool insertVoltageLevelDB(QString m_voltage_level_name, QString &errMsg);
    bool deleteVoltageLevelDB(QString m_voltage_level_id, QString &errMsg);

 	bool insertDeviceAreaDB(WheelRobortDeviceAreaStruct m_device_area);
    bool insertDeviceAreaDB(WheelRobortDeviceAreaStruct m_device_area, QString &errMsg);
	bool deleteDeviceAreaDB(QString m_device_area_uuid);
    bool deleteDeviceAreaDB(QString m_device_area_uuid, QString &errMsg);

 	bool insertEquipmentIntervalDB(WheelRobortEquipmentIntervalStruct m_equipmentIntervalStru);
    bool insertEquipmentIntervalDB(WheelRobortEquipmentIntervalStruct m_equipmentIntervalStru, QString &errMsg);
	bool deleteEquipmentIntervalDB(QString m_equipment_interval_uuid);
    bool deleteEquipmentIntervalDB(QString m_equipment_interval_uuid, QString &errMsg);
	bool updateEquipmentIntervalDB(WheelRobortEquipmentIntervalStruct equipmentIntervalStru);


	bool getWheelRobortAlarmSearchDB(int m_count, int m_showCount, QList<WheelRobortAlarmSearchStruct>& m_alarmSearchStru, QString m_start_time, QString m_stop_time);

	bool getWheelRobortAlarmSearchCountDB(int & m_count, QString m_start_time, QString m_stop_time);

    bool getUserRole(QString username, QString password, userLoginRetVal &ret);

	bool getNodeAlarmStatusFromDevicesDB(QString device_uuid, QList<WheelRobortAlarmPathStruct> &c_alarmPath);

	bool getDeviceNodePathFromDevicesDB(QString m_device_uuid, QStringList &c_devicePath);

    bool getDevicesByTaskUuid(QStringList &devs, QString task_uuid);

	//标准巡检点位库维护
	bool getWheelStandardPatrolVindicateDB(QStringList m_device_type_uuid, QList<WheelStandardPatrolVindicateStruct> &m_standardPatrolVindicateStru);
	bool updataWheelStandardPatrolVindicateDB(WheelDevicePointNameStruct m_devicePointNameStru);
	bool deleteWheelStandardPatrolVindicateDB(QString m_device_point_type_uuid);
	bool getWheelStandardForPointUUidDB(WheelStandardPatrolVindicateStruct &c_data, QString device_point_uuid);

	bool getWheelDeviceParameterDB(QString m_device_uuid, WheelRobortDeviceParameterStruct &m_deviceParameter);

	bool getWheelDeviceParameterToTextDB(QList<QStringList> &m_devP, QStringList &errorMsg);
	//报表
	bool getWheelReportHeadMessageDB(QString m_task_uuid, WheelReportHeadMessage &m_ReportHeadMessage);
	bool getWheelAlarmPointDB(QString m_TaskUUid, QList<AlarmUnusualNormalPoint>& m_AlarmPoint);
	bool getUnusualPointDB(QString m_TaskUUid, QList<AlarmUnusualNormalPoint>& m_AlarmPoint);
	bool getWheelNormalPointDB(QString m_TaskUUid, QList<AlarmUnusualNormalPoint>& m_AlarmPoint);

	//任务下选择的设备
	bool getEditDevicesForEditUUidDB(QMap<QString, QString>& m_deviceMap, QString m_edit_uuid);

	bool getEditDevicesForEditUUidListDB(QStringList & m_deviceMap, QString m_edit_uuid);

	bool getTaskEditTypeIdDB(WheelTaskAdminType &m_taskAdminType, QString m_edit_uuid);

	bool getWheelTaskStatusDB(std::map<int, QString> &m_WheelTaskStatusData);

	bool getCalendarTaskMapDB(QString m_start_time, QString m_end_time, QMap<int, QList<WheelCalendarData>> &taskMap);

	bool getWheelAllDeviceUUidDB(QStringList &m_dev);

	bool getWheelTaskListShowDB(QString m_start_time, QString m_end_time, QList<WheelTaskShow> &taskList);
	bool getWheelTaskStruShowDB(WheelTaskShow &taskStru,QString task_uuid);

	bool getDevicePointTypeUUidFromDevices(QString m_device_uuid,QString &device_point_type_uuid);

	bool getTaskEditStruForTaskTemplateUUidDB(QString m_task_edit_uuid, WheelTaskEditStruct &taskEdit);

    bool getThresholdFileName(QString device_uuid, WheelRobotMeterType meter_type_id, QString &threshold_filename);
	bool getThresholdFileNameForVirtual(QString virtual_uuid, QString &threshold_filename);

	bool getWheelUnusualPointSearchListDB(int m_count, int m_pageCount, QList<DeviceAlarmSearchStruct> &m_UnusualStru, QStringList status);
	bool getWheelUnusualPointSearchCountDB(int &m_count, QStringList status);

	bool isAuditFinishDB(QString task_uuid);

	bool insertDeviceType(QString c_uuid, QString c_name);
	bool insertSubDeviceType(QString c_uuid, QString c_typeuuid, QString c_name);
	bool insertDevicePointName(QStringList c_data, QString &retMsg);
	bool insertRecognitionType(QString recognitionTypeName);
	bool insertMeterType(QString meterTypeName);
	bool insertFeverType(QString feverTypeName);
	bool insertSaveType(QString saveTypeName);
	bool insertVoltageLevel(QString c_uuid, QString c_name);
	bool insertEquipmentInterval(QString c_uuid, QString c_typeuuid, QString c_name);
	bool insertDeviceArea(QString c_uuid, QString c_name);

	bool updataSubDeviceType(QString subDeviceTypeUUid, QString deviceTypeUUid);
	bool updataDevicePointName(QStringList c_data, QString &retMsg);

	bool deleteDeviceWithDevicePointUUid(QString uuid, QString &Msg);

	bool getWheelTaskHistoryStruDB(QString m_start_time, QString m_end_time, QString taskName, QList<WheelTaskEditStruct> &stru);

	bool getCreateExcelAlarmSearchDB(int icho, int m_page, int m_pageCount, int &m_count, QList<QStringList> &c_data, WheelPatrolParameter m_wheelPatrolPara);

	bool getDeviceListForPointId(QString pointId, QStringList &deviceList);
	bool getDeviceListForPointId(QString pointId, QList<wheelDeviceDetailMsg> &data);

	bool getPointIdForEditTaskUUid(std::vector<int> &data, QString task_uuid);

	bool getWheelUserConfigDataDB(QList<WheelUserConfig> &conData, WheelUserType nowType);
	bool getWheelAllUserConfigDataDB(QList<WheelUserConfig> &conData);

	bool addUserConfigDB(WheelUserConfig data, QString &Msg);
	bool deleteUserConfigDB(QString user_uuid, QString &Msg);

	bool searchDeviceUUidWithDeviceAreaDB(std::map<QString, QStringList> &data);

	bool getCreateReportDB(QList<WheelCreateReport> &data, QString start_time, QString end_time);

	bool getPatrolPointSetDataDB(int m_page, int m_pageCount, QList<WheelPatrolPointSet> &data, WheelPatrolParameter m_wheelPatrolPara);
	bool getPatrolPointSetDataCountDB(int &m_count, WheelPatrolParameter m_wheelPatrolPara);

	bool updataDeviceStartUsingDB(QStringList device_uuid, int start_using, QString &retMsg);

	bool insertPatrolPointForDevices(QStringList insertData, QString &retMsg);
	bool updatePatrolPointForDevices(QStringList updateData, QString &retMsg);

	bool getAlarmCountSearchDB(QString device_uuid, int &count);

	bool getDeviceTypeNameForDeviceUUidDB(QString device_uuid, QString &device_Type);

	bool getDeviceSnForDeviceUUidDB(QString device_uuid, QString &device_sn, QString &device_sn_name);
	bool getDeviceUuidForDeviceSn(QString device_ferver_type, QList<QString> &device_uuid_list);
	bool getDeviceSnNameForDeviceSn(QString device_sn, QString& device_sn_name);

	bool updateTaskStatusDB(QString task_uuid, int task_status, QString &retMsg);

	bool getMeterEnumWithDeviceUUidDB(QString device_uuid, WheelRobotMeterType &meterId);

//	bool getAlarmMessageSubscribeDB(QList<WheelNoteMessage> &data, WheelAlarmMessageIf msg);
	bool getAlarmMessageSubscribeDB(int m_count, int m_pageCount, QList<WheelNoteMessage> &data, WheelAlarmMessageIf msg);
	bool getAlarmMessageSubscribeCountDB(int &m_count, WheelAlarmMessageIf msg);

	bool getSystemMessageSubscribeDB(int m_count, int m_pageCount, QList<WheelNoteMessage> &data, WheelAlarmMessageIf msg);
	bool getSystemMessageSubscribeCountDB(int &m_count, WheelAlarmMessageIf msg);

	bool getDeviceUUidWithType(QString fault_name_uuid, QStringList &m_dev, WheelDeviceTreePath type);

	bool insertNoteMessageDB(QString noteUUid, WheelSubMsgInsert noteMsg, QStringList fault_uuid, QString &retMsg);
	bool deleteNoteMessageDB(QString noteUUid, QString fault_name_uuid, QString &retMsg);

    bool getUserPhoneNumberByDeviceNumber(QString &userPhoneNumber, QString device_uuid);

	bool getSystemNameAndUUidDB(QList<WeelSystemAlarm> &data);
	///////////////////////////////////////////
	//三项温差设备查询
	bool getWheelVirtualDeviceThreeCompareMap(QMap<QString, DeviceVirtualSort> &deviceMap);
	//泄露电流表动作次数设备查询
	bool getWheelVirtualDeviceRevealAmpereMap(QMap<QString, DeviceVirtualSort> &deviceMap);

	bool updateDeviceVirtualAndInsertVirtual(QString virtualUUid, QList<QString> updateDevice, QString subDeviceUUid);
	//获取虚拟设备字段
	bool getVirtualRelevanceDeviceUUid(QString taskUUid, QString deviceUUid, QString &releDev, QString &virtualUUid);
	bool getInspectResultValueForDeviceUUid(QString taskUUid, QString devUUid, QString &resultValue);

	bool getVirtualDeviceUUid(QString deviceUUid, virtualDeviceConfig &cfg);
	bool dataClearForVirtual();
	//获取某个任务下的设备名
	bool getDeviceNameForTaskUUid(QString &taskUUid, QMap<QString, QString> &data);

	bool updateFastAuditTaskDB(QString taskUUid);

	bool getNewWheelCollectTreeDataDB(QList<QStringList> &m_collectTreeDat, QStringList selectUUid, RootNodeType selectType);

	bool selectDeviceDataWithIntervalUUid(QList<QStringList> &_deviceData, QString intervalUUid, QString originalVoltageLevelUUid);
	//开启事务管理模式
	bool insertDeviceWithCopyInterval(QList<QStringList> deviceData, QString newVoltageLevelUUid, QString equipment_interval_uuid);

	bool selectPointUUidAndDeviceTypeUUid(QList<QStringList> &deviceData, QStringList deviceTypeUUidList, QString voltageLevelUUid);

	bool deleteDeviceTypeForDevices(QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid);

	bool selectPointAllDataWithPointList(QList<QStringList> &deviceData, QStringList _data, QString voltageLevelUUid);

	bool getPointId(QString strDeviceUUid, QString &strPointId);

    bool getThresholdValue(QString deviceUUid, QStringList &jsonValue);

    bool getBJDeviceUUidWithEquipment(QStringList &devList, QString eqUUid);

    bool getBJDeviceUUidWithDevicetype(QStringList &devList, QString tyUUid);

    bool getBJDeviceModelData(QList<BJDeviceModel> &_data);

    bool getBJTaskData(QString taskUUid, BJRobotTaskStatusData &data);

    bool getBJDeviceResultFromDB(QString taskUUid, QString deviceUUid, BJDeviceResultSendData &data);

    bool getBJDeviceUUidWithDeviceId(QStringList &devList);

    bool getDeviceNameWithDeviceUUid(QString deviceUUid, QString &stdDeviceName);

    bool getDeviceUUidWhihTaskUUid(QString taskUUid, QStringList &deviceList);

    bool getCheTaskDeviceNameResult(QString taskUUid, QList<QStringList> &data);
private:
    QSqlDatabase m_myDb;
    boost::mutex m_lock;
};                    

#define WHEEL_ROBOT_DB LibDLWheelRobotDBOperation::GetSingleton()

#endif