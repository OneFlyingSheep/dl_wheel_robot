#ifndef __DL_COMMON_DL_HANGRAIL_ROBOT_OPER_H__
#define __DL_COMMON_DL_HANGRAIL_ROBOT_OPER_H__

#include <map>
#include <QtSql/QSql>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlRecord>
#include <QtSql/QSqlTableModel>
#include "common/Singleton.hpp"
#include "common/DLHangRailRobotGlobalDef.hpp"
#include "LibCommonFileOperation/LibCommonFileOperation.h"
#include <boost/serialization/singleton.hpp>
#include <boost/thread/mutex.hpp>
#include <QStandardItemModel>
#include <QTableView>
#include <QTreeView>

class LibDLHangRailRobotDBOperation : public Singleton<LibDLHangRailRobotDBOperation>
{
public:
    LibDLHangRailRobotDBOperation();
    ~LibDLHangRailRobotDBOperation();

    //stationCfg
    bool getStationCfg(stationCfg &cfg);
    bool updateStationCfg(stationCfg cfg);

    //deviceCfg
    bool getAllDevicesDeviceSsId(QStringList &devicesLists);
    bool getAllDevices(QList<deviceConfigType> &devs);
    bool getAllDevicesSortByDeviceAreaName(QList<deviceConfigType> &devs);
    
    bool insertSingleDevice(deviceConfigType dev, QString &errMsg);
    bool insertMultiDevices(QList<deviceConfigType> devs);

    bool updateSingleDeviceBySsid(deviceConfigType dev);
    bool updateMultiDevicesBySsid(QList<deviceConfigType> devs);

	bool deleteSingleDeviceBySsId(QString ssid, QString &errMsg);
    bool deleteMultiDevicesBySsId(QList<QString> ssids);

    bool getSingleDeviceBySsId(QString ssid, deviceConfigType &dev);
    bool getMultiDevicesBySsId(QStringList ssids, QList<deviceConfigType> &devs);

	bool getMultiRobotTaskDeviceSsid(QStringList ssids, std::vector<hangRailRobotTaskDevice> &devs);

	bool getDeviceAreaNames(QStringList &areaNames);

    bool getExistDeviceAreaNames(QStringList &areaNames);

    bool getMultiDevicesByAreaName(QString areaName, QList<deviceConfigType> &devs);
    bool getDeviceByAreaNameInnerJoinDeviceName(QString areaName, QList<QString> &devs);

    bool getDeviceNameByDeviceNameId(int ssid, QString &devName);

    // virdev
    bool getAllVirtualDeviceSsid(QStringList &virDevLists);
    bool getAllVirDevices(QList<virtualDeviceType> &devs);
    bool getAllVirDevicesSortByDeviceAreaName(QList<virtualDeviceType> &devs);
    
	bool insertSingleVirDevice(virtualDeviceType dev);
    bool insertMultiVirDevices(QList<virtualDeviceType> devs);

    bool updateSingleVirDeviceBySsid(virtualDeviceType dev);
    bool updateMultiVirDevicesBySsid(QList<virtualDeviceType> devs);

    bool deleteSingleVirDeviceBySsId(QString ssid);
    bool deleteMultiVirDevicesBySsId(QList<QString> ssids);

	bool getTaskCoreBackData(QString dev_ssid, hangRailRobotTaskDeviceCom &backCom);

	bool getSingleVirDeviceBySsId(QString ssid, virtualDeviceType &dev);
    bool getMultiVirDevicesBySsId(QStringList ssids, QList<virtualDeviceType> &devs);

    bool getExistVirDeviceAreaNames(QStringList &areaNames);

    bool getMultiVirDevicesByAreaName(QString areaName, QList<virtualDeviceType> &devs);
    bool getVirDeviceByAreaNameInnerJoinDeviceName(QString areaName, QList<QString> &devs);

    //
    bool getDeviceDetailType(std::map<int, deviceDetailType> &detail);
    bool getDeviceAreaType(std::map<int, deviceAreaType> &areas);
    bool getDevicePatrolType(std::map<int, devicePatrolType> &ptrls);
    bool getDeviceUnitType(std::map<int, deviceUnitType> &units);
    bool getDeviceNameType(std::map<int, deviceNameType> &devNames);
    bool getDeviceType(std::map<int, deviceType> &devType);
    bool getDeviceAlternateName(std::map<int, deviceAlternateNameType> &devType);

	bool getThresholdEnvironment(std::map<int, thresholdEnvi>& threEnvi);

	bool getThresholdEnvironmentID(std::map<int, thresholdEnvi>& m_thresholdEnviID);

	bool insertThresholdEnvironment(thresholdEnvi TEnvi);

	bool getThresholdPatrol(std::map<int, thresholdPatrol>& threPatrol);

	bool getThresholdPatrolID(std::map<int, thresholdPatrol>& m_thresholdPatrolID);

	bool insertThresholdPatrol(thresholdPatrol TPatrol);

	//inspectResult
    bool getResultByTaskAndDeviceId(QString taskId, QString deviceId, inspectResultType &inpectRes);

    //task
    bool getTaskByTaskSsid(QString taskId, QList<taskType> &tasks);
    bool insertSingleTask(taskType task);
    bool insertTaskAtBeginning(taskType task);
    bool updateSingleTask(taskType task);

	bool getEnvironmentTypeData(QList<QString>& m_enviTypeName);

	bool getStationCfgStationName(QList<QString>& m_stationName);

    //taskTemplate
    bool getAllTaskTemplate(QList<taskTemplateType> &tasks);
    bool getAllTaskTemplatesSortByTaskTemplateName(QList<taskTemplateType> &tasks);
    bool getTodayTimeTask(QList<taskTemplateType> &tasks);
    bool getTaskExecType(std::map<int, taskExecuteType> &types);

    //taskTemplate
    bool insertSingleTaskTemplate(taskTemplateType taskTemplate);
	bool insertSingleInpectResult(inspectResultType inspectResult);
    bool deleteSingleTaskBySsid(QString ssid);

	bool ReadDatabaseStorageFirstList(QString task_ssid, FirstListPortion & m_FiListData);
	bool ReadDatabaseStorageSecondList(QString task_ssid, QList<SecondListPortion> &m_SecondData);
	bool ReadDatabaseStorageForthList(QString task_ssid, QString & strRet);
	int ReadDatabaseStoFifth_AllSumList(QString task_ssid);
	int ReadDatabaseStoFifth_AllCountDevAreName(QString task_ssid,QString area_name);
	bool ReadDatabaseStoFifth_AllData(QList<FifthListPortion> &m_FifthData, QString m_task_ssid, QString m_device_area_name);
	bool ReadDatabaseStoFifth_DevAreName(QString task_ssid, QStringList &deviceAreaName);
	bool ReadDatabaseStorageSeventhList(QString task_ssid, SeventhListPortion &m_SeventhData);

    bool getUserRole(QString username, QString password, hangRobotUserLoginRetVal &ret);

	bool SearchTaskTableViewData(QStandardItemModel * SearchTaskDataModel);

	bool DeleteDatalistTaskTemplate(QString taskTemplateSsid);
    
	void TaskTableViewData(int row, int trow , QStandardItemModel *m_taskDataModel);
	void TaskSelectTableViewData(int row, int trow, QString m_StartTime, QString m_StopTime ,QString m_taskName, QStandardItemModel *m_taskSearchDataModel);
	int TaskDeviceTableViewDataCount(QString m_task_ssid);
	void TaskDeviceTableViewData(int row, int trow, QString m_task_ssid, QStandardItemModel *m_taskDoubleClickDataModel);
	int DevicesTableViewDataCount(bool b_Select, QString m_StartTime, QString m_StopTime, QString m_deviceName);
	void DevicesTableViewData(int row, int trow, QStandardItemModel *m_devicesDataModel);
	int TaskTableViewDataCount(bool b_Select, QString m_StartTime, QString m_StopTime, QString m_taskName);
	void DevicesSelectTableViewData(int row, int trow, QString m_StartTime, QString m_StopTime, QString m_deviceName, QStandardItemModel *m_devicesSearchDataModel);
	int DeviceClickTableViewDataCount(QString m_device_ssid);
	void DeviceClickTableViewData(int row, int trow, QString m_device_ssid, QStandardItemModel *m_devicesDoubleClickDataModel);
	int DeviceAlarmTableViewDataCount(bool b_Select = false, QString m_StartTime = "", QString m_StopTime = "", QString m_AreaName = "");
	int DeviceAlarmSearchAreaNameDataCount(QString m_AreaName);
	void DeviceAlarmTableViewData(int row, int trow, QStandardItemModel * DeviceAlarm_tableModel);
	void DeviceAlarmSearchTableViewData(int row, int trow, QString m_StartTime, QString m_StopTime, QString m_AreaName, QStandardItemModel * DeviceAlarmSearch_tableModel);
	void DeviceAlarmUpDateViewData(QString task_ssid,QString device_ssid,QString dealed_info);
    bool DeviceListAlarmUpDateViewData(QVector<updateAlarmInfoStruct> infoList, QString dealed_info, QString &errMsg);
	void TreeViewConfirmPatrolModel(QStandardItemModel * m_ConfirmPatrolModel);
	void TreeViewParticularPatrolModel(QStandardItemModel * m_ParticularPatrolModel);
	void getDeviceCurveData(int &x, int &y,QString device_ssid, QStandardItemModel * m_DeviceCurveModel);
	void ThresholdEnvironmentViewData(QStandardItemModel * m_ThresholdEnviDataModel);
	void ThresholdPatrolViewData(QStandardItemModel * m_ThresholdPatrolDataModel);
	bool insertAndUpdateMapData(QList<MapItemData> MapDataStruct, QString station_id);
	bool getMapDataList(QList<MapItemData>& m_MapDataList, QString station_id);
    bool getMapCornerItem(QList<MapItemData>& m_MapDataList, QString station_id);
	bool getDeviceAreaNameList(QList<QString>& m_DeviceAreaName);
	bool getDeviceAlarmSearchAreaNameData(int m_Page, int trow, QString m_AreaName, QStandardItemModel *DeviceAlarmAreaName_tableModel);
	bool DoubleClickIntPatrolTableViewData(QString m_taskSsid, QString m_deviceSsid, QStandardItemModel *m_doubleClickIntPatrolDataModel);
	bool insertEnvironmentResultData(EnvironmentResult m_enviResult);
	bool EnvironmentResultEnviViewData(QStandardItemModel * m_enviResultModel);
	bool deleteEnvironmentResult(QString DeletedateTime);
	bool EnvironmentResultEnviListData(QString m_enviKind,QList<EnvironmentResult> &m_enviData);
	bool getRestorationValueForDB(std::map<int, RestorationValue> &m_restorationValue);
	bool insertRestorationValue(RestorationValue m_restorationValue);
	bool getVirtualDeviceList(QList<RelevanceDevice>& m_virDeviceInitializeList);
	bool getVirtualDeviceNameMap(std::map<int, QString>& m_virDeviceNameMap);
	bool getSearchAreaDeviceList(QString m_areaName, QList<QString>& m_searchAreaDeviceList);
	bool upDateDevicesForRelative(RelevanceDevice m_releDevice);
	//.///////////////////////////////////////////////////.//
	bool openDb(QString hostName, QString dbName, QString userName, QString passwd, int port = 3306);
	//.///////////////////////////////////////////////////.//
	bool getTaskHistoryDataFromDB(TaskSearchCondition condi, QList<TaskShowItemInfo> &data);
	bool getDeviceInspectResultData(QString task_ssid, QList<DeviceInspectResult>& data, TaskSearchCondition condi);
	bool getTreeConfirmPatrolForPadDB(QMap<QString, QList<TreeTaskIsuData>> &map);
	bool getTreeParticularPatrolForPadDB(QMap<QString, QList<TreeTaskIsuData>> &map);
	bool getDeviceHistoryInspectResultDataDB(TaskSearchCondition condi, QStringList deviceAreaName, QStringList deviceTypeId, QList<DeviceRecordInfo>& data);
	bool getHistoryDeviceByDeviceSsidDB(TaskSearchCondition condi, QString device_ssid, DeviceHistoryRecord &data);
	bool getHistoryDeviceAlarmDataDB(TaskSearchCondition condi, QList<DeviceAlarmHistory>& data);
	bool getAlarmDeviceCountDB(int &iCount);
	//.///////////////////////////////////////////////////.//
	bool getReturnInsertDeviceDB(QString deviceSsid, InsertDeviceReturn &rdev);
	bool updateWalkThresholdToStationCfg(int start_value, int terminus_value);
	//.///////////////////////////////////////////////////.//
	bool updateDeviceNameIdForDevices(QList<updateDeviceNameId> data);
	bool selectDeviceNameIdWithDeviceSSid(QString deviceSSid, QString &deviceNameId);
private:
    void closeDb();
    bool execSqlString(QSqlQuery &qry);
    bool querySqlString(QString sqlString, QSqlQuery &qry);

private:
    QSqlDatabase m_myDb;
    boost::mutex m_lock;
};

#define ROBOTDB_DB LibDLHangRailRobotDBOperation::GetSingleton()

#endif