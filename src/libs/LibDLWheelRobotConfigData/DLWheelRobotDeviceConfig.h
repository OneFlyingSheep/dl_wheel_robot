#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <QList>

// #if _MSC_VER >= 1600  
// #pragma execution_character_set("utf-8")  
// #endif 

class DLWheelRobotDeviceConfig : public Singleton<DLWheelRobotDeviceConfig>
{
public:
	DLWheelRobotDeviceConfig();
	~DLWheelRobotDeviceConfig();

    void refreshData();

public:
	//设备sn列表
	void loadDeviceSnData();
	QString getDeviceSnNameQString(QString device_sn);

	//设备区域
	void loadWheelDeviceAreaData();
	std::map<QString, QString> getWheelDeviceAreaDataMap();
	QList<QString> getWheelDeviceAreaNameQList();
	QString getWheelDeviceAreaNameQString(QString m_device_area_uuid);
	QString getWheelDeviceAreaUUidQString(QString m_device_area_name);
	
	//点位名称
	void loadWheelDevicePointNameData();
	std::map<QString, WheelDevicePointNameStruct> getWheelDevicePointNameDataMap();
	QList<QString> getWheelDevicePointNameQList();
	QString getWheelSubDeviceTypeUUidFromPointName(QString m_device_point_type_uuid);
	QString getWheelDevicePointNameQString(QString m_device_point_type_uuid);
	QString getWheelDevicePointTypeUUidQString(QString m_device_point_type_name);
	std::map<QString, WheelDevicePointNameStruct> getWheelDevicePointNameFromSubDeviceType(QString m_sub_device_uuid);

	//小设备类型
	void loadWheelSubDeviceTypeData();
	std::map<QString, WheelSubDeviceTypeStruct> getWheelSubDeviceTypeDataMap();
	QList<QString> getWheelSubDeviceNameQList();
	QString getWheelDeviceTypeUUidFromSubDevice(QString m_sub_device_type_uuid);
	QString getWheelSubDeviceNameQString(QString m_sub_device_type_uuid);
	QString getWheelSubDeviceTypeUUidQString(QString m_sub_device_name);
	std::map<QString, QString> getWheelSubDeviceNameFromDeviceType(QString m_device_type_uuid);

	//设备类型
	void loadWheelDeviceTypeData();
	std::map<QString, QString> getWheelDeviceTypeDataMap();
	QList<QString> getWheelDeviceTypeNameQList();
	QString getWheelDeviceTypeNameQString(QString m_device_type_uuid);
	QString getWheelDeviceTypeUUidQString(QString m_device_type_name);

	//电压等级
	void loadWheelVoltageLevelData();
	std::map<QString, QString> getWheelVoltageLevelDataMap();
	QList<QString> getWheelVoltageLevelNameQList();
	QString getWheelVoltageLevelNameQString(QString m_voltage_level_id);
	QString getWheelVoltageLevelIdInt(QString m_voltage_level_name);

	//发热类型
	void loadWheelFeverTypeData();
	std::map<int, QString> getWheelFeverTypeDataMap();
	QList<QString> getWheelFeverTypeNameQList();
	QString getWheelFeverTypeNameQString(int m_fever_type_id);
	int getWheelFeverTypeIdInt(QString m_fever_type_name);

	//仪表类型
	void loadWheelMeterTypeData();
	std::map<int, WheelRobortMeterTypeStruct> getWheelMeterTypeDataMap();
	QList<QString> getWheelMeterTypeNameQList();
	QString getWheelMeterTypeNameQString(int m_meter_type_id);
	int getWheelMeterTypeIdInt(QString m_meter_type_name);
	QString getWheelThresholdUUidFromMeterType(int m_meter_type_id);

	//识别类型
	void loadWheelRecognitionTypeData();
	std::map<int, QString> getWheelRecognitionTypeDataMap();
	QList<QString> getWheelRecognitionTypeQList();
	QString getWheelRecognitionTypeNameQString(int recognition_type_id);
	int getWheelRecognitionTypeIdInt(QString recognition_type_name);

	//告警等级
	void loadWheelAlarmLevelData();
	std::map<int, QString> getWheelAlarmLevelDataMap();
	QList<QString> getWheelAlarmLevelDataQList();
	int getWheelAlarmLevelId(QString alarm_level_name);

	//保存类型
	void loadWheelSaveTypeData();
	std::map<int, QString> getWheelSavaTypeDataMap();
	QList<QString> getWheelSavaTypeDataQList();
	QString getWheelSaveTypeNameQString(int save_type_id);
	int getWheelSaveTypeIdInt(QString save_type_name);

	//间隔名
	void loadWheelEquipmentIntervalData();
	std::map<QString, WheelRobortEquipmentIntervalStruct> getWheelRobortEquipmentIntervalDataMap();
    std::map<QString, QString> getWheelRobortEquipmentIntervalFromVoltageLevelId(QString m_voltage_level_id);
	QString getWheelEquipmentIntervalQString(QString equipment_interval_name);

	//标准巡检点位库维护界面
	QList<WheelStandardPatrolVindicateStruct> getWheelStandardPatrolVindicate(QStringList m_device_type_uuid);
	WheelDevicePointNameStruct getPointUUidForStandardPatrolVin(WheelStandardPatrolVindicateStruct m_standPatrol);

	//设备参数2_WheelRobortDeviceParameterStruct
	WheelRobortDeviceParameterStruct getWheelDeviceParameterData(QString m_device_uuid);

	//DeviceParameter数据保存到txt
	void getWheelDeviceParameterDataToText();

	//获取全部设备uuid
	QStringList getWheelAllDeviceUUid();

	//根据点位uuid判断拍照类型
	WheelJudgeTakePhoto getWheelChooseRecognitionBool(QString m_device_point_type_uuid);

	//根据device_uuid判断拍照类型
	WheelJudgeTakePhoto getWheelChooseRecForDeviceUUidBool(QString m_device_uuid);

	QString dealNameLetter(QString name, int &phase_id);
	QString getUUid();

	//巡检点位设置
	QList<WheelPatrolPointSet> getPatrolPointSetData(int m_page, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);
	void getPatrolPointSetDataCount(int & m_page, int & m_count, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);

	QString getDeviceTypeNameForDeviceUUid(QString device_uuid);

	WheelRobotMeterType getMeterEnumWithDeviceUUid(QString device_uuid);

	//设备-查询消息订阅数据
	QList<WheelNoteMessage> getAlarmMessageSubscribe(int m_page, int m_pageCount, WheelAlarmMessageIf msg);
	void getAlarmMessageSubscribeCount(int & m_page, int & m_count, int m_pageCount, WheelAlarmMessageIf msg);

	//系统-查询消息订阅数据
	QList<WheelNoteMessage> getSystemMessageSubscribe(int m_page, int m_pageCount, WheelAlarmMessageIf msg);
	void getSystemMessageSubscribeCount(int & m_page, int & m_count, int m_pageCount, WheelAlarmMessageIf msg);

	//获取系统告警名称和uuid
	QList<WeelSystemAlarm> getSystemNameAndUUid();

private:
	int m_noteMsgCountF;
	int m_noteMsgCountS;

	std::map<QString, QString> m_WheelDeviceSnData;

	std::map<QString, QString> m_WheelDeviceAreaData;
	QList<QString> m_WheelDeviceAreaQList;

	std::map<QString, WheelDevicePointNameStruct> m_WheelDevicePointNameData;
	QList<QString> m_WheelDevicePointNameQList;

	std::map<QString, WheelSubDeviceTypeStruct> m_WheelSubDeviceTypeData;
	QList<QString> m_WheelSubDeviceNameQList;

	std::map<QString, QString> m_WheelDeviceTypeData;
	QList<QString> m_WheelDeviceTypeNameQList;

	std::map<QString, QString> m_WheelVoltageLevelData;
	QList<QString> m_WheelVoltageLevelNameQList;

	std::map<int, QString> m_WheelFeverTypeData;
	QList<QString> m_WheelFeverTypeNameQList;

	std::map<int, WheelRobortMeterTypeStruct> m_WheelMeterTypeData;
	QList<QString> m_WheelMeterTypeNameQList;

	std::map<int, QString> m_WheelRecognitionTypeData;
	QList<QString> m_WheelRecognitionTypeQList;

	std::map<int, QString> m_WheelAlarmLevelData;
	QList<QString> m_WheelAlarmLevelQList;

	std::map<int, QString> m_WheelSaveTypeData;
	QList<QString> m_WheelSaveTypeQList;

	std::map<QString, WheelRobortEquipmentIntervalStruct> m_WheelEquipmentIntervalData;
};

#define WHEEL_DEVICE_CONFIG DLWheelRobotDeviceConfig::GetSingleton()

