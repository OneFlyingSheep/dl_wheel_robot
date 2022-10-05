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
	//�豸sn�б�
	void loadDeviceSnData();
	QString getDeviceSnNameQString(QString device_sn);

	//�豸����
	void loadWheelDeviceAreaData();
	std::map<QString, QString> getWheelDeviceAreaDataMap();
	QList<QString> getWheelDeviceAreaNameQList();
	QString getWheelDeviceAreaNameQString(QString m_device_area_uuid);
	QString getWheelDeviceAreaUUidQString(QString m_device_area_name);
	
	//��λ����
	void loadWheelDevicePointNameData();
	std::map<QString, WheelDevicePointNameStruct> getWheelDevicePointNameDataMap();
	QList<QString> getWheelDevicePointNameQList();
	QString getWheelSubDeviceTypeUUidFromPointName(QString m_device_point_type_uuid);
	QString getWheelDevicePointNameQString(QString m_device_point_type_uuid);
	QString getWheelDevicePointTypeUUidQString(QString m_device_point_type_name);
	std::map<QString, WheelDevicePointNameStruct> getWheelDevicePointNameFromSubDeviceType(QString m_sub_device_uuid);

	//С�豸����
	void loadWheelSubDeviceTypeData();
	std::map<QString, WheelSubDeviceTypeStruct> getWheelSubDeviceTypeDataMap();
	QList<QString> getWheelSubDeviceNameQList();
	QString getWheelDeviceTypeUUidFromSubDevice(QString m_sub_device_type_uuid);
	QString getWheelSubDeviceNameQString(QString m_sub_device_type_uuid);
	QString getWheelSubDeviceTypeUUidQString(QString m_sub_device_name);
	std::map<QString, QString> getWheelSubDeviceNameFromDeviceType(QString m_device_type_uuid);

	//�豸����
	void loadWheelDeviceTypeData();
	std::map<QString, QString> getWheelDeviceTypeDataMap();
	QList<QString> getWheelDeviceTypeNameQList();
	QString getWheelDeviceTypeNameQString(QString m_device_type_uuid);
	QString getWheelDeviceTypeUUidQString(QString m_device_type_name);

	//��ѹ�ȼ�
	void loadWheelVoltageLevelData();
	std::map<QString, QString> getWheelVoltageLevelDataMap();
	QList<QString> getWheelVoltageLevelNameQList();
	QString getWheelVoltageLevelNameQString(QString m_voltage_level_id);
	QString getWheelVoltageLevelIdInt(QString m_voltage_level_name);

	//��������
	void loadWheelFeverTypeData();
	std::map<int, QString> getWheelFeverTypeDataMap();
	QList<QString> getWheelFeverTypeNameQList();
	QString getWheelFeverTypeNameQString(int m_fever_type_id);
	int getWheelFeverTypeIdInt(QString m_fever_type_name);

	//�Ǳ�����
	void loadWheelMeterTypeData();
	std::map<int, WheelRobortMeterTypeStruct> getWheelMeterTypeDataMap();
	QList<QString> getWheelMeterTypeNameQList();
	QString getWheelMeterTypeNameQString(int m_meter_type_id);
	int getWheelMeterTypeIdInt(QString m_meter_type_name);
	QString getWheelThresholdUUidFromMeterType(int m_meter_type_id);

	//ʶ������
	void loadWheelRecognitionTypeData();
	std::map<int, QString> getWheelRecognitionTypeDataMap();
	QList<QString> getWheelRecognitionTypeQList();
	QString getWheelRecognitionTypeNameQString(int recognition_type_id);
	int getWheelRecognitionTypeIdInt(QString recognition_type_name);

	//�澯�ȼ�
	void loadWheelAlarmLevelData();
	std::map<int, QString> getWheelAlarmLevelDataMap();
	QList<QString> getWheelAlarmLevelDataQList();
	int getWheelAlarmLevelId(QString alarm_level_name);

	//��������
	void loadWheelSaveTypeData();
	std::map<int, QString> getWheelSavaTypeDataMap();
	QList<QString> getWheelSavaTypeDataQList();
	QString getWheelSaveTypeNameQString(int save_type_id);
	int getWheelSaveTypeIdInt(QString save_type_name);

	//�����
	void loadWheelEquipmentIntervalData();
	std::map<QString, WheelRobortEquipmentIntervalStruct> getWheelRobortEquipmentIntervalDataMap();
    std::map<QString, QString> getWheelRobortEquipmentIntervalFromVoltageLevelId(QString m_voltage_level_id);
	QString getWheelEquipmentIntervalQString(QString equipment_interval_name);

	//��׼Ѳ���λ��ά������
	QList<WheelStandardPatrolVindicateStruct> getWheelStandardPatrolVindicate(QStringList m_device_type_uuid);
	WheelDevicePointNameStruct getPointUUidForStandardPatrolVin(WheelStandardPatrolVindicateStruct m_standPatrol);

	//�豸����2_WheelRobortDeviceParameterStruct
	WheelRobortDeviceParameterStruct getWheelDeviceParameterData(QString m_device_uuid);

	//DeviceParameter���ݱ��浽txt
	void getWheelDeviceParameterDataToText();

	//��ȡȫ���豸uuid
	QStringList getWheelAllDeviceUUid();

	//���ݵ�λuuid�ж���������
	WheelJudgeTakePhoto getWheelChooseRecognitionBool(QString m_device_point_type_uuid);

	//����device_uuid�ж���������
	WheelJudgeTakePhoto getWheelChooseRecForDeviceUUidBool(QString m_device_uuid);

	QString dealNameLetter(QString name, int &phase_id);
	QString getUUid();

	//Ѳ���λ����
	QList<WheelPatrolPointSet> getPatrolPointSetData(int m_page, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);
	void getPatrolPointSetDataCount(int & m_page, int & m_count, int m_pageCount, WheelPatrolParameter m_wheelPatrolPara);

	QString getDeviceTypeNameForDeviceUUid(QString device_uuid);

	WheelRobotMeterType getMeterEnumWithDeviceUUid(QString device_uuid);

	//�豸-��ѯ��Ϣ��������
	QList<WheelNoteMessage> getAlarmMessageSubscribe(int m_page, int m_pageCount, WheelAlarmMessageIf msg);
	void getAlarmMessageSubscribeCount(int & m_page, int & m_count, int m_pageCount, WheelAlarmMessageIf msg);

	//ϵͳ-��ѯ��Ϣ��������
	QList<WheelNoteMessage> getSystemMessageSubscribe(int m_page, int m_pageCount, WheelAlarmMessageIf msg);
	void getSystemMessageSubscribeCount(int & m_page, int & m_count, int m_pageCount, WheelAlarmMessageIf msg);

	//��ȡϵͳ�澯���ƺ�uuid
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

