#define COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, str, val) str.val = rec.value(rec.indexOf(#val)).toInt()
#define COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, str, val) str.val = rec.value(rec.indexOf(#val)).toString()

#define COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, str, val) rec.setValue(#val, QString::number(str.val))
#define COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, str, val) rec.setValue(#val, str.val)

#include "LibDLWheelRobotDBOperation.h"
#include <QtSql/QSqlError>
#include <QDebug>
#include <QSettings>
#include <QApplication>
#include "LibDLHangRailCommonTools/DLHangRailCommonTools.h"
#include <QRegExp>
#include <QMap>
#include <QUuid>
LibDLWheelRobotDBOperation::LibDLWheelRobotDBOperation()
{
    //DBInfoData_T dbInfoData = DLHangRailCommonTools::getDBInfoData();

    //openDb(dbInfoData.hostName, dbInfoData.dbName, dbInfoData.userName, dbInfoData.password);
    //openDb("192.168.192.234","dlwheelrobotdb" , "root","q1w2e3r4" );
	//openDb("localhost", "dlwheelrobotdb", "root", "q1w2e3r4");
}

LibDLWheelRobotDBOperation::~LibDLWheelRobotDBOperation()
{
    closeDb();
}

bool LibDLWheelRobotDBOperation::openDb(QString hostName, QString dbName, QString userName, QString passwd, int port)
{
//	dbName = "dlwheelrobotdb";
    m_myDb = QSqlDatabase::addDatabase("QMYSQL");
    m_myDb.setHostName(hostName);
    m_myDb.setPort(port);
    m_myDb.setDatabaseName(dbName);
    m_myDb.setUserName(userName);
    m_myDb.setPassword(passwd);
    m_myDb.setConnectOptions("MYSQL_OPT_RECONNECT=1;");

    if (m_myDb.open())
    {
        ROS_INFO("database is established!");
		qDebug() << "链接数据库成功.";
        return true;
    }
    else
    {
        ROS_ERROR("build error! err:%s;", m_myDb.lastError().text().toStdString().c_str());
        return false;
    }
}

void LibDLWheelRobotDBOperation::closeDb()
{
    m_myDb.close();
}

bool LibDLWheelRobotDBOperation::execSqlString(QSqlQuery &qry)
{
    boost::mutex::scoped_lock lock(m_lock);
    bool bRet = qry.exec();
    if (!bRet)
    {
        ROS_ERROR("execSqlString failed; sqlString: %s", qry.executedQuery().toStdString().c_str());
    }
    else
    {
        ROS_INFO("execSqlString success; sqlString: %s", qry.executedQuery().toStdString().c_str())
    }
    return bRet;
}

bool LibDLWheelRobotDBOperation::querySqlString(QString sqlString, QSqlQuery &qry)
{
    boost::mutex::scoped_lock lock(m_lock);
    bool bRet = false;
    if (!m_myDb.isOpen())
    {
        return bRet;
    }

    int count = 1;
    bRet = qry.exec(sqlString);
    while (!bRet)
    {
        ROS_ERROR("querySqlString failed; Count: %d, sqlString: %s, qry execed: %s ,qry error : %s", count, sqlString.toStdString().c_str(), qry.executedQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
        bRet = qry.exec(sqlString);
		if (!bRet)
		{
		//	emit databaseDisconnect(bRet,qry.lastError().text());
		}
		break;
    }
    if (!bRet)
    {
        ROS_ERROR("querySqlString failed; Count: %d , sqlString: %s, qry execed: %s ,qry error : %s", count, sqlString.toStdString().c_str(), qry.executedQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }
    else
    {
        ROS_INFO("querySqlString success; Count: %d , sqlString: %s, qry execed: %s", count, sqlString.toStdString().c_str(), qry.executedQuery().toStdString().c_str());
    }
    return bRet;
}

bool LibDLWheelRobotDBOperation::insertDeviceSn(QString device_sn, QString device_name)
{
	return false;
}

bool LibDLWheelRobotDBOperation::getDeviceSn(std::map<QString, QString>& bim_device_map)
{
	QSqlQuery query;
	bool bReturn = false;

	QString sqlString = QString("select * from device_sn;");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		bim_device_map.insert(std::make_pair(query.value(0).toString(), query.value(1).toString()));
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDevice(WheelRobotInsertDeviceStruct dev, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    int deviceCount = 0;
//     QString sqlString = QString("SELECT COUNT(*) from devices WHERE device_uuid = %1;").arg(dev.device_uuid);
//     bReturn = querySqlString(sqlString, query);
//     query.next();
//     int deviceCount = query.value(0).toInt();
// 
//     if (deviceCount > 0)
//     {
//         sqlString = QString("UPDATE devices SET voltage_level_id = '%1', equipment_interval_uuid = '%2', device_area_uuid = '%3',"
//             "device_type_uuid = '%4', sub_device_type_uuid = '%5', device_point_type_uuid = '%6', unit_type_uuid = '%7', recognition_type_id= '%8', save_type_id = '%9',"
//             "meter_type_id = '%10', fever_type_id = '%11', threshold_filename = '%12', device_phase_id = '%13' WHERE device_uuid = '%14'")
//             .arg(dev.voltage_level_id).arg(dev.equipment_interval_uuid)
//             .arg(dev.device_area_uuid).arg(dev.device_type_uuid).arg(dev.sub_device_type_uuid)
//             .arg(dev.device_point_type_uuid).arg(dev.unit_type_uuid).arg(dev.recognition_type_id)
//             .arg(dev.save_type_id).arg(dev.meter_type_id).arg(dev.fever_type_id)
//             .arg(dev.threshold_filename).arg(dev.device_phase_id).arg(dev.device_uuid);
//         bReturn = querySqlString(sqlString, query);
//     }
//     else
//     {
        QString sqlString = QString("SELECT COUNT(*) from devices WHERE voltage_level_id = '%1' AND device_area_uuid = '%2' AND device_type_uuid = '%3' "
            " AND sub_device_type_uuid = '%4' AND device_point_type_uuid = '%5' AND device_phase_id = %6;")
            .arg(dev.device.voltage_level_id)
            //.arg(dev.device.equipment_interval_uuid)
            .arg(dev.device.device_area_uuid)
            .arg(dev.device.device_type_uuid)
            .arg(dev.device.sub_device_type_uuid)
            .arg(dev.device.device_point_type_uuid)
            .arg(dev.device.device_phase_id)
            ;//equipment_interval_uuid = '%2' AND 
        bReturn = querySqlString(sqlString, query);
        query.next();
        deviceCount = query.value(0).toInt();

        if (deviceCount > 0)
        {
            errMsg = "相同设备已存在,请勿重复插入";
            return false;
        }

        m_myDb.database().transaction();
        sqlString = QString("INSERT INTO devices SET voltage_level_id = '%1', equipment_interval_uuid = '%2', device_area_uuid = '%3',"
            "device_type_uuid = '%4', sub_device_type_uuid = '%5', device_point_type_uuid = '%6', unit_type_uuid = '%7', recognition_type_id= '%8', save_type_id = '%9',"
            "meter_type_id = '%10', fever_type_id = '%11', threshold_filename = '%12', device_phase_id = '%13', device_uuid = '%14',alarm_level_id='%15',start_using='1';")
            .arg(dev.device.voltage_level_id).arg(dev.device.equipment_interval_uuid)
            .arg(dev.device.device_area_uuid).arg(dev.device.device_type_uuid).arg(dev.device.sub_device_type_uuid)
            .arg(dev.device.device_point_type_uuid).arg(dev.device.unit_type_uuid).arg(dev.device.recognition_type_id)
            .arg(dev.device.save_type_id).arg(dev.device.meter_type_id).arg(dev.device.fever_type_id)
            .arg(dev.device.threshold_filename).arg(dev.device.device_phase_id).arg(dev.device.device_uuid).arg((int)dev.device.alarm_level_id);
        bReturn = querySqlString(sqlString, query);

        if (!bReturn)
        {
            m_myDb.database().rollback();
            errMsg = query.lastError().text();
            return bReturn;
        }

        sqlString = QString("INSERT INTO device_parameter SET device_uuid = '%1', point_id = %2, ptz_pan = %3,"
            "ptz_tilt = %4, hc_zoom_near = %5, hc_focus_near = %6, hc_zoom_far = %7, hc_focus_far= %8, mag_focus = %9,"
            "video_length = %10, audio_length = %11;")
            .arg(dev.paratmeter.device_uuid).arg(dev.paratmeter.point_id)
            .arg(dev.paratmeter.ptz_pan).arg(dev.paratmeter.ptz_tilt).arg(dev.paratmeter.hc_zoom_near)
            .arg(dev.paratmeter.hc_focus_near).arg(dev.paratmeter.hc_zoom_far).arg(dev.paratmeter.hc_focus_far)
            .arg(dev.paratmeter.mag_focus).arg(dev.paratmeter.video_length).arg(dev.paratmeter.audio_length);
        bReturn = querySqlString(sqlString, query);
    //}

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
    }
    m_myDb.database().commit();
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updateDeviceParameter(WheelRobortDeviceParameterStruct dev, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    int deviceCount = 0;

    QString sqlString = QString("SELECT COUNT(*) from device_parameter WHERE device_uuid = '%1';").arg(dev.device_uuid);
    bReturn = querySqlString(sqlString, query);
    query.next();
    deviceCount = query.value(0).toInt();
    if (deviceCount > 0)
    {
        sqlString = QString("UPDATE device_parameter SET point_id = %1, ptz_pan = %2,"
            "ptz_tilt = %3, hc_zoom_near = %4, hc_focus_near = %5, hc_zoom_far = %6, hc_focus_far= %7, mag_focus = %8,"
            "video_length = %9, audio_length = %10 WHERE device_uuid = '%11';")
            .arg(dev.point_id)
            .arg(dev.ptz_pan).arg(dev.ptz_tilt).arg(dev.hc_zoom_near)
            .arg(dev.hc_focus_near).arg(dev.hc_zoom_far).arg(dev.hc_focus_far)
            .arg(dev.mag_focus).arg(dev.video_length).arg(dev.audio_length).arg(dev.device_uuid);
        bReturn = querySqlString(sqlString, query);

        if (!bReturn)
        {
            errMsg = query.lastError().text();
            return bReturn;
        }
    }
    else
    {
        int iAllCount = 0;
        sqlString = QString("SELECT max(device_id) from device_parameter;");
        bReturn = querySqlString(sqlString, query);
        while (query.next())
        {
            iAllCount = query.value(0).toInt();
        }

        sqlString = QString("INSERT INTO device_parameter SET device_uuid = '%1', point_id = %2, ptz_pan = %3,"
            "ptz_tilt = %4, hc_zoom_near = %5, hc_focus_near = %6, hc_zoom_far = %7, hc_focus_far= %8, mag_focus = %9,"
            "video_length = %10, audio_length = %11, device_id=%12;")
            .arg(dev.device_uuid).arg(dev.point_id)
            .arg(dev.ptz_pan).arg(dev.ptz_tilt).arg(dev.hc_zoom_near)
            .arg(dev.hc_focus_near).arg(dev.hc_zoom_far).arg(dev.hc_focus_far)
            .arg(dev.mag_focus).arg(dev.video_length).arg(dev.audio_length).arg(iAllCount + 1);
        bReturn = querySqlString(sqlString, query);

        if (!bReturn)
        {
            errMsg = query.lastError().text();
            return bReturn;
        }
    }
}

bool LibDLWheelRobotDBOperation::updateDeviceAlarmLevel(DeviceAlarmLevel level, QString device_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE devices SET alarm_level_id = %1 WHERE device_uuid = '%2';").arg(level).arg(device_uuid);
    bReturn = querySqlString(sqlString, query);

    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceFromDeviceUUid(QString uuid, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM devices WHERE device_uuid = '%1';").arg(uuid);

    m_myDb.database().transaction();

    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
        return bReturn;
    }

    sqlString = QString("DELETE FROM device_parameter WHERE device_uuid = '%1';").arg(uuid);

    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
        return bReturn;
    }

    m_myDb.database().commit();
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceTypeByIntervalAndDeviceType(QString interval_uuid, QString device_uuid, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    
    QString sqlString = QString("SELECT device_uuid FROM devices WHERE equipment_interval_uuid = '%1' AND device_type_uuid = '%2';").arg(interval_uuid).arg(device_uuid);

    QStringList deviceList;
    bReturn = querySqlString(sqlString, query);

    while (query.next())
    {
        deviceList.push_back(query.value(0).toString());
    }

    if (!bReturn)
    {
        errMsg = query.lastError().text();
        return bReturn;
    }

    m_myDb.database().transaction();
    sqlString = QString("DELETE FROM devices WHERE equipment_interval_uuid = '%1' AND device_type_uuid = '%2';").arg(interval_uuid).arg(device_uuid);

    
    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
        return bReturn;
    }

    bReturn = deleteDeviceParameterFromDeviceList(deviceList);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
        return bReturn;
    }

    m_myDb.database().commit();
    return bReturn;

}

bool LibDLWheelRobotDBOperation::deleteIntervalByIntervalUuid(QString interval_uuid, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;

    QString sqlString = QString("SELECT device_uuid FROM devices WHERE equipment_interval_uuid = '%1';").arg(interval_uuid);

    QStringList deviceList;
    bReturn = querySqlString(sqlString, query);

    while (query.next())
    {
        deviceList.push_back(query.value(0).toString());
    }

    if (!bReturn)
    {
        errMsg = query.lastError().text();
        return bReturn;
    }

    m_myDb.database().transaction();

    sqlString = QString("DELETE FROM devices WHERE equipment_interval_uuid = '%1';").arg(interval_uuid);
    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
        return bReturn;
    }

    bReturn = deleteDeviceParameterFromDeviceList(deviceList);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
        return bReturn;
    }

    m_myDb.database().commit();

    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteVoltageDevicesByVoltageId(QString voltage_id, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;

    QString sqlString = QString("SELECT device_uuid FROM devices WHERE voltage_level_id = '%1';").arg(voltage_id);

    QStringList deviceList;
    bReturn = querySqlString(sqlString, query);

    while (query.next())
    {
        deviceList.push_back(query.value(0).toString());
    }

    if (!bReturn)
    {
        errMsg = query.lastError().text();
        return bReturn;
    }

    m_myDb.database().transaction();

    sqlString = QString("DELETE FROM devices WHERE voltage_level_id = '%1';").arg(voltage_id);
    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
        return bReturn;
    }

    bReturn = deleteDeviceParameterFromDeviceList(deviceList);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        errMsg = query.lastError().text();
        return bReturn;
    }

    m_myDb.database().commit();

    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceFromDeviceUUidList(QStringList devicesUUidList)
{
	boost::mutex::scoped_lock lock(m_lock);
	QSqlQuery query;
	query.exec("START TRANSACTION");
	//输入插入
	for (int i = 0; i < devicesUUidList.size(); i++)
	{
		QString str = QString("DELETE FROM devices WHERE device_uuid='%1';")
			.arg(devicesUUidList[i]);
		query.exec(str);
	}
	//提交事务
	query.exec("COMMIT");
	return true;
}

bool LibDLWheelRobotDBOperation::getDeviceFullNameByDeviceUuid(QString device_uuid, QString &fullName)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT * from deviceFullName WHERE device_uuid = '%1';").arg(device_uuid);
    bReturn = querySqlString(sqlString, query);
    fullName = "";
    while (query.next())
    {
        int i = 1;
        fullName = "";
        fullName += query.value(i++).toString();
        fullName += query.value(i++).toString();
        fullName += query.value(i++).toString();
        fullName += query.value(i++).toString();
        fullName += query.value(i++).toString();
        fullName += query.value(i++).toString();
        QString phaseName = query.value(i++).toString();
        fullName.replace("X", phaseName);

    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::selectDeviceNodeUUidForDeviceUUid(QString device_uuid, QStringList &nodeUUid)
{
	QString sqlString = QString("SELECT voltage_level_id,equipment_interval_uuid,device_type_uuid,device_point_type_uuid FROM devices WHERE device_uuid='%1';")
		.arg(device_uuid);
	QSqlQuery query;
	bool bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		nodeUUid.append(query.value(0).toString());
		nodeUUid.append(query.value(1).toString());
		nodeUUid.append(query.value(2).toString());
		nodeUUid.append(query.value(3).toString());
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceFromDeviceList(QStringList deviceList)
{
    QString sqlString = QString("DELETE FROM devices WHERE device_uuid in (");
    QSqlQuery query;
    bool bReturn = false;

    for (int i = 0; i < deviceList.size(); i++)
    {
        if (i != 0)
        {
            sqlString += ',';
        }
        sqlString += QString("'%1'").arg(deviceList[i]);
    }
    sqlString += ");";

    bReturn = querySqlString(sqlString, query);

    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceParameterFromDeviceList(QStringList deviceList)
{
    QString sqlString = QString("DELETE FROM device_parameter WHERE device_uuid in (");
    QSqlQuery query;
    bool bReturn = false;

    for (int i = 0; i < deviceList.size(); i++)
    {
        if (i != 0)
        {
            sqlString += ',';
        }
        sqlString += deviceList[i];
    }
    sqlString += ");";

    bReturn = querySqlString(sqlString, query);

    return bReturn;
}

bool LibDLWheelRobotDBOperation::queryDeviceByDeviceUuid(QString uuid, WheelRobotDeviceStruct &dev)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT * from devices WHERE device_uuid = '%1';").arg(uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        dev.device_uuid = query.value(i++).toString();
        dev.voltage_level_id = query.value(i++).toString();
        dev.equipment_interval_uuid = query.value(i++).toString();
        dev.device_area_uuid = query.value(i++).toString();
        dev.device_type_uuid = query.value(i++).toString();
        dev.sub_device_type_uuid = query.value(i++).toInt();
        dev.device_point_type_uuid = query.value(i++).toString();
        dev.unit_type_uuid = query.value(i++).toInt();
        dev.recognition_type_id = (WheelRobotRecognizeType)query.value(i++).toInt();
		dev.save_type_id = (WheelRobotSaveType)query.value(i++).toInt();
        dev.meter_type_id = (WheelRobotMeterType)query.value(i++).toInt();
        dev.fever_type_id = (WheelRobotFeverType)query.value(i++).toInt();
        dev.threshold_filename = query.value(i++).toInt();
        dev.device_phase_id = (WheelRobotPhaseType)query.value(i++).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelStationConfigDataDB(std::map<int, WheelStationConfigStruct> &m_WheelStationConfigData)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelStationConfigStruct wheelStationCfgStu;
    QString sqlString = QString("select * from station_cfg;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        wheelStationCfgStu.station_pms_id = query.value(i++).toString();
        wheelStationCfgStu.company_name = query.value(i++).toString();
        wheelStationCfgStu.station_name = query.value(i++).toString();
        wheelStationCfgStu.core_ip = query.value(i++).toString();
        wheelStationCfgStu.core_port = query.value(i++).toInt();
        wheelStationCfgStu.hcnetcamera_username = query.value(i++).toString();
        wheelStationCfgStu.hcnetcamera_userpassword = query.value(i++).toString();
        wheelStationCfgStu.hcnetcamera_ip = query.value(i++).toString();
        wheelStationCfgStu.hcnetcamera_port = query.value(i++).toInt();
        wheelStationCfgStu.infrared_ip = query.value(i++).toString();
        wheelStationCfgStu.infrared_port = query.value(i++).toInt();
        wheelStationCfgStu.calibfile_path = query.value(i++).toString();
        wheelStationCfgStu.report_path = query.value(i++).toString();
        m_WheelStationConfigData.insert(std::make_pair(query.value(0).toInt(), wheelStationCfgStu));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceAreaDataDB(std::map<QString, QString> &m_WheelDeviceAreaData)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelStationConfigStruct wheelStationCfgStu;
    QString sqlString = QString("select * from device_area;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelDeviceAreaData.insert(std::make_pair(query.value(0).toString(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDevicePointNameDB(std::map<QString, WheelDevicePointNameStruct> &m_WheelDevicePointNameData)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelDevicePointNameStruct wheelDevicePointNameStu;
    QString sqlString = QString("select * from device_point_name;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        wheelDevicePointNameStu.device_point_type_uuid = query.value(i++).toString();
        wheelDevicePointNameStu.sub_device_type_uuid = query.value(i++).toString();
        wheelDevicePointNameStu.device_point_type_name = query.value(i++).toString();
        wheelDevicePointNameStu.recognition_type_id = query.value(i++).toInt();
        wheelDevicePointNameStu.meter_type_id = query.value(i++).toInt();
        wheelDevicePointNameStu.fever_type_id = query.value(i++).toInt();
        wheelDevicePointNameStu.save_type_id = query.value(i++).toInt();
        m_WheelDevicePointNameData.insert(std::make_pair(query.value(0).toString(), wheelDevicePointNameStu));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelSubDeviceTypeDB(std::map<QString, WheelSubDeviceTypeStruct> &m_WheelSubDeviceTypeData)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelSubDeviceTypeStruct wheelSubDeviceTypeStu;
    QString sqlString = QString("select * from sub_device_type;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        wheelSubDeviceTypeStu.sub_device_type_uuid = query.value(0).toString();
        wheelSubDeviceTypeStu.device_type_uuid = query.value(1).toString();
        wheelSubDeviceTypeStu.sub_device_name = query.value(2).toString();
        m_WheelSubDeviceTypeData.insert(std::make_pair(query.value(0).toString(), wheelSubDeviceTypeStu));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceTypeDB(std::map<QString, QString> &m_WheelDeviceTypeData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from device_type;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelDeviceTypeData.insert(std::make_pair(query.value(0).toString(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskEndActionDB(std::map<int, QString> &m_WheelTaskEndActionData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from task_end_action;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelTaskEndActionData.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskEndTypeDB(std::map<int, QString> &m_WheelTaskEndTypeData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from task_end_type;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelTaskEndTypeData.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskTypeDB(std::map<int, QString> &m_WheelTaskTypeData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from task_type;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelTaskTypeData.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelVoltageLevelDB(std::map<QString, QString> &m_WheelVoltageLevelData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from voltage_level;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelVoltageLevelData.insert(std::make_pair(query.value(0).toString(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelFeverTypeDB(std::map<int, QString> &m_WheelFeverTypeData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from fever_type;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelFeverTypeData.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelRecognitionTypeDB(std::map<int, QString> &m_WheelRecognitionTypeData)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelRecognitionTypeStruct wheelRecognitionTypeStu;
    QString sqlString = QString("select * from recognition_type;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelRecognitionTypeData.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelMeterTypeDB(std::map<int, WheelRobortMeterTypeStruct>& m_WheelMeterTypeData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT * FROM meter_type;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        WheelRobortMeterTypeStruct meterType;
        meterType.meter_type_id = query.value(i++).toInt();
        meterType.meter_type_name = query.value(i++).toString();
        meterType.threshold_filename = query.value(i++).toString();

        m_WheelMeterTypeData.insert(std::make_pair(query.value(0).toInt(), meterType));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelPointTreeDataDB(QList<WheelPointTreeDataStruct> &m_wheelPointTreeData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT devices.device_uuid,voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_type.device_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,devices.alarm_level_id,device_phase.device_phase_id FROM devices JOIN voltage_level JOIN equipment_interval JOIN device_type JOIN device_point_name JOIN device_phase WHERE devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id ORDER BY voltage_level.voltage_level_id,equipment_interval.equipment_interval_name,device_type.device_type_name,device_point_name.device_point_type_name DESC,device_phase.device_phase_name ASC");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        WheelPointTreeDataStruct wheelPointTreeDataStu;
        wheelPointTreeDataStu.device_uuid = query.value(i++).toString();
        wheelPointTreeDataStu.wheel_pointTree_data.append(query.value(i++).toString());
        wheelPointTreeDataStu.wheel_pointTree_data.append(query.value(i++).toString());
        wheelPointTreeDataStu.wheel_pointTree_data.append(query.value(i++).toString());
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        wheelPointTreeDataStu.wheel_pointTree_data.append(pointName);
        wheelPointTreeDataStu.alarm_level = ((DeviceAlarmLevel)query.value(i++).toInt());
		if (query.value(i++).toInt() != 5)
		{
			m_wheelPointTreeData.append(wheelPointTreeDataStu);
		}
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelPointTreeDataDB(QList<WheelPointTreeDataStruct> &m_wheelPointTreeData, WheelPatrolParameter m_wheelPatrolPara)
{
    QSqlQuery query;
    bool bReturn = false;

    QString Device_Area_UUid = QString("");
    if (m_wheelPatrolPara.m_device_area_uuid.size() == 0)
    {
        Device_Area_UUid = QString("devices.device_area_uuid like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_device_area_uuid.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_device_area_uuid.size() - 1))
            {
                Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
            }
            else
            {
                Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
            }

        }
    }

    QString Device_Type_UUid = QString("");
    if (m_wheelPatrolPara.m_device_type_uuid.size() == 0)
    {
        Device_Type_UUid = QString("devices.device_type_uuid like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_device_type_uuid.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_device_type_uuid.size() - 1))
            {
                Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
            }
            else
            {
                Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
            }

        }
    }

    QString Recognition_Type_Id = QString("");
    if (m_wheelPatrolPara.m_recognition_type_uuid.size() == 0)
    {
        Recognition_Type_Id = QString("devices.recognition_type_id like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_recognition_type_uuid.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_recognition_type_uuid.size() - 1))
            {
                Recognition_Type_Id = Recognition_Type_Id + QString("devices.recognition_type_id = '%1' ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
            }
            else
            {
                Recognition_Type_Id = Recognition_Type_Id + QString("devices.recognition_type_id = '%1' OR ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
            }

        }
    }

    QString Meter_Type_Id = QString("");
    if (m_wheelPatrolPara.m_meter_type_id.size() == 0)
    {
        Meter_Type_Id = QString("devices.meter_type_id like '%%' OR devices.meter_type_id IS NULL");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_meter_type_id.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_meter_type_id.size() - 1))
            {
                Meter_Type_Id = Meter_Type_Id + QString("devices.meter_type_id = '%1' ").arg(m_wheelPatrolPara.m_meter_type_id[i]);
            }
            else
            {
                Meter_Type_Id = Meter_Type_Id + QString("devices.meter_type_id = '%1' OR ").arg(m_wheelPatrolPara.m_meter_type_id[i]);
            }

        }
    }

    QString Alarm_Level_Id = QString("");
    if (m_wheelPatrolPara.m_alarm_level_id.size() == 0)
    {
        Alarm_Level_Id = QString("devices.alarm_level_id like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_alarm_level_id.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_alarm_level_id.size() - 1))
            {
                Alarm_Level_Id = Alarm_Level_Id + QString("devices.alarm_level_id = '%1' ").arg(m_wheelPatrolPara.m_alarm_level_id[i]);
            }
            else
            {
                Alarm_Level_Id = Alarm_Level_Id + QString("devices.alarm_level_id = '%1' OR ").arg(m_wheelPatrolPara.m_alarm_level_id[i]);
            }

        }
    }
    
    QString sqlString = QString("SELECT devices.device_uuid,voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_type.device_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,devices.alarm_level_id,device_phase.device_phase_id FROM devices JOIN voltage_level JOIN equipment_interval JOIN device_type JOIN device_point_name JOIN device_phase WHERE devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND (%1) AND (%2) AND (%3) AND (%4) AND (%5) AND device_point_name.device_point_type_name like '%%6%' ORDER BY voltage_level.voltage_level_id,equipment_interval.equipment_interval_name,device_type.device_type_name,device_point_name.device_point_type_name DESC,device_phase.device_phase_name ASC")
        .arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id).arg(Alarm_Level_Id).arg(m_wheelPatrolPara.device_point_name);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        WheelPointTreeDataStruct wheelPointTreeDataStu;
        wheelPointTreeDataStu.device_uuid = query.value(i++).toString();
        wheelPointTreeDataStu.wheel_pointTree_data.append(query.value(i++).toString());
        wheelPointTreeDataStu.wheel_pointTree_data.append(query.value(i++).toString());
        wheelPointTreeDataStu.wheel_pointTree_data.append(query.value(i++).toString());
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        wheelPointTreeDataStu.wheel_pointTree_data.append(pointName);
        wheelPointTreeDataStu.alarm_level = ((DeviceAlarmLevel)query.value(i++).toInt());
		if (query.value(i++) != 5)
		{
			m_wheelPointTreeData.append(wheelPointTreeDataStu);
		}
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelEquipmentTreeDataDB(QList<QStringList> &lstStrings)
{
	QSqlQuery query;
	bool bReturn = false;
	//QString sqlString = QString("SELECT voltage_level.voltage_level_id,voltage_level.voltage_level_name,equipment_interval.equipment_interval_uuid, equipment_interval.equipment_interval_name,device_type.device_type_uuid, device_type.device_type_name,device_point_name.device_point_type_uuid,device_point_name.device_point_type_name,devices.device_uuid, device_phase.device_phase_id FROM device_phase JOIN voltage_level JOIN equipment_interval JOIN device_type JOIN device_point_name JOIN devices WHERE devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id");
	QString sqlString = QString("SELECT voltage_level.voltage_level_id,voltage_level.voltage_level_name,equipment_interval.equipment_interval_uuid, equipment_interval.equipment_interval_name,device_type.device_type_uuid, device_type.device_type_name,device_point_name.device_point_type_uuid,device_point_name.device_point_type_name,devices.device_uuid, device_phase.device_phase_id FROM device_phase JOIN voltage_level JOIN equipment_interval JOIN device_type JOIN device_point_name JOIN devices WHERE devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 8;
		QStringList lstCurEquipmentInfo;
		lstCurEquipmentInfo << query.value(i--).toString()			//电压等级 uid
			<< query.value(i--).toString()					//电压等级 name
			<< query.value(i--).toString()					//间隔 id
			<< query.value(i--).toString()					//间隔 name
			<< query.value(i--).toString()					//类型 id
			<< query.value(i--).toString()					//类型 name
			<< query.value(i--).toString()					//设备 id
			<< query.value(i--).toString()					//设备 name
			<< query.value(i--).toString();					//device id
		lstStrings.push_back(lstCurEquipmentInfo);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelLevelDataDB(QList<QStringList> &lstStrings)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT voltage_level.voltage_level_id,voltage_level.voltage_level_name FROM voltage_level");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		QStringList lstCurEquipmentInfo;
		lstCurEquipmentInfo << query.value(i++).toString()			//电压等级 uid
			<< query.value(i).toString();					//电压等级 name
		lstStrings.push_back(lstCurEquipmentInfo);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelEquipmentTypeDataDB(QList<QStringList> &lstStrings)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT device_type.device_type_uuid, device_type.device_type_name FROM device_type");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		QStringList lstCurEquipmentInfo;
		lstCurEquipmentInfo << query.value(i++).toString()			//设备类型 uid
			<< query.value(i).toString();					//设备类型 name
		lstStrings.push_back(lstCurEquipmentInfo);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelEquipmentDataDB(QList<QStringList> &lstStrings, QString strTypeID)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT device_point_name.device_point_type_uuid,device_point_name.device_point_type_name FROM device_point_name JOIN sub_device_type WHERE device_point_name.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND sub_device_type.device_type_uuid='%1';").arg(strTypeID);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		QStringList lstCurEquipmentInfo;
		lstCurEquipmentInfo << query.value(i++).toString()			//设备类型 uid
			<< query.value(i).toString();					//设备类型 name
		lstStrings.push_back(lstCurEquipmentInfo);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelAlarmLevelDB(std::map<int, QString> &m_WheelAlarmLevelData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from alarm_level;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelAlarmLevelData.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelSaveTypeDB(std::map<int, QString> &m_WheelSaveTypeData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from save_type;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelSaveTypeData.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelEquipmentIntervalDB(std::map<QString, WheelRobortEquipmentIntervalStruct> &m_WheelEquipmentIntervalData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from equipment_interval;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        WheelRobortEquipmentIntervalStruct robortEquipmentInterval;
        robortEquipmentInterval.equipment_interval_uuid = query.value(i++).toString();
        robortEquipmentInterval.voltage_level_id = query.value(i++).toString();
        robortEquipmentInterval.equipment_interval_name = query.value(i++).toString();
        m_WheelEquipmentIntervalData.insert(std::make_pair(query.value(0).toString(), robortEquipmentInterval));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertTaskDB(WheelTaskStruct m_wheelTaskStruct)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelRecognitionTypeStruct wheelRecognitionTypeStu;
    QString sqlString = QString("INSERT INTO task VALUE ('%1', '%2', '%3', '%4', %5, %6, '%7', '%8', '%9', '%10','%11','%12','%13','%14', %15, %16,'%17','%18','%19','%20', '%21', '%22');")
        .arg(m_wheelTaskStruct.task_uuid)
        .arg(m_wheelTaskStruct.task_name)
        .arg(m_wheelTaskStruct.task_edit_uuid)
        .arg(m_wheelTaskStruct.task_template_uuid)
        .arg((int)m_wheelTaskStruct.task_type_id)
        .arg((int)m_wheelTaskStruct.task_status_id)
        .arg(m_wheelTaskStruct.task_start_time)
        .arg(m_wheelTaskStruct.task_end_time)
        .arg(m_wheelTaskStruct.task_end_type_id)
        .arg(m_wheelTaskStruct.task_duration)
        .arg(m_wheelTaskStruct.task_total_devices)
        .arg(m_wheelTaskStruct.task_total_bugs)
        .arg(m_wheelTaskStruct.task_total_mistake)
        .arg(m_wheelTaskStruct.task_end_action)
        .arg((int)m_wheelTaskStruct.task_curr_action_type_id)
        .arg((int)m_wheelTaskStruct.task_audit_id)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_temperature)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_humidity)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_pm_2_5)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_wind_direct)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_wind_speed)
        .arg(m_wheelTaskStruct.task_code);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertTaskDB(WheelTaskStruct m_wheelTaskStruct, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelRecognitionTypeStruct wheelRecognitionTypeStu;
    QString sqlString = QString("INSERT INTO task VALUE ('%1', '%2', '%3', '%4', %5, %6, '%7', %8, '%9', '%10','%11','%12','%13','%14', %15, %16,'%17','%18','%19','%20', '%21','%22');")
        .arg(m_wheelTaskStruct.task_uuid)
        .arg(m_wheelTaskStruct.task_name)
        .arg(m_wheelTaskStruct.task_edit_uuid)
        .arg(m_wheelTaskStruct.task_template_uuid)
        .arg((int)m_wheelTaskStruct.task_type_id)
        .arg((int)m_wheelTaskStruct.task_status_id)
        .arg(m_wheelTaskStruct.task_start_time)
        .arg(NULL)
        .arg(m_wheelTaskStruct.task_end_type_id)
        .arg(m_wheelTaskStruct.task_duration)
        .arg(m_wheelTaskStruct.task_total_devices)
        .arg(m_wheelTaskStruct.task_total_bugs)
        .arg(m_wheelTaskStruct.task_total_mistake)
        .arg(m_wheelTaskStruct.task_end_action)
        .arg((int)m_wheelTaskStruct.task_curr_action_type_id)
        .arg((int)m_wheelTaskStruct.task_audit_id)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_temperature)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_humidity)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_pm_2_5)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_wind_direct)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_wind_speed)
        .arg(m_wheelTaskStruct.task_code);
    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        errMsg = query.lastError().text();
    }

    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertImmediateTaskDB(WheelTaskStruct m_wheelTaskStruct, QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelRecognitionTypeStruct wheelRecognitionTypeStu;
    QString sqlString = QString("INSERT INTO task SET task_uuid = '%1',task_name='%2',task_edit_uuid='%3',task_type_id='%4',task_status_id='%5',task_start_time='%6',task_total_devices='%7';")
        .arg(m_wheelTaskStruct.task_uuid).arg(m_wheelTaskStruct.task_name).arg(m_wheelTaskStruct.task_edit_uuid)
        .arg(m_wheelTaskStruct.task_type_id).arg(m_wheelTaskStruct.task_status_id).arg(m_wheelTaskStruct.task_start_time)
        .arg(m_wheelTaskStruct.task_total_devices);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        retMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataTaskDB(WheelTaskStruct m_wheelTaskStruct)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE task SET task_name = '%1',task_edit_uuid = '%2',task_template_uuid = '%3', task_type_id = '%4', task_status_id = '%5', task_start_time = '%6', task_end_time = '%7', task_end_type_id = '%8', task_duration = '%9', task_total_devices = '%10', task_total_bugs = '%11', task_total_mistake = '%12', task_end_action = '%13',task_curr_action_type_id = '%14', envi_temperature = '%15', envi_humidity = '%16', envi_pm_2_5 = '%17', envi_wind_direct = '%18', envi_wind_speed = '%19' WHERE task_uuid = '%20';")
        .arg(m_wheelTaskStruct.task_name)
        .arg(m_wheelTaskStruct.task_edit_uuid)
        .arg(m_wheelTaskStruct.task_template_uuid)
        .arg((int)m_wheelTaskStruct.task_type_id)
        .arg((int)m_wheelTaskStruct.task_status_id)
        .arg(m_wheelTaskStruct.task_start_time)
        .arg(m_wheelTaskStruct.task_end_time)
        .arg(m_wheelTaskStruct.task_end_type_id)
        .arg(m_wheelTaskStruct.task_duration)
        .arg(m_wheelTaskStruct.task_total_devices)
        .arg(m_wheelTaskStruct.task_total_bugs)
        .arg(m_wheelTaskStruct.task_total_mistake)
        .arg(m_wheelTaskStruct.task_end_action)
        .arg((int)m_wheelTaskStruct.task_curr_action_type_id)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_temperature)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_humidity)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_pm_2_5)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_wind_direct)
        .arg(m_wheelTaskStruct.envi_environment_struct.envi_wind_speed)
        .arg(m_wheelTaskStruct.task_uuid);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updateTaskDB(QString task_uuid, QString start_time)
{
    ROS_INFO("updateTaskDB task: task_uuid,start_time ");
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE task SET task_status_id = '%1', task_start_time = '%2' WHERE task_uuid = '%3';")
        .arg(3)
        .arg(start_time)
        .arg(task_uuid);
    bReturn = querySqlString(sqlString, query);
    ROS_INFO("updateTaskDB task start: task_uuid:%s task_status_id:3", task_uuid.toStdString().data());
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updateTaskDB(QString taskUUid, QStringList taskQList)
{
    ROS_INFO("updateTaskDB task: task_uuid ");
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString;
    if (!taskUUid.isEmpty())
    {
        sqlString = QString("UPDATE task SET task_status_id = 3 WHERE task_uuid = '%1';")
            .arg(taskUUid);
        bReturn = querySqlString(sqlString, query);
        ROS_INFO("updateTaskDB task list1: task_uuid:%s task_status_id:3", taskUUid.toStdString().data());
    }
    

    for (int i = 0; i < taskQList.size(); i++)
    {
        sqlString = QString("UPDATE task SET task_status_id = 1 WHERE task_uuid = '%1';")
            .arg(taskQList.at(i));

        bReturn = querySqlString(sqlString, query);
        ROS_INFO("updateTaskDB task list2: task_uuid:%s task_status_id:1", taskUUid.toStdString().data());
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updateTaskEndStatus(WheelTaskStruct wheelTaskStruct)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE task SET task_status_id = '%1', task_end_time = '%2', task_end_type_id = '%3' WHERE task_uuid = '%4';")
        .arg(wheelTaskStruct.task_status_id)
        .arg(wheelTaskStruct.task_end_time)
        .arg(wheelTaskStruct.task_end_type_id)
        .arg(wheelTaskStruct.task_uuid);

    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        ROS_ERROR("updateTaskEndStatus error.");
    }

	int allCount = 0;
	int bugCount = 0;
	int mistakeCount = 0;
	sqlString = QString("SELECT alarm_level_id,count(*) FROM inspect_result WHERE inspect_result.task_uuid='%1' GROUP BY alarm_level_id;")
		.arg(wheelTaskStruct.task_uuid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int iRet1 = query.value(0).toInt();
		int iRet2 = query.value(1).toInt();

		if (iRet1 == 1)
		{
		}
		else if (iRet1 > 1 && iRet1 < 6)
		{
			bugCount = bugCount + iRet2;
		}
		else
		{
			mistakeCount = mistakeCount + iRet2;
		}
		allCount = allCount + iRet2;
	}

	if (bReturn)
	{
		sqlString = QString("UPDATE task SET task_total_devices='%1',task_total_bugs='%2',task_total_mistake='%3' WHERE task_uuid='%4';")
			.arg(allCount)
			.arg(bugCount)
			.arg(mistakeCount)
			.arg(wheelTaskStruct.task_uuid);
		bReturn = querySqlString(sqlString, query);
	}
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataTaskForAuditIdDB(QString taskUUid)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("UPDATE task SET task_audit_id = '1' WHERE task_uuid = '%1';")
		.arg(taskUUid);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::updataTaskDBWhenCollectFinished(QString task_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE task SET task_status_id = '%1' WHERE task_uuid = '%2';")
        .arg((int)WHEEL_ROBOT_TASK_STATUS_FINISH)
        .arg(task_uuid);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteTaskDB(QString m_task_uuid, QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM task where task_uuid='%1';").arg(m_task_uuid);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        retMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getTaskNameByTaskUuid(QString task_uuid, QString &task_name)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT task_name FROM task where task_uuid='%1';").arg(task_uuid);
    bReturn = querySqlString(sqlString, query);
    
    while (query.next())
    {
        task_name = query.value(0).toString();
    }

    return bReturn;
}

bool LibDLWheelRobotDBOperation::getTaskPropertyByTaskUuid(QString task_uuid, QString &task_property)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT task_type_name FROM task_type INNER JOIN task ON task.task_type_id = task_type.task_type_id WHERE task.task_uuid = '%1';").arg(task_uuid);
    bReturn = querySqlString(sqlString, query);
    
    while (query.next())
    {
        task_property = query.value(0).toString();
    }

    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertTaskTemplateDB(WheelTaskTemplateStruct m_wheelTaskTemplateStruct,QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO task_template VALUE('%1','%2','%3','%4','%5','%6','%7', '%8', '%9', '%10');")
        .arg(m_wheelTaskTemplateStruct.task_template_uuid)
        .arg(m_wheelTaskTemplateStruct.task_edit_uuid)
        .arg(m_wheelTaskTemplateStruct.task_type_id)
        .arg(m_wheelTaskTemplateStruct.task_end_action_id)
        .arg(m_wheelTaskTemplateStruct.task_template_name)
        .arg(m_wheelTaskTemplateStruct.task_start_date)
        .arg(m_wheelTaskTemplateStruct.task_repeat_duration)
        .arg(m_wheelTaskTemplateStruct.task_status_id)
        .arg(m_wheelTaskTemplateStruct.task_loop_type_id)
        .arg(m_wheelTaskTemplateStruct.task_start_time.toString("hh:mm:ss"));
    bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		retMsg = query.lastError().text();
	}
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteTaskTemplateDB(QString m_task_template_uuid, QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM task_template where task_template_uuid='%1';").arg(m_task_template_uuid);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        retMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updateTaskTemplateDB(QString task_template_uuid, int type, QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE task_template SET task_status_id = '%1' WHERE task_template_uuid = '%2';")
        .arg(type)
        .arg(task_template_uuid);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        retMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskTemplateAllTimedTasksSortByStartTimeAfterCurrent(QList<WheelTaskTemplateStruct> &tasks)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT * FROM task_template WHERE (task_type_id = %1 OR task_type_id = %2) AND task_start_time >= '%3' AND task_status_id!=6 ORDER BY task_start_time ASC;")
        .arg(WHEEL_ROBOT_TASK_TIMED_TASK).arg(WHEEL_ROBOT_TASK_LOOP_TASK).arg(QTime::currentTime().toString("hh:mm:ss"));
    bReturn = querySqlString(sqlString, query);

    while (query.next())
    {
        int i = 0;
        WheelTaskTemplateStruct task;

        task.task_template_uuid = query.value(i++).toString();
        task.task_edit_uuid = query.value(i++).toString();
        task.task_type_id = (WheelRobotTaskType)query.value(i++).toInt();
        task.task_end_action_id = (WheelRobotTaskEndActionType)query.value(i++).toInt();
        task.task_template_name = query.value(i++).toString();
        task.task_start_date = query.value(i++).toString();
        task.task_repeat_duration = query.value(i++).toInt();
        task.task_status_id = (WheelRobotTaskStatusType)query.value(i++).toInt();
        task.task_loop_type_id = (WheelRobotTaskLoopType)query.value(i++).toInt();
        task.task_start_time = QTime::fromString(query.value(i++).toString(), "hh:mm:ss");

        tasks.push_back(task);
    }

    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertInspectResultDB(WheelInspectResultStruct m_wheelInspectResultStruct)
{
    QSqlQuery query;
    bool bReturn = false;

    //QString sqlString = QString("INSERT INTO inspect_result VALUE('%1','%2','%3','%4','%5','%6','%7','%8');")
   // QString sqlString = QString("INSERT INTO inspect_result SET task_uuid='%1',device_uuid='%2',inspect_times='%3',inspect_result='%4',inspect_status_id='%5',is_dealed='%6',alarm_level_id='%7',deal_info_uuid='%8',inspect_result='%9';")
    QString sqlString = QString("INSERT IGNORE INTO inspect_result SET task_uuid='%1',device_uuid='%2',inspect_time='%3',inspect_result='%4',inspect_status_id='%5',is_dealed='%6',alarm_level_id='%7',deal_info_uuid='%8';")
        .arg(m_wheelInspectResultStruct.task_uuid)
        .arg(m_wheelInspectResultStruct.device_uuid)
        .arg(m_wheelInspectResultStruct.inspect_time)
        .arg(m_wheelInspectResultStruct.inspect_result)
        .arg((int)m_wheelInspectResultStruct.inspect_status_id)
        .arg(m_wheelInspectResultStruct.is_dealed)
        .arg(m_wheelInspectResultStruct.alarm_level_id)
        .arg(m_wheelInspectResultStruct.deal_info_uuid);
 //       .arg(m_wheelInspectResultStruct.inspect_result2);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataInspectResultForDealedDB(WheelRobotPartrolReaultVerify resultVerify, QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE inspect_result SET is_dealed = '%1',alarm_level_id='%2',deal_info_uuid ='%3' where task_uuid = '%4' and device_uuid='%5';")
        .arg(resultVerify.is_dealed)
        .arg((int)resultVerify.alarm_level_id)
        .arg(resultVerify.wheelResultInfo.deal_info_uuid)
        .arg(resultVerify.task_uuid)
        .arg(resultVerify.device_uuid);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        retMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataInspectResultAndInsertDealInfoDB(WheelPartolResult resultVerify, QString &rectMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	m_myDb.database().transaction();
	QString sqlString = QString("UPDATE inspect_result SET inspect_status_id='%1',is_dealed = '%2',alarm_level_id='%3',deal_info_uuid ='%4' where task_uuid = '%5' and device_uuid='%6';")
		.arg(resultVerify.inspect_status_id)
		.arg(1)
		.arg(resultVerify.alarm_level_id)
		.arg(resultVerify.deal_info_uuid)
		.arg(resultVerify.task_uuid)
		.arg(resultVerify.device_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		m_myDb.database().rollback();
		rectMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}
	//sqlString = QString("INSERT INTO deal_info VALUE ('%1', '%2', '%3', '%4', '%5','','');")
	sqlString = QString("INSERT INTO deal_info set deal_info_uuid='%1',deal_task_uuid='%2',dealed_info='%3',dealed_result='%4',dealed_status_id='%5';")
		.arg(resultVerify.deal_info_uuid)
		.arg(resultVerify.task_uuid)
		.arg(resultVerify.dealed_info)
		.arg(resultVerify.dealed_result)
		.arg(resultVerify.dealed_status_id);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		m_myDb.database().rollback();
		rectMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}
	m_myDb.database().commit();
	return bReturn;
}

bool LibDLWheelRobotDBOperation::updataInspectResultAndUpdataDealInfoDB(WheelPartolResult resultVerify, QString &rectMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	m_myDb.database().transaction();

	QString sqlString = QString("UPDATE inspect_result SET inspect_status_id='%1',is_dealed = '%2',alarm_level_id='%3',deal_info_uuid ='%4' where task_uuid = '%5' and device_uuid='%6';")
		.arg(resultVerify.inspect_status_id)
		.arg(resultVerify.is_dealed)
		.arg(resultVerify.alarm_level_id)
		.arg(resultVerify.deal_info_uuid)
		.arg(resultVerify.task_uuid)
		.arg(resultVerify.device_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		m_myDb.database().rollback();
		rectMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}

	sqlString = QString("UPDATE deal_info SET deal_task_uuid='%1',dealed_info = '%2',dealed_result='%3',dealed_status_id='%4' where deal_info_uuid='%5';")
		.arg(resultVerify.task_uuid)
		.arg(resultVerify.dealed_info)
		.arg(resultVerify.dealed_result)
		.arg(resultVerify.dealed_status_id)
		.arg(resultVerify.deal_info_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		m_myDb.database().rollback();
		rectMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}
	m_myDb.database().commit();
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertThresholdByMeterType(QString normalString, QString warningString, QString commonString, QString serialString, QString dangerString, WheelRobotMeterType type, QString threshold_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO threshold VALUE('%1','%2','%3','%4','%5','%6');")
        .arg(threshold_uuid).arg(normalString).arg(warningString).arg(commonString).arg(serialString).arg(dangerString);

    m_myDb.database().transaction();

    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        //rectMsg = query.lastError().text();
        m_myDb.database().commit();
        return bReturn;
    }

    sqlString = QString("UPDATE meter_type SET threshold_filename = '%1' WHERE meter_type_id = %2 ").arg(threshold_uuid).arg((int)type);

    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        //rectMsg = query.lastError().text();
        m_myDb.database().commit();
        return bReturn;
    }
    m_myDb.database().commit();
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertThresholdByDeviceUuid(QString normalString, QString warningString, QString commonString, QString serialString, QString dangerString, QVector<QString> device_uuid, QString threshold_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO threshold VALUE('%1','%2','%3','%4','%5','%6');")
        .arg(threshold_uuid).arg(normalString).arg(warningString).arg(commonString).arg(serialString).arg(dangerString);
    
    m_myDb.database().transaction();

    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        //rectMsg = query.lastError().text();
        m_myDb.database().commit();
        return bReturn;
    }

    sqlString = QString("UPDATE devices SET threshold_filename = '%1' WHERE device_uuid IN (").arg(threshold_uuid);
    for (int i = 0; i < device_uuid.size(); i++)
    {
        if (i != 0)
        {
            sqlString += ", ";
        }
        sqlString += "'" + device_uuid[i] + "'";
    }

    sqlString += ");";

    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        m_myDb.database().rollback();
        //rectMsg = query.lastError().text();
        m_myDb.database().commit();
        return bReturn;
    }
    m_myDb.database().commit();
    return bReturn;
}

// bool LibDLWheelRobotDBOperation::insertThresholdDB(WheelThresholdStruct m_wheelThresholdStruct)
// {
//     QSqlQuery query;
//     bool bReturn = false;
//     QString sqlString = QString("INSERT INTO inspect_result VALUE('%1','%2','%3','%4','%5','%6','%7','%8','%9','%10','%11','%12');")
// //        .arg(m_wheelThresholdStruct.threshold_id)
//         .arg(m_wheelThresholdStruct.threshold_type)
// //        .arg(m_wheelThresholdStruct.thre_normal_bottom)
// //        .arg(m_wheelThresholdStruct.thre_normal_top)
//         .arg(m_wheelThresholdStruct.thre_earlywarn_bottom)
//         .arg(m_wheelThresholdStruct.thre_earlywarn_top)
//         .arg(m_wheelThresholdStruct.thre_generalwarn_bottom)
//         .arg(m_wheelThresholdStruct.thre_generalwarn_top)
//         .arg(m_wheelThresholdStruct.thre_severitywarn_bottom)
//         .arg(m_wheelThresholdStruct.thre_severitywarn_top)
//         .arg(m_wheelThresholdStruct.thre_dangerwarn_bottom)
//         .arg(m_wheelThresholdStruct.thre_dangerwarn_top);
//     bReturn = querySqlString(sqlString, query);
//     return bReturn;
// }
// 
// bool LibDLWheelRobotDBOperation::updataThresholdDB(WheelThresholdStruct m_wheelThresholdStruct)
// {
//     QSqlQuery query;
//     bool bReturn = false;
//     QString sqlString = QString("UPDATE inspect_result SET threshold_type = '%1', thre_normal_bottom = '%2', thre_normal_top = '%3', thre_earlywarn_bottom = '%4', thre_earlywarn_top = '%5'，thre_generalwarn_bottom = '%6', thre_generalwarn_top = '%7', thre_severitywarn_bottom = '%8', thre_severitywarn_top = '%9',thre_dangerwarn_bottom = '%10', thre_dangerwarn_top = '%11' where threshold_id = '%12';")
//         .arg(m_wheelThresholdStruct.threshold_type)
//         //        .arg(m_wheelThresholdStruct.thre_normal_bottom)
//         //        .arg(m_wheelThresholdStruct.thre_normal_top)
//         .arg(m_wheelThresholdStruct.thre_earlywarn_bottom)
//         .arg(m_wheelThresholdStruct.thre_earlywarn_top)
//         .arg(m_wheelThresholdStruct.thre_generalwarn_bottom)
//         .arg(m_wheelThresholdStruct.thre_generalwarn_top)
//         .arg(m_wheelThresholdStruct.thre_severitywarn_bottom)
//         .arg(m_wheelThresholdStruct.thre_severitywarn_top)
//         .arg(m_wheelThresholdStruct.thre_dangerwarn_bottom)
//         .arg(m_wheelThresholdStruct.thre_dangerwarn_top);
// //        .arg(m_wheelThresholdStruct.threshold_id);
//     bReturn = querySqlString(sqlString, query);
//     return bReturn;
// }

bool LibDLWheelRobotDBOperation::deleteThresholdDB(QString m_threshold_id)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM threshold where threshold_id='%1';").arg(m_threshold_id);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getThresholdeByMeterType(QString &normalString, QString &warningString, QString &commonString, QString &serialString, QString &dangerString, WheelRobotMeterType type, QString &threshold_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelTaskEditStruct wheelTaskEditStru;
    QString sqlString = QString("SELECT * FROM threshold JOIN meter_type WHERE meter_type.threshold_filename = threshold.threshold_uuid AND meter_type.meter_type_id = %1;")
        .arg((int)type);
    bReturn = querySqlString(sqlString, query);

    while (query.next())
    {
        int i = 0;
        threshold_uuid = query.value(i++).toString();
        normalString = query.value(i++).toString();
        warningString = query.value(i++).toString();
        commonString = query.value(i++).toString();
        serialString = query.value(i++).toString();
        dangerString = query.value(i++).toString();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getThresholdeByDeviceUuid(QString &normalString, QString &warningString, QString &commonString, QString &serialString, QString &dangerString, QString device_uuid, QString &threshold_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelTaskEditStruct wheelTaskEditStru;
    QString sqlString = QString("SELECT * FROM threshold JOIN devices WHERE devices.threshold_filename = threshold.threshold_uuid AND devices.device_uuid = '%1';")
        .arg(device_uuid);
    bReturn = querySqlString(sqlString, query);

    while (query.next())
    {
        int i = 0;
        threshold_uuid = query.value(i++).toString();
        normalString = query.value(i++).toString();
        warningString = query.value(i++).toString();
        commonString = query.value(i++).toString();
        serialString = query.value(i++).toString();
        dangerString = query.value(i++).toString();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertTaskEditDB(WheelTaskEditStruct m_wheelTaskEditStruct, QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO task_edit VALUE('%1','%2','%3','%4');")
        .arg(m_wheelTaskEditStruct.task_edit_uuid)
        .arg(m_wheelTaskEditStruct.task_edit_name)
        .arg(m_wheelTaskEditStruct.task_edit_date)
        .arg((int)m_wheelTaskEditStruct.task_edit_type_id);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        retMsg = query.lastError().text();
        return bReturn;
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteTaskEditDB(QString m_task_edit_uuid, QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM task_edit where task_edit_uuid='%1';")
        .arg(m_task_edit_uuid);
    bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		retMsg = query.lastError().text();
	}
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataTaskEditDB(WheelTaskEditStruct m_wheelTaskEditStruct, QString &retMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE task_edit SET task_edit_name = '%1',task_edit_date = '%2', task_edit_type_id = '%3' WHERE task_edit_uuid = '%4';")
        .arg(m_wheelTaskEditStruct.task_edit_name)
        .arg(m_wheelTaskEditStruct.task_edit_date)
        .arg((int)m_wheelTaskEditStruct.task_edit_type_id)
        .arg(m_wheelTaskEditStruct.task_edit_uuid);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        retMsg = query.lastError().text();
        return bReturn;
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::importTaskEditDB(QString c_task_edit_uuid_old, WheelTaskEditStruct stru, QString &Msg)
{
	QSqlQuery query;
	bool bReturn = false;
	m_myDb.database().transaction();
	QString sqlString = QString("INSERT INTO task_edit VALUE('%1','%2','%3','%4');")
		.arg(stru.task_edit_uuid)
		.arg(stru.task_edit_name)
		.arg(stru.task_edit_date)
		.arg((int)stru.task_edit_type_id);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		Msg = query.lastError().text();
		m_myDb.database().rollback();
		return bReturn;
	}

	sqlString = QString("INSERT INTO task_devices(task_edit_uuid, device_uuid)(SELECT '%1', device_uuid FROM task_devices WHERE task_edit_uuid = '%2');")
		.arg(stru.task_edit_uuid).arg(c_task_edit_uuid_old);
	bReturn = querySqlString(sqlString, query);

	if (!bReturn)
	{
		Msg = Msg + query.lastError().text();
		m_myDb.database().rollback();
		m_myDb.database().commit();
		return bReturn;
	}
	m_myDb.database().commit();
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getTaskEditListDB(int m_count, int m_showCount, WheelTaskAdminType m_task_edit_type_id, QList<WheelTaskEditStruct> &m_wheelTaskEditStruct)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelTaskEditStruct wheelTaskEditStru;
    QString sqlString = QString("SELECT * FROM task_edit WHERE task_edit_type_id='%1' ORDER BY task_edit_date DESC LIMIT %2,%3;")
        .arg((int)m_task_edit_type_id).arg(m_count).arg(m_showCount);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        wheelTaskEditStru.task_edit_uuid = query.value(i++).toString();
        wheelTaskEditStru.task_edit_name = query.value(i++).toString();
        wheelTaskEditStru.task_edit_date = query.value(i++).toString().replace(QRegExp("T"),"_");
        wheelTaskEditStru.task_edit_type_id = (WheelTaskAdminType)query.value(i++).toInt();
        m_wheelTaskEditStruct.append(wheelTaskEditStru);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getTaskEditPageDB(int &m_count, WheelTaskAdminType m_task_edit_type_id)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelTaskEditStruct wheelTaskEditStru;
    QString sqlString = QString("SELECT COUNT(*) FROM task_edit where task_edit_type_id='%1';")
        .arg((int)m_task_edit_type_id);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_count = query.value(0).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getTaskEditDB(WheelTaskEditStruct & m_taskEditStruct, QString m_edit_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT * FROM task_edit WHERE task_edit_uuid='%1';")
        .arg(m_edit_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        m_taskEditStruct.task_edit_uuid = query.value(i++).toString();
        m_taskEditStruct.task_edit_name = query.value(i++).toString();
        m_taskEditStruct.task_edit_date = query.value(i++).toString();
        m_taskEditStruct.task_edit_type_id = (WheelTaskAdminType)query.value(i++).toInt();
    }
    return bReturn;
    return false;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceAlarmSearchDB(int &m_count, int m_showCount, WheelPatrolParameter m_wheelPatrolPara, QList<DeviceAlarmSearchStruct> &m_deviceAlarmSearchStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QString Device_UUid = QString("");
    if (m_wheelPatrolPara.m_device_uuid.size() == 0)
    {
        Device_UUid = QString("inspect_result.device_uuid like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_device_uuid.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_device_uuid.size() - 1))
            {
                Device_UUid = Device_UUid + QString("inspect_result.device_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_uuid[i]);
            }
            else
            {
                Device_UUid = Device_UUid + QString("inspect_result.device_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_uuid[i]);
            }

        }
    }
    
    QString Device_Area_UUid = QString("");
    if (m_wheelPatrolPara.m_device_area_uuid.size() == 0)
    {
        Device_Area_UUid = QString("devices.device_area_uuid like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_device_area_uuid.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_device_area_uuid.size() - 1))
            {
                Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
            }
            else
            {
                Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
            }

        }
    }

    QString Device_Type_UUid = QString("");
    if (m_wheelPatrolPara.m_device_type_uuid.size() == 0)
    {
        Device_Type_UUid = QString("devices.device_type_uuid like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_device_type_uuid.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_device_type_uuid.size() - 1))
            {
                Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
            }
            else
            {
                Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
            }

        }
    }
    
    QString Recognition_Type_UUid = QString("");
    if (m_wheelPatrolPara.m_recognition_type_uuid.size() == 0)
    {
        Recognition_Type_UUid = QString("devices.recognition_type_id like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_recognition_type_uuid.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_recognition_type_uuid.size() - 1))
            {
                Recognition_Type_UUid = Recognition_Type_UUid + QString("devices.recognition_type_id = '%1' ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
            }
            else
            {
                Recognition_Type_UUid = Recognition_Type_UUid + QString("devices.recognition_type_id = '%1' OR ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
            }

        }
    }

    QString Alarm_Level_Id = QString("");
    if (m_wheelPatrolPara.m_alarm_level_id.size() == 0)
    {
        Alarm_Level_Id = QString("inspect_result.alarm_level_id like '%%'");
    }
    else
    {
        for (int i = 0; i < m_wheelPatrolPara.m_alarm_level_id.size(); i++)
        {
            if (i == (m_wheelPatrolPara.m_alarm_level_id.size() - 1))
            {
                Alarm_Level_Id = Alarm_Level_Id + QString("inspect_result.alarm_level_id = '%1' ").arg(m_wheelPatrolPara.m_alarm_level_id[i]);
            }
            else
            {
                Alarm_Level_Id = Alarm_Level_Id + QString("inspect_result.alarm_level_id = '%1' OR ").arg(m_wheelPatrolPara.m_alarm_level_id[i]);
            }

        }
    }
    if (m_count < 0)
    {
        if (m_wheelPatrolPara.m_start_time.isEmpty())
        {
            QString sqlString = QString("SELECT COUNT(*) FROM recognition_type,device_point_name,task,devices,alarm_level,save_type,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.recognition_type_id=recognition_type.recognition_type_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND (%1) AND (%2) AND (%3) AND (%4) AND (%5);")
                .arg(Device_UUid).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Alarm_Level_Id).arg(Recognition_Type_UUid);
            bReturn = querySqlString(sqlString, query);
            while (query.next())
            {
                m_count = query.value(0).toInt();
            }
        }
        else
        {
            QString sqlString = QString("SELECT COUNT(*) FROM recognition_type,device_point_name,task,devices,alarm_level,save_type,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.recognition_type_id=recognition_type.recognition_type_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND (%1) AND (%2) AND (%3) AND (%4) AND (%5) AND inspect_time>'%6' AND inspect_time<'%7';")
                .arg(Device_UUid).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Alarm_Level_Id).arg(Recognition_Type_UUid).arg(m_wheelPatrolPara.m_start_time).arg(m_wheelPatrolPara.m_stop_time);
            bReturn = querySqlString(sqlString, query);
            while (query.next())
            {
                m_count = query.value(0).toInt();
            }
        }
    }
    else
    {
        if (m_wheelPatrolPara.m_start_time.isEmpty())
        {
            DeviceAlarmSearchStruct deviceAlarmSearch;
            QString sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,alarm_level.alarm_level_name,inspect_result.inspect_time,save_type.save_type_name,inspect_result.inspect_status_id,inspect_result.is_dealed,deal_info.deal_info_uuid,deal_info.dealed_result,deal_info.dealed_status_id FROM recognition_type,device_point_name,task,devices,alarm_level,save_type,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.recognition_type_id=recognition_type.recognition_type_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND (%1) AND (%2) AND (%3) AND (%4) AND(%5) ORDER BY inspect_result.inspect_time DESC LIMIT %6,%7;")
                .arg(Device_UUid).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Alarm_Level_Id).arg(Recognition_Type_UUid).arg(m_count).arg(m_showCount);
            bReturn = querySqlString(sqlString, query);
            while (query.next())
            {
                int i = 0;
                deviceAlarmSearch.task_uuid = query.value(i++).toString();
                deviceAlarmSearch.device_uuid = query.value(i++).toString();
                deviceAlarmSearch.recognition_type_name = query.value(i++).toString();
                QString pointName = query.value(i++).toString();
                QString pointPhase = query.value(i++).toString();
                if (pointName.contains("X", Qt::CaseSensitive))
                {
                    pointName = pointName.replace(QRegExp("X"), pointPhase);
                }
                deviceAlarmSearch.device_point_type_name = pointName;
                deviceAlarmSearch.inspect_result = query.value(i++).toString();
                deviceAlarmSearch.alarm_level_name = query.value(i++).toString();
                deviceAlarmSearch.inspect_time = query.value(i++).toString().replace(QRegExp("T"), "_");
                deviceAlarmSearch.save_type_name = query.value(i++).toString();
                deviceAlarmSearch.inspect_status_id = (WheelInspectStatus)query.value(i++).toInt();
                deviceAlarmSearch.is_dealed = query.value(i++).toInt();

                if (deviceAlarmSearch.is_dealed == 1)
                {
                    deviceAlarmSearch.deal_uuid = query.value(i++).toString();
                    deviceAlarmSearch.deal_result = query.value(i++).toString();
                    deviceAlarmSearch.deal_status_id = (WheelInspectStatus)query.value(i++).toInt();
                }
                m_deviceAlarmSearchStru.append(deviceAlarmSearch);
            }
        }
        else
        {
            DeviceAlarmSearchStruct deviceAlarmSearch;
            QString sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,alarm_level.alarm_level_name,inspect_result.inspect_time,save_type.save_type_name,inspect_result.inspect_status_id,inspect_result.is_dealed,deal_info.deal_info_uuid,deal_info.dealed_result,deal_info.dealed_status_id FROM recognition_type,device_point_name,task,devices,alarm_level,save_type,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.recognition_type_id=recognition_type.recognition_type_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND (%1) AND (%2) AND (%3) AND (%4) AND (%5) AND inspect_time>'%6' AND inspect_time<'%7' ORDER BY inspect_result.inspect_time DESC LIMIT %8,%9;")
                .arg(Device_UUid).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Alarm_Level_Id).arg(Recognition_Type_UUid).arg(m_wheelPatrolPara.m_start_time).arg(m_wheelPatrolPara.m_stop_time).arg(m_count).arg(m_showCount);
            bReturn = querySqlString(sqlString, query);
            while (query.next())
            {
                int i = 0;
                deviceAlarmSearch.task_uuid = query.value(i++).toString();
                deviceAlarmSearch.device_uuid = query.value(i++).toString();
                deviceAlarmSearch.recognition_type_name = query.value(i++).toString();
                QString pointName = query.value(i++).toString();
                QString pointPhase = query.value(i++).toString();
                if (pointName.contains("X", Qt::CaseSensitive))
                {
                    pointName = pointName.replace(QRegExp("X"), pointPhase);
                }
                deviceAlarmSearch.device_point_type_name = pointName;
                deviceAlarmSearch.inspect_result = query.value(i++).toString();
                deviceAlarmSearch.alarm_level_name = query.value(i++).toString();
                deviceAlarmSearch.inspect_time = query.value(i++).toString().replace(QRegExp("T"), "_");
                deviceAlarmSearch.save_type_name = query.value(i++).toString();
                deviceAlarmSearch.inspect_status_id = (WheelInspectStatus)query.value(i++).toInt();
                deviceAlarmSearch.is_dealed = query.value(i++).toInt();

                if (deviceAlarmSearch.is_dealed == 1)
                {
                    deviceAlarmSearch.deal_uuid = query.value(i++).toString();
                    deviceAlarmSearch.deal_result = query.value(i++).toString();
                    deviceAlarmSearch.deal_status_id = (WheelInspectStatus)query.value(i++).toInt();
                }
                m_deviceAlarmSearchStru.append(deviceAlarmSearch);
            }
        }
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getTaskDeviceDataDB(QString m_task_edit_uuid, QList<QString> &m_device_uuid)
{
    QSqlQuery query;
    QString taskDevice="";
    bool bReturn = false;
    QString sqlString = QString("SELECT * FROM task_devices where task_edit_uuid='%1';")
        .arg(m_task_edit_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        taskDevice = query.value(1).toString();
        m_device_uuid.append(taskDevice);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertTaskDevicesDB(QString m_task_edit_uuid, QList<QString> m_device_uuid, QString &rectMsg)
{
    QSqlQuery query;
    bool bReturn = false;
//    query.exec("START TRANSACTION");
    int strLength = m_device_uuid.size();
    int insertCount = 0;
    if (strLength % 2000 != 0)
    {
        insertCount = strLength / 2000 + 1;
    }
    else
    {
        insertCount = strLength / 2000;
    }

    for (int j = 0; j < insertCount; j++)
    {
        QString sqlString = QString("INSERT INTO task_devices ");
        for (int i = j * 2000; i < (j + 1) * 2000; i++)
        {
            qDebug() << "正在插入数据库表task_devices:当前第" << i+1 << "条！";
            QString sqlString_2;
            if (i == j * 2000)
            {
                sqlString_2 = QString("select '%1','%2'").arg(m_task_edit_uuid).arg(m_device_uuid[i]);
            }
            else
            {
                sqlString_2 = QString("union all select '%1','%2'").arg(m_task_edit_uuid).arg(m_device_uuid[i]);
            }

            sqlString = sqlString + sqlString_2;

            if (i == (j + 1) * 2000 - 1)
            {
                sqlString = sqlString + ";";
            }
            if (i == strLength - 1)
            {
                sqlString = sqlString + ";";
                break;
            }
        }
        bReturn = querySqlString(sqlString, query);
    }
    if (!bReturn)
    {
        rectMsg = query.lastError().text();
        return bReturn;
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteTaskDevicesDB(QString m_task_edit_uuid, QString &rectMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM task_devices where task_edit_uuid='%1';")
        .arg(m_task_edit_uuid);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        rectMsg = query.lastError().text();
        return bReturn;
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataTaskDevicesDB(QString m_task_edit_uuid, QList<QString> m_add_device_uuid, QList<QString> m_delete_device_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    for (int i = 0; i < m_add_device_uuid.size(); i++)
    {
        QString sqlString = QString("INSERT INTO task_devices VALUE('%1','%2');")
            .arg(m_task_edit_uuid).arg(m_add_device_uuid.at(i));
        bReturn = querySqlString(sqlString, query);
    }

    for (int i = 0; i < m_delete_device_uuid.size(); i++)
    {
        QString sqlString = QString("DELETE FROM task_devices where task_edit_uuid='%1' and device_uuid='%2';")
            .arg(m_task_edit_uuid).arg(m_delete_device_uuid.at(i));
        bReturn = querySqlString(sqlString, query);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertTaskEditAndInsertTaskDevicesDB(WheelTaskEditStruct edit, QList<QString> m_device_uuid, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;

	m_myDb.database().transaction();

	QString sqlString = QString("INSERT INTO task_edit VALUE('%1','%2','%3','%4');")
		.arg(edit.task_edit_uuid)
		.arg(edit.task_edit_name)
		.arg(edit.task_edit_date)
		.arg((int)edit.task_edit_type_id);

	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		m_myDb.database().rollback();
		retMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}

	int strLength = m_device_uuid.size();
	int insertCount = 0;
	if (strLength % 2000 != 0)
	{
		insertCount = strLength / 2000 + 1;
	}
	else
	{
		insertCount = strLength / 2000;
	}

	for (int j = 0; j < insertCount; j++)
	{
		QString sqlString = QString("INSERT INTO task_devices ");
		for (int i = j * 2000; i < (j + 1) * 2000; i++)
		{
			//qDebug() << "正在插入数据库表task_devices:当前第" << i + 1 << "条！";
			QString sqlString_2;
			if (i == j * 2000)
			{
				sqlString_2 = QString("select '%1','%2'").arg(edit.task_edit_uuid).arg(m_device_uuid[i]);
			}
			else
			{
				sqlString_2 = QString("union all select '%1','%2'").arg(edit.task_edit_uuid).arg(m_device_uuid[i]);
			}

			sqlString = sqlString + sqlString_2;

			if (i == (j + 1) * 2000 - 1)
			{
				sqlString = sqlString + ";";
			}
			if (i == strLength - 1)
			{
				sqlString = sqlString + ";";
				break;
			}
		}
		bReturn = querySqlString(sqlString, query);
	}
	if (!bReturn)
	{
		m_myDb.database().rollback();
		retMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}

	m_myDb.database().commit();
	return bReturn;
}

bool LibDLWheelRobotDBOperation::updataTaskEditAndUpdataTaskDevicesDB(WheelTaskEditStruct edit, QList<QString> m_device_uuid, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;

	m_myDb.database().transaction();

	QString sqlString = QString("UPDATE task_edit SET task_edit_name = '%1',task_edit_date = '%2', task_edit_type_id = '%3' WHERE task_edit_uuid = '%4';")
		.arg(edit.task_edit_name)
		.arg(edit.task_edit_date)
		.arg((int)edit.task_edit_type_id)
		.arg(edit.task_edit_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		m_myDb.database().rollback();
		retMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}

	sqlString = QString("DELETE FROM task_devices where task_edit_uuid='%1';")
		.arg(edit.task_edit_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		m_myDb.database().rollback();
		retMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}

	int strLength = m_device_uuid.size();
	int insertCount = 0;
	if (strLength % 2000 != 0)
	{
		insertCount = strLength / 2000 + 1;
	}
	else
	{
		insertCount = strLength / 2000;
	}

	for (int j = 0; j < insertCount; j++)
	{
		QString sqlString = QString("INSERT INTO task_devices ");
		for (int i = j * 2000; i < (j + 1) * 2000; i++)
		{
			qDebug() << "正在插入数据库表task_devices:当前第" << i + 1 << "条！";
			QString sqlString_2;
			if (i == j * 2000)
			{
				sqlString_2 = QString("select '%1','%2'").arg(edit.task_edit_uuid).arg(m_device_uuid[i]);
			}
			else
			{
				sqlString_2 = QString("union all select '%1','%2'").arg(edit.task_edit_uuid).arg(m_device_uuid[i]);
			}

			sqlString = sqlString + sqlString_2;

			if (i == (j + 1) * 2000 - 1)
			{
				sqlString = sqlString + ";";
			}
			if (i == strLength - 1)
			{
				sqlString = sqlString + ";";
				break;
			}
		}
		bReturn = querySqlString(sqlString, query);
	}

	if (!bReturn)
	{
		m_myDb.database().rollback();
		retMsg = query.lastError().text();
		m_myDb.database().commit();
		return bReturn;
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelPatrolResultDB(int m_page, int m_showCount, QList<WheelPatrolResultStruct> &m_wheelPatrolResultStru,QString m_device_uuid,QString m_start_time,QString m_stop_time)
{
    QSqlQuery query;
    WheelPatrolResultStruct wheelPatrolResultStru;
    bool bReturn = false;
    QString sqlString;
    if (m_start_time=="")
    {
        sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,task.task_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_time,inspect_result.inspect_result,save_type.save_type_name FROM inspect_result join save_type join task join device_point_name join devices join device_phase WHERE inspect_result.task_uuid = task.task_uuid and inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and devices.device_phase_id = device_phase.device_phase_id and inspect_result.device_uuid='%1' limit %2,%3;")
            .arg(m_device_uuid).arg(m_page).arg(m_showCount);
    }
    else
    {
        sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,task.task_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_time,inspect_result.inspect_result,save_type.save_type_name FROM inspect_result join save_type join task join device_point_name join devices join device_phase WHERE inspect_result.task_uuid = task.task_uuid and inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and devices.device_phase_id = device_phase.device_phase_id and inspect_result.device_uuid='%1' and inspect_time>'%2' and inspect_time <'%3' limit %4,%5;")
            .arg(m_device_uuid).arg(m_start_time).arg(m_stop_time).arg(m_page).arg(m_showCount);
    }
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        wheelPatrolResultStru.task_uuid = query.value(i++).toString();
        wheelPatrolResultStru.device_uuid = query.value(i++).toString();
        wheelPatrolResultStru.task_name = query.value(i++).toString();
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        wheelPatrolResultStru.device_point_type_name = pointName;
        wheelPatrolResultStru.inspect_time = query.value(i++).toString();
        wheelPatrolResultStru.inspect_result = query.value(i++).toString();
        wheelPatrolResultStru.save_type_name = query.value(i++).toString();
        m_wheelPatrolResultStru.append(wheelPatrolResultStru);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelPatrolResultDB(QList<WheelPatrolResultStruct>& m_wheelPatrolResultStru, QString m_start_time, QString m_stop_time)
{
	QSqlQuery query;
	WheelPatrolResultStruct wheelPatrolResultStru;
	bool bReturn = false;
	QString sqlString;
	if (m_start_time == "")
	{
		sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,task.task_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_time,inspect_result.inspect_result,save_type.save_type_name FROM inspect_result join save_type join task join device_point_name join devices join device_phase WHERE inspect_result.task_uuid = task.task_uuid and inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and devices.device_phase_id = device_phase.device_phase_id;");
	}
	else
	{
		sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,task.task_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_time,inspect_result.inspect_result,save_type.save_type_name FROM inspect_result join save_type join task join device_point_name join devices join device_phase WHERE inspect_result.task_uuid = task.task_uuid and inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and devices.device_phase_id = device_phase.device_phase_id and inspect_time>'%1' and inspect_time <'%2';")
			.arg(m_start_time).arg(m_stop_time);
	}
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		wheelPatrolResultStru.task_uuid = query.value(i++).toString();
		wheelPatrolResultStru.device_uuid = query.value(i++).toString();
		wheelPatrolResultStru.task_name = query.value(i++).toString();
		QString pointName = query.value(i++).toString();
		QString pointPhase = query.value(i++).toString();
		if (pointName.contains("X", Qt::CaseSensitive))
		{
			pointName = pointName.replace(QRegExp("X"), pointPhase);
		}
		wheelPatrolResultStru.device_point_type_name = pointName;
		wheelPatrolResultStru.inspect_time = query.value(i++).toString();
		wheelPatrolResultStru.inspect_result = query.value(i++).toString();
		wheelPatrolResultStru.save_type_name = query.value(i++).toString();
		m_wheelPatrolResultStru.append(wheelPatrolResultStru);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelPatrolResultCountDB(int &m_count, QString m_device_uuid, QString m_start_time, QString m_stop_time)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString;
    if (m_start_time.isEmpty())
    {
        sqlString = QString("SELECT count(*) FROM inspect_result join save_type join task join device_point_name join devices WHERE inspect_result.task_uuid = task.task_uuid and inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and inspect_result.device_uuid='%1';")
            .arg(m_device_uuid);
    }
    else
    {
        sqlString = QString("SELECT count(*) FROM inspect_result join save_type join task join device_point_name join devices WHERE inspect_result.task_uuid = task.task_uuid and inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and inspect_result.device_uuid='%1' and inspect_time>'%2' and inspect_time<'%3';")
            .arg(m_device_uuid).arg(m_start_time).arg(m_stop_time);
    }
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_count = query.value(0).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelPatrolResultCompareDB(int m_count, int m_showCount, QList<WheelPatrolResultCompareStruct> &m_wheelPatrolResultCompareStru, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelPatrolResultCompareStruct wheelPatrolResultCom;
    QString sqlString;
    QString DeviceUUid="";
    if (m_device_uuid.size() == 0)
    {
        DeviceUUid = QString("devices.device_uuid LIKE '%%'");
    }
    else
    {
        for (int i = 0; i < m_device_uuid.size(); i++)
        {
            if (i == (m_device_uuid.size() - 1))
            {
                DeviceUUid = DeviceUUid + QString("devices.device_uuid = '%1' ").arg(m_device_uuid[i]);
            }
            else
            {
                DeviceUUid = DeviceUUid + QString("devices.device_uuid = '%1' OR ").arg(m_device_uuid[i]);
            }

        }
    }


	QString SaveTypeId = "";
	if (m_save_type_id.size() == 0)
	{
		SaveTypeId = QString("devices.save_type_id LIKE '%%'");
	}
	else
	{
		for (int i = 0; i < m_save_type_id.size(); i++)
		{
			if (i == (m_save_type_id.size() - 1))
			{
				SaveTypeId = SaveTypeId + QString("devices.save_type_id = '%1' ").arg(m_save_type_id[i]);
			}
			else
			{
				SaveTypeId = SaveTypeId + QString("devices.save_type_id = '%1' OR ").arg(m_save_type_id[i]);
			}

		}
	}

    if (m_start_time=="")
    {
        sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,device_point_name.device_point_type_name,inspect_result.inspect_time,inspect_result.inspect_result,save_type.save_type_name FROM inspect_result join save_type join device_point_name join devices WHERE inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and (%1) and (%2) ORDER BY inspect_result.inspect_time desc limit %3,%4;")
			.arg(DeviceUUid).arg(SaveTypeId).arg(m_count).arg(m_showCount);
    }
    else
    {
        sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,device_point_name.device_point_type_name,inspect_result.inspect_time,inspect_result.inspect_result,save_type.save_type_name FROM inspect_result join save_type join device_point_name join devices WHERE inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and (%1) and (%2) and inspect_time>'%3' and inspect_time<'%4' ORDER BY inspect_result.inspect_time desc limit %5,%6;")
			.arg(DeviceUUid).arg(SaveTypeId).arg(m_start_time).arg(m_stop_time).arg(m_count).arg(m_showCount);
    }
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        wheelPatrolResultCom.task_uuid = query.value(0).toString();
        wheelPatrolResultCom.device_uuid = query.value(1).toString();
        wheelPatrolResultCom.device_point_type_name = query.value(2).toString();
        wheelPatrolResultCom.inspect_time = query.value(3).toString().replace("T","_");
        wheelPatrolResultCom.inspect_result = query.value(4).toString();
        wheelPatrolResultCom.save_type_name = query.value(5).toString();
        m_wheelPatrolResultCompareStru.append(wheelPatrolResultCom);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelPatrolResultCompareCountDB(int &m_count, QStringList m_device_uuid, QStringList m_save_type_id, QString m_start_time, QString m_stop_time)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString;
    QString DeviceUUid = "";
    if (m_device_uuid.size() == 0)
    {
        DeviceUUid = QString("devices.device_uuid LIKE '%%'");
    }
    else
    {
        for (int i = 0; i < m_device_uuid.size(); i++)
        {
            if (i == (m_device_uuid.size() - 1))
            {
                DeviceUUid = DeviceUUid + QString("devices.device_uuid = '%1' ").arg(m_device_uuid[i]);
            }
            else
            {
                DeviceUUid = DeviceUUid + QString("devices.device_uuid = '%1' OR ").arg(m_device_uuid[i]);
            }

        }
    }
	QString SaveTypeId = "";
	if (m_save_type_id.size() == 0)
	{
		SaveTypeId = QString("devices.save_type_id LIKE '%%'");
	}
	else
	{
		for (int i = 0; i < m_save_type_id.size(); i++)
		{
			if (i == (m_save_type_id.size() - 1))
			{
				SaveTypeId = SaveTypeId + QString("devices.save_type_id = '%1' ").arg(m_save_type_id[i]);
			}
			else
			{
				SaveTypeId = SaveTypeId + QString("devices.save_type_id = '%1' OR ").arg(m_save_type_id[i]);
			}

		}
	}
    if (m_start_time=="")
    {
        sqlString = QString("SELECT COUNT(*) FROM inspect_result join save_type join device_point_name join devices WHERE inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and (%1) and (%2); ")
			.arg(DeviceUUid).arg(SaveTypeId);
    }
    else
    {
        sqlString = QString("SELECT COUNT(*) FROM inspect_result join save_type join device_point_name join devices WHERE inspect_result.device_uuid = devices.device_uuid and devices.save_type_id = save_type.save_type_id and devices.device_point_type_uuid = device_point_name.device_point_type_uuid and (%1) and (%2) and inspect_time>'%3' and inspect_time<'%4';")
			.arg(DeviceUUid).arg(SaveTypeId).arg(m_start_time).arg(m_stop_time);
    }
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_count = query.value(0).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getTaskListShowDB(QList<WheelTaskListShowStruct> &m_wheelTaskListShowStru, WheelTaskListSearchIndex m_index, QList<WheelTaskTemplateList> &m_taskTemplateList)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString;
    WheelTaskListShowStruct wheelTaskListShowStruct;
    QString TaskStatus = "";
    if (m_index.m_task_status_id.size() == 0)
    {
        TaskStatus = QString("task_status.task_status_id like '%%'");
    }
    else
    {
        for (int i = 0; i < m_index.m_task_status_id.size(); i++)
        {
            if (i == (m_index.m_task_status_id.size() - 1))
            {
                TaskStatus = TaskStatus + QString("task_status.task_status_id = '%1' ").arg(m_index.m_task_status_id[i]);
            }
            else
            {
                TaskStatus = TaskStatus + QString("task_status.task_status_id = '%1' OR ").arg(m_index.m_task_status_id[i]);
            }

        }
    }

    if (m_index.m_start_time=="")
    {
//         sqlString = QString("SELECT task.task_edit_uuid, task.task_uuid,task_edit.task_edit_type_id, task.task_name, task.task_start_time, task_status.task_status_name from task,task_status,task_edit where task.task_edit_uuid=task_edit.task_edit_uuid and task.task_status_id = task_status.task_status_id and (%1) and task.task_name like '%%2%' ORDER BY task.task_start_time DESC;")
//             .arg(TaskStatus).arg(m_index.m_task_name);
        sqlString = QString("SELECT task.task_edit_uuid, task.task_uuid,task_edit.task_edit_type_id, task.task_name, task.task_start_time, task_status.task_status_name from task,task_status,task_edit where task.task_edit_uuid=task_edit.task_edit_uuid AND task.task_status_id = task_status.task_status_id and (%1) and task.task_name like '%%2%' ORDER BY task.task_start_time DESC;")
            .arg(TaskStatus).arg(m_index.m_task_name);
    }
    else
    {
//         sqlString = QString("SELECT task.task_edit_uuid, task.task_uuid,task_edit.task_edit_type_id, task.task_name, task.task_start_time, task_status.task_status_name from task,task_status,task_edit where task.task_edit_uuid=task_edit.task_edit_uuid and task.task_status_id = task_status.task_status_id and (%1) and task.task_name like '%%2%' and task.task_start_time>'%3' and task.task_start_time<'%4' ORDER BY task.task_start_time DESC;")
//             .arg(TaskStatus).arg(m_index.m_task_name).arg(m_index.m_start_time).arg(m_index.m_stop_time);
        sqlString = QString("SELECT task.task_edit_uuid, task.task_uuid,task_edit.task_edit_type_id, task.task_name, task.task_start_time, task_status.task_status_name from task,task_status,task_edit where task.task_edit_uuid=task_edit.task_edit_uuid AND task.task_status_id = task_status.task_status_id and (%1) and task.task_name like '%%2%' and task.task_start_time>'%3' and task.task_start_time<'%4' ORDER BY task.task_start_time DESC;")
            .arg(TaskStatus).arg(m_index.m_task_name).arg(m_index.m_start_time).arg(m_index.m_stop_time);
    }
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        wheelTaskListShowStruct.task_edit_uuid = query.value(i++).toString();
        wheelTaskListShowStruct.task_template_uuid = "";
        wheelTaskListShowStruct.task_uuid = query.value(i++).toString();
        wheelTaskListShowStruct.task_edit_type_id = (WheelTaskAdminType)query.value(i++).toInt();
        wheelTaskListShowStruct.task_name = query.value(i++).toString();
        wheelTaskListShowStruct.task_start_time = query.value(i++).toString().replace(QRegExp("T"), " ");
        wheelTaskListShowStruct.task_status_name = query.value(i++).toString();
        m_wheelTaskListShowStru.append(wheelTaskListShowStruct);
    }

	WheelTaskTemplateList tasktemp;
	QString sqlString2 = QString("SELECT task_template.task_template_uuid,task_template.task_edit_uuid,task_template.task_template_name,task_template.task_start_date,task_template.task_start_time,task_template.task_repeat_duration,task_template.task_loop_type_id,task_status.task_status_name,task_edit_type.task_edit_type_id FROM task_template,task_status,task_edit_type,task_edit WHERE task_template.task_status_id=task_status.task_status_id AND task_template.task_edit_uuid=task_edit.task_edit_uuid AND task_edit.task_edit_type_id=task_edit_type.task_edit_type_id ORDER BY task_template.task_loop_type_id;");
	bReturn = querySqlString(sqlString2, query);
	while (query.next())
	{
		int i = 0;
		tasktemp.task_template_uuid = query.value(i++).toString();
		tasktemp.task_edit_uuid = query.value(i++).toString();
		tasktemp.task_template_name = query.value(i++).toString();
		tasktemp.task_start_date = query.value(i++).toString();
		tasktemp.task_start_time = query.value(i++).toString();
		tasktemp.task_repeat_duration = query.value(i++).toInt();
		tasktemp.task_loop_type_id = query.value(i++).toInt();
		tasktemp.task_status_name = query.value(i++).toString();
		tasktemp.task_edit_type_id = query.value(i++).toInt();
		m_taskTemplateList.append(tasktemp);
	}
    return bReturn;
}

// bool LibDLWheelRobotDBOperation::getTaskListShowCountDB(int &m_count, WheelTaskListSearchIndex m_index)
// {
//     QSqlQuery query;
//     bool bReturn = false;
//     QString sqlString;
// 
//     QString TaskStatus = "";
//     if (m_index.m_task_status_id.size() == 0)
//     {
//         TaskStatus = QString("task_status_id like '%%'");
//     }
//     else
//     {
//         for (int i = 0; i < m_index.m_task_status_id.size(); i++)
//         {
//             if (i == (m_index.m_task_status_id.size() - 1))
//             {
//                 TaskStatus = TaskStatus + QString("task_status.task_status_id = '%1' ").arg(m_index.m_task_status_id[i]);
//             }
//             else
//             {
//                 TaskStatus = TaskStatus + QString("task_status.task_status_id = '%1' OR ").arg(m_index.m_task_status_id[i]);
//             }
// 
//         }
//     }
// 
//     if (m_index.m_start_time == "")
//     {
//         sqlString = QString("SELECT count(*) from task_edit join task_template join task join task_status where task.task_template_uuid = task_template.task_template_uuid and task.task_edit_uuid = task_edit.task_edit_uuid and task.task_status_id = task_status.task_status_id and (%1) and task.task_name like '%%2%';")
//             .arg(TaskStatus).arg(m_index.m_task_name);
//     }
//     else
//     {
//         sqlString = QString("SELECT count(*) from task_edit join task_template join task join task_status where task.task_template_uuid = task_template.task_template_uuid and task.task_edit_uuid = task_edit.task_edit_uuid and task.task_status_id = task_status.task_status_id and (%1) and task.task_name like '%%2%' and task.task_start_time>'%3' and task.task_end_time<'%4';")
//             .arg(TaskStatus).arg(m_index.m_task_name).arg(m_index.m_start_time).arg(m_index.m_stop_time);
//     }
//     bReturn = querySqlString(sqlString, query);
//     while (query.next())
//     {
//         m_count = m_count + query.value(0).toInt();
//     }
// 
//     if (m_index.m_start_time == "")
//     {
//         sqlString = QString("SELECT count(*) from task_template join task_status join task_edit where task_edit.task_edit_uuid = task_template.task_edit_uuid and task_template.task_status_id = task_status.task_status_id and (%1) and task_template.task_template_name like '%%2%';")
//             .arg(TaskStatus).arg(m_index.m_task_name);
//     }
//     else
//     {
//         sqlString = QString("SELECT count(*) from task_template join task_status join task_edit where task_edit.task_edit_uuid = task_template.task_edit_uuid and task_template.task_status_id = task_status.task_status_id and (%1) and task_template.task_template_name like '%%2%' and task_template.task_start_date>'%3' and task_template.task_start_date<'%4';")
//             .arg(TaskStatus).arg(m_index.m_task_name).arg(m_index.m_start_time).arg(m_index.m_stop_time);
//     }
//     bReturn = querySqlString(sqlString, query);
//     while (query.next())
//     {
//         m_count = m_count + query.value(0).toInt();
//     }
//     return bReturn;
// }

bool LibDLWheelRobotDBOperation::getWheelDeviceAlarmColorDataDB(WheelRobortDevicesAlarmColorStruct &m_devicesAlarmColorStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT `t`.`alarm_level_id` AS `alarm_level_id`,`t`.`device_uuid` AS `device_uuid` FROM `inspect_result` `t` WHERE (`t`.`alarm_level_id` = (SELECT max(`inspect_result`.`alarm_level_id`) FROM inspect_result WHERE (`inspect_result`.`device_uuid` = `t`.`device_uuid`))) GROUP BY `t`.`device_uuid`;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        if (query.value(0).toInt() == 1)
        {
            m_devicesAlarmColorStru.alarmColor_Green.append(query.value(1).toString());
        }
        else if (query.value(0).toInt() == 2)
        {
            m_devicesAlarmColorStru.alarmColor_Blue.append(query.value(1).toString());
        }
        else if (query.value(0).toInt() == 3)
        {
            m_devicesAlarmColorStru.alarmColor_Yellow.append(query.value(1).toString());
        }
        else if (query.value(0).toInt() == 4)
        {
            m_devicesAlarmColorStru.alarmColor_Orange.append(query.value(1).toString());
        }
        else
        {
            m_devicesAlarmColorStru.alarmColor_Red.append(query.value(1).toString());
        }
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelIntervalShowDB(QList<WheelRobortIntervalShowStruct> &m_wheelRobortIntervalShowStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QList<QString> voltage_id;
    QString sqlString1 = QString("SELECT voltage_level.voltage_level_id FROM voltage_level ORDER BY voltage_level_id;");
    bReturn = querySqlString(sqlString1, query);
    while (query.next())
    {
        voltage_id.append(query.value(0).toString());
    }
    for (int i = 0; i < voltage_id.size(); i++)
    {
        WheelRobortIntervalShowStruct intervalShow;
        int Level_1 = 1;
        int Level_2 = 1;
        QString sqlString2 = QString("SELECT voltage_level.voltage_level_id,voltage_level.voltage_level_name,equipment_interval.equipment_interval_uuid,equipment_interval.equipment_interval_name,max(inspect_result.alarm_level_id) FROM equipment_interval JOIN devices JOIN inspect_result JOIN voltage_level WHERE inspect_result.device_uuid=devices.device_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND voltage_level.voltage_level_id=equipment_interval.voltage_level_id AND voltage_level.voltage_level_id='%1' GROUP BY equipment_interval.equipment_interval_uuid;")
            .arg(voltage_id.at(i));
        bReturn = querySqlString(sqlString2, query);
        while (query.next())
        {
            intervalShow.voltage_level_id = query.value(0).toString();
            intervalShow.voltage_level_name = query.value(1).toString();
            WheelRobortEquiInterAlarmStruct equipmentInterva;
            equipmentInterva.equipment_interval_uuid = query.value(2).toString();
            equipmentInterva.equipment_interval_name = query.value(3).toString();
            equipmentInterva.IntervalAlarmLevel = (DeviceAlarmLevel)query.value(4).toInt();
            Level_1 = query.value(4).toInt();

            if (Level_2 < Level_1)
            {
                Level_2 = Level_1;
            }
            intervalShow.equipmentIntervalStru.append(equipmentInterva);
        }
		if (!intervalShow.equipmentIntervalStru.isEmpty())
		{
			intervalShow.VoltageAlarmLevel = DeviceAlarmLevel(Level_2);
			m_wheelRobortIntervalShowStru.append(intervalShow);
		}
        
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceFromIntervalShowDB(QString m_equipment_interval_uuid, QList<WheelRobortDeviceFromIntervalStruct> &m_wrDeviceFromIntervalStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QList<int> voltage_id;
    QString sqlString1 = QString("SELECT devices.device_uuid,device_point_name.device_point_type_name,device_phase.device_phase_name ,max(inspect_result.alarm_level_id) FROM devices join device_phase JOIN device_point_name JOIN inspect_result WHERE inspect_result.device_uuid=devices.device_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.device_point_type_uuid = device_point_name.device_point_type_uuid AND devices.equipment_interval_uuid='%1' GROUP BY devices.device_uuid;")
        .arg(m_equipment_interval_uuid);
    bReturn = querySqlString(sqlString1, query);
    while (query.next())
    {
        int i = 0;
        WheelRobortDeviceFromIntervalStruct m_wrDeviceFInterval;
        m_wrDeviceFInterval.device_uuid = query.value(i++).toString();
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        m_wrDeviceFInterval.device_name = pointName;
        m_wrDeviceFInterval.DeviceAlarmLevel = DeviceAlarmLevel(query.value(i++).toInt());
        m_wrDeviceFromIntervalStru.append(m_wrDeviceFInterval);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceUUidForCheckBoxDB(QString m_choose, QString checkBoxTypeUUid, QList<QString> &m_devices)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select device_uuid from devices where '%1'='%2';")
        .arg(m_choose).arg(checkBoxTypeUUid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_devices.append(query.value(0).toString());
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDealInfoDB(WheelRobortDealInfoStruct m_dealInfoStru, QString &rectMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO deal_info VALUE ('%1', '%2', '%3', '%4', '%5');")
        .arg(m_dealInfoStru.deal_info_uuid)
        .arg(m_dealInfoStru.dealed_info)
        .arg(m_dealInfoStru.dealed_result)
        .arg(m_dealInfoStru.dealed_status_id)
        .arg(m_dealInfoStru.dealed_user);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        rectMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataDealInfoDB(WheelRobortDealInfoStruct m_dealInfoStru, QString & rectMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE deal_info SET dealed_result = '%1',dealed_status_id='%2' where deal_info_uuid='%3';")
        .arg(m_dealInfoStru.dealed_result)
        .arg(m_dealInfoStru.dealed_status_id)
        .arg(m_dealInfoStru.deal_info_uuid);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        rectMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataDealInfoForUserDB(QString deal_task_uuid, QString deal_time, QString deal_info, QString & rectMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("UPDATE deal_info SET dealed_time = '%1',dealed_user='%2' where deal_task_uuid='%3';")
		.arg(deal_time)
		.arg(deal_info)
		.arg(deal_task_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		rectMsg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getUserRole(QString username, QString password, userLoginRetVal &ret)
{
    QSqlQuery query;
    bool bReturn = false;

    if (username.isNull() || password.isNull())
    {
        ret.role = WHEEL_USER_NONE;
        ret.retCode = WHEEL_LOGIN_USERNAME_NOT_EXIST;
        ret.errMsg = "用户为空";
        ROS_ERROR("user login name or password is null");
        return false;
    }

    QString sqlString = QString("SELECT COUNT(*) FROM user_config WHERE user_name='%1';").arg(username);
    bReturn = querySqlString(sqlString, query);
    query.next();
    int count = query.value(0).toInt();
    if (count != 0)
    {
        sqlString = QString("SELECT * FROM user_config WHERE user_name='%1';").arg(username);
        bReturn = querySqlString(sqlString, query);

        query.next();
        QString userPassword = query.value(4).toString();

        if (password.compare(userPassword) == 0)
        {
            ret.role = (WheelUserType)query.value(query.record().indexOf("user_role")).toInt();
            ret.retCode = WHEEL_LOGIN_SUCCESS;
            ret.errMsg = "成功了！";
            return true;
        }
        else
        {
            ret.role = WHEEL_USER_NONE;
            ret.retCode = WHEEL_LOGIN_PASSWORD_INCORRECT;
            ret.errMsg = "密码错误！";
            return false;
        }
    }
    else
    {
        ret.role = WHEEL_USER_NONE;
        ret.retCode = WHEEL_LOGIN_USERNAME_NOT_EXIST;
        ret.errMsg = "用户不存在";
        ROS_ERROR("user does not exists");
        return false;
    }
}

bool LibDLWheelRobotDBOperation::getWheelDeviceAlarmSearchVerifyDB(int m_count, int m_showCount, QList<WheelRobortTaskSearchStruct> &m_wheelRobortTaskSearchStru)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelRobortTaskSearchStruct wheelTaskSearchStru;
    QString sqlString = QString("SELECT task.task_uuid,task_audit.task_audi_status,task.task_name,task_status.task_status_name,task.task_start_time,task.task_end_time,task.task_total_devices,task.task_total_bugs,task.task_total_mistake FROM task JOIN task_audit JOIN task_status WHERE task.task_audit_id=task_audit.task_audit_id AND task.task_status_id=task_status.task_status_id limit %1,%2;")
        .arg(m_count).arg(m_showCount);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        wheelTaskSearchStru.task_uuid = query.value(i++).toString();
        wheelTaskSearchStru.task_audi_status = query.value(i++).toString();
        wheelTaskSearchStru.task_name = query.value(i++).toString();
        wheelTaskSearchStru.task_status_name = query.value(i++).toString();
        wheelTaskSearchStru.task_start_time = query.value(i++).toString().replace("T"," ");
        wheelTaskSearchStru.task_end_time = query.value(i++).toString().replace("T", " ");
        wheelTaskSearchStru.task_total_devices = query.value(i++).toInt();
        wheelTaskSearchStru.task_total_bugs = query.value(i++).toInt();
        wheelTaskSearchStru.task_total_mistake = query.value(i++).toInt();
        wheelTaskSearchStru.task_total_nonrecognition = 0;
        wheelTaskSearchStru.task_total_unusual = 0;
        m_wheelRobortTaskSearchStru.append(wheelTaskSearchStru);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceAlarmSearchVerifyCountDB(int &m_count)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelTaskEditStruct wheelTaskEditStru;
    QString sqlString = QString("SELECT count(*) FROM task JOIN task_audit JOIN task_status WHERE task.task_audit_id=task_audit.task_audit_id AND task.task_status_id=task_status.task_status_id;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_count = query.value(0).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceAlarmForTaskUUidDB(int m_count, int m_showCount, QString m_task_uuid, QString m_start_time, QString m_stop_time,QList<DeviceAlarmSearchStruct> &m_deviceAlarmSearchStru)
{
    QSqlQuery query;
    bool bReturn = false;
    DeviceAlarmSearchStruct deviceAlarmSearch;
	QString sqlString;
	if (m_start_time.isEmpty())
	{
		sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,alarm_level.alarm_level_name,inspect_result.inspect_time,save_type.save_type_name,inspect_result.inspect_status_id,inspect_result.is_dealed,deal_info.deal_info_uuid,deal_info.dealed_result,deal_info.dealed_status_id FROM recognition_type,device_point_name,task,devices,alarm_level,save_type,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.recognition_type_id=recognition_type.recognition_type_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND task.task_uuid='%1' ORDER BY inspect_result.inspect_time DESC limit %2,%3;")
			.arg(m_task_uuid).arg(m_count).arg(m_showCount);
	}
	else
	{
		sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,alarm_level.alarm_level_name,inspect_result.inspect_time,save_type.save_type_name,inspect_result.inspect_status_id,inspect_result.is_dealed,deal_info.deal_info_uuid,deal_info.dealed_result,deal_info.dealed_status_id FROM recognition_type,device_point_name,task,devices,alarm_level,save_type,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.recognition_type_id=recognition_type.recognition_type_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND task.task_uuid='%1' AND inspect_result.inspect_time>'%2' AND inspect_result.inspect_time<'%3' ORDER BY inspect_result.inspect_time DESC;")
			.arg(m_task_uuid).arg(m_start_time).arg(m_stop_time);
	}

    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        deviceAlarmSearch.task_uuid = query.value(i++).toString();
        deviceAlarmSearch.device_uuid = query.value(i++).toString();
        deviceAlarmSearch.recognition_type_name = query.value(i++).toString();
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        deviceAlarmSearch.device_point_type_name = pointName;
        deviceAlarmSearch.inspect_result = query.value(i++).toString();
        deviceAlarmSearch.alarm_level_name = query.value(i++).toString();
        deviceAlarmSearch.inspect_time = query.value(i++).toString().replace(QRegExp("T"), "_");
        deviceAlarmSearch.save_type_name = query.value(i++).toString();
        deviceAlarmSearch.inspect_status_id = (WheelInspectStatus)query.value(i++).toInt();
        deviceAlarmSearch.is_dealed = query.value(i++).toInt();

        if (deviceAlarmSearch.is_dealed == 1)
        {
            deviceAlarmSearch.deal_uuid = query.value(i++).toString();
            deviceAlarmSearch.deal_result = query.value(i++).toString();
            deviceAlarmSearch.deal_status_id = (WheelInspectStatus)query.value(i++).toInt();
        }
        m_deviceAlarmSearchStru.append(deviceAlarmSearch);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceAlarmForTaskUUidCountDB(int &m_count, QString m_task_uuid, QString m_start_time, QString m_stop_time)
{
    QSqlQuery query;
    bool bReturn = false;
	QString sqlString;
	if (m_start_time.isEmpty())
	{
		sqlString = QString("SELECT COUNT(*) FROM task JOIN inspect_result JOIN devices JOIN device_point_name JOIN device_phase JOIN alarm_level JOIN save_type WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND inspect_result.task_uuid='%1';")
			.arg(m_task_uuid);
	}
	else
	{
		sqlString = QString("SELECT COUNT(*) FROM task JOIN inspect_result JOIN devices JOIN device_point_name JOIN device_phase JOIN alarm_level JOIN save_type WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND inspect_result.task_uuid='%1' AND inspect_result.inspect_time>'%2' AND inspect_result.inspect_time<'%3';")
			.arg(m_task_uuid).arg(m_start_time).arg(m_stop_time);
	}
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_count = query.value(0).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelAlarmDataForDeviceTaskUUidDB(QString task_uuid, QString device_uuid, DeviceAlarmSearchStruct &stru)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString;
	sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,alarm_level.alarm_level_name,inspect_result.inspect_time,save_type.save_type_name,inspect_result.inspect_status_id,inspect_result.is_dealed,deal_info.deal_info_uuid,deal_info.dealed_result,deal_info.dealed_status_id FROM recognition_type,device_point_name,task,devices,alarm_level,save_type,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.recognition_type_id=recognition_type.recognition_type_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND task.task_uuid='%1' AND devices.device_uuid='%2';")
			.arg(task_uuid).arg(device_uuid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		stru.task_uuid = query.value(i++).toString();
		stru.device_uuid = query.value(i++).toString();
		stru.recognition_type_name = query.value(i++).toString();
		QString pointName = query.value(i++).toString();
		QString pointPhase = query.value(i++).toString();
		if (pointName.contains("X", Qt::CaseSensitive))
		{
			pointName = pointName.replace(QRegExp("X"), pointPhase);
		}
		stru.device_point_type_name = pointName;
		stru.inspect_result = query.value(i++).toString();
		stru.alarm_level_name = query.value(i++).toString();
		stru.inspect_time = query.value(i++).toString().replace(QRegExp("T"), "_");
		stru.save_type_name = query.value(i++).toString();
		stru.inspect_status_id = (WheelInspectStatus)query.value(i++).toInt();
		stru.is_dealed = query.value(i++).toInt();

		if (stru.is_dealed == 1)
		{
			stru.deal_uuid = query.value(i++).toString();
			stru.deal_result = query.value(i++).toString();
			stru.deal_status_id = (WheelInspectStatus)query.value(i++).toInt();
		}
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskDataForDeviceUUidDB(QString deviceUUid, QList<WheelTaskShow> &taskData)
{
	QSqlQuery query;
	bool bReturn = false;
	WheelTaskShow ctr;
	QString sqlString;
// 	sqlString = QString("SELECT inspect_result.task_uuid,task.task_name ,task.task_end_time FROM inspect_result,task WHERE inspect_result.task_uuid=task.task_uuid AND inspect_result.device_uuid='%1' ORDER BY inspect_result.inspect_time DESC;")
// 		.arg(deviceUUid);
	sqlString = QString("SELECT task.task_uuid,task.task_name ,task.task_start_time FROM task ORDER BY task.task_start_time DESC;");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		ctr.task_uuid = query.value(i++).toString();
        ctr.task_name = query.value(i++).toString();
        ctr.task_time = query.value(i++).toString().replace("T"," ");
		taskData.append(ctr);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDeviceParameterDB(WheelRobortDeviceParameterStruct m_deviceParamterStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO device_parameter VALUE ('%1', '%2', '%3', '%4', '%5', '%6', '%7', '%8', '%9', '%10','%11');")
        .arg(m_deviceParamterStru.device_uuid)
        .arg(m_deviceParamterStru.point_id)
        .arg(m_deviceParamterStru.ptz_pan)
        .arg(m_deviceParamterStru.ptz_tilt)
        .arg(m_deviceParamterStru.hc_zoom_near)
        .arg(m_deviceParamterStru.hc_focus_near)
        .arg(m_deviceParamterStru.hc_zoom_far)
        .arg(m_deviceParamterStru.hc_focus_far)
        .arg(m_deviceParamterStru.mag_focus)
        .arg(m_deviceParamterStru.video_length)
        .arg(m_deviceParamterStru.audio_length);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceParameterDB(QString m_device_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM device_parameter where device_uuid='%1';").arg(m_device_uuid);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataDeviceParameterDB(WheelRobortDeviceParameterStruct m_deviceParamterStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("UPDATE device_parameter SET point_id='%1',ptz_pan='%2',ptz_tilt='%3',hc_zoom_near='%4',hc_focus_near='%5',hc_zoom_far = '%6', hc_focus_far = '%7', mag_focus = '%8', video_length = '%9', audio_length = '%10' WHERE device_uuid = '%11'; ")
        .arg(m_deviceParamterStru.point_id)
        .arg(m_deviceParamterStru.ptz_pan)
        .arg(m_deviceParamterStru.ptz_tilt)
        .arg(m_deviceParamterStru.hc_zoom_near)
        .arg(m_deviceParamterStru.hc_focus_near)
        .arg(m_deviceParamterStru.hc_zoom_far)
        .arg(m_deviceParamterStru.hc_focus_far)
        .arg(m_deviceParamterStru.mag_focus)
        .arg(m_deviceParamterStru.video_length)
        .arg(m_deviceParamterStru.audio_length)
        .arg(m_deviceParamterStru.device_uuid);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertVoltageLevelDB(QString m_voltage_level_name)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO voltage_level(voltage_level_name) VALUES ('%1');")
        .arg(m_voltage_level_name);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertVoltageLevelDB(QString m_voltage_level_name, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    int count = 0;
    QString sqlString = QString("SELECT COUNT(*) from voltage_level WHERE voltage_level_name = '%1';").arg(m_voltage_level_name);
    bReturn = querySqlString(sqlString, query);
    query.next();
    count = query.value(0).toInt();

    if (count > 0)
    {
        errMsg = "重复电压等级名称，请核实后再输入";
        return false;
    }

    QString uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
    sqlString = QString("INSERT INTO voltage_level VALUES ('%1', '%2');")
        .arg(uuid)
        .arg(m_voltage_level_name);
    bReturn = querySqlString(sqlString, query);

	sqlString = QString("INSERT INTO device_area VALUES ('%1', '%2');")
		.arg(uuid)
		.arg(m_voltage_level_name);
	bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        errMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteVoltageLevelDB(QString m_voltage_level_id, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM voltage_level where voltage_level_id='%1';")
        .arg(m_voltage_level_id);
    bReturn = querySqlString(sqlString, query);

    sqlString = QString("DELETE FROM device_area where device_area_uuid='%1';")
        .arg(m_voltage_level_id);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        errMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDeviceAreaDB(WheelRobortDeviceAreaStruct m_device_area)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO device_area VALUE ('%1','%2');")
        .arg(m_device_area.device_area_uuid).arg(m_device_area.device_area_name);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDeviceAreaDB(WheelRobortDeviceAreaStruct m_device_area, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    int count = 0;

    QString sqlString = QString("SELECT COUNT(*) from device_area WHERE device_area_name = '%1';").arg(m_device_area.device_area_name);
    bReturn = querySqlString(sqlString, query);
    query.next();
    count = query.value(0).toInt();

    if (count > 0)
    {
        errMsg = "重复区域名称，请核实后再输入";
        return false;
    }

    sqlString = QString("INSERT INTO device_area VALUE ('%1','%2');")
        .arg(m_device_area.device_area_uuid).arg(m_device_area.device_area_name);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        errMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceAreaDB(QString m_device_area_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM device_area where device_area_uuid='%1';")
        .arg(m_device_area_uuid);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceAreaDB(QString m_device_area_uuid, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM device_area where device_area_uuid='%1';")
        .arg(m_device_area_uuid);
    bReturn = querySqlString(sqlString, query);

    if (!bReturn)
    {
        errMsg = query.lastError().text();
    }

    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertEquipmentIntervalDB(WheelRobortEquipmentIntervalStruct m_equipmentIntervalStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("INSERT INTO equipment_interval VALUE ('%1','%2','%3');")
        .arg(m_equipmentIntervalStru.equipment_interval_uuid)
        .arg(m_equipmentIntervalStru.voltage_level_id)
        .arg(m_equipmentIntervalStru.equipment_interval_name);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::insertEquipmentIntervalDB(WheelRobortEquipmentIntervalStruct m_equipmentIntervalStru, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    int count = 0;

    QString sqlString = QString("SELECT COUNT(*) from equipment_interval WHERE equipment_interval_name = '%1';").arg(m_equipmentIntervalStru.equipment_interval_name);
    bReturn = querySqlString(sqlString, query);
    query.next();
    count = query.value(0).toInt();

    if (count > 0)
    {
        errMsg = "重复间隔名称，请核实后再输入";
        return false;
    }

    sqlString = QString("INSERT INTO equipment_interval VALUE ('%1','%2','%3');")
        .arg(m_equipmentIntervalStru.equipment_interval_uuid)
        .arg(m_equipmentIntervalStru.voltage_level_id)
        .arg(m_equipmentIntervalStru.equipment_interval_name);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        errMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteEquipmentIntervalDB(QString m_equipment_interval_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM equipment_interval where equipment_interval_uuid='%1';")
        .arg(m_equipment_interval_uuid);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteEquipmentIntervalDB(QString m_equipment_interval_uuid, QString &errMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM equipment_interval where equipment_interval_uuid='%1';")
        .arg(m_equipment_interval_uuid);
    bReturn = querySqlString(sqlString, query);
    if (!bReturn)
    {
        errMsg = query.lastError().text();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updateEquipmentIntervalDB(WheelRobortEquipmentIntervalStruct equipmentIntervalStru)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("UPDATE equipment_interval SET equipment_interval_name='%1' WHERE equipment_interval_uuid='%2';")
		.arg(equipmentIntervalStru.equipment_interval_name)
		.arg(equipmentIntervalStru.equipment_interval_uuid);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelRobortAlarmSearchDB(int m_count, int m_showCount, QList<WheelRobortAlarmSearchStruct>& m_alarmSearchStru, QString m_start_time, QString m_stop_time)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelRobortAlarmSearchStruct robortAlarmSearch;
    QString sqlString = QString("SELECT recognition_type.recognition_type_name,device_point_name.device_point_type_name, device_phase.device_phase_name,alarm_level.alarm_level_name,inspect_result.inspect_result, inspect_result.inspect_time FROM recognition_type,devices,device_point_name,device_phase,alarm_level,inspect_result WHERE devices.recognition_type_id=recognition_type.recognition_type_id AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.device_uuid=inspect_result.device_uuid AND inspect_result.alarm_level_id>1 AND inspect_result.inspect_time>'%1' AND inspect_result.inspect_time<'%2' ORDER BY inspect_result.inspect_time DESC LIMIT %3,%4;")
        .arg(m_start_time).arg(m_stop_time).arg(m_count).arg(m_showCount);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        robortAlarmSearch.AlarmType = query.value(i++).toString();
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        robortAlarmSearch.AlarmContent = QString("%1%2%3").arg(pointName).arg(query.value(i++).toString()).arg(query.value(i++).toString());
        robortAlarmSearch.AlarmTime = query.value(i++).toString().replace(QRegExp("T"), "_");
        m_alarmSearchStru.append(robortAlarmSearch);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelRobortAlarmSearchCountDB(int &m_count, QString m_start_time, QString m_stop_time)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT count(*) FROM recognition_type,devices,device_point_name,device_phase,alarm_level,inspect_result WHERE devices.recognition_type_id=recognition_type.recognition_type_id AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.device_uuid=inspect_result.device_uuid AND inspect_result.alarm_level_id>1 AND inspect_result.inspect_time>'%1' AND inspect_result.inspect_time<'%2' ORDER BY inspect_result.inspect_time DESC;")
        .arg(m_start_time).arg(m_stop_time);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_count = query.value(0).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getNodeAlarmStatusFromDevicesDB(QString device_uuid, QList<WheelRobortAlarmPathStruct> &c_alarmPath)
{
    WheelRobortAlarmPathStruct c_alarmPathStru;
    QSqlQuery query;
    bool bReturn = false;

    QList<int> c_level;
    QList<QString> deviceMsg;

    QString sqlString = QString("SELECT devices.voltage_level_id,devices.equipment_interval_uuid,devices.device_type_uuid,devices.alarm_level_id,device_point_name.device_point_type_name,device_phase.device_phase_name FROM devices,device_point_name,device_phase WHERE devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.device_uuid='%1';")
        .arg(device_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        deviceMsg.append(query.value(i++).toString());
        deviceMsg.append(query.value(i++).toString());
        deviceMsg.append(query.value(i++).toString());
        deviceMsg.append(query.value(i++).toString());
        deviceMsg.append(query.value(i++).toString());
        deviceMsg.append(query.value(i++).toString());
    }
    c_alarmPathStru.alarm_level_id = 0;
    QString sqlStr2 = QString("SELECT max(devices.alarm_level_id) FROM devices GROUP BY devices.voltage_level_id;");
    bReturn = querySqlString(sqlStr2, query);
    while (query.next())
    {
        c_level.append(query.value(0).toInt());
    }

    for (int i = 0; i < c_level.size(); i++)
    {
        if (c_alarmPathStru.alarm_level_id < c_level.at(i))
        {
            c_alarmPathStru.alarm_level_id = c_level.at(i);
        }
    }
    c_alarmPath.append(c_alarmPathStru);
    c_alarmPath.append(c_alarmPathStru);

    QString sqlStr3 = QString("SELECT voltage_level.voltage_level_name,max(devices.alarm_level_id) FROM devices,voltage_level WHERE devices.voltage_level_id=voltage_level.voltage_level_id AND voltage_level.voltage_level_id='%1';")
        .arg(deviceMsg.at(0));
    bReturn = querySqlString(sqlStr3, query);
    while (query.next())
    {
    //    DeviceAlarmLevel voltage_level_alarm_level = (DeviceAlarmLevel)query.value(0).toInt();
    //    int voltage_level_alarm_level = query.value(1).toInt();
        c_alarmPathStru.alarm_path_name = query.value(0).toString();
        c_alarmPathStru.alarm_level_id = query.value(1).toInt();
        c_alarmPath.append(c_alarmPathStru);
    }

    QString sqlStr4 = QString("SELECT equipment_interval.equipment_interval_name,max(devices.alarm_level_id) FROM devices,equipment_interval WHERE devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.equipment_interval_uuid='%1';")
        .arg(deviceMsg.at(1));
    bReturn = querySqlString(sqlStr4, query);
    while (query.next())
    {
        //    DeviceAlarmLevel voltage_level_alarm_level = (DeviceAlarmLevel)query.value(0).toInt();
        //int equipment_interval_alarm_level = query.value(1).toInt();
        c_alarmPathStru.alarm_path_name = query.value(0).toString();    
        c_alarmPathStru.alarm_level_id = query.value(1).toInt();
        c_alarmPath.append(c_alarmPathStru);
    }

    QString sqlStr5 = QString("SELECT device_type.device_type_name,max(devices.alarm_level_id) FROM devices,device_type WHERE devices.device_type_uuid=device_type.device_type_uuid AND devices.device_type_uuid='%1' AND devices.equipment_interval_uuid='%2';")
        .arg(deviceMsg.at(2)).arg(deviceMsg.at(1));
    bReturn = querySqlString(sqlStr5, query);
    while (query.next())
    {
        //    DeviceAlarmLevel voltage_level_alarm_level = (DeviceAlarmLevel)query.value(0).toInt();
        c_alarmPathStru.alarm_path_name = query.value(0).toString();
        c_alarmPathStru.alarm_level_id = query.value(1).toInt();
        c_alarmPath.append(c_alarmPathStru);
    }
    QString pointName = deviceMsg.at(4);
    QString pointPhase = deviceMsg.at(5);
    if (pointName.contains("X", Qt::CaseSensitive))
    {
        pointName = pointName.replace(QRegExp("X"), pointPhase);
    }
    c_alarmPathStru.alarm_path_name = pointName;
    c_alarmPathStru.alarm_level_id = deviceMsg.at(3).toInt();
    c_alarmPath.append(c_alarmPathStru);

    return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceNodePathFromDevicesDB(QString m_device_uuid, QStringList &c_devicePath)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_type.device_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name FROM devices,equipment_interval,voltage_level,device_type,device_point_name,device_phase WHERE devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.device_uuid='%1';")
        .arg(m_device_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        c_devicePath.append(query.value(i++).toString());
        c_devicePath.append(query.value(i++).toString());
        c_devicePath.append(query.value(i++).toString());
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        c_devicePath.append(pointName);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getDevicesByTaskUuid(QStringList &devs, QString task_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    devs.clear();
    QString sqlString = QString("SELECT device_uuid FROM task_devices INNER JOIN task ON task.task_edit_uuid = task_devices.task_edit_uuid WHERE task.task_uuid = '%1'")
        .arg(task_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        devs.push_back(query.value(0).toString());
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelStandardPatrolVindicateDB(QStringList m_device_type_uuid, QList<WheelStandardPatrolVindicateStruct> &m_standardPatrolVindicateStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString;
    QString Device_Type_UUid = QString("");
    if (m_device_type_uuid.size() == 0)
    {
        sqlString = QString("SELECT device_point_name.device_point_type_uuid,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,recognition_type.recognition_type_name,meter_type.meter_type_name,fever_type.fever_type_name,save_type.save_type_name FROM device_type,sub_device_type,device_point_name LEFT JOIN recognition_type ON device_point_name.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON device_point_name.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON device_point_name.fever_type_id=fever_type.fever_type_id LEFT JOIN save_type ON device_point_name.save_type_id=save_type.save_type_id WHERE device_point_name.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND sub_device_type.device_type_uuid=device_type.device_type_uuid ORDER BY device_type.device_type_uuid,sub_device_type.sub_device_name,device_point_name.device_point_type_name;");
    }
    else
    {
        for (int i = 0; i < m_device_type_uuid.size(); i++)
        {
            if (i == (m_device_type_uuid.size() - 1))
            {
                Device_Type_UUid = Device_Type_UUid + QString("device_type.device_type_uuid = '%1' ").arg(m_device_type_uuid[i]);
            }
            else
            {
                Device_Type_UUid = Device_Type_UUid + QString("device_type.device_type_uuid = '%1' OR ").arg(m_device_type_uuid[i]);
            }

        }
        sqlString = QString("SELECT device_point_name.device_point_type_uuid,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,recognition_type.recognition_type_name,meter_type.meter_type_name,fever_type.fever_type_name,save_type.save_type_name FROM device_type,sub_device_type,device_point_name LEFT JOIN recognition_type ON device_point_name.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON device_point_name.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON device_point_name.fever_type_id=fever_type.fever_type_id LEFT JOIN save_type ON device_point_name.save_type_id=save_type.save_type_id WHERE device_point_name.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND sub_device_type.device_type_uuid=device_type.device_type_uuid AND (%1) ORDER BY device_type.device_type_uuid,sub_device_type.sub_device_name,device_point_name.device_point_type_name;")
            .arg(Device_Type_UUid);
    }
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        WheelStandardPatrolVindicateStruct standPatrol;
        standPatrol.m_device_point_uuid = query.value(i++).toString();
        standPatrol.m_device_type_name = query.value(i++).toString();
        standPatrol.m_sub_device_name = query.value(i++).toString();
        standPatrol.m_device_point_name = query.value(i++).toString();
        standPatrol.m_recognition_type_name = query.value(i++).toString();
        standPatrol.m_meter_type_name = query.value(i++).toString();
        standPatrol.m_fever_type_name = query.value(i++).toString();
        standPatrol.m_save_type_name = query.value(i++).toString();
        m_standardPatrolVindicateStru.append(standPatrol);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::updataWheelStandardPatrolVindicateDB(WheelDevicePointNameStruct m_devicePointNameStru)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString;
    sqlString = QString("UPDATE device_point_name SET sub_device_type_uuid='%1',device_point_type_name='%2',recognition_type_id='%3'")
        .arg(m_devicePointNameStru.sub_device_type_uuid)
        .arg(m_devicePointNameStru.device_point_type_name)
        .arg(m_devicePointNameStru.recognition_type_id);
    QString sqlStr_1 = "";
    if (m_devicePointNameStru.meter_type_id == -1)
    {
    }
    else
    {
        sqlStr_1 = QString(",meter_type_id='%1'").arg(m_devicePointNameStru.meter_type_id);
    }
    QString sqlStr_2 = "";
    if (m_devicePointNameStru.fever_type_id == -1)
    {
    }
    else
    {
        sqlStr_2 = QString(",fever_type_id='%1'").arg(m_devicePointNameStru.fever_type_id);
    }
    QString sqlStr_3 = "";
    if (m_devicePointNameStru.save_type_id == -1)
    {
    }
    else
    {
        sqlStr_3 = QString(",save_type_id='%1'").arg(m_devicePointNameStru.save_type_id);
    }
    QString sqlStr_4 = QString(" WHERE device_point_type_uuid = '%1';").arg(m_devicePointNameStru.device_point_type_uuid);
    sqlString = sqlString + sqlStr_1 + sqlStr_2 + sqlStr_3 + sqlStr_4;
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteWheelStandardPatrolVindicateDB(QString m_device_point_type_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("DELETE FROM device_point_name where device_point_type_uuid='%1';").arg(m_device_point_type_uuid);
    bReturn = querySqlString(sqlString, query);
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelStandardForPointUUidDB(WheelStandardPatrolVindicateStruct &c_data, QString device_point_uuid)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT device_point_name.device_point_type_uuid,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,recognition_type.recognition_type_name,meter_type.meter_type_name,fever_type.fever_type_name,save_type.save_type_name FROM device_type,sub_device_type,device_point_name LEFT JOIN recognition_type ON device_point_name.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON device_point_name.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON device_point_name.fever_type_id=fever_type.fever_type_id LEFT JOIN save_type ON device_point_name.save_type_id=save_type.save_type_id WHERE device_point_name.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND sub_device_type.device_type_uuid=device_type.device_type_uuid AND device_point_name.device_point_type_uuid='%1' ORDER BY device_type.device_type_uuid,sub_device_type.sub_device_name;").arg(device_point_uuid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		c_data.m_device_point_uuid = query.value(i++).toString();
		c_data.m_device_type_name = query.value(i++).toString();
		c_data.m_sub_device_name = query.value(i++).toString();
		c_data.m_device_point_name = query.value(i++).toString();
		c_data.m_recognition_type_name = query.value(i++).toString();
		c_data.m_meter_type_name = query.value(i++).toString();
		c_data.m_fever_type_name = query.value(i++).toString();
		c_data.m_save_type_name = query.value(i++).toString();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceParameterDB(QString m_device_uuid, WheelRobortDeviceParameterStruct &m_deviceParameter)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT * FROM device_parameter WHERE device_uuid='%1';").arg(m_device_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        m_deviceParameter.device_uuid = query.value(i++).toString();
        m_deviceParameter.point_id = query.value(i++).toInt();
        m_deviceParameter.ptz_pan = query.value(i++).toInt();
        m_deviceParameter.ptz_tilt = query.value(i++).toInt();
        m_deviceParameter.hc_zoom_near = query.value(i++).toInt();
        m_deviceParameter.hc_focus_near = query.value(i++).toInt();
        m_deviceParameter.hc_zoom_far = query.value(i++).toInt();
        m_deviceParameter.hc_focus_far = query.value(i++).toInt();
        m_deviceParameter.mag_focus = query.value(i++).toInt();
        m_deviceParameter.video_length = query.value(i++).toInt();
        m_deviceParameter.audio_length = query.value(i++).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelDeviceParameterToTextDB(QList<QStringList> &m_devP, QStringList &errorMsg)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT device_parameter.device_uuid,device_parameter.point_id,devices.save_type_id,device_parameter.ptz_pan,device_parameter.ptz_tilt,device_parameter.hc_zoom_far,device_parameter.hc_focus_far,device_parameter.hc_zoom_near,device_parameter.hc_focus_near,device_parameter.mag_focus,device_parameter.video_length,device_parameter.audio_length,voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,device_phase.device_phase_name FROM device_parameter,devices,voltage_level,equipment_interval,device_type,sub_device_type,device_point_name,device_phase WHERE device_parameter.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        QStringList m_deviceParameter;
        m_deviceParameter.append(query.value(0).toString());
        for (int i = 1; i < 12; i++)
        {
            m_deviceParameter.append(query.value(i).toString());
        }
		int k = 12;
		QString voltageLevel = query.value(k++).toString();
		QString equipment = query.value(k++).toString();
	//	QString deviceArea = query.value(k++).toString();
		QString devicetype = query.value(k++).toString();
		QString subDevice = query.value(k++).toString();
		QString devicePoint = query.value(k++).toString();
		QString devicePhase = query.value(k++).toString();
    //    QString deviceName = voltageLevel + equipment + devicetype + subDevice.replace("X", devicePhase) + devicePoint.replace("X", devicePhase);
        QString deviceName = voltageLevel + equipment + devicetype + devicePoint.replace("X", devicePhase);
		m_deviceParameter.append(deviceName);
		if (m_deviceParameter[3] == "-1")
		{
			errorMsg.append(m_deviceParameter[0] + devicePoint);
		}
		else
		{
			m_devP.append(m_deviceParameter);
		}
		
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelReportHeadMessageDB(QString m_task_uuid, WheelReportHeadMessage &m_ReportHeadMessage)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT task.task_name,task.task_start_time,task.task_end_time,task_status.task_status_name,task.task_total_devices,task.task_total_bugs,task.task_total_mistake,task.envi_temperature,task.envi_humidity,task.envi_pm_2_5,task.envi_wind_direct,task.envi_wind_speed FROM task,task_status WHERE task.task_status_id=task_status.task_status_id AND task.task_uuid='%1';").arg(m_task_uuid);
    bReturn = querySqlString(sqlString, query);

    while (query.next())
    {
        int i = 0;
        m_ReportHeadMessage.TaskName = query.value(i++).toString();
        m_ReportHeadMessage.TaskStartTime = query.value(i++).toString();
        m_ReportHeadMessage.TaskStopTime = query.value(i++).toString();

        m_ReportHeadMessage.TaskStatus = query.value(i++).toString();
        m_ReportHeadMessage.PatrolPointNum.AllPatrolPointNum = query.value(i++).toInt();
        m_ReportHeadMessage.PatrolPointNum.AlarmPatrolPointNum = query.value(i++).toInt();
        m_ReportHeadMessage.PatrolPointNum.UnusualPatrolPointNum = query.value(i++).toInt();
        m_ReportHeadMessage.PatrolPointNum.NormalPatrolPointNum = 
            ((m_ReportHeadMessage.PatrolPointNum.AllPatrolPointNum - m_ReportHeadMessage.PatrolPointNum.AlarmPatrolPointNum)
                - m_ReportHeadMessage.PatrolPointNum.UnusualPatrolPointNum);

        m_ReportHeadMessage.EnvironmentMessage.envi_temperature = query.value(i++).toFloat();
        m_ReportHeadMessage.EnvironmentMessage.envi_humidity = query.value(i++).toFloat();
        m_ReportHeadMessage.EnvironmentMessage.envi_pm_2_5 = query.value(i++).toFloat();
        m_ReportHeadMessage.EnvironmentMessage.envi_wind_direct = query.value(i++).toFloat();
        m_ReportHeadMessage.EnvironmentMessage.envi_wind_speed = query.value(i++).toFloat();
    }
    return bReturn;
}


bool LibDLWheelRobotDBOperation::getWheelAlarmPointDB(QString m_TaskUUid, QList<AlarmUnusualNormalPoint> &m_AlarmPoint)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,alarm_level.alarm_level_name,inspect_result.inspect_time,recognition_type.recognition_type_id,devices.device_uuid FROM task,inspect_result,alarm_level,devices,recognition_type,device_point_name,device_phase WHERE devices.recognition_type_id=recognition_type.recognition_type_id AND task.task_uuid=inspect_result.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND inspect_result.alarm_level_id>1 AND inspect_result.alarm_level_id<=5 AND task.task_uuid='%1' ORDER BY inspect_result.inspect_time;").arg(m_TaskUUid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        AlarmUnusualNormalPoint alarmPoint;
        alarmPoint.DiscernType = query.value(i++).toString();
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        alarmPoint.PointName = pointName;
        alarmPoint.DiscernResult = query.value(i++).toString();
        alarmPoint.AlarmGradeOrAuditResult = query.value(i++).toString();
        QString c_date = query.value(i++).toString().replace(QRegExp("T"), "_"); // 2018-10-19 09:59:35
        alarmPoint.DiscernTime = QString("%1年%2月%3日 %4时%5分%6秒")
            .arg(c_date.mid(0,4)).arg(c_date.mid(5,2)).arg(c_date.mid(8, 2)).arg(c_date.mid(11, 2))
            .arg(c_date.mid(14, 2)).arg(c_date.mid(17, 2));
        int reco = query.value(i++).toInt();
		alarmPoint.RecognitionType = reco;
		QString device_uuid = query.value(i++).toString();
		if (reco == 1 || reco == 2 || reco == 3)
		{
			//alarmPoint.CollectMessage.VisibleLightPath = "D:\\test\\cc.jpg";
			alarmPoint.CollectMessage.VisibleLightPath = QString("/task/%1/%2/%3_scaled.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
		}
		else if (reco == 4 || reco == 6)
		{
			//alarmPoint.CollectMessage.VisibleLightPath = "D:\\test\\gcc.jpg";
			alarmPoint.CollectMessage.VisibleLightPath = QString("/task/%1/%2/%3_scaled.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
			//alarmPoint.CollectMessage.InfraredLightPath = "D:\\test\\bb.jpg";
			alarmPoint.CollectMessage.InfraredLightPath = QString("/task/%1/%2/%3_result.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
		}
        else{}
        m_AlarmPoint.append(alarmPoint);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getUnusualPointDB(QString m_TaskUUid, QList<AlarmUnusualNormalPoint> &m_AlarmPoint)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,deal_info.dealed_result,inspect_result.inspect_time,recognition_type.recognition_type_id,inspect_result.is_dealed,devices.device_uuid FROM task,devices,recognition_type,device_point_name,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE devices.recognition_type_id=recognition_type.recognition_type_id AND task.task_uuid=inspect_result.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.device_uuid=inspect_result.device_uuid AND inspect_result.alarm_level_id='6' AND task.task_uuid='%1' ORDER BY inspect_result.inspect_time;").arg(m_TaskUUid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        AlarmUnusualNormalPoint alarmPoint;
        alarmPoint.DiscernType = query.value(i++).toString();
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        alarmPoint.PointName = pointName;
        alarmPoint.DiscernResult = query.value(i++).toString();
        alarmPoint.AlarmGradeOrAuditResult = query.value(i++).toString();
        QString c_date = query.value(i++).toString().replace(QRegExp("T"), "_"); // 2018-10-19 09:59:35
        alarmPoint.DiscernTime = QString("%1年%2月%3日 %4时%5分%6秒")
            .arg(c_date.mid(0, 4)).arg(c_date.mid(5, 2)).arg(c_date.mid(8, 2)).arg(c_date.mid(11, 2))
            .arg(c_date.mid(14, 2)).arg(c_date.mid(17, 2));

        int reco = query.value(i++).toInt();
		alarmPoint.RecognitionType = reco;
        if (query.value(i++).toInt() == 0)
        {
            alarmPoint.AlarmGradeOrAuditResult = "";
        }

		QString device_uuid = query.value(i++).toString();
		if (reco == 1 || reco == 2 || reco == 3)
		{
			//alarmPoint.CollectMessage.VisibleLightPath = "D:\\test\\cc.jpg";
			alarmPoint.CollectMessage.VisibleLightPath = QString("/task/%1/%2/%3_scaled.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
		}
		else if (reco == 4 || reco == 6)
		{
			//alarmPoint.CollectMessage.VisibleLightPath = "D:\\test\\gcc.jpg";
			alarmPoint.CollectMessage.VisibleLightPath = QString("/task/%1/%2/%3_scaled.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
			//alarmPoint.CollectMessage.InfraredLightPath = "D:\\test\\bb.jpg";
			alarmPoint.CollectMessage.InfraredLightPath = QString("/task/%1/%2/%3_result.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
		}
		else {}
        m_AlarmPoint.append(alarmPoint);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelNormalPointDB(QString m_TaskUUid, QList<AlarmUnusualNormalPoint> &m_AlarmPoint)
{
    QSqlQuery query;
    bool bReturn = false;
//	QString sqlString = QString("SELECT recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,deal_info.dealed_result,inspect_result.inspect_time,recognition_type.recognition_type_id,inspect_result.is_dealed,devices.device_uuid FROM task,devices,recognition_type,device_point_name,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE devices.recognition_type_id=recognition_type.recognition_type_id AND task.task_uuid=inspect_result.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.device_uuid=inspect_result.device_uuid AND inspect_result.alarm_level_id='1' AND task.task_uuid='%1' ORDER BY inspect_result.inspect_time;").arg(m_TaskUUid);
	QString sqlString = QString("SELECT recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,deal_info.dealed_result,inspect_result.inspect_time,recognition_type.recognition_type_id,devices.device_uuid FROM task,devices,recognition_type,device_point_name,device_phase,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE devices.recognition_type_id=recognition_type.recognition_type_id AND task.task_uuid=inspect_result.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.device_uuid=inspect_result.device_uuid AND inspect_result.alarm_level_id='1' AND task.task_uuid='%1' ORDER BY inspect_result.inspect_time;").arg(m_TaskUUid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        int i = 0;
        AlarmUnusualNormalPoint alarmPoint;
        alarmPoint.DiscernType = query.value(i++).toString();
        QString pointName = query.value(i++).toString();
        QString pointPhase = query.value(i++).toString();
        if (pointName.contains("X", Qt::CaseSensitive))
        {
            pointName = pointName.replace(QRegExp("X"), pointPhase);
        }
        alarmPoint.PointName = pointName;
        alarmPoint.DiscernResult = query.value(i++).toString();
        alarmPoint.AlarmGradeOrAuditResult = query.value(i++).toString();
        QString c_date = query.value(i++).toString().replace(QRegExp("T"), "_"); // 2018-10-19 09:59:35
        alarmPoint.DiscernTime = QString("%1年%2月%3日 %4时%5分%6秒")
            .arg(c_date.mid(0, 4)).arg(c_date.mid(5, 2)).arg(c_date.mid(8, 2)).arg(c_date.mid(11, 2))
            .arg(c_date.mid(14, 2)).arg(c_date.mid(17, 2));

        int reco = query.value(i++).toInt();
		alarmPoint.RecognitionType = reco;
		QString device_uuid = query.value(i++).toString();
		if (reco == 1 || reco == 2 || reco == 3)
		{
			//alarmPoint.CollectMessage.VisibleLightPath = "D:\\test\\cc.jpg";
				alarmPoint.CollectMessage.VisibleLightPath = QString("/task/%1/%2/%3_scaled.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
		}
		else if (reco == 4 || reco == 6)
		{
			//alarmPoint.CollectMessage.VisibleLightPath = "D:\\test\\gcc.jpg";
				alarmPoint.CollectMessage.VisibleLightPath = QString("/task/%1/%2/%3_scaled.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
			//alarmPoint.CollectMessage.InfraredLightPath = "D:\\test\\bb.jpg";
				alarmPoint.CollectMessage.InfraredLightPath = QString("/task/%1/%2/%3_result.jpg").arg(m_TaskUUid).arg(device_uuid).arg(device_uuid);
		}
		else {}
        m_AlarmPoint.append(alarmPoint);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getEditDevicesForEditUUidDB(QMap<QString,QString> &m_deviceMap,QString m_edit_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT device_uuid FROM task_devices WHERE task_edit_uuid='%1';").arg(m_edit_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_deviceMap.insert(query.value(0).toString(), "");
    }
    return bReturn;
}
bool LibDLWheelRobotDBOperation::getEditDevicesForEditUUidListDB(QStringList &m_deviceMap, QString m_edit_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT task_devices.device_uuid FROM task_devices,devices,device_point_name WHERE task_devices.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task_devices.task_edit_uuid='%1' ORDER BY device_point_name.device_point_type_name;")
        .arg(m_edit_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_deviceMap.append(query.value(0).toString());
    }
    return bReturn;
}
bool LibDLWheelRobotDBOperation::getTaskEditTypeIdDB(WheelTaskAdminType &m_taskAdminType, QString m_edit_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT task_edit_type_id FROM task_edit WHERE task_edit_uuid='%1';").arg(m_edit_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_taskAdminType = (WheelTaskAdminType)query.value(0).toInt();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskStatusDB(std::map<int, QString> &m_WheelTaskStatusData)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("select * from task_status;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_WheelTaskStatusData.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getCalendarTaskMapDB(QString m_start_time, QString m_end_time, QMap<int, QList<WheelCalendarData>> &taskMap)
{
    QSqlQuery query;
    bool bReturn = false;
    WheelCalendarData calend;

    QString sqlString = QString("SELECT task.task_name, task.task_start_time, task_status.task_status_name from task,task_status where task.task_status_id = task_status.task_status_id and task.task_start_time>'%1' and task.task_start_time<'%2' ORDER BY task.task_start_time;")
        .arg(m_start_time).arg(m_end_time);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        QString task_name = query.value(0).toString();
        QString task_time = query.value(1).toString();
        QString task_status = query.value(2).toString();
        int task_month = task_time.mid(8, 2).toInt();
        QString task_name_add;
        if (task_time.mid(11, 2).toInt() >= 12)
        {
            task_name_add = task_time.mid(11, 5) + QString("下午 ") + task_name;
        }
        else
        {
            task_name_add = task_time.mid(11, 5) + QString("上午 ") + task_name;

        }

        calend.task_name = task_name_add;
        calend.task_status = task_status;

        if (taskMap.contains(task_month))
        {
            taskMap[task_month].append(calend);
        }
        else
        {
            QList<WheelCalendarData> calendList;
            calendList.append(calend);
            taskMap.insert(task_month, calendList);
        }
    }

    sqlString = QString("SELECT task_template.task_template_name,task_template.task_start_date, task_status.task_status_name from task_edit join task_template join task_status where task_edit.task_edit_uuid = task_template.task_edit_uuid and task_template.task_status_id = task_status.task_status_id and task_template.task_start_date>'%1' and task_template.task_start_date<'%2' ORDER BY task_template.task_start_time;")
        .arg(m_start_time).arg(m_end_time);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        QString task_name = query.value(0).toString();
        QString task_time = query.value(1).toString();
        QString task_status = query.value(2).toString();
        int task_month = task_time.mid(8, 2).toInt();
        QString task_name_add;
        if (task_time.mid(11, 2).toInt() >= 12)
        {
            task_name_add = task_time.mid(11, 5) + QString("下午 ") + task_name;
        }
        else
        {
            task_name_add = task_time.mid(11, 5) + QString("上午 ") + task_name;

        }

        calend.task_name = task_name_add;
        calend.task_status = task_status;

        if (taskMap.contains(task_month))
        {
            int hour = taskMap[task_month].last().task_name.mid(0, 2).toInt();
            int minu = taskMap[task_month].last().task_name.mid(3, 2).toInt();
            int hour_n = task_time.mid(11, 2).toInt();
            int minu_n = task_time.mid(14, 2).toInt();
            if (hour < hour_n)
            {
                taskMap[task_month].append(calend);
            }
            else if(hour > hour_n)
            {
                taskMap[task_month].insert(taskMap[task_month].size()-1, calend);
            }
            else
            {
                if (minu <= minu_n)
                {
                    taskMap[task_month].append(calend);
                }
                else
                {
                    taskMap[task_month].insert(taskMap[task_month].size()-1, calend);
                }
            }
        }
        else
        {
            QList<WheelCalendarData> calendList;
            calendList.append(calend);
            taskMap.insert(task_month, calendList);
        }
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelAllDeviceUUidDB(QStringList &m_dev)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT device_uuid FROM devices;");
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        m_dev.append(query.value(0).toString());
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskListShowDB(QString m_start_time, QString m_end_time, QList<WheelTaskShow> &taskList)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT task_uuid,task_name FROM task WHERE task_start_time>'%1' AND task_start_time<'%2' ORDER BY task_start_time DESC;")
        .arg(m_start_time).arg(m_end_time);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        WheelTaskShow tk;
        tk.task_uuid = query.value(0).toString();
        tk.task_name = query.value(1).toString();
        taskList.append(tk);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskStruShowDB(WheelTaskShow &taskStru, QString task_uuid)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT task_name,task_start_time FROM task WHERE task_uuid='%1';").arg(task_uuid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		taskStru.task_uuid = task_uuid;
		taskStru.task_name = query.value(0).toString();
		taskStru.task_time = query.value(1).toString();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDevicePointTypeUUidFromDevices(QString m_device_uuid,QString &device_point_type_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT device_point_type_uuid from devices where device_uuid='%1';")
        .arg(m_device_uuid);
    bReturn = querySqlString(sqlString, query);
    while (query.next())
    {
        device_point_type_uuid = query.value(0).toString();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getTaskEditStruForTaskTemplateUUidDB(QString m_task_edit_uuid, WheelTaskEditStruct & taskEdit)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT * FROM task_edit WHERE task_edit_uuid='%1';")
		.arg(m_task_edit_uuid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		taskEdit.task_edit_uuid = query.value(i++).toString();
		taskEdit.task_edit_name = query.value(i++).toString();
		taskEdit.task_edit_date = query.value(i++).toString().replace(QRegExp("T"), "_");
		taskEdit.task_edit_type_id = (WheelTaskAdminType)query.value(i++).toInt();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getThresholdFileName(QString device_uuid, WheelRobotMeterType meter_type_id, QString &threshold_filename)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT threshold_filename FROM devices WHERE device_uuid ='%1';")
        .arg(device_uuid);
    bReturn = querySqlString(sqlString, query);

    while (query.next())
    {
        threshold_filename = query.value(0).toString();
    }

    if (threshold_filename == NULL)
    {
        QString sqlString = QString("SELECT threshold_filename FROM meter_type WHERE meter_type_id =%1;")
            .arg(meter_type_id);
        bReturn = querySqlString(sqlString, query);
        while (query.next())
        {
            threshold_filename = query.value(0).toString();
        }
    }

    return bReturn;
}

bool LibDLWheelRobotDBOperation::getThresholdFileNameForVirtual(QString virtual_uuid, QString &threshold_filename)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT threshold_uuid FROM virtual_device WHERE virtual_device_uuid ='%1';")
		.arg(virtual_uuid);
	bReturn = querySqlString(sqlString, query);

	while (query.next())
	{
		threshold_filename = query.value(0).toString();
	}

	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelUnusualPointSearchListDB(int m_count, int m_pageCount, QList<DeviceAlarmSearchStruct> &m_UnusualStru, QStringList status)
{
	QSqlQuery query;
	bool bReturn = false;

	QString Inspect_Status_Id = QString("");
	
	if (status.size() == 0)
	{
		Inspect_Status_Id = QString("inspect_result.inspect_status_id like '%%'");
	}
	else
	{
		if (status.size() == 1 && status[0] == "3")
		{
			Inspect_Status_Id = QString("deal_info.dealed_result<>''");
		}
		else
		{
			for (int i = 0; i < status.size(); i++)
			{
				if (i == (status.size() - 1))
				{
					Inspect_Status_Id = Inspect_Status_Id + QString("inspect_result.inspect_status_id = '%1' ").arg(status[i]);
				}
				else
				{
					Inspect_Status_Id = Inspect_Status_Id + QString("inspect_result.inspect_status_id = '%1' OR ").arg(status[i]);
				}

			}
		}
		
	}

	QString sqlString = QString("SELECT task.task_uuid,devices.device_uuid,recognition_type.recognition_type_name,device_point_name.device_point_type_name,device_phase.device_phase_name,inspect_result.inspect_result,deal_info.dealed_result,save_type.save_type_name,max(inspect_result.inspect_time) FROM task,devices,recognition_type,device_point_name,device_phase,save_type,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE devices.recognition_type_id=recognition_type.recognition_type_id AND task.task_uuid=inspect_result.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.device_uuid=inspect_result.device_uuid AND devices.save_type_id=save_type.save_type_id AND (%1) GROUP BY devices.device_uuid ORDER BY inspect_result.inspect_time DESC limit %2,%3;")
		.arg(Inspect_Status_Id).arg(m_count).arg(m_pageCount);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		DeviceAlarmSearchStruct dev;
		dev.task_uuid = query.value(i++).toString();
		dev.device_uuid = query.value(i++).toString();
		dev.recognition_type_name = query.value(i++).toString();
		QString pointName = query.value(i++).toString();
		QString pointPhase = query.value(i++).toString();
		if (pointName.contains("X", Qt::CaseSensitive))
		{
			pointName = pointName.replace(QRegExp("X"), pointPhase);
		}
		dev.device_point_type_name = pointName;
		dev.inspect_result = query.value(i++).toString();
		dev.deal_result = query.value(i++).toString();
		dev.save_type_name = query.value(i++).toString();
        m_UnusualStru.append(dev);
	}
	return bReturn;
}
bool LibDLWheelRobotDBOperation::getWheelUnusualPointSearchCountDB(int &m_count, QStringList status)
{
	QSqlQuery query;
	bool bReturn = false;

	QString Inspect_Status_Id = QString("");

	if (status.size() == 0)
	{
		Inspect_Status_Id = QString("inspect_result.inspect_status_id like '%%'");
	}
	else
	{
		if (status.size() == 1 && status[0] == "3")
		{
			Inspect_Status_Id = QString("deal_info.dealed_result<>''");
		}
		else
		{
			for (int i = 0; i < status.size(); i++)
			{
				if (i == (status.size() - 1))
				{
					Inspect_Status_Id = Inspect_Status_Id + QString("inspect_result.inspect_status_id = '%1' ").arg(status[i]);
				}
				else
				{
					Inspect_Status_Id = Inspect_Status_Id + QString("inspect_result.inspect_status_id = '%1' OR ").arg(status[i]);
				}

			}
		}

	}

	QString sqlString = QString("SELECT COUNT(*) FROM (SELECT COUNT(*) FROM task,devices,recognition_type,device_point_name,device_phase,save_type,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid WHERE devices.recognition_type_id=recognition_type.recognition_type_id AND task.task_uuid=inspect_result.task_uuid AND inspect_result.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND devices.device_uuid=inspect_result.device_uuid AND devices.save_type_id=save_type.save_type_id AND (%1) GROUP BY devices.device_uuid) aa;")
		.arg(Inspect_Status_Id);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		m_count = query.value(0).toInt();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::isAuditFinishDB(QString task_uuid)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT count(*) FROM inspect_result,devices WHERE devices.device_uuid=inspect_result.device_uuid AND inspect_result.is_dealed='0' AND inspect_result.task_uuid='%1';")
		.arg(task_uuid);
	bReturn = querySqlString(sqlString, query);

	while (query.next())
	{
		int cho = query.value(0).toInt();
		if (!cho == 0)
		{
			bReturn = false;
		}
		else
		{
			bReturn = true;
		}
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDeviceType(QString c_uuid, QString c_name)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO device_type VALUE ('%1','%2');")
		.arg(c_uuid).arg(c_name);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertSubDeviceType(QString c_uuid, QString c_typeuuid, QString c_name)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO sub_device_type VALUE ('%1','%2','%3');")
		.arg(c_uuid).arg(c_typeuuid).arg(c_name);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDevicePointName(QStringList c_data, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO device_point_name SET device_point_type_uuid='%1',sub_device_type_uuid='%2',device_point_type_name='%3'")
		.arg(c_data[0]).arg(c_data[1]).arg(c_data[2]);

	if (c_data[3] == "-1" || c_data[3] == "-2")
	{
	}
	else
	{
		sqlString = sqlString + QString(",recognition_type_id='%1'").arg(c_data[3]);
	}

	if (c_data[4] == "-1" || c_data[4] == "-2")
	{
	}
	else
	{
		sqlString = sqlString + QString(",meter_type_id='%1'").arg(c_data[4]);
	}

	if (c_data[5] == "-1" || c_data[5] == "-2")
	{
	}
	else
	{
		sqlString = sqlString + QString(",fever_type_id='%1'").arg(c_data[5]);
	}

	if (c_data[6] == "-1" || c_data[6] == "-2")
	{
		sqlString = sqlString + QString(";");
	}
	else
	{
		sqlString = sqlString + QString(",save_type_id='%1';").arg(c_data[6]);
	}
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		retMsg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertRecognitionType(QString recognitionTypeName)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO recognition_type SET recognition_type_name='%1';")
		.arg(recognitionTypeName);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertMeterType(QString meterTypeName)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO meter_type SET meter_type_name='%1';")
		.arg(meterTypeName);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertFeverType(QString feverTypeName)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO fever_type SET fever_type_name='%1';")
		.arg(feverTypeName);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertSaveType(QString saveTypeName)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO save_type SET save_type_name='%1';")
		.arg(saveTypeName);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertVoltageLevel(QString c_uuid, QString c_name)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO voltage_level VALUE ('%1','%2');")
		.arg(c_uuid).arg(c_name);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertEquipmentInterval(QString c_uuid, QString c_typeuuid, QString c_name)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO equipment_interval VALUE ('%1','%2','%3');")
		.arg(c_uuid).arg(c_typeuuid).arg(c_name);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDeviceArea(QString c_uuid, QString c_name)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("INSERT INTO device_area VALUE ('%1','%2');")
		.arg(c_uuid).arg(c_name);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::updataSubDeviceType(QString subDeviceTypeUUid, QString deviceTypeUUid)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("UPDATE sub_device_type SET device_type_uuid = '%1' where sub_device_type_uuid='%2';")
		.arg(deviceTypeUUid).arg(subDeviceTypeUUid);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::updataDevicePointName(QStringList c_data, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("UPDATE device_point_name SET sub_device_type_uuid='%1',device_point_type_name='%2'")
		.arg(c_data[1]).arg(c_data[2]);
	QString sqlString2 = QString("UPDATE devices SET");
	if (c_data[3] == "-1" || c_data[3] == "-2")
	{
		sqlString = sqlString + QString(",recognition_type_id=NULL");
		sqlString2 = sqlString2 + QString(" recognition_type_id=NULL");
	}
	else
	{
		sqlString = sqlString + QString(",recognition_type_id='%1'").arg(c_data[3]);
		sqlString2 = sqlString2 + QString(" recognition_type_id='%1'").arg(c_data[3]);
	}

	if (c_data[4] == "-1" || c_data[4] == "-2")
	{
		sqlString = sqlString + QString(",meter_type_id=NULL");
		sqlString2 = sqlString2 + QString(",meter_type_id=NULL");
	}
	else
	{
		sqlString = sqlString + QString(",meter_type_id='%1'").arg(c_data[4]);
		sqlString2 = sqlString2 + QString(",meter_type_id='%1'").arg(c_data[4]);
	}

	if (c_data[5] == "-1" || c_data[5] == "-2")
	{
		sqlString = sqlString + QString(",fever_type_id=NULL");
		sqlString2 = sqlString2 + QString(",fever_type_id=NULL");
	}
	else
	{
		sqlString = sqlString + QString(",fever_type_id='%1'").arg(c_data[5]);
		sqlString2 = sqlString2 + QString(",fever_type_id='%1'").arg(c_data[5]);
	}

	if (c_data[6] == "-1" || c_data[6] == "-2")
	{
		sqlString = sqlString + QString(",save_type_id=NULL");
		sqlString2 = sqlString2 + QString(",save_type_id=NULL");
	}
	else
	{
		sqlString = sqlString + QString(",save_type_id='%1'").arg(c_data[6]);
		sqlString2 = sqlString2 + QString(",save_type_id='%1'").arg(c_data[6]);
	}
	sqlString = sqlString + QString(" WHERE device_point_type_uuid='%1';").arg(c_data[0]);
	sqlString2 = sqlString2 + QString(" WHERE device_point_type_uuid='%1';").arg(c_data[0]);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		retMsg = query.lastError().text();
	}
	bReturn = querySqlString(sqlString2, query);
	if (!bReturn)
	{
		retMsg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceWithDevicePointUUid(QString uuid, QString &Msg)
{
	QSqlQuery query;
	bool bReturn = false;

	QString sqlStr = QString("SELECT COUNT(*) FROM device_point_name where device_point_type_uuid='%1';")
		.arg(uuid);
	bReturn = querySqlString(sqlStr, query);
	while (query.next())
	{
		if (query.value(0).toInt() == 0)
		{
			Msg = QString("删除失败！该点位不存在或已被删除！");
			return false;
		}
	}

	QString sqlString = QString("DELETE FROM device_point_name where device_point_type_uuid='%1';")
		.arg(uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		Msg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelTaskHistoryStruDB(QString m_start_time, QString m_end_time, QString taskName, QList<WheelTaskEditStruct> &stru)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlStr;
	WheelTaskEditStruct edit;
	if (m_start_time.isEmpty())
	{
		sqlStr = QString("SELECT task_edit_uuid,task_name FROM task WHERE task_name LIKE '%%1%' ORDER BY task_start_time DESC;")
			.arg(taskName);
	}
	else
	{
		sqlStr = QString("SELECT task_edit_uuid,task_name FROM task WHERE task_start_time>'%1' and task_start_time<'%2' and task_name LIKE '%%3%' ORDER BY task_start_time DESC;")
			.arg(m_start_time).arg(m_end_time).arg(taskName);
	}
	
	bReturn = querySqlString(sqlStr, query);
	while (query.next())
	{
		edit.task_edit_uuid = query.value(0).toString();
		edit.task_edit_name = query.value(1).toString();
		stru.append(edit);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getCreateExcelAlarmSearchDB(int icho, int m_page, int m_pageCount, int &m_count, QList<QStringList> &c_data, WheelPatrolParameter m_wheelPatrolPara)
{
	QString Device_Area_UUid = QString("");
	if (m_wheelPatrolPara.m_device_area_uuid.size() == 0)
	{
		Device_Area_UUid = QString("devices.device_area_uuid like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_device_area_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_device_area_uuid.size() - 1))
			{
				Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
			}
			else
			{
				Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
			}

		}
	}

	QString Device_Type_UUid = QString("");
	if (m_wheelPatrolPara.m_device_type_uuid.size() == 0)
	{
		Device_Type_UUid = QString("devices.device_type_uuid like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_device_type_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_device_type_uuid.size() - 1))
			{
				Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
			}
			else
			{
				Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
			}

		}
	}

	QString Recognition_Type_Id = QString("");
	if (m_wheelPatrolPara.m_recognition_type_uuid.size() == 0)
	{
		Recognition_Type_Id = QString("devices.recognition_type_id like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_recognition_type_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_recognition_type_uuid.size() - 1))
			{
				Recognition_Type_Id = Recognition_Type_Id + QString("devices.recognition_type_id = '%1' ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
			}
			else
			{
				Recognition_Type_Id = Recognition_Type_Id + QString("devices.recognition_type_id = '%1' OR ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
			}

		}
	}

	QString Meter_Type_Id = QString("");
	if (m_wheelPatrolPara.m_meter_type_id.size() == 0)
	{
		Meter_Type_Id = QString("devices.meter_type_id like '%%' OR devices.meter_type_id IS NULL");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_meter_type_id.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_meter_type_id.size() - 1))
			{
				Meter_Type_Id = Meter_Type_Id + QString("devices.meter_type_id = '%1' ").arg(m_wheelPatrolPara.m_meter_type_id[i]);
			}
			else
			{
				Meter_Type_Id = Meter_Type_Id + QString("devices.meter_type_id = '%1' OR ").arg(m_wheelPatrolPara.m_meter_type_id[i]);
			}

		}
	}

	QString Msg;
	QSqlQuery query;
	
	QString sqlString;
	bool bReturn = false;
	if (icho == 1)
	{
		if (m_wheelPatrolPara.m_start_time.isEmpty())
		{
			if (m_wheelPatrolPara.m_device_uuid.isEmpty())
			{
				sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,device_area.device_area_name,equipment_interval.equipment_interval_name,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,recognition_type.recognition_type_name,meter_type.meter_type_name,fever_type.fever_type_name,task_status.task_status_name,inspect_result.inspect_result,deal_info.dealed_result,inspect_result.inspect_time,alarm_level.alarm_level_name,task.envi_temperature,task.envi_humidity,task.envi_wind_speed,task.envi_wind_direct,task.envi_pm_2_5,save_type.save_type_name,device_phase.device_phase_name FROM device_phase,task,device_area,equipment_interval,device_type,sub_device_type,device_point_name,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON devices.fever_type_id=fever_type.fever_type_id,task_status,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid,alarm_level,save_type WHERE devices.device_phase_id=device_phase.device_phase_id AND inspect_result.task_uuid = task.task_uuid AND inspect_result.device_uuid = devices.device_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_status_id=task_status.task_status_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND (%1) AND (%2) AND (%3) AND (%4) ORDER BY inspect_result.inspect_time DESC LIMIT %5,%6;")
					.arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id).arg(m_page).arg(m_pageCount);
			}
			else
			{
				sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,device_area.device_area_name,equipment_interval.equipment_interval_name,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,recognition_type.recognition_type_name,meter_type.meter_type_name,fever_type.fever_type_name,task_status.task_status_name,inspect_result.inspect_result,deal_info.dealed_result,inspect_result.inspect_time,alarm_level.alarm_level_name,task.envi_temperature,task.envi_humidity,task.envi_wind_speed,task.envi_wind_direct,task.envi_pm_2_5,save_type.save_type_name,device_phase.device_phase_name FROM device_phase,task,device_area,equipment_interval,device_type,sub_device_type,device_point_name,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON devices.fever_type_id=fever_type.fever_type_id,task_status,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid,alarm_level,save_type WHERE devices.device_phase_id=device_phase.device_phase_id AND inspect_result.task_uuid = task.task_uuid AND inspect_result.device_uuid = devices.device_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_status_id=task_status.task_status_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND devices.device_uuid='%1' AND (%2) AND (%3) AND (%4) AND (%5) ORDER BY inspect_result.inspect_time DESC LIMIT %6,%7;")
					.arg(m_wheelPatrolPara.m_device_uuid[0]).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id).arg(m_page).arg(m_pageCount);
			}
			
		}
		else
		{
			if (m_wheelPatrolPara.m_device_uuid.isEmpty())
			{
				sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,device_area.device_area_name,equipment_interval.equipment_interval_name,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,recognition_type.recognition_type_name,meter_type.meter_type_name,fever_type.fever_type_name,task_status.task_status_name,inspect_result.inspect_result,deal_info.dealed_result,inspect_result.inspect_time,alarm_level.alarm_level_name,task.envi_temperature,task.envi_humidity,task.envi_wind_speed,task.envi_wind_direct,task.envi_pm_2_5,save_type.save_type_name,device_phase.device_phase_name FROM device_phase,task,device_area,equipment_interval,device_type,sub_device_type,device_point_name,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON devices.fever_type_id=fever_type.fever_type_id,task_status,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid,alarm_level,save_type WHERE devices.device_phase_id=device_phase.device_phase_id AND inspect_result.task_uuid = task.task_uuid AND inspect_result.device_uuid = devices.device_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_status_id=task_status.task_status_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND inspect_result.inspect_time>'%1' AND inspect_result.inspect_time<'%2' AND (%3) AND (%4) AND (%5) AND (%6) ORDER BY inspect_result.inspect_time DESC LIMIT %7,%8;")
					.arg(m_wheelPatrolPara.m_start_time).arg(m_wheelPatrolPara.m_stop_time).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id).arg(m_page).arg(m_pageCount);
			}
			else
			{
				sqlString = QString("SELECT inspect_result.task_uuid,inspect_result.device_uuid,device_area.device_area_name,equipment_interval.equipment_interval_name,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,recognition_type.recognition_type_name,meter_type.meter_type_name,fever_type.fever_type_name,task_status.task_status_name,inspect_result.inspect_result,deal_info.dealed_result,inspect_result.inspect_time,alarm_level.alarm_level_name,task.envi_temperature,task.envi_humidity,task.envi_wind_speed,task.envi_wind_direct,task.envi_pm_2_5,save_type.save_type_name,device_phase.device_phase_name FROM device_phase,task,device_area,equipment_interval,device_type,sub_device_type,device_point_name,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON devices.fever_type_id=fever_type.fever_type_id,task_status,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid,alarm_level,save_type WHERE devices.device_phase_id=device_phase.device_phase_id AND inspect_result.task_uuid = task.task_uuid AND inspect_result.device_uuid = devices.device_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_status_id=task_status.task_status_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND inspect_result.inspect_time>'%1' AND inspect_result.inspect_time<'%2' AND devices.device_uuid='%3' AND (%4) AND (%5) AND (%6) AND (%7) ORDER BY inspect_result.inspect_time DESC LIMIT %8,%9;")
					.arg(m_wheelPatrolPara.m_start_time).arg(m_wheelPatrolPara.m_stop_time).arg(m_wheelPatrolPara.m_device_uuid[0]).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id).arg(m_page).arg(m_pageCount);
			}
		}
		
		bReturn = querySqlString(sqlString, query);
		while (query.next())
		{
			QStringList data;
			QString qu;
			for (int i = 0; i < 21; i++)
			{
				qu = query.value(i).toString();
				if (i == ((int)FIELDS_DEVICE_POINT_TYPE_NAME + 2))
				{
					if (qu.contains("X", Qt::CaseSensitive))
					{
						qu = qu.replace(QRegExp("X"), query.value(21).toString());
					}

				}
				else if (i == ((int)FIELDS_INSPECT_TIME + 2))
				{
					qu = qu.replace(QRegExp("T"), "_");
				}
				data.append(qu);
			}
			c_data.append(data);
		}
		if (!bReturn)
		{
			Msg = query.lastError().text();
		}
	}
	else
	{
		if (m_wheelPatrolPara.m_start_time.isEmpty())
		{
			if (m_wheelPatrolPara.m_device_uuid.isEmpty())
			{
				sqlString = QString("SELECT COUNT(*) FROM task,device_area,equipment_interval,device_type,sub_device_type,device_point_name,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON devices.fever_type_id=fever_type.fever_type_id,task_status,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid,alarm_level,save_type,device_phase WHERE devices.device_phase_id=device_phase.device_phase_id AND inspect_result.task_uuid = task.task_uuid AND inspect_result.device_uuid = devices.device_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_status_id=task_status.task_status_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND (%1) AND (%2) AND (%3) AND (%4) ORDER BY inspect_result.inspect_time DESC;")
					.arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id);
			}
			else
			{
				sqlString = QString("SELECT COUNT(*) FROM task,device_area,equipment_interval,device_type,sub_device_type,device_point_name,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON devices.fever_type_id=fever_type.fever_type_id,task_status,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid,alarm_level,save_type,device_phase WHERE devices.device_phase_id=device_phase.device_phase_id AND inspect_result.task_uuid = task.task_uuid AND inspect_result.device_uuid = devices.device_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_status_id=task_status.task_status_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND devices.device_uuid='%1' AND (%2) AND (%3) AND (%4) AND (%5) ORDER BY inspect_result.inspect_time DESC;")
					.arg(m_wheelPatrolPara.m_device_uuid[0]).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id);
			}
		}
		else
		{
			if (m_wheelPatrolPara.m_device_uuid.isEmpty())
			{
				sqlString = QString("SELECT COUNT(*) FROM task,device_area,equipment_interval,device_type,sub_device_type,device_point_name,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON devices.fever_type_id=fever_type.fever_type_id,task_status,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid,alarm_level,save_type,device_phase WHERE devices.device_phase_id=device_phase.device_phase_id AND inspect_result.task_uuid = task.task_uuid AND inspect_result.device_uuid = devices.device_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_status_id=task_status.task_status_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND inspect_result.inspect_time>'%1' AND inspect_result.inspect_time<'%2' AND (%3) AND (%4) AND (%5) AND (%6) ORDER BY inspect_result.inspect_time DESC;")
					.arg(m_wheelPatrolPara.m_start_time).arg(m_wheelPatrolPara.m_stop_time).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id);
			}
			else
			{
				sqlString = QString("SELECT COUNT(*) FROM task,device_area,equipment_interval,device_type,sub_device_type,device_point_name,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id LEFT JOIN fever_type ON devices.fever_type_id=fever_type.fever_type_id,task_status,inspect_result LEFT JOIN deal_info ON inspect_result.deal_info_uuid=deal_info.deal_info_uuid,alarm_level,save_type,device_phase WHERE devices.device_phase_id=device_phase.device_phase_id AND inspect_result.task_uuid = task.task_uuid AND inspect_result.device_uuid = devices.device_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_status_id=task_status.task_status_id AND inspect_result.alarm_level_id=alarm_level.alarm_level_id AND devices.save_type_id=save_type.save_type_id AND inspect_result.inspect_time>'%1' AND inspect_result.inspect_time<'%2' AND devices.device_uuid='%3' AND (%4) AND (%5) AND (%6) AND (%7) ORDER BY inspect_result.inspect_time DESC;")
					.arg(m_wheelPatrolPara.m_start_time).arg(m_wheelPatrolPara.m_stop_time).arg(m_wheelPatrolPara.m_device_uuid[0]).arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_Id).arg(Meter_Type_Id);
			}
		}
		bReturn = querySqlString(sqlString, query);
		while (query.next())
		{
			m_count = query.value(0).toInt();
		}
		if (!bReturn)
		{
			Msg = query.lastError().text();
		}
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceListForPointId(QString pointId, QStringList &deviceList)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlStr = QString("SELECT device_uuid FROM device_parameter WHERE point_id='%1';")
		.arg(pointId);
	bReturn = querySqlString(sqlStr, query);
	while (query.next())
	{
		deviceList.append(query.value(0).toString());
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceListForPointId(QString pointId, QList<wheelDeviceDetailMsg> &data)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlStr = QString("SELECT devices.device_uuid,voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_area.device_area_name,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,device_phase.device_phase_name FROM device_parameter,devices,voltage_level,equipment_interval,device_area,device_type,sub_device_type,device_point_name,device_phase WHERE device_parameter.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_area_uuid=device_area.device_area_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_phase_id=device_phase.device_phase_id AND device_parameter.point_id='%1';")
		.arg(pointId);
	bReturn = querySqlString(sqlStr, query);
	while (query.next())
	{
		wheelDeviceDetailMsg _data;
		int i = 0;
		_data.deviceUUid = query.value(i++).toString();
		_data.VoltageLevel = query.value(i++).toString();
		_data.equipmentInterval = query.value(i++).toString();
		_data.deviceArea = query.value(i++).toString();
		_data.deviceType = query.value(i++).toString();
		_data.subDevice = query.value(i++).toString();
		_data.devicePointType = query.value(i++).toString();
		QString devicePhase = query.value(i++).toString();

		_data.subDevice = _data.subDevice.replace("X", devicePhase);
		_data.devicePointType = _data.devicePointType.replace("X", devicePhase);
		data.append(_data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getPointIdForEditTaskUUid(std::vector<int> &data, QString task_uuid)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlStr = QString("SELECT device_parameter.point_id FROM task,task_edit,devices,task_devices,device_parameter WHERE task_edit.task_edit_uuid=task_devices.task_edit_uuid AND task_devices.device_uuid=devices.device_uuid AND devices.device_uuid=device_parameter.device_uuid AND task.task_edit_uuid=task_edit.task_edit_uuid AND task.task_uuid='%1' GROUP BY device_parameter.point_id;")
		.arg(task_uuid);
	bReturn = querySqlString(sqlStr, query);
	while (query.next())
	{
		data.push_back(query.value(0).toInt());
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelUserConfigDataDB(QList<WheelUserConfig> &conData, WheelUserType nowType)
{
	QSqlQuery query;
	bool bReturn = false;
	
	QString sqlStr = QString("SELECT * FROM user_config WHERE user_role<'%1';")
		.arg((int)nowType);
	bReturn = querySqlString(sqlStr, query);
	while (query.next())
	{
		WheelUserConfig data;
		int i = 0;
		data.user_uuid = query.value(i++).toString();
		data.user_name = query.value(i++).toString();
		data.user_role = (WheelUserType)query.value(i++).toInt();
		data.user_authority = query.value(i++).toInt();
		data.user_password = query.value(i++).toString();
		conData.append(data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelAllUserConfigDataDB(QList<WheelUserConfig> &conData)
{
	QSqlQuery query;
	bool bReturn = false;
	WheelUserConfig data;
	QString sqlStr = QString("SELECT user_uuid,user_name FROM user_config ORDER BY user_name;");
	bReturn = querySqlString(sqlStr, query);
	while (query.next())
	{
		int i = 0;
		data.user_uuid = query.value(i++).toString();
		data.user_name = query.value(i++).toString();
		conData.append(data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::addUserConfigDB(WheelUserConfig data, QString &Msg)
{
	QSqlQuery query;
	bool bReturn = false;
	m_myDb.database().transaction();
	int iCount = -1;
	QString sqlStr = QString("SELECT count(*) FROM user_config WHERE user_name='%1';")
		.arg(data.user_name);
	bReturn = querySqlString(sqlStr, query);
	while (query.next())
	{
		iCount = query.value(0).toInt();
	}
	if (iCount > 0 && bReturn)
	{
		Msg = QString("用户名已存在！");
		m_myDb.database().rollback();
		return false;
	}
	if (!bReturn)
	{
		Msg = query.lastError().text();
		m_myDb.database().rollback();
		return bReturn;
	}
	QString sqlString = QString("INSERT INTO user_config VALUE('%1','%2','%3','%4','%5','%6');")
		.arg(data.user_uuid)
		.arg(data.user_name)
		.arg((int)data.user_role)
		.arg(data.user_authority)
		.arg(data.user_password)
		.arg(QString("86") + data.user_telephone);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		Msg = query.lastError().text();
		m_myDb.database().rollback();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteUserConfigDB(QString user_uuid, QString &Msg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("DELETE FROM user_config where user_uuid='%1';")
		.arg(user_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		Msg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::searchDeviceUUidWithDeviceAreaDB(std::map<QString, QStringList> &data)
{
	QSqlQuery query;
	bool bReturn = false;
	QString iniDeviceArea;
	QString nextDeviceArea="";
	QStringList listData;
	int i = 0;

	QString sqlString = QString("SELECT device_area.device_area_name,devices.device_uuid FROM devices,device_area WHERE devices.device_area_uuid=device_area.device_area_uuid ORDER BY devices.device_area_uuid;");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		iniDeviceArea = query.value(0).toString();
		if (i == 0)
		{
			listData.append(query.value(1).toString());
		}
		else
		{
			if (iniDeviceArea == nextDeviceArea)
			{
				listData.append(query.value(1).toString());
			}
			else
			{
				data.insert(std::make_pair(nextDeviceArea, listData));
				listData.clear();
				listData.append(query.value(1).toString());
			}
		}
		nextDeviceArea = iniDeviceArea;
		i++;
	}
	if (iniDeviceArea.isEmpty())
	{
	}
	else
	{
		data.insert(std::make_pair(iniDeviceArea, listData));
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getCreateReportDB(QList<WheelCreateReport> &data,QString start_time,QString end_time)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString;
	if (start_time.isEmpty())
	{
		sqlString = QString("SELECT task.task_uuid,task.task_name,task.task_start_time,task.task_total_devices,task_status.task_status_name,task_edit_type.task_edit_type_name FROM task,task_status,task_edit,task_edit_type WHERE task.task_status_id=task_status.task_status_id AND task.task_edit_uuid=task_edit.task_edit_uuid AND task_edit.task_edit_type_id=task_edit_type.task_edit_type_id order by task.task_start_time desc;");
	}
	else
	{
		sqlString = QString("SELECT task.task_uuid,task.task_name,task.task_start_time,task.task_total_devices,task_status.task_status_name,task_edit_type.task_edit_type_name FROM task,task_status,task_edit,task_edit_type WHERE task.task_status_id=task_status.task_status_id AND task.task_edit_uuid=task_edit.task_edit_uuid AND task_edit.task_edit_type_id=task_edit_type.task_edit_type_id AND task.task_start_time>'%1' AND task.task_start_time<'%2' order by task.task_start_time desc;")
			.arg(start_time).arg(end_time);
	}
	
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		WheelCreateReport c_data;
		c_data.task_uuid = query.value(i++).toString();
		c_data.task_name = query.value(i++).toString();
		c_data.task_time = query.value(i++).toString().replace("T"," ");
		c_data.all_point = query.value(i++).toInt();
		c_data.task_status = query.value(i++).toString();
		c_data.patrol_type = query.value(i++).toString();
		data.append(c_data);
	}

	return bReturn;
}

bool LibDLWheelRobotDBOperation::getPatrolPointSetDataDB(int m_page, int m_pageCount, QList<WheelPatrolPointSet> &data, WheelPatrolParameter m_wheelPatrolPara)
{
	QSqlQuery query;
	bool bReturn = false;
	//.///////////////////////////////////.//

	QString Device_Area_UUid = QString("");
	if (m_wheelPatrolPara.m_device_area_uuid.size() == 0)
	{
		Device_Area_UUid = QString("devices.device_area_uuid like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_device_area_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_device_area_uuid.size() - 1))
			{
				Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
			}
			else
			{
				Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
			}

		}
	}

	QString Device_Type_UUid = QString("");
	if (m_wheelPatrolPara.m_device_type_uuid.size() == 0)
	{
		Device_Type_UUid = QString("devices.device_type_uuid like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_device_type_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_device_type_uuid.size() - 1))
			{
				Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
			}
			else
			{
				Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
			}

		}
	}

	QString Recognition_Type_UUid = QString("");
	if (m_wheelPatrolPara.m_recognition_type_uuid.size() == 0)
	{
		Recognition_Type_UUid = QString("devices.recognition_type_id like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_recognition_type_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_recognition_type_uuid.size() - 1))
			{
				Recognition_Type_UUid = Recognition_Type_UUid + QString("devices.recognition_type_id = '%1' ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
			}
			else
			{
				Recognition_Type_UUid = Recognition_Type_UUid + QString("devices.recognition_type_id = '%1' OR ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
			}

		}
	}

	//.///////////////////////////////////.//
	QString sqlString = QString("SELECT devices.device_uuid,device_point_name.device_point_type_uuid,voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_area.device_area_name,device_type.device_type_name,sub_device_type.sub_device_name,device_point_name.device_point_type_name,device_phase.device_phase_name,recognition_type.recognition_type_name,meter_type.meter_type_name,devices.start_using FROM device_phase,voltage_level,device_point_name,equipment_interval,device_area,device_type,sub_device_type,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id WHERE devices.device_phase_id=device_phase.device_phase_id AND devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid AND devices.device_area_uuid = device_area.device_area_uuid AND devices.device_type_uuid = device_type.device_type_uuid AND devices.device_point_type_uuid = device_point_name.device_point_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND (%1) AND (%2) AND (%3) order by device_point_name.device_point_type_name limit %4,%5;")
		.arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_UUid).arg(m_page).arg(m_pageCount);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		WheelPatrolPointSet c_data;
		c_data.device_uuid = query.value(i++).toString();
		c_data.device_point_type_uuid = query.value(i++).toString();
		c_data.voltahe_level_name = query.value(i++).toString();
		c_data.equipment_interval_name = query.value(i++).toString();
		c_data.device_area_name = query.value(i++).toString();
		c_data.device_type_name = query.value(i++).toString();
		c_data.sub_device_type = query.value(i++).toString();
		QString pointName = query.value(i++).toString();
		QString pointPhase = query.value(i++).toString();
		if (pointName.contains("X", Qt::CaseSensitive))
		{
			pointName = pointName.replace(QRegExp("X"), pointPhase);
		}
		c_data.device_point_type_name = pointName;
		c_data.recognition_type_name = query.value(i++).toString();
		c_data.meter_type_name = query.value(i++).toString();
		if (query.value(i++).toInt() == 0)
		{
			c_data.start_using = QString("禁用");
		}
		else
		{
			c_data.start_using = QString("启用");
		}
		data.append(c_data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getPatrolPointSetDataCountDB(int &m_count, WheelPatrolParameter m_wheelPatrolPara)
{
	QSqlQuery query;
	bool bReturn = false;
	//.///////////////////////////////////.//

	QString Device_Area_UUid = QString("");
	if (m_wheelPatrolPara.m_device_area_uuid.size() == 0)
	{
		Device_Area_UUid = QString("devices.device_area_uuid like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_device_area_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_device_area_uuid.size() - 1))
			{
				Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
			}
			else
			{
				Device_Area_UUid = Device_Area_UUid + QString("devices.device_area_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_area_uuid[i]);
			}

		}
	}

	QString Device_Type_UUid = QString("");
	if (m_wheelPatrolPara.m_device_type_uuid.size() == 0)
	{
		Device_Type_UUid = QString("devices.device_type_uuid like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_device_type_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_device_type_uuid.size() - 1))
			{
				Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
			}
			else
			{
				Device_Type_UUid = Device_Type_UUid + QString("devices.device_type_uuid = '%1' OR ").arg(m_wheelPatrolPara.m_device_type_uuid[i]);
			}

		}
	}

	QString Recognition_Type_UUid = QString("");
	if (m_wheelPatrolPara.m_recognition_type_uuid.size() == 0)
	{
		Recognition_Type_UUid = QString("devices.recognition_type_id like '%%'");
	}
	else
	{
		for (int i = 0; i < m_wheelPatrolPara.m_recognition_type_uuid.size(); i++)
		{
			if (i == (m_wheelPatrolPara.m_recognition_type_uuid.size() - 1))
			{
				Recognition_Type_UUid = Recognition_Type_UUid + QString("devices.recognition_type_id = '%1' ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
			}
			else
			{
				Recognition_Type_UUid = Recognition_Type_UUid + QString("devices.recognition_type_id = '%1' OR ").arg(m_wheelPatrolPara.m_recognition_type_uuid[i]);
			}

		}
	}

	//.///////////////////////////////////.//
	QString sqlString = QString("SELECT count(*) FROM device_phase,voltage_level,device_point_name,equipment_interval,device_area,device_type,sub_device_type,devices LEFT JOIN recognition_type ON devices.recognition_type_id=recognition_type.recognition_type_id LEFT JOIN meter_type ON devices.meter_type_id=meter_type.meter_type_id WHERE devices.device_phase_id=device_phase.device_phase_id AND devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid AND devices.device_area_uuid = device_area.device_area_uuid AND devices.device_type_uuid = device_type.device_type_uuid AND devices.device_point_type_uuid = device_point_name.device_point_type_uuid AND devices.sub_device_type_uuid=sub_device_type.sub_device_type_uuid AND (%1) AND (%2) AND (%3);")
		.arg(Device_Area_UUid).arg(Device_Type_UUid).arg(Recognition_Type_UUid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		m_count = query.value(0).toInt();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::updataDeviceStartUsingDB(QStringList device_uuid, int start_using, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString str="";
	for (int i = 0; i < device_uuid.size(); i++)
	{
		if (i != 0)
		{
			str += QString(",");
		}
		str += QString("'%1'").arg(device_uuid[i]);
	}


	QString sqlString = QString("UPDATE devices SET start_using = '%1' WHERE device_uuid in (%2);")
		.arg(start_using).arg(str);
	bReturn = querySqlString(sqlString, query);

	if(!bReturn)
	{
		retMsg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertPatrolPointForDevices(QStringList insertData, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	QStringList c_data;
	QString str = QString("SELECT sub_device_type.device_type_uuid,device_point_name.sub_device_type_uuid,device_point_name.recognition_type_id,device_point_name.meter_type_id,device_point_name.fever_type_id,device_point_name.save_type_id FROM device_point_name,sub_device_type WHERE device_point_name.sub_device_type_uuid=sub_device_type.sub_device_type_uuid and device_point_name.device_point_type_uuid='%1';")
		.arg(insertData[1]);
	bReturn = querySqlString(str, query);
	while (query.next())
	{
		int i = 0;
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
	}
	QString sqlString = QString("INSERT INTO devices SET device_uuid = '%1',voltage_level_id='%2',equipment_interval_uuid='%3',device_area_uuid='%4',"
		"device_type_uuid='%5',sub_device_type_uuid='%6',device_point_type_uuid='%7',recognition_type_id='%8',save_type_id='%9',meter_type_id='%10',"
		"fever_type_id='%11',device_phase_id='%12',start_using='1';")
		.arg(insertData[0]).arg(insertData[2]).arg(insertData[3]).arg(insertData[4]).arg(c_data[0]).arg(c_data[1]).arg(insertData[1])
		.arg(c_data[2]).arg(c_data[5]).arg(c_data[3]).arg(c_data[4]).arg(insertData[5]);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::updatePatrolPointForDevices(QStringList updateData, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	QStringList c_data;
	QString str = QString("SELECT sub_device_type.device_type_uuid,device_point_name.sub_device_type_uuid,device_point_name.recognition_type_id,device_point_name.meter_type_id,device_point_name.fever_type_id,device_point_name.save_type_id FROM device_point_name,sub_device_type WHERE device_point_name.sub_device_type_uuid=sub_device_type.sub_device_type_uuid and device_point_name.device_point_type_uuid='%1';")
		.arg(updateData[1]);
	bReturn = querySqlString(str, query);
	while (query.next())
	{
		int i = 0;
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
		c_data.append(query.value(i++).toString());
	}
	QString sqlString = QString("UPDATE INTO devices SET voltage_level_id='%1',equipment_interval_uuid='%2',device_area_uuid='%3',"
		"device_type_uuid='%4',sub_device_type_uuid='%5',device_point_type_uuid='%6',recognition_type_id='%7',save_type_id='%8',meter_type_id='%9',"
		"fever_type_id='%10',device_phase_id='%11' WHERE device_uuid='%12';")
		.arg(updateData[2]).arg(updateData[3]).arg(updateData[4]).arg(c_data[0]).arg(c_data[1]).arg(updateData[1])
		.arg(c_data[2]).arg(c_data[5]).arg(c_data[3]).arg(c_data[4]).arg(updateData[5]).arg(updateData[0]);
	bReturn = querySqlString(sqlString, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getAlarmCountSearchDB(QString device_uuid, int &count)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT count(*) FROM inspect_result WHERE alarm_level_id>'1' AND alarm_level_id<'6' AND device_uuid='%1';")
		.arg(device_uuid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		count = query.value(0).toInt();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceTypeNameForDeviceUUidDB(QString device_uuid, QString &device_Type)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT device_type.device_type_name FROM devices,device_type WHERE devices.device_type_uuid=device_type.device_type_uuid AND devices.device_uuid='%1';")
		.arg(device_uuid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		device_Type = query.value(0).toString();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceSnForDeviceUUidDB(QString device_uuid, QString & device_sn, QString & device_sn_name)
{
	QSqlQuery query;
	bool bReturn = false;

	QString fever_type_name;
	QString sqlString = QString("SELECT fever_type.fever_type_name FROM devices, fever_type WHERE devices.fever_type_id=fever_type.fever_type_id AND devices.device_uuid='%1';")
		.arg(device_uuid);

	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		fever_type_name = query.value(0).toString();
	}

	sqlString = QString("SELECT device_sn_id, device_sn_name FROM device_sn WHERE device_sn_id='%1';")
		.arg(fever_type_name);
	
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		device_sn = query.value(0).toString();
		device_sn_name = query.value(1).toString();
	}

	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceUuidForDeviceSn(QString device_ferver_type, QList<QString> &device_uuid_list)
{
	QSqlQuery query;
	bool bReturn = false;

	device_uuid_list.clear();

	QString device_uuid;

	QString sqlString = QString("SELECT devices.device_uuid FROM devices, fever_type WHERE fever_type.fever_type_id=devices.fever_type_id AND fever_type.fever_type_name='%1';")
		.arg(device_ferver_type);

	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		device_uuid = query.value(0).toString();
		device_uuid_list.append(device_uuid);
	}

	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceSnNameForDeviceSn(QString device_sn, QString &device_sn_name)
{
	QSqlQuery query;
	bool bReturn = false;

	QString sqlString = QString("SELECT device_sn_name FROM device_sn WHERE device_sn_id='%1';")
		.arg(device_sn);

	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		device_sn_name = query.value(0).toString();
	}

	return bReturn;
}


bool LibDLWheelRobotDBOperation::updateTaskStatusDB(QString task_uuid, int task_status, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("UPDATE task SET task_status_id = '%1' WHERE task_uuid = '%2';")
		.arg(task_status).arg(task_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		retMsg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getMeterEnumWithDeviceUUidDB(QString device_uuid, WheelRobotMeterType &meterId)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT meter_type.meter_type_id FROM devices,meter_type WHERE devices.meter_type_id=meter_type.meter_type_id AND devices.device_uuid='%1';")
		.arg(device_uuid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		meterId = (WheelRobotMeterType)query.value(0).toInt();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getAlarmMessageSubscribeDB(int m_count, int m_pageCount, QList<WheelNoteMessage> &data, WheelAlarmMessageIf msg)
{
	QSqlQuery query;
	QString sqlString;
	bool bReturn = false;
	WheelNoteMessage c_data;
	if (msg.equipment_interval_uuid.isEmpty())
	{
		sqlString = QString("SELECT note_message.note_uuid,note_message.fault_name_uuid,voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_point_name.device_point_type_name,user_config.user_name,note_message.send_time,send_freq.send_freq_name FROM note_message,devices,voltage_level,equipment_interval,device_point_name,user_config,send_freq WHERE note_message.fault_name_uuid=devices.device_uuid AND devices.voltage_level_id = voltage_level.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid AND device_point_name.device_point_type_uuid = devices.device_point_type_uuid AND note_message.send_freq_id=send_freq.send_freq_id AND note_message.user_receive_uuid = user_config.user_uuid limit %1,%2;")
			.arg(m_count).arg(m_pageCount);
	}
	else
	{
		sqlString = QString("SELECT note_message.note_uuid,note_message.fault_name_uuid,voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_point_name.device_point_type_name,user_config.user_name,note_message.send_time,send_freq.send_freq_name FROM note_message,devices,voltage_level,equipment_interval,device_point_name,user_config,send_freq WHERE note_message.fault_name_uuid=devices.device_uuid AND devices.voltage_level_id = voltage_level.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid AND device_point_name.device_point_type_uuid = devices.device_point_type_uuid AND note_message.send_freq_id=send_freq.send_freq_id AND note_message.user_receive_uuid = user_config.user_uuid AND voltage_level.voltage_level_id='%1' AND equipment_interval.equipment_interval_uuid='%2' AND device_point_name.device_point_type_name like '%%3%' AND user_config.user_name like '%%4%' limit %5,%6;")
			.arg(msg.voltage_level_id).arg(msg.equipment_interval_uuid).arg(msg.device_point_name).arg(msg.revice_user_name).arg(m_count).arg(m_pageCount);
	}
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		c_data.note_uuid = query.value(i++).toString();
		c_data.fault_name_uuid = query.value(i++).toString();
		c_data.voltage_level_name = query.value(i++).toString();
		c_data.equipment_interval_name = query.value(i++).toString();
		c_data.fault_name = query.value(i++).toString();
		c_data.revice_user_name = query.value(i++).toString();
		c_data.send_freq = query.value(i++).toString();
		c_data.alarm_type = query.value(i++).toString();
		data.append(c_data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getAlarmMessageSubscribeCountDB(int &m_count, WheelAlarmMessageIf msg)
{
	QSqlQuery query;
	QString sqlString;
	bool bReturn = false;
	if (msg.equipment_interval_uuid.isEmpty())
	{
		sqlString = QString("SELECT count(*) FROM note_message,devices,voltage_level,equipment_interval,device_point_name,user_config,send_freq WHERE note_message.fault_name_uuid=devices.device_uuid AND devices.voltage_level_id = voltage_level.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid AND device_point_name.device_point_type_uuid = devices.device_point_type_uuid AND note_message.send_freq_id=send_freq.send_freq_id AND note_message.user_receive_uuid = user_config.user_uuid;")
			.arg(msg.voltage_level_id).arg(msg.equipment_interval_uuid).arg(msg.device_point_name).arg(msg.revice_user_name);
	}
	else
	{
		sqlString = QString("SELECT count(*) FROM note_message,devices,voltage_level,equipment_interval,device_point_name,user_config,send_freq WHERE note_message.fault_name_uuid=devices.device_uuid AND devices.voltage_level_id = voltage_level.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid AND device_point_name.device_point_type_uuid = devices.device_point_type_uuid AND note_message.send_freq_id=send_freq.send_freq_id AND note_message.user_receive_uuid = user_config.user_uuid AND voltage_level.voltage_level_id='%1' AND equipment_interval.equipment_interval_uuid='%2' AND device_point_name.device_point_type_name like '%%3%' AND user_config.user_name like '%%4%';")
			.arg(msg.voltage_level_id).arg(msg.equipment_interval_uuid).arg(msg.device_point_name).arg(msg.revice_user_name);
	}

	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		m_count = query.value(0).toInt();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getSystemMessageSubscribeDB(int m_count, int m_pageCount, QList<WheelNoteMessage> &data, WheelAlarmMessageIf msg)
{
	QSqlQuery query;
	bool bReturn = false;
	WheelNoteMessage c_data;
	QString sqlString2;
	if (msg.revice_user_name.isEmpty())
	{
		sqlString2 = QString("SELECT note_message.note_uuid,note_message.fault_name_uuid,alarm_system_name.sys_alarm_name,user_config.user_name,note_message.send_time,send_freq.send_freq_name FROM note_message,user_config,send_freq,alarm_system_name WHERE note_message.send_freq_id=send_freq.send_freq_id AND note_message.user_receive_uuid = user_config.user_uuid AND note_message.fault_name_uuid = alarm_system_name.sys_alarm_id limit %1,%2;")
			.arg(m_count).arg(m_pageCount);
	}
	else
	{
		sqlString2 = QString("SELECT note_message.note_uuid,note_message.fault_name_uuid,alarm_system_name.sys_alarm_name,user_config.user_name,note_message.send_time,send_freq.send_freq_name FROM note_message,user_config,send_freq,alarm_system_name WHERE note_message.send_freq_id=send_freq.send_freq_id AND note_message.user_receive_uuid = user_config.user_uuid AND note_message.fault_name_uuid = alarm_system_name.sys_alarm_id AND user_config.user_name like '%%1%' limit %2,%3;")
			.arg(msg.revice_user_name).arg(m_count).arg(m_pageCount);
	}

    bReturn = querySqlString(sqlString2, query);
	while (query.next())
	{
		int i = 0;
		c_data.note_uuid = query.value(i++).toString();
		c_data.fault_name_uuid = query.value(i++).toString();
		c_data.fault_name = query.value(i++).toString();
		c_data.revice_user_name = query.value(i++).toString();
		c_data.send_freq = query.value(i++).toString();
		c_data.alarm_type = query.value(i++).toString();
		data.append(c_data);
	}
	return bReturn;
}
bool LibDLWheelRobotDBOperation::getSystemMessageSubscribeCountDB(int &m_count, WheelAlarmMessageIf msg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString2;
	if (msg.equipment_interval_uuid.isEmpty())
	{
		sqlString2 = QString("SELECT count(*) FROM note_message,user_config,send_freq,alarm_system_name WHERE note_message.send_freq_id=send_freq.send_freq_id AND note_message.user_receive_uuid = user_config.user_uuid AND note_message.fault_name_uuid = alarm_system_name.sys_alarm_id;")
			.arg(msg.revice_user_name);
	}
	else
	{
		sqlString2 = QString("SELECT count(*) FROM note_message,user_config,send_freq,alarm_system_name WHERE note_message.send_freq_id=send_freq.send_freq_id AND note_message.user_receive_uuid = user_config.user_uuid AND note_message.fault_name_uuid = alarm_system_name.sys_alarm_id AND user_config.user_name like '%%';")
			.arg(msg.revice_user_name);
	}

    bReturn = querySqlString(sqlString2, query);
	while (query.next())
	{
		m_count = query.value(0).toInt();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceUUidWithType(QString fault_name_uuid, QStringList &m_dev, WheelDeviceTreePath type)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString;

	switch (type)
	{
	case Voltage_Level:
	{
		sqlString = QString("SELECT devices.device_uuid FROM devices,voltage_level WHERE voltage_level.voltage_level_id=devices.voltage_level_id AND voltage_level.voltage_level_id='%1';")
			.arg(fault_name_uuid);
		break;
	}
	case Equipment_Interval:
	{
		sqlString = QString("SELECT devices.device_uuid FROM devices,equipment_interval WHERE equipment_interval.equipment_interval_uuid=devices.equipment_interval_uuid AND equipment_interval.equipment_interval_uuid='%1';")
			.arg(fault_name_uuid);
		break;
	}
	case Device_Type:
	{
		sqlString = QString("SELECT devices.device_uuid FROM devices,device_type WHERE device_type.device_type_uuid=devices.device_type_uuid AND device_type.device_type_uuid='%1';")
			.arg(fault_name_uuid);
		break;
	}
	default:
		break;
	}

	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		m_dev.append(query.value(0).toString());
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertNoteMessageDB(QString noteUUid, WheelSubMsgInsert noteMsg, QStringList fault_uuid, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	int strLength = fault_uuid.size();
	int insertCount = 0;
	if (strLength % 2000 != 0)
	{
		insertCount = strLength / 2000 + 1;
	}
	else
	{
		insertCount = strLength / 2000;
	}

	for (int j = 0; j < insertCount; j++)
	{
		QString sqlString = QString("INSERT INTO note_message ");
		for (int i = j * 2000; i < (j + 1) * 2000; i++)
		{
			qDebug() << "正在插入数据库表task_devices:当前第" << i + 1 << "条！";
			QString sqlString_2;
			if (i == j * 2000)
			{
				sqlString_2 = QString("select '%1','%2','%3','%4','%5','%6'")
					.arg(noteUUid)
					.arg(noteMsg.user_receive_uuid)
					.arg(noteMsg.alarm_level_id)
					.arg(noteMsg.send_time)
					.arg(noteMsg.send_freq_id)
					.arg(fault_uuid[i]);
			}
			else
			{
				sqlString_2 = QString("union all select '%1','%2','%3','%4','%5','%6'")
					.arg(noteUUid)
					.arg(noteMsg.user_receive_uuid)
					.arg(noteMsg.alarm_level_id)
					.arg(noteMsg.send_time)
					.arg(noteMsg.send_freq_id)
					.arg(fault_uuid[i]);
			}

			sqlString = sqlString + sqlString_2;

			if (i == (j + 1) * 2000 - 1)
			{
				sqlString = sqlString + ";";
			}
			if (i == strLength - 1)
			{
				sqlString = sqlString + ";";
				break;
			}
		}
		bReturn = querySqlString(sqlString, query);
	}
	if (!bReturn)
	{
		retMsg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteNoteMessageDB(QString noteUUid, QString fault_name_uuid, QString &retMsg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("DELETE FROM note_message WHERE note_uuid='%1' AND fault_name_uuid='%2';")
		.arg(noteUUid).arg(fault_name_uuid);
	bReturn = querySqlString(sqlString, query);
	if (!bReturn)
	{
		retMsg = query.lastError().text();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getUserPhoneNumberByDeviceNumber(QString &userPhoneNumber, QString device_uuid)
{
    QSqlQuery query;
    bool bReturn = false;
    QString sqlString = QString("SELECT user_telephone FROM user_config JOIN note_message WHERE note_message.fault_name_uuid = '%1' AND user_config.user_uuid = note_message.user_receive_uuid;")
        .arg(device_uuid);
    bReturn = querySqlString(sqlString, query);
    while(query.next())
	{
        userPhoneNumber = query.value(0).toString();
    }
	ROS_ERROR("DB error:getUserPhoneNumberByDeviceNumber:sql:%s,bool:%d,device_uuid:%s,userNumber:%s ", sqlString.toStdString().c_str(), bReturn, device_uuid.toStdString().c_str(), userPhoneNumber.toStdString().c_str());
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getSystemNameAndUUidDB(QList<WeelSystemAlarm> &data)
{
	QSqlQuery query;
	bool bReturn = false;
	WeelSystemAlarm sys;
	QString sqlString = QString("SELECT * FROM alarm_system_name;");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		sys.SysAlarmUUid = query.value(i++).toString();
		sys.SysAlarmName = query.value(i++).toString();
		data.append(sys);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelVirtualDeviceThreeCompareMap(QMap<QString, DeviceVirtualSort> &deviceMap)
{
	QSqlQuery query;
	bool bReturn = false;
	DeviceVirtualSort data;
	QString sqlString = QString("SELECT device_uuid,equipment_interval_uuid,device_point_type_uuid,device_phase_id,sub_device_type_uuid FROM devices WHERE device_phase_id !='1' AND (recognition_type_id = '4' OR recognition_type_id = '6') ORDER BY equipment_interval_uuid,device_point_type_uuid,device_phase_id;");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		data.device_uuid = query.value(i++).toString();
		data.equipment_interval_uuid = query.value(i++).toString();
		data.device_point_type_uuid = query.value(i++).toString();
		data.device_phase_id = query.value(i++).toString();
		data.sub_device_type_uuid = query.value(i++).toString();

		deviceMap.insert(data.device_uuid, data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getWheelVirtualDeviceRevealAmpereMap(QMap<QString, DeviceVirtualSort> &deviceMap)
{
	QSqlQuery query;
	bool bReturn = false;
	DeviceVirtualSort data;
	QString sqlString = QString("SELECT device_uuid,equipment_interval_uuid,sub_device_type_uuid,device_point_type_uuid,device_phase_id FROM devices WHERE device_phase_id !='1' AND meter_type_id='3' ORDER BY equipment_interval_uuid,device_point_type_uuid,device_phase_id;");
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		int i = 0;
		data.device_uuid = query.value(i++).toString();
		data.equipment_interval_uuid = query.value(i++).toString();
		data.sub_device_type_uuid = query.value(i++).toString();
		data.device_point_type_uuid = query.value(i++).toString();
		data.device_phase_id = query.value(i++).toString();

		deviceMap.insert(data.device_uuid, data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::updateDeviceVirtualAndInsertVirtual(QString virtualUUid, QList<QString> updateDevice, QString subDeviceUUid)
{
	QSqlQuery query;
	bool bReturn = false;

	if (updateDevice.size() < 2)
		return false;

	for (int i = 0; i < updateDevice.size(); i++)
	{
		QString sqlString = QString("UPDATE devices SET virtual_uuid='%1' WHERE device_uuid='%2';")
			.arg(virtualUUid)
			.arg(updateDevice[i]);
		bReturn = querySqlString(sqlString, query);
		if (!bReturn)
		{
			return bReturn;
		}
	}
	
	QString relevanceDeviceUUid = updateDevice.join(" ");
	QString sqlString = QString("INSERT INTO virtual_device SET virtual_device_uuid='%1',relevance_device_uuid='%2',sub_device_type_uuid = '%3',threshold_uuid='%4';")
		.arg(virtualUUid)
		.arg(relevanceDeviceUUid)
		.arg(subDeviceUUid)
		.arg("temporary");
	bReturn = querySqlString(sqlString, query);

	QSqlQuery query2;
	QString sqlString2 = QString("SELECT * FROM devices WHERE devices.device_uuid='%1'").arg(updateDevice[0]);
	bReturn = querySqlString(sqlString2, query2);
	QStringList vieDevice;
	while (query2.next())
	{
		vieDevice.append(virtualUUid);
		for (int i = 1; i < 16; i++)
		{
			vieDevice.append(query2.value(i).toString());
		}
	}

	QString sqlString3 = QString("INSERT INTO devices SET device_uuid='%1',voltage_level_id='%2',equipment_interval_uuid='%3',device_area_uuid='%4',device_type_uuid='%5',sub_device_type_uuid='%6',device_point_type_uuid='%7',unit_type_uuid='%8',recognition_type_id='%9',save_type_id='%10',meter_type_id='%11',fever_type_id='%12',device_phase_id=5;")
		.arg(vieDevice[0]).arg(vieDevice[1]).arg(vieDevice[2]).arg(vieDevice[3]).arg(vieDevice[4])
		.arg(vieDevice[5]).arg(vieDevice[6]).arg(vieDevice[7]).arg(vieDevice[8]).arg(vieDevice[9])
		.arg(vieDevice[10]).arg(vieDevice[11]);
	bReturn = querySqlString(sqlString3, query);
	
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getVirtualRelevanceDeviceUUid(QString taskUUid, QString deviceUUid, QString &releDev, QString &virtualUUid)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT virtual_device.relevance_device_uuid,virtual_device.virtual_device_uuid FROM devices,inspect_result,virtual_device WHERE inspect_result.device_uuid = devices.device_uuid AND devices.virtual_uuid = virtual_device.virtual_device_uuid AND inspect_result.task_uuid='%1' AND inspect_result.device_uuid='%2';")
		.arg(taskUUid)
		.arg(deviceUUid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		releDev = query.value(0).toString();
		virtualUUid = query.value(1).toString();
	}
	if (releDev.isEmpty())
	{
		return false;
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getInspectResultValueForDeviceUUid(QString taskUUid, QString devUUid, QString &resultValue)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT inspect_result FROM inspect_result WHERE task_uuid='%1' AND device_uuid='%2';")
		.arg(taskUUid)
		.arg(devUUid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		resultValue = query.value(0).toString();
	}
	if (resultValue.isEmpty())
	{
		return false;
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getVirtualDeviceUUid(QString deviceUUid, virtualDeviceConfig &cfg)
{
	QSqlQuery query;
	bool bReturn = false;
	QString sqlString = QString("SELECT virtual_device.virtual_device_uuid,virtual_device.sub_device_type_uuid,sub_device_type.sub_device_name,device_point_name.device_point_type_name,devices.device_phase_id FROM device_point_name,devices,virtual_device,sub_device_type WHERE devices.device_uuid='%1' AND devices.virtual_uuid=virtual_device.virtual_device_uuid AND sub_device_type.sub_device_type_uuid=virtual_device.sub_device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid;")
		.arg(deviceUUid);
	bReturn = querySqlString(sqlString, query);
	while (query.next())
	{
		cfg.virtual_uuid = query.value(0).toString();
		cfg.sub_device_type_uuid = query.value(1).toString();
		cfg.sub_device_type_name = query.value(2).toString() + query.value(3).toString();
		cfg.device_phase_id = query.value(4).toInt();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::dataClearForVirtual()
{
	QSqlQuery query;
	bool bReturn = false;

	bReturn = querySqlString("UPDATE devices SET virtual_uuid = NULL;", query);
	bReturn = querySqlString("delete devices from devices,virtual_device where devices.device_uuid=virtual_device.virtual_device_uuid;", query);
	bReturn = querySqlString("truncate table virtual_device;", query);

	return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceNameForTaskUUid(QString &taskUUid, QMap<QString, QString> &data)
{
	QSqlQuery query;
	bool bReturn = false;

	QString str = QString("SELECT inspect_result.device_uuid, device_point_name.device_point_type_name, device_phase.device_phase_name FROM inspect_result, devices, device_point_name, device_phase WHERE inspect_result.device_uuid = devices.device_uuid AND devices.device_point_type_uuid = device_point_name.device_point_type_uuid AND devices.device_phase_id = device_phase.device_phase_id AND inspect_result.task_uuid = '%1';").arg(taskUUid);
	bReturn = querySqlString(str, query);
	while (query.next())
	{
		QString deviceUUid = query.value(0).toString();
		QString devicePointName = query.value(1).toString();
		QString devicePhase = query.value(2).toString();
		data[deviceUUid] = devicePointName.replace("X", devicePhase);
	}

	return bReturn;
}

bool LibDLWheelRobotDBOperation::updateFastAuditTaskDB(QString taskUUid)
{
	QSqlQuery query;
	bool bReturn = false;

	QString str = QString("UPDATE inspect_result SET is_dealed=1 WHERE task_uuid='%1';").arg(taskUUid);
	bReturn = querySqlString(str, query);

	return bReturn;
}

bool LibDLWheelRobotDBOperation::getNewWheelCollectTreeDataDB(QList<QStringList> &m_collectTreeDat, QStringList selectUUid, RootNodeType selectType)
{
	QSqlQuery query;
	bool bReturn = false;
	QString str;
	switch (selectType)
	{
	case RootNode_VoltageLevel:
		str = QString("SELECT voltage_level.voltage_level_id,voltage_level.voltage_level_name,equipment_interval.equipment_interval_uuid, equipment_interval.equipment_interval_name,devices.device_type_uuid, device_type.device_type_name, devices.device_point_type_uuid, devices.device_phase_id, device_point_name.device_point_type_name, devices.device_uuid, device_parameter.ptz_tilt FROM voltage_level LEFT JOIN equipment_interval ON voltage_level.voltage_level_id = equipment_interval.voltage_level_id LEFT JOIN devices ON voltage_level.voltage_level_id = devices.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid LEFT JOIN device_type ON devices.device_type_uuid = device_type.device_type_uuid LEFT JOIN device_point_name  ON devices.device_point_type_uuid = device_point_name.device_point_type_uuid LEFT JOIN device_parameter ON devices.device_uuid = device_parameter.device_uuid WHERE voltage_level.voltage_level_id = '%1' ORDER BY voltage_level.voltage_level_name, equipment_interval.equipment_interval_name,device_type.device_type_name, device_point_name.device_point_type_name;")
			.arg(selectUUid[0]);
		break;
	case RootNode_Interval:
		str = QString("SELECT voltage_level.voltage_level_id,voltage_level.voltage_level_name,equipment_interval.equipment_interval_uuid, equipment_interval.equipment_interval_name,devices.device_type_uuid, device_type.device_type_name, devices.device_point_type_uuid, devices.device_phase_id, device_point_name.device_point_type_name, devices.device_uuid, device_parameter.ptz_tilt FROM voltage_level LEFT JOIN equipment_interval ON voltage_level.voltage_level_id = equipment_interval.voltage_level_id LEFT JOIN devices ON voltage_level.voltage_level_id = devices.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid LEFT JOIN device_type ON devices.device_type_uuid = device_type.device_type_uuid LEFT JOIN device_point_name  ON devices.device_point_type_uuid = device_point_name.device_point_type_uuid LEFT JOIN device_parameter ON devices.device_uuid = device_parameter.device_uuid WHERE voltage_level.voltage_level_id = '%1' AND equipment_interval.equipment_interval_uuid='%2' ORDER BY voltage_level.voltage_level_name, equipment_interval.equipment_interval_name,device_type.device_type_name, device_point_name.device_point_type_name;")
			.arg(selectUUid[0]).arg(selectUUid[1]);
		break;
	case RootNode_DeviceType:
		str = QString("SELECT voltage_level.voltage_level_id,voltage_level.voltage_level_name,equipment_interval.equipment_interval_uuid, equipment_interval.equipment_interval_name,devices.device_type_uuid, device_type.device_type_name, devices.device_point_type_uuid, devices.device_phase_id, device_point_name.device_point_type_name, devices.device_uuid, device_parameter.ptz_tilt FROM voltage_level LEFT JOIN equipment_interval ON voltage_level.voltage_level_id = equipment_interval.voltage_level_id LEFT JOIN devices ON voltage_level.voltage_level_id = devices.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid LEFT JOIN device_type ON devices.device_type_uuid = device_type.device_type_uuid LEFT JOIN device_point_name  ON devices.device_point_type_uuid = device_point_name.device_point_type_uuid LEFT JOIN device_parameter ON devices.device_uuid = device_parameter.device_uuid WHERE voltage_level.voltage_level_id = '%1' AND equipment_interval.equipment_interval_uuid='%2' AND device_type.device_type_uuid='%3' ORDER BY voltage_level.voltage_level_name, equipment_interval.equipment_interval_name,device_type.device_type_name, device_point_name.device_point_type_name;")
			.arg(selectUUid[0]).arg(selectUUid[1]).arg(selectUUid[2]);
		break;
	case RootNode_Device:
		return true;
		break;
	case RootNode_Init:
		str = QString("SELECT voltage_level.voltage_level_id,voltage_level.voltage_level_name,equipment_interval.equipment_interval_uuid, equipment_interval.equipment_interval_name,devices.device_type_uuid, device_type.device_type_name, devices.device_point_type_uuid, devices.device_phase_id, device_point_name.device_point_type_name, devices.device_uuid, device_parameter.ptz_tilt FROM voltage_level LEFT JOIN equipment_interval ON voltage_level.voltage_level_id = equipment_interval.voltage_level_id LEFT JOIN devices ON voltage_level.voltage_level_id = devices.voltage_level_id AND devices.equipment_interval_uuid = equipment_interval.equipment_interval_uuid LEFT JOIN device_type ON devices.device_type_uuid = device_type.device_type_uuid LEFT JOIN device_point_name  ON devices.device_point_type_uuid = device_point_name.device_point_type_uuid LEFT JOIN device_parameter ON devices.device_uuid = device_parameter.device_uuid ORDER BY voltage_level.voltage_level_name, equipment_interval.equipment_interval_name, device_type.device_type_name,device_point_name.device_point_type_name;");
		break;
	default:
		break;
	}
	bReturn = querySqlString(str, query);
	while (query.next())
	{
	
		QStringList _data;
		if (query.value(0).toString() == "")
		{
			continue;
		}
		else
		{
			_data.append(query.value(0).toString());
			_data.append(query.value(1).toString());
		}

		if (query.value(2).toString() == "")
		{
			_data.append(""); _data.append("");
			_data.append(""); _data.append("");
			_data.append(""); _data.append("");
			_data.append(""); _data.append("");
			m_collectTreeDat.append(_data);
			continue;
		}
		else
		{
			_data.append(query.value(2).toString());
			_data.append(query.value(3).toString());
		}

		if (query.value(4).toString() == "")
		{
			_data.append(""); _data.append("");
			_data.append(""); _data.append("");
			_data.append(""); _data.append("");
			m_collectTreeDat.append(_data);
			continue;
		}
		else
		{
			_data.append(query.value(4).toString());
			_data.append(query.value(5).toString());
		}

		if (query.value(6).toString() == "")
		{
			_data.append(""); _data.append("");
			_data.append(""); _data.append("");
			m_collectTreeDat.append(_data);
			continue;
		}
		else
		{
			_data.append(query.value(6).toString());

			if (query.value(7).toInt() == 2)
			{
				_data.append(query.value(8).toString().replace("X", "A"));
			}
			else if (query.value(7).toInt() == 3)
			{
				_data.append(query.value(8).toString().replace("X", "B"));
			}
			else if (query.value(7).toInt() == 4)
			{
				_data.append(query.value(8).toString().replace("X", "C"));
			}
			else
			{
				_data.append(query.value(8).toString());
			}
			_data.append(query.value(9).toString());
		}
		if (query.value(10).toInt() == -1)
		{
			_data.append("0");
		}
		else
		{
			_data.append("1");
		}
		m_collectTreeDat.append(_data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::selectDeviceDataWithIntervalUUid(QList<QStringList> &_deviceData, QString intervalUUid, QString originalVoltageLevelUUid)
{
	QSqlQuery query;
	bool bReturn = false;

	QString str = QString("SELECT device_area_uuid,device_type_uuid,sub_device_type_uuid,device_point_type_uuid,unit_type_uuid,recognition_type_id,save_type_id,meter_type_id,fever_type_id,threshold_filename,device_phase_id,alarm_level_id,start_using FROM devices WHERE voltage_level_id='%1'AND equipment_interval_uuid='%2';")
		.arg(originalVoltageLevelUUid)
		.arg(intervalUUid);
	bReturn = querySqlString(str, query);
	while (query.next())
	{
		QStringList data;
		for (int i = 0; i < 13; i++)
		{
			data.append(query.value(i).toString());
		}
		_deviceData.append(data);
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::insertDeviceWithCopyInterval(QList<QStringList> deviceData, QString newVoltageLevelUUid, QString equipment_interval_uuid)
{
	boost::mutex::scoped_lock lock(m_lock);
	QSqlQuery query;
	query.exec("START TRANSACTION");
	//输入插入
	for (int i = 0; i < deviceData.size(); i++)
	{
		QString deviceUUid = QUuid::createUuid().toString().remove("{").remove("}").remove("-");
		QString str = QString("INSERT INTO devices(device_uuid,voltage_level_id,equipment_interval_uuid,device_area_uuid,device_type_uuid,sub_device_type_uuid,device_point_type_uuid,unit_type_uuid,recognition_type_id,save_type_id,meter_type_id,fever_type_id,threshold_filename,device_phase_id,alarm_level_id,start_using)"
			" VALUES('%1','%2','%3','%4','%5','%6','%7','%8','%9','%10','%11','%12','%13','%14','%15','%16');")
			.arg(deviceUUid)
			.arg(newVoltageLevelUUid)
			.arg(equipment_interval_uuid)
			.arg(deviceData[i][0])
			.arg(deviceData[i][1])
			.arg(deviceData[i][2])
			.arg(deviceData[i][3])
			.arg(deviceData[i][4])
			.arg(deviceData[i][5])
			.arg(deviceData[i][6])
			.arg(deviceData[i][7])
			.arg(deviceData[i][8])
			.arg(deviceData[i][9])
			.arg(deviceData[i][10])
			.arg(deviceData[i][11])
			.arg(deviceData[i][12]);

		query.exec(str);
		QString parameterStr = QString("INSERT INTO device_parameter(device_uuid,point_id,ptz_pan,ptz_tilt,hc_zoom_near,hc_focus_near,hc_zoom_far,hc_focus_far,mag_focus,video_length,audio_length) VALUES('%1',-1,-1,-1,-1,-1,-1,-1,-1,-1,-1);")
			.arg(deviceUUid);
		query.exec(parameterStr);
	}
	//提交事务
	query.exec("COMMIT");
	return true;
}

bool LibDLWheelRobotDBOperation::selectPointUUidAndDeviceTypeUUid(QList<QStringList> &deviceData, QStringList deviceTypeUUidList, QString voltageLevelUUid)
{
	QSqlQuery query;
	bool bReturn = false;

	QString deviceType = "";
	for (int i = 0; i < deviceTypeUUidList.size(); i++)
	{
		deviceType = deviceType + QString(" device_type.device_type_uuid='%1'").arg(deviceTypeUUidList[i]);
		if (i != deviceTypeUUidList.size() - 1)
		{
			deviceType = deviceType + QString(" OR ");
		}
	}

	QString str = QString("SELECT device_type.device_type_uuid,sub_device_type.sub_device_type_uuid,device_point_name.device_point_type_uuid,device_point_name.unit_type_id,device_point_name.recognition_type_id,device_point_name.save_type_id,device_point_name.meter_type_id,device_point_name.fever_type_id,device_point_name.device_point_type_name FROM device_type, sub_device_type, device_point_name WHERE device_type.device_type_uuid = sub_device_type.device_type_uuid AND sub_device_type.sub_device_type_uuid = device_point_name.sub_device_type_uuid AND (%1);")
		.arg(deviceType);
	bReturn = querySqlString(str, query);
	while (query.next())
	{
		QStringList data;
		data.append(voltageLevelUUid);
		for (int i = 0; i < 8; i++)
		{
			data.append(query.value(i).toString());
		}
		data.append("");
		data.append("1");
		data.append("6");
		data.append("1");
		if (query.value(8).toString().contains("X", Qt::CaseSensitive))
		{
			data[10] = QString("2");
			deviceData.append(data);
			data[10] = QString("3");
			deviceData.append(data);
			data[10] = QString("4");
			deviceData.append(data);
		}
		else
		{
			deviceData.append(data);
		}
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::deleteDeviceTypeForDevices(QString voltageLevelUUid, QString intervalUUid, QString deviceTypeUUid)
{
	QSqlQuery query;
	QString str = QString("DELETE FROM devices WHERE voltage_level_id='%1' AND equipment_interval_uuid = '%2' AND device_type_uuid = '%3';")
		.arg(voltageLevelUUid)
		.arg(intervalUUid)
		.arg(deviceTypeUUid);
	bool bReturn = querySqlString(str, query);
	return bReturn;
}

bool LibDLWheelRobotDBOperation::selectPointAllDataWithPointList(QList<QStringList> &deviceData, QStringList _data, QString voltageLevelUUid)
{
	QSqlQuery query;
	bool bReturn = false;

	QString pointNameUUid = "";
	for (int i = 0; i < _data.size(); i++)
	{
		pointNameUUid = pointNameUUid + QString(" device_point_name.device_point_type_uuid='%1'").arg(_data[i]);
		if (i != _data.size() - 1)
		{
			pointNameUUid = pointNameUUid + QString(" OR ");
		}
	}

	QString str = QString("SELECT device_type.device_type_uuid,sub_device_type.sub_device_type_uuid,device_point_name.device_point_type_uuid,device_point_name.unit_type_id,device_point_name.recognition_type_id,device_point_name.save_type_id,device_point_name.meter_type_id,device_point_name.fever_type_id,device_point_name.device_point_type_name FROM device_type, sub_device_type, device_point_name WHERE device_type.device_type_uuid = sub_device_type.device_type_uuid AND sub_device_type.sub_device_type_uuid = device_point_name.sub_device_type_uuid AND (%1);")
		.arg(pointNameUUid);
	bReturn = querySqlString(str, query);
	while (query.next())
	{
		QStringList data;
		data.append(voltageLevelUUid);
		for (int i = 0; i < 8; i++)
		{
			data.append(query.value(i).toString());
		}
		data.append("");
		data.append("1");
		data.append("6");
		data.append("1");
		if (query.value(8).toString().contains("A", Qt::CaseSensitive))
		{
			data[10] = QString("2");
			deviceData.append(data);
		}
		else if (query.value(8).toString().contains("B", Qt::CaseSensitive))
		{
			data[10] = QString("3");
			deviceData.append(data);
		}
		else if (query.value(8).toString().contains("C", Qt::CaseSensitive))
		{
			data[10] = QString("4");
			deviceData.append(data);
		}
		else
		{
			deviceData.append(data);
		}
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getPointId(QString strDeviceUUid, QString &strPointId)
{
	QSqlQuery query;
	QString str = QString("SELECT point_id FROM device_parameter WHERE device_uuid='%1';")
		.arg(strDeviceUUid);
	bool bReturn = querySqlString(str, query);
	while (query.next())
	{
		strPointId = query.value(0).toString();
	}
	return bReturn;
}

bool LibDLWheelRobotDBOperation::getThresholdValue(QString deviceUUid, QStringList &jsonValue)
{
    QSqlQuery query;

    QString str = QString("SELECT threshold.alarm_normal,threshold.alarm_warning,threshold.alarm_common,threshold.alarm_serial,threshold.alarm_danger FROM threshold,devices WHERE threshold.threshold_uuid=devices.threshold_filename AND devices.device_uuid='%2';")
        .arg(deviceUUid);
    bool bReturn = querySqlString(str, query);
    while (query.next())
    {
        jsonValue.append(query.value(0).toString());
        jsonValue.append(query.value(1).toString());
        jsonValue.append(query.value(2).toString());
        jsonValue.append(query.value(3).toString());
        jsonValue.append(query.value(4).toString());
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getBJDeviceUUidWithEquipment(QStringList &devList, QString eqUUid)
{
    QSqlQuery query;
    QString str = QString("SELECT device_uuid FROM devices WHERE equipment_interval_uuid='%！';")
        .arg(eqUUid);
    bool bReturn = querySqlString(str, query);
    while (query.next())
    {
        devList.append(query.value(0).toString());
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getBJDeviceUUidWithDevicetype(QStringList &devList, QString tyUUid)
{
    QSqlQuery query;
    QString str = QString("SELECT device_uuid FROM devices WHERE device_type_uuid='%！';")
        .arg(tyUUid);
    bool bReturn = querySqlString(str, query);
    while (query.next())
    {
        devList.append(query.value(0).toString());
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getBJDeviceModelData(QList<BJDeviceModel> &_data)
{
    QSqlQuery query;
    QString str = QString("SELECT device_parameter.device_id,device_point_name.device_point_type_name,equipment_interval.equipment_interval_uuid,voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_type.device_type_uuid,device_type.device_type_name,meter_type.meter_type_name,save_type.save_type_name,recognition_type.recognition_type_name,device_phase.device_phase_name FROM voltage_level,device_parameter,equipment_interval,device_type,device_point_name,save_type,recognition_type,device_phase,devices LEFT JOIN meter_type ON devices.meter_type_id = meter_type.meter_type_id WHERE devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.device_point_type_uuid = device_point_name.device_point_type_uuid AND devices.save_type_id = save_type.save_type_id AND devices.recognition_type_id = recognition_type.recognition_type_id AND devices.device_phase_id = device_phase.device_phase_id AND device_parameter.device_uuid=devices.device_uuid AND devices.voltage_level_id=voltage_level.voltage_level_id;");
    bool bReturn = querySqlString(str, query);
    while (query.next())
    {
        int i = 0;
        BJDeviceModel data;
        data.deviceId = query.value(i++).toString();
        data.deviceName = query.value(i++).toString();
        data.bayId = query.value(i++).toString();
        data.bayName = query.value(i++).toString();
        data.bayName = data.bayName + "," + query.value(i++).toString();
        data.mainDeviceId = query.value(i++).toString();
        data.device_type_item_name = query.value(i++).toString();
        //data.deviceType = query.value(i++).toString();
        data.meterType = query.value(i++).toString();
        //data.appearanceType = query.value(i++).toString();
        data.saveTypeList = query.value(i++).toString();
        data.recognitionTypeList = query.value(i++).toString();
        data.phase = query.value(i++).toString();
        if (data.device_type_item_name.contains("隔离开关", Qt::CaseSensitive))
        {
            data.mainDeviceName = "隔离开关";
        }
        else
        {
            data.mainDeviceName = data.device_type_item_name;
        }
        //data.deviceInfo = query.value(i++).toString();
        _data.append(data);
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getBJTaskData(QString taskUUid, BJRobotTaskStatusData &data)
{
    QSqlQuery query;
    QString str = QString("SELECT task.task_code,task.task_start_time,task.task_status_id FROM task WHERE task.task_uuid='%1';")
        .arg(taskUUid);
    bool bReturn = querySqlString(str, query);
    while (query.next())
    {
        int i = 0;
        data.task_code = query.value(i++).toString();
        data.task_start_time = query.value(i++).toString();
        data.task_state = query.value(i++).toString();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getBJDeviceResultFromDB(QString taskUUid, QString deviceUUid, BJDeviceResultSendData &data)
{
    QSqlQuery query;
    QString str = QString("SELECT task.task_name,task.task_code,device_point_name.device_point_type_name,devices.device_uuid,inspect_result.inspect_result,unit_type.unit_type_name,inspect_result.inspect_time,recognition_type.recognition_type_name,task.task_uuid FROM task,device_point_name,inspect_result,recognition_type,devices LEFT JOIN unit_type ON devices.unit_type_uuid=unit_type.unit_type_id WHERE device_point_name.recognition_type_id=recognition_type.recognition_type_id AND task.task_uuid=inspect_result.task_uuid AND devices.device_uuid=inspect_result.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_uuid='%1' AND devices.device_uuid='%2';")
        .arg(taskUUid)
        .arg(deviceUUid);
    bool bReturn = querySqlString(str, query);
    while (query.next())
    {
        int i = 0;
        data.task_name = query.value(i++).toString();
        data.task_code = query.value(i++).toString();
        data.device_name = query.value(i++).toString();
        data.device_id = query.value(i++).toString();
        data.value = query.value(i++).toString();
        data.unit = query.value(i++).toString();
        data.time = query.value(i++).toString().replace("T", " ");
        data.recognition_type = query.value(i++).toString();
        data.task_patrolled_id = query.value(i++).toString();
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getBJDeviceUUidWithDeviceId(QStringList &devList)
{
    QStringList deviceLs = devList;
    devList.clear();
    QSqlQuery query;
    bool bReturn = false;
    for (int i = 0; i < deviceLs.size(); i++)
    {
        QString str = QString("SELECT device_uuid FROM device_parameter WHERE device_id='%1';")
            .arg(deviceLs[i]);
        bReturn = querySqlString(str, query);
        while (query.next())
        {
            devList.append(query.value(0).toString());
        }
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceNameWithDeviceUUid(QString deviceUUid, QString &stdDeviceName)
{
    QSqlQuery query;
    bool bReturn = false;

    QString str = QString("SELECT voltage_level.voltage_level_name,equipment_interval.equipment_interval_name,device_type.device_type_name,device_point_name.device_point_type_name FROM devices,voltage_level,equipment_interval,device_type,device_point_name WHERE devices.voltage_level_id=voltage_level.voltage_level_id AND devices.equipment_interval_uuid=equipment_interval.equipment_interval_uuid AND devices.device_type_uuid=device_type.device_type_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND devices.device_uuid='%1';")
        .arg(deviceUUid);
    bReturn = querySqlString(str, query);
    while (query.next())
    {
        stdDeviceName = stdDeviceName + query.value(0).toString() + "-";
        stdDeviceName = stdDeviceName + query.value(1).toString() + "-";
        stdDeviceName = stdDeviceName + query.value(2).toString() + "-";
        stdDeviceName = stdDeviceName + query.value(3).toString();
    }

    return bReturn;
}

bool LibDLWheelRobotDBOperation::getDeviceUUidWhihTaskUUid(QString taskUUid, QStringList &deviceList)
{
    QSqlQuery query;
    bool bReturn = false;

    QString str = QString("select task_devices.device_uuid from task,task_devices where task.task_edit_uuid=task_devices.task_edit_uuid and task.task_uuid='%1';")
        .arg(taskUUid);
    bReturn = querySqlString(str, query);
    while (query.next())
    {
        deviceList.append(query.value(0).toString());
    }
    return bReturn;
}

bool LibDLWheelRobotDBOperation::getCheTaskDeviceNameResult(QString taskUUid, QList<QStringList> &data)
{
    QSqlQuery query;
    bool bReturn = false;

    QString str = QString("SELECT device_point_name.device_point_type_name,inspect_result.inspect_result FROM device_point_name,devices,task,task_devices,inspect_result WHERE task.task_edit_uuid=task_devices.task_edit_uuid AND task_devices.device_uuid=devices.device_uuid AND devices.device_point_type_uuid=device_point_name.device_point_type_uuid AND task.task_uuid=inspect_result.task_uuid AND task_devices.device_uuid=inspect_result.device_uuid AND task.task_uuid='%1' ORDER BY device_point_name.device_point_type_name;")
        .arg(taskUUid);
    bReturn = querySqlString(str, query);
    while (query.next())
    {
        QStringList _data;
        _data.append(query.value(0).toString());
        _data.append(query.value(1).toString());
        data.append(_data);
    }
    return bReturn;
}
