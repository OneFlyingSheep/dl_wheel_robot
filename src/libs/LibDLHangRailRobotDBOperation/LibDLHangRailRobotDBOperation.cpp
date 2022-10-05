#define COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, str, val) str.val = rec.value(rec.indexOf(#val)).toInt()
#define COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, str, val) str.val = rec.value(rec.indexOf(#val)).toString()

#define COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, str, val) rec.setValue(#val, QString::number(str.val))
#define COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, str, val) rec.setValue(#val, str.val)

#include "LibDLHangRailRobotDBOperation.h"
#include <QtSql/QSqlError>
#include <QDebug>
#include <QSettings>
#include "LibDLHangRailCommonTools/DLHangRailCommonTools.h"

#include <QApplication>
LibDLHangRailRobotDBOperation::LibDLHangRailRobotDBOperation()
{
// 	 DBInfoData_T dbInfoData =  DLHangRailCommonTools::getDBInfoData();
// 
// 	 openDb(dbInfoData.hostName, dbInfoData.dbName, dbInfoData.userName, dbInfoData.password);
}

LibDLHangRailRobotDBOperation::~LibDLHangRailRobotDBOperation()
{
    closeDb();
}

bool LibDLHangRailRobotDBOperation::getStationCfg(stationCfg &cfg)
{
    bool bRet = false;
    QString sqlStr = "SELECT * FROM station_cfg";
    QSqlQuery qry;
    bRet = querySqlString(sqlStr, qry);

    if (bRet)
    {
        while (qry.next())
        {
			cfg.station_ssid = qry.value(0).toString();
			cfg.station_name = qry.value(1).toString();
        }
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::updateStationCfg(stationCfg cfg)
{
    bool bRet = false;
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getAllDevicesDeviceSsId(QStringList &devicesLists)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT device_ssid FROM devices ORDER BY device_robot_offset ASC", qry);
    while (qry.next())
    {
        QSqlRecord rec = qry.record();
        QString ssid = rec.value(rec.indexOf("device_ssid")).toString();
        devicesLists.push_back(ssid);
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getAllDevices(QList<deviceConfigType> &devs)
{
    QSqlQuery qry;
    //QString qryStr = QString("SELECT * FROM devices WHERE (device_name_id != %1 AND device_name_id != %2 AND  device_name_id != %3)").arg().arg(156).arg(157);
    bool bRet = querySqlString("SELECT * FROM devices", qry);
    while (qry.next())
    {
        deviceConfigType d;
        QSqlRecord rec = qry.record();
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, device_ssid);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_alternate_name_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_name_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, device_area_name);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_area_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_robot_offset);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_lift_offset);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_body_rotate);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_cam_rotate);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_zoom);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_focus);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_thermo_focus);
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, station_ssid);

        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_zoom_two_stage);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_focus_two_stage);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_thermo_focus_two_stage);

        devs.push_back(d);
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getAllDevicesSortByDeviceAreaName(QList<deviceConfigType> &devs)
{
    QSqlQuery qry;
    //QString qryStr = QString("SELECT * FROM devices ORDER BY device_area_name WHERE device_name_id != %1 AND device_name_id != %2 AND  device_name_id != %3 ASC").arg(155).arg(156).arg(157);
    bool bRet = querySqlString("SELECT * FROM devices ORDER BY device_area_name ASC", qry);
    while (qry.next())
    {
        deviceConfigType d;
        QSqlRecord rec = qry.record();
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, device_ssid);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_alternate_name_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_name_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, device_area_name);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_area_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_robot_offset);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_lift_offset);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_body_rotate);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_cam_rotate);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_zoom);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_focus);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_thermo_focus);
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, station_ssid);
        //COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_body_rotate_two_stage);
        //COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_cam_rotate_two_stage);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_zoom_two_stage);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_focus_two_stage);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_thermo_focus_two_stage);

        devs.push_back(d);
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::insertSingleDevice(deviceConfigType dev, QString &errMsg)
{
//     QSqlTableModel devicesModel;
//      devicesModel.setTable("devices");
// //     devicesModel.setFilter(QString("device_area_id = %1 AND device_name_id = %2").arg(dev.device_area_id).arg(dev.device_name_id));    
//     devicesModel.select();
// 
// //     if (devicesModel.rowCount() > 0)
// //     {
// //         ROS_ERROR("same device, please delete former device first!, area_id:%d, name_id:%d", dev.device_area_id, dev.device_name_id);
// //         return false;
// //     }
// 
//     devicesModel.database().transaction();
// 
//     QSqlRecord rec = devicesModel.record();
//     COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, device_ssid);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_alternate_name_id);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_name_id);
// 	COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, device_area_name);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_area_id);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_robot_offset);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_lift_offset);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_body_rotate);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_cam_rotate);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_zoom);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_focus);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_thermo_focus);
//     COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, station_ssid);
//     //COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_body_rotate_two_stage);
//     //COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_cam_rotate_two_stage);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_zoom_two_stage);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_focus_two_stage);
//     COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_thermo_focus_two_stage);
// 
//     if (!devicesModel.insertRecord(-1, rec))
//     {                            
//         devicesModel.database().rollback();
//         ROS_ERROR("insertSingleDevice error, area_id:%d, name_id:%d", dev.device_area_id, dev.device_name_id);
//         return false;
//     }
// 
//     if (devicesModel.submitAll())
//     {
//         devicesModel.database().commit();
//         ROS_INFO("insertSingleDevice succeed");
//         return true;
//     }
//     else
//     {
//         devicesModel.database().rollback();
//         ROS_ERROR("insertSingleDevice submitAll error, area_id:%d, name_id:%d", dev.device_area_id, dev.device_name_id);
//         return false;
//     }
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("INSERT INTO devices SET device_ssid = '%1', device_alternate_name_id = '%2', device_name_id = '%3', device_area_name = '%4', device_area_id = '%5',device_robot_offset = '%6', device_lift_offset = '%7', device_body_rotate = '%8', device_cam_rotate = '%9', device_hc_zoom = '%10', device_hc_focus = '%11',device_thermo_focus = '%12', station_ssid = '%13', device_hc_zoom_two_stage = '%14',device_hc_focus_two_stage = '%15', device_thermo_focus_stage = '%16';")
		.arg(dev.device_ssid)
		.arg(dev.device_alternate_name_id)
		.arg(dev.device_name_id)
		.arg(dev.device_area_name)
		.arg(dev.device_area_id)
		.arg(dev.device_robot_offset)
		.arg(dev.device_lift_offset)
		.arg(dev.device_body_rotate)
		.arg(dev.device_cam_rotate)
		.arg(dev.device_hc_zoom)
		.arg(dev.device_hc_focus)
		.arg(dev.device_thermo_focus)
		.arg(dev.station_ssid)
		.arg(dev.device_hc_zoom_two_stage)
		.arg(dev.device_hc_focus_two_stage)
		.arg(dev.device_thermo_focus_two_stage);
	bRet = querySqlString(sqlStr, query);
	errMsg = query.lastError().text();
	return bRet;
}

bool LibDLHangRailRobotDBOperation::insertMultiDevices(QList<deviceConfigType> devs)
{
    QSqlTableModel devicesModel;
    bool bRet = true;
    devicesModel.setTable("devices");

    devicesModel.database().transaction();

    for (int i = 0; i < devs.size(); i++)
    {
        deviceConfigType dev = devs.at(i);

        devicesModel.setFilter(QString("device_area_name = %1 AND device_name_id = %2").arg(dev.device_area_name).arg(dev.device_name_id));
        devicesModel.select();

        if (devicesModel.record().count() > 0)
        {
            ROS_ERROR("same device, please delete former device first!, area_name:%s, name_id:%d", dev.device_area_name.toStdString().c_str(), dev.device_name_id);
            continue;
        }        

        QSqlRecord rec = devicesModel.record();
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, device_ssid);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_alternate_name_id);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_name_id);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_area_id);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_robot_offset);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_lift_offset);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_body_rotate);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_cam_rotate);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_zoom);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_focus);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_thermo_focus);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, station_ssid);
        //COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_body_rotate_two_stage);
        //COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_cam_rotate_two_stage);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_zoom_two_stage);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_focus_two_stage);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_thermo_focus_two_stage);

        if (!devicesModel.insertRecord(-1, rec))
        {
            bRet = false;
            ROS_ERROR("insertMultiDevices error, area_name:%d, name_id:%d", dev.device_area_name, dev.device_name_id);
            break;
        }
    }

    if (bRet)
    {
        if (devicesModel.submitAll())
        {
            ROS_INFO("insertMultiDevices succeed");
            devicesModel.database().commit();
        }
        else
        {
            bRet = false;
            ROS_ERROR("insertMultiDevices submitAll error");
            devicesModel.database().rollback();
        }
    }
    else
    {
        devicesModel.database().rollback();
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::updateSingleDeviceBySsid(deviceConfigType dev)
{
	QSqlQuery query;

    QString sqlStr = QString("update devices set device_name_id='%1',device_alternate_name_id=%2,"
        "device_area_name='%3', device_area_id='%4', relative_dev='%5',device_robot_offset='%6',device_lift_offset='%7',"
        "device_body_rotate='%8',device_cam_rotate='%9',device_hc_zoom='%10',device_hc_focus='%11',device_thermo_focus='%12',station_ssid='%13',"
        "device_hc_zoom_two_stage='%14',device_hc_focus_two_stage='%15',device_thermo_focus_stage='%16' where device_ssid='%17';")
        .arg(dev.device_name_id).arg(dev.device_alternate_name_id)
        .arg(dev.device_area_name).arg(dev.device_area_id)
        .arg(dev.relative_dev).arg(dev.device_robot_offset)
        .arg(dev.device_lift_offset).arg(dev.device_body_rotate)
        .arg(dev.device_cam_rotate).arg(dev.device_hc_zoom)
        .arg(dev.device_hc_focus).arg(dev.device_thermo_focus)
        .arg(dev.station_ssid)
        .arg(dev.device_hc_zoom_two_stage).arg(dev.device_hc_focus_two_stage)
        .arg(dev.device_thermo_focus_two_stage).arg(dev.device_ssid);

    bool bRet = querySqlString(sqlStr, query);

    if (!bRet)
    {
        ROS_ERROR("updateSingleVirDeviceBySsid failed: qrySql:%s, err:%s", query.lastQuery().toStdString().c_str(), query.lastError().text().toStdString().c_str());
    }
	return bRet;
}

bool LibDLHangRailRobotDBOperation::updateMultiDevicesBySsid(QList<deviceConfigType> devs)
{
    QSqlTableModel devicesModel;
    bool bRet = true;
    devicesModel.setTable("devices");

    devicesModel.database().transaction();

    for (int i = 0; i < devs.size(); i++)
    {
        deviceConfigType dev = devs[i];

        devicesModel.setFilter(QString("device_area_name = %1 AND device_name_id = %2").arg(dev.device_area_name).arg(dev.device_name_id));
        devicesModel.select();

        if (devicesModel.rowCount() != 1)
        {
            ROS_ERROR("updateMultiDevicesBySsid!error device num found, num:%d, please delete former device first!, device ssid:%s", devicesModel.rowCount(), dev.device_ssid.toStdString().c_str());
            continue;
        }

        QSqlRecord rec = devicesModel.record(i);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, device_ssid);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_alternate_name_id);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_name_id);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_area_id);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_robot_offset);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_lift_offset);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_body_rotate);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_cam_rotate);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_zoom);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_focus);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_thermo_focus);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, station_ssid);
        //COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_body_rotate_two_stage);
        //COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_cam_rotate_two_stage);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_zoom_two_stage);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_hc_focus_two_stage);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, device_thermo_focus_two_stage);

        if (!devicesModel.setRecord(i, rec))
        {
            bRet = false;
            ROS_ERROR("updateMultiDevicesBySsid error, area_name:%s, name_id:%d", dev.device_area_name.toStdString().c_str(), dev.device_name_id);
            break;
        }
    }

    if (bRet)
    {
        if (devicesModel.submitAll())
        {
            ROS_INFO("updateMultiDevicesBySsid succeed");
            devicesModel.database().commit();
        }
        else
        {
            bRet = false;
            ROS_ERROR("updateMultiDevicesBySsid submitAll error");
            devicesModel.database().rollback();
        }
    }
    else
    {
        devicesModel.database().rollback();
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::deleteSingleDeviceBySsId(QString ssid, QString &errMsg)
{
	bool bRet = false;
	QSqlQuery query;
    QString sqlStr = QString("DELETE from devices where device_ssid='%1';").arg(ssid);
    bRet = querySqlString(sqlStr, query);
	errMsg = query.lastError().text();
    if (!bRet)
    {
        ROS_ERROR("updateSingleVirDeviceBySsid failed: qrySql:%s, err:%s", query.lastQuery().toStdString().c_str(), errMsg.toStdString().c_str());
    }
	return bRet;
}

bool LibDLHangRailRobotDBOperation::deleteMultiDevicesBySsId(QList<QString> ssids)
{
    return true;
}

bool LibDLHangRailRobotDBOperation::getSingleDeviceBySsId(QString ssid, deviceConfigType &dev)
{
    QString qsqlStr(QString("SELECT * FROM devices WHERE device_ssid = '%1'").arg(ssid));
    QSqlQuery qry;
    bool bRet = querySqlString(qsqlStr, qry);
    if (!bRet)
    {
        ROS_ERROR("getSingleDeviceBySsId failed: qrySql:%s, err:%s", qsqlStr.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
        return false;
    }

    deviceConfigType d;
    QSqlRecord rec;
    while (qry.next())
    {
        int i = 0;

        d.device_ssid = qry.value(i++).toString();
        d.device_alternate_name_id = qry.value(i++).toInt();
        d.device_name_id = qry.value(i++).toInt();
        d.device_area_name = qry.value(i++).toString();
        d.device_area_id = qry.value(i++).toInt();
        d.relative_dev = qry.value(i++).toString();
        d.device_robot_offset = qry.value(i++).toInt();
        d.device_lift_offset = qry.value(i++).toInt();
        d.device_body_rotate = qry.value(i++).toInt();
        d.device_cam_rotate = qry.value(i++).toInt();
        d.device_hc_zoom = qry.value(i++).toInt();
        d.device_hc_focus = qry.value(i++).toInt();
        d.device_thermo_focus = qry.value(i++).toInt();
        d.station_ssid = qry.value(i++).toString();
        d.device_hc_zoom_two_stage = qry.value(i++).toInt();
        d.device_hc_focus_two_stage = qry.value(i++).toInt();
        d.device_thermo_focus_two_stage = qry.value(i++).toInt();
    }
    dev = d;
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getMultiDevicesBySsId(QStringList ssids, QList<deviceConfigType> &devs)
{
	int bRet = false;
	for (int i = 0; i < ssids.size(); i++)
	{
		QString qsqlStr(QString("SELECT * FROM devices WHERE device_ssid = '%1'").arg(ssids[i]));
		QSqlQuery qry;
		bRet = querySqlString(qsqlStr, qry);
		if (!bRet)
		{
			ROS_ERROR("getMultiDevicesBySsId failed: qrySql:%s, err:%s", qsqlStr.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
			break;
		}
		while (qry.next())
		{
			deviceConfigType d;
			QSqlRecord rec;

			int i = 0;

			d.device_ssid = qry.value(i++).toString();
			d.device_alternate_name_id = qry.value(i++).toInt();
			d.device_name_id = qry.value(i++).toInt();
			d.device_area_name = qry.value(i++).toString();
			d.device_area_id = qry.value(i++).toInt();
			d.relative_dev = qry.value(i++).toString();
			d.device_robot_offset = qry.value(i++).toInt();
			d.device_lift_offset = qry.value(i++).toInt();
			d.device_body_rotate = qry.value(i++).toInt();
			d.device_cam_rotate = qry.value(i++).toInt();
			d.device_hc_zoom = qry.value(i++).toInt();
			d.device_hc_focus = qry.value(i++).toInt();
			d.device_thermo_focus = qry.value(i++).toInt();
			d.station_ssid = qry.value(i++).toString();
			d.device_hc_zoom_two_stage = qry.value(i++).toInt();
			d.device_hc_focus_two_stage = qry.value(i++).toInt();
			d.device_thermo_focus_two_stage = qry.value(i++).toInt();

			devs.push_back(d);
		}
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getMultiRobotTaskDeviceSsid(QStringList ssids, std::vector<hangRailRobotTaskDevice> &devs)
{
	int bRet = false;
	for (int i = 0; i < ssids.size(); i++)
	{
		QString qsqlStr(QString("SELECT device_ssid,device_robot_offset,device_lift_offset FROM devices WHERE device_ssid = '%1'").arg(ssids[i]));
		QSqlQuery qry;
		bRet = querySqlString(qsqlStr, qry);
		while (qry.next())
		{
			hangRailRobotTaskDevice d;
			QSqlRecord rec;
			int i = 0;
			d.dev_uuid = qry.value(i++).toString().toStdString();//
			d.x = qry.value(i++).toInt();//
			d.y = qry.value(i++).toInt();//

			devs.push_back(d);
		}
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceAreaNames(QStringList &areaNames)
{
    bool bRet = false;

    QString str = "SELECT * FROM device_area_type";

    QSqlQuery qry;
    bRet = querySqlString(str, qry);

    while (qry.next())
    {
        QSqlRecord rec = qry.record();
        areaNames.push_back(qry.value(rec.indexOf("device_area_name")).toString());
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getExistDeviceAreaNames(QStringList &areaNames)
{
    QString str = "SELECT DISTINCT device_area_name FROM devices ORDER BY device_area_name ASC";

    QSqlQuery qry;
    bool bRet = querySqlString(str, qry);

    while (qry.next())
    {
        QSqlRecord rec = qry.record();
        areaNames.push_back(qry.value(rec.indexOf("device_area_name")).toString());
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getMultiDevicesByAreaName(QString areaName, QList<deviceConfigType> &devs)
{
    bool bRet = false;

    QSqlTableModel devicesModel;
    devicesModel.setFilter(QString("device_area_name = %1").arg(areaName));
    devicesModel.setTable("devices");

    if (!devicesModel.select())
    {
        ROS_ERROR("getSingleDeviceBySsId select error");
        return false;
    }
    else
    {
        ROS_INFO("getSingleDeviceBySsId model row count:%d", devicesModel.rowCount());
    }

    for (int i = 0; i < devicesModel.rowCount(); i++)
    {
        deviceConfigType d;
        QSqlRecord rec;
        rec = devicesModel.record(0);

        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, device_ssid);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_alternate_name_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_name_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_area_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_robot_offset);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_lift_offset);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_body_rotate);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_cam_rotate);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_zoom);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_focus);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_thermo_focus);
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, station_ssid);
        //COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_body_rotate_two_stage);
        //COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_cam_rotate_two_stage);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_zoom_two_stage);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_hc_focus_two_stage);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, device_thermo_focus_two_stage);
        devs.push_back(d);
    }

    return true;
}

bool LibDLHangRailRobotDBOperation::getDeviceByAreaNameInnerJoinDeviceName(QString areaName, QList<QString> &devs)
{
    QSqlQuery qry;
    QString str = QString("SELECT device_name.device_name FROM device_name INNER JOIN devices ON device_name.device_name_id = devices.device_name_id WHERE devices.device_area_name = '%1' ORDER BY device_name.device_name ASC").arg(areaName);
    
    bool bRet = querySqlString(str, qry);

    while (qry.next())
    {
        QSqlRecord rec = qry.record();
        devs.push_back(qry.value(rec.indexOf("device_name")).toString());
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceNameByDeviceNameId(int ssid, QString &devName)
{
    QSqlQuery qry;
    QString str = QString("SELECT * FROM device_name WHERE device_name_id = %1").arg(ssid);

    bool bRet = querySqlString(str, qry);

    if (!bRet)
    {
        ROS_ERROR("getDeviceNameByDeviceNameId failed: qrySql:%s, err:%s", qry.lastQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
        return bRet;
    }

    while (qry.next())
    {
        QSqlRecord rec = qry.record();
        devName = qry.value(rec.indexOf("device_name")).toString();
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getAllVirtualDeviceSsid(QStringList &virDevLists)
{
    QSqlQuery qry;

    bool bRet = querySqlString("SELECT * FROM virtualdevices", qry);

    if (bRet)
    {
        while (qry.next())
        {
            QSqlRecord rec = qry.record();
            QString ssid = rec.value(rec.indexOf("vir_dev_ssid")).toString();
            virDevLists.push_back(ssid);
        }
    }
    else
    {
        ROS_ERROR("getAllVirtualDeviceSsid failed: qrySql:%s, err:%s", qry.lastQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }    

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getAllVirDevices(QList<virtualDeviceType> &devs)
{
    QSqlQuery qry;

    bool bRet = querySqlString("SELECT * FROM virtualdevices", qry);

    if (bRet)
    {
        while (qry.next())
        {
            virtualDeviceType d;
            QSqlRecord rec = qry.record();
            COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, vir_dev_ssid);
            COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, vir_dev_name_id);
            COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, vir_dev_area_name);
            COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, vir_dev_area_id);
            COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, relative_dev);
            COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, station_ssid);
            devs.push_back(d);
        }
    }
    else
    {
        ROS_ERROR("getAllVirDevices failed: qrySql:%s, err:%s", qry.lastQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getAllVirDevicesSortByDeviceAreaName(QList<virtualDeviceType> &devs)
{
    QSqlQuery qry;
    //QString qryStr = QString("SELECT * FROM devices ORDER BY device_area_name WHERE device_name_id != %1 AND device_name_id != %2 AND  device_name_id != %3 ASC").arg(155).arg(156).arg(157);

    bool bRet = querySqlString("SELECT * FROM virtualdevices ORDER BY vir_dev_area_name ASC", qry);

    if (bRet)
    {
        while (qry.next())
        {
            virtualDeviceType d;
            QSqlRecord rec = qry.record();
            COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, vir_dev_ssid);
            COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, vir_dev_name_id);
            COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, vir_dev_area_name);
            COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, vir_dev_area_id);
            COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, relative_dev);
            COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, station_ssid);
            devs.push_back(d);
        }
    } 
    else
    {
        ROS_ERROR("getAllVirDevicesSortByDeviceAreaName failed: qrySql:%s, err:%s", qry.lastQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::insertSingleVirDevice(virtualDeviceType dev)
{
    boost::mutex::scoped_lock lock(m_lock);

    QSqlTableModel devicesModel;
    devicesModel.setTable("virtualdevices");
    //     devicesModel.setFilter(QString("device_area_id = %1 AND device_name_id = %2").arg(dev.device_area_id).arg(dev.device_name_id));    
    devicesModel.select();

    //     if (devicesModel.rowCount() > 0)
    //     {
    //         ROS_ERROR("same device, please delete former device first!, area_id:%d, name_id:%d", dev.device_area_id, dev.device_name_id);
    //         return false;
    //     }

    devicesModel.database().transaction();

    QSqlRecord rec = devicesModel.record();

    COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, vir_dev_ssid);
    COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, vir_dev_name_id);
    COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, vir_dev_area_name);
    COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, vir_dev_area_id);
    COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, relative_dev);
    COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, station_ssid);

    if (!devicesModel.insertRecord(-1, rec))
    {
        devicesModel.database().rollback();
        ROS_ERROR("insertSingleVirDevice error, ssid:%s", dev.vir_dev_ssid.toStdString().c_str());
        return false;
    }

    if (devicesModel.submitAll())
    {
        devicesModel.database().commit();
        ROS_INFO("insertSingleVirDevice succeed, ssid:%s", dev.vir_dev_ssid.toStdString().c_str());
        return true;
    }
    else
    {
        devicesModel.database().rollback();
        ROS_ERROR("insertSingleVirDevice error, ssid:%s", dev.vir_dev_ssid.toStdString().c_str());
        return false;
    }
}

bool LibDLHangRailRobotDBOperation::insertMultiVirDevices(QList<virtualDeviceType> devs)
{
    boost::mutex::scoped_lock lock(m_lock);
    QSqlTableModel devicesModel;
    bool bRet = true;
    devicesModel.setTable("virtualdevices");

    devicesModel.database().transaction();

    for (int i = 0; i < devs.size(); i++)
    {
        virtualDeviceType dev = devs.at(i);

        devicesModel.setFilter(QString("vir_dev_area_name = %1 AND vir_dev_name_id = %2").arg(dev.vir_dev_area_name).arg(dev.vir_dev_area_id));
        devicesModel.select();

        if (devicesModel.record().count() > 0)
        {
            ROS_ERROR("same device, please delete former device first!, area_name:%s, name_id:%d", dev.vir_dev_area_name.toStdString().c_str(), dev.vir_dev_area_id);
            continue;
        }

        QSqlRecord rec = devicesModel.record();
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, vir_dev_ssid);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, vir_dev_name_id);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, vir_dev_area_name);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, vir_dev_area_id);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, relative_dev);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, station_ssid);

        if (!devicesModel.insertRecord(-1, rec))
        {
            bRet = false;
            ROS_ERROR("insertMultiVirDevices error, area_name:%s, name_id:%d", dev.vir_dev_area_name.toStdString().c_str(), dev.vir_dev_area_id);
            break;
        }
    }

    if (bRet)
    {
        if (devicesModel.submitAll())
        {
            ROS_INFO("insertMultiVirDevices succeed");
            devicesModel.database().commit();
        }
        else
        {
            bRet = false;
            ROS_ERROR("insertMultiVirDevices submitAll error");
            devicesModel.database().rollback();
        }
    }
    else
    {
        devicesModel.database().rollback();
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::updateSingleVirDeviceBySsid(virtualDeviceType dev)
{
    QSqlQuery query;
    QString sqlStr = QString("update virtualdevices set vir_dev_name_id='%1',"
        "vir_dev_area_name='%2', vir_dev_area_id='%3', relative_dev='%4',station_ssid='%5' where vir_dev_ssid='%6';")
        .arg(dev.vir_dev_name_id).arg(dev.vir_dev_area_name)
        .arg(dev.vir_dev_area_id).arg(dev.relative_dev)
        .arg(dev.station_ssid).arg(dev.vir_dev_ssid);

    bool bRet = querySqlString(sqlStr, query);

    if (!bRet)
    {
        ROS_ERROR("updateSingleVirDeviceBySsid failed: qrySql:%s, err:%s", query.lastQuery().toStdString().c_str(), query.lastError().text().toStdString().c_str());
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::updateMultiVirDevicesBySsid(QList<virtualDeviceType> devs)
{
    boost::mutex::scoped_lock lock(m_lock);
    QSqlTableModel devicesModel;
    bool bRet = true;
    devicesModel.setTable("devices");

    devicesModel.database().transaction();

    for (int i = 0; i < devs.size(); i++)
    {
        virtualDeviceType dev = devs[i];

        devicesModel.setFilter(QString("vir_dev_area_name = %1 AND vir_dev_name_id = %2").arg(dev.vir_dev_area_name).arg(dev.vir_dev_name_id));
        devicesModel.select();

        if (devicesModel.rowCount() != 1)
        {
            ROS_ERROR("updateMultiVirDevicesBySsid!error device num found, num:%d, please delete former device first!, device ssid:%s", devicesModel.rowCount(), dev.vir_dev_ssid.toStdString().c_str());
            continue;
        }

        QSqlRecord rec = devicesModel.record(i);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, vir_dev_ssid);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, vir_dev_name_id);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, vir_dev_area_name);
        COPY_VAR_FROM_STRUC_TO_SQL_INT(rec, dev, vir_dev_area_id);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, relative_dev);
        COPY_VAR_FROM_STRUC_TO_SQL_STRING(rec, dev, station_ssid);

        if (!devicesModel.setRecord(i, rec))
        {
            bRet = false;
            ROS_ERROR("updateMultiVirDevicesBySsid error, area_name:%s, name_id:%d", dev.vir_dev_area_name.toStdString().c_str(), dev.vir_dev_name_id);
            break;
        }
    }

    if (bRet)
    {
        if (devicesModel.submitAll())
        {
            ROS_INFO("updateMultiVirDevicesBySsid succeed");
            devicesModel.database().commit();
        }
        else
        {
            bRet = false;
            ROS_ERROR("updateMultiVirDevicesBySsid submitAll error");
            devicesModel.database().rollback();
        }
    }
    else
    {
        devicesModel.database().rollback();
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::deleteSingleVirDeviceBySsId(QString ssid)
{
    QSqlQuery query;
    QString sqlStr = QString("DELETE from virtualdevices where vir_dev_ssid='%1';").arg(ssid);

    bool bRet = querySqlString(sqlStr, query);

    if (!bRet)
    {
        ROS_ERROR("updateSingleVirDeviceBySsid failed: qrySql:%s, err:%s", query.lastQuery().toStdString().c_str(), query.lastError().text().toStdString().c_str());
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::deleteMultiVirDevicesBySsId(QList<QString> ssids)
{
    return true;
}

bool LibDLHangRailRobotDBOperation::getTaskCoreBackData(QString dev_ssid, hangRailRobotTaskDeviceCom &backCom)
{
	QSqlQuery query;
	QString sqlStr = QString("SELECT devices.relative_dev,station_cfg.station_name,devices.device_area_name,device_name.device_name,device_type.device_type_name FROM devices,device_name,device_type,station_cfg WHERE devices.device_name_id=device_name.device_detail_id and device_type.device_type_id=device_name.device_type_id AND devices.station_ssid=station_cfg.station_ssid AND devices.device_ssid='%1';").arg(dev_ssid);
	backCom.device_ssid = dev_ssid;

	bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		int i = 0;
		backCom.relative_dev = query.value(i++).toString();
		backCom.station_name = query.value(i++).toString();
		backCom.device_area = query.value(i++).toString();
		backCom.device_name = query.value(i++).toString();
		backCom.device_type = query.value(i++).toString();
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getSingleVirDeviceBySsId(QString ssid, virtualDeviceType &dev)
{
    QString qsqlStr(QString("SELECT * FROM virtualdevices WHERE vir_dev_ssid = '%1'").arg(ssid));
    QSqlQuery qry;

    bool bRet = querySqlString(qsqlStr, qry);

    if (!bRet)
    {
        ROS_ERROR("getSingleVirDeviceBySsId failed: qrySql:%s, err:%s", qsqlStr.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
        return false;
    }

    virtualDeviceType d;
    QSqlRecord rec;
    while (qry.next())
    {
        int i = 0;

        d.vir_dev_ssid = qry.value(i++).toString();
        d.vir_dev_name_id = qry.value(i++).toInt();
        d.vir_dev_area_name = qry.value(i++).toString();
        d.vir_dev_area_id = qry.value(i++).toInt();
        d.relative_dev = qry.value(i++).toString();
        d.station_ssid = qry.value(i++).toString();
    }

    dev = d;
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getMultiVirDevicesBySsId(QStringList ssids, QList<virtualDeviceType> &devs)
{
    int bRet = false;
    for (int i = 0; i < ssids.size(); i++)
    {
        QString qsqlStr(QString("SELECT * FROM virtualdevices WHERE vir_dev_ssid = '%1'").arg(ssids[i]));
        QSqlQuery qry;

        bRet = querySqlString(qsqlStr, qry);

        if (!bRet)
        {
            ROS_ERROR("getMultiVirDevicesBySsId failed: qrySql:%s, err:%s", qsqlStr.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
            break;
        }
        while (qry.next())
        {
            virtualDeviceType d;
            int i = 0;

            d.vir_dev_ssid = qry.value(i++).toString();
            d.vir_dev_name_id = qry.value(i++).toInt();
            d.vir_dev_area_name = qry.value(i++).toString();
            d.vir_dev_area_id = qry.value(i++).toInt();
            d.relative_dev = qry.value(i++).toString();
            d.station_ssid = qry.value(i++).toString();

            devs.push_back(d);
        }
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getExistVirDeviceAreaNames(QStringList &areaNames)
{
    QString str = "SELECT DISTINCT vir_dev_area_name FROM virtualdevices ORDER BY vir_dev_area_name ASC";

    QSqlQuery qry;
    bool bRet = querySqlString(str, qry);

    while (qry.next())
    {
        QSqlRecord rec = qry.record();
        areaNames.push_back(qry.value(rec.indexOf("vir_dev_area_name")).toString());
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getMultiVirDevicesByAreaName(QString areaName, QList<virtualDeviceType> &devs)
{
    boost::mutex::scoped_lock lock(m_lock);
    bool bRet = false;

    QSqlTableModel devicesModel;
    devicesModel.setFilter(QString("device_area_name = %1").arg(areaName));
    devicesModel.setTable("devices");

    if (!devicesModel.select())
    {
        ROS_ERROR("getMultiVirDevicesByAreaName select error");
        return false;
    }
    else
    {
        ROS_INFO("getMultiVirDevicesByAreaName model row count:%d", devicesModel.rowCount());
    }

    for (int i = 0; i < devicesModel.rowCount(); i++)
    {
        virtualDeviceType d;
        QSqlRecord rec;
        rec = devicesModel.record(0);

        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, vir_dev_ssid);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, vir_dev_name_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, vir_dev_area_name);
        COPY_VAR_FROM_SQL_TO_STRUCT_INT(rec, d, vir_dev_area_id);
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, relative_dev);
        COPY_VAR_FROM_SQL_TO_STRUCT_STRING(rec, d, station_ssid);

        devs.push_back(d);
    }

    return true;
}

bool LibDLHangRailRobotDBOperation::getVirDeviceByAreaNameInnerJoinDeviceName(QString areaName, QList<QString> &devs)
{
    QSqlQuery qry;
    QString qsqlStr = QString("SELECT device_name.device_name FROM device_name INNER JOIN virtualdevices ON device_name.device_name_id = virtualdevices.vir_dev_name_id WHERE virtualdevices.vir_dev_area_name = '%1' ORDER BY device_name.device_name ASC").arg(areaName);
    
    bool bRet = querySqlString(qsqlStr, qry);

    if (!bRet)
    {
        ROS_ERROR("getMultiVirDevicesBySsId failed: qrySql:%s, err:%s", qry.lastQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
        return bRet;
    }
    while (qry.next())
    {
        QSqlRecord rec = qry.record();
        devs.push_back(qry.value(rec.indexOf("device_name")).toString());
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceDetailType(std::map<int, deviceDetailType> &detail)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT * FROM dev_detail_type", qry);
    while (qry.next())
    {
        deviceDetailType det;
        int i = 0;
        det.device_detail_id = qry.value(i++).toInt();
        det.device_classifier_name = qry.value(i++).toString();
        detail.insert(std::make_pair(det.device_detail_id, det));
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceAreaType(std::map<int, deviceAreaType> &areas)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT * FROM device_area_type", qry);
    while (qry.next())
    {
        deviceAreaType area;
        int i = 0;
        area.device_area_id = qry.value(i++).toInt();
        area.device_area_name = qry.value(i++).toString();

        areas.insert(std::make_pair(area.device_area_id, area));
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDevicePatrolType(std::map<int, devicePatrolType> &ptrls)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT * FROM device_type", qry);
    while (qry.next())
    {
        devicePatrolType ptrl;
        int i = 0;
        ptrl.device_patrol_type_id = qry.value(i++).toInt();
        ptrl.device_patrol_type_name = qry.value(i++).toString();

        ptrls.insert(std::make_pair(ptrl.device_patrol_type_id, ptrl));
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceUnitType(std::map<int, deviceUnitType> &units)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT * FROM device_unit", qry);
    while (qry.next())
    {
        deviceUnitType unit;
        int i = 0;
        unit.device_unit_id = qry.value(i++).toInt();
        unit.device_unit_name = qry.value(i++).toString();

        units.insert(std::make_pair(unit.device_unit_id, unit));
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceNameType(std::map<int, deviceNameType> &devNames)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT * FROM device_name", qry);
    while (qry.next())
    {
        deviceNameType name;
        int i = 0;
        name.device_detail_id = qry.value(i++).toInt();
        name.device_name = qry.value(i++).toString();
        name.device_type_id = qry.value(i++).toInt();
        name.device_unit_id = qry.value(i++).toInt();

        devNames.insert(std::make_pair(name.device_detail_id, name));
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceType(std::map<int, deviceType> &devType)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT * FROM device_type", qry);
    while (qry.next())
    {
        deviceType type;
        int i = 0;
        type.device_type_id = qry.value(i++).toInt();
        type.device_type_name = qry.value(i++).toString();

        devType.insert(std::make_pair(type.device_type_id, type));
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceAlternateName(std::map<int, deviceAlternateNameType> &devType)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT * FROM device_alternate_name", qry);
    if (!bRet)
    {
        ROS_ERROR("getDeviceAlternateName failed: qrySql:%s, err:%s", qry.lastQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }
    while (qry.next())
    {
        deviceAlternateNameType type;
        int i = 0;
        type.device_alternate_name_id = qry.value(i++).toInt();
        type.device_alternate_name = qry.value(i++).toString();

        devType.insert(std::make_pair(type.device_alternate_name_id, type));
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getThresholdEnvironment(std::map<int, thresholdEnvi> &threEnvi)
{
	QSqlQuery query;
    bool bRet = querySqlString("SELECT * FROM threshold_envi", query);
	while (query.next())
	{
		thresholdEnvi threE;
		int i = 0;
		threE.threshold_envi_id = query.value(i++).toInt();
		threE.envi_station_name = query.value(i++).toString();
		threE.envi_device_type_name = query.value(i++).toString();
		threE.envi_alarm_up = query.value(i++).toFloat();
		threE.envi_alarm_down = query.value(i++).toFloat();

		threEnvi.insert(std::make_pair(threE.threshold_envi_id, threE));
	}

	return bRet;
}

bool LibDLHangRailRobotDBOperation::getThresholdEnvironmentID(std::map<int, thresholdEnvi> &m_thresholdEnviID)
{
	QSqlQuery query;
    bool bRet = querySqlString(QString("select threshold_envi.threshold_envi_id,station_cfg.station_ssid,environment_type.envi_device_type_id,"
        "station_cfg.station_ssid,threshold_envi.envi_alarm_up,threshold_envi.envi_alarm_down from threshold_envi "
        "inner join environment_type inner join station_cfg where threshold_envi.envi_station_name=station_cfg.station_name "
        "and threshold_envi.envi_device_type_name=environment_type.envi_device_type_name order by threshold_envi.threshold_envi_id;"), query);
	while (query.next())
	{
		thresholdEnvi threE;
		int i = 0;
		threE.threshold_envi_id = query.value(i++).toInt();
		threE.envi_station_ssid = query.value(i++).toInt();
		threE.envi_device_type_id = query.value(i++).toInt();
		threE.envi_alarm_up = query.value(i++).toFloat();
		threE.envi_alarm_down = query.value(i++).toFloat();

		m_thresholdEnviID.insert(std::make_pair(threE.envi_device_type_id, threE));
	}

	return bRet;
}

bool LibDLHangRailRobotDBOperation::insertThresholdEnvironment(thresholdEnvi TEnvi)
{
	QSqlQuery query;
	bool bRet;
	if (TEnvi.envi_alarm_up < TEnvi.envi_alarm_down)
	{
		bRet = false;
	}
	else
	{
		QString sqlStr = QString("update threshold_envi set envi_station_name='%1', envi_alarm_up='%2', envi_alarm_down='%3' where envi_device_type_name='%4';")
			.arg(TEnvi.envi_station_name)
			.arg(TEnvi.envi_alarm_up)
			.arg(TEnvi.envi_alarm_down)
			.arg(TEnvi.envi_device_type_name);
        bRet = querySqlString(sqlStr, query);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getThresholdPatrol(std::map<int, thresholdPatrol> &threPatrol)
{
	QSqlQuery query;
//	bool bRet = query.exec(QString("SELECT * FROM threshold_patrol"));
    bool bRet = querySqlString("SELECT * FROM threshold_patrol", query);
	while (query.next())
	{
		thresholdPatrol threP;
		int i = 0;
		threP.threshold_patrol_id = query.value(i++).toInt();
		threP.patrol_device_type_name = query.value(i++).toString();
		threP.patrol_station_name = query.value(i++).toString();
		threP.patrol_alarm_up = query.value(i++).toFloat();
		threP.patrol_alarm_down = query.value(i++).toFloat();

		threPatrol.insert(std::make_pair(threP.threshold_patrol_id, threP));
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getThresholdPatrolID(std::map<int, thresholdPatrol> &m_thresholdPatrolID)
{
	QSqlQuery query;
	QString sqlStr = QString("select threshold_patrol.threshold_patrol_id,device_type.device_type_id,"
		"station_cfg.station_ssid,threshold_patrol.patrol_alarm_up,threshold_patrol.patrol_alarm_down from "
		"threshold_patrol inner join device_type inner join station_cfg where threshold_patrol.patrol_device_type_name="
		"device_type.device_type_name and threshold_patrol.patrol_station_name=station_cfg.station_name order by threshold_patrol.threshold_patrol_id;");
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		thresholdPatrol threP;
		int i = 0;
		threP.threshold_patrol_id = query.value(i++).toInt();
		threP.patrol_device_type_id = query.value(i++).toInt();
		threP.patrol_station_ssid = query.value(i++).toInt();
		threP.patrol_alarm_up = query.value(i++).toFloat();
		threP.patrol_alarm_down = query.value(i++).toFloat();
		m_thresholdPatrolID.insert(std::make_pair(threP.patrol_device_type_id, threP));
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::insertThresholdPatrol(thresholdPatrol TPatrol)
{
	QSqlQuery query;
	bool bRet;
	if (TPatrol.patrol_alarm_up < TPatrol.patrol_alarm_down)
	{
		bRet = false;
	}
	else
	{
		QString qrDate = QString("update threshold_patrol set patrol_station_name='%1', patrol_alarm_up='%2', patrol_alarm_down='%3' where patrol_device_type_name='%4';")
			.arg(TPatrol.patrol_station_name)
			.arg(TPatrol.patrol_alarm_up)
			.arg(TPatrol.patrol_alarm_down)
			.arg(TPatrol.patrol_device_type_name);
        bRet = querySqlString(qrDate, query);
	}
	
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getResultByTaskAndDeviceId(QString taskId, QString deviceId, inspectResultType &inpectRes)
{
    QString qsqlStr(QString("SELECT * FROM inpect_result WHERE task_ssid = '%1' AND device_ssid = '%2'").arg(taskId).arg(deviceId));

    QSqlQuery qry;
    bool bRet = querySqlString(qsqlStr, qry);

    if (!bRet)
    {
        ROS_ERROR("getResultByTaskAndDeviceId failed: qrySql:%s, err:%s", qsqlStr.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
        return bRet;
    }

    while (qry.next())
    {
        inspectResultType inspRes;
        int i = 0;
        inspRes.inspect_result_id = qry.value(i++).toInt();
        inspRes.task_ssid = qry.value(i++).toString();
        inspRes.device_ssid = qry.value(i++).toString();
        inspRes.inspect_time = qry.value(i++).toDateTime();
        inspRes.inspect_result = qry.value(i++).toString();
        inspRes.inspect_status = qry.value(i++).toBool();
        inspRes.visible_file_path = qry.value(i++).toString();
        inspRes.thermo_file_path = qry.value(i++).toString();
        inspRes.is_dealed = qry.value(i++).toBool();
        inspRes.dealed_info = qry.value(i++).toString();
        inspRes.deal_user = qry.value(i++).toString();

        inpectRes = inspRes;
        //inpectRes.push_back(inspRes);
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getTaskByTaskSsid(QString taskId, QList<taskType> &tasks)
{
    char buff[DB_BUFF_MIDIUM];
    memset(buff, 0, DB_BUFF_MIDIUM);

    sprintf(buff, "SELECT * FROM task WHERE task_ssid = %s", taskId.toStdString().c_str());

    QSqlQuery qry;
    bool bRet = querySqlString(QString(buff), qry);
    while (qry.next())
    {
        taskType task;
        int i = 0;
        task.task_ssid = qry.value(i++).toString();
        task.task_name = qry.value(i++).toString();
        task.task_template_ssid = qry.value(i++).toString();
        task.task_start_time = qry.value(i++).toDateTime();
        task.task_end_time = qry.value(i++).toDateTime();
        task.task_duration = qry.value(i++).toInt();
        task.task_total_devices = qry.value(i++).toInt();
        task.task_total_bugs = qry.value(i++).toInt();
        task.task_type_id = qry.value(i++).toInt();

        tasks.push_back(task);
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::insertSingleTask(taskType task)
{
    ROS_INFO("insertSingleTask, taskSsid:%s", task.task_ssid.toStdString().c_str());
    QString str = QString("INSERT INTO task VALUE ('%1', '%2', '%3', '%4', '%5', '%6', %7, %8, %9, %10)").arg(task.task_ssid).arg(task.task_name).arg(task.task_template_ssid)
        .arg(task.station_ssid).arg(task.task_start_time.toString("yyyy-MM-dd hh:mm:ss")).arg(task.task_end_time.toString("yyyy-MM-dd hh:mm:ss"))
        .arg(task.task_duration).arg(task.task_total_devices).arg(task.task_total_bugs).arg(task.task_type_id);

    QSqlQuery qry;
    bool bRet = querySqlString(str, qry);

    if (!bRet)
    {
        ROS_ERROR("insertSingleTask failed: qrySql:%s, err:%s", str.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::insertTaskAtBeginning(taskType task)
{
    ROS_INFO("insertTaskAtBeginning, taskSsid:%s", task.task_ssid.toStdString().c_str());

    QString str = QString("INSERT INTO task (task_ssid, task_name, task_template_ssid, station_ssid, task_start_time, task_type_id) VALUE ('%1', '%2', '%3', '%4', '%5', '%6')")
        .arg(task.task_ssid).arg(task.task_name).arg(task.task_template_ssid).arg(task.station_ssid).arg(task.task_start_time.toString("yyyy-MM-dd hh:mm:ss")).arg(task.task_type_id);

    QSqlQuery qry;
    bool bRet = querySqlString(str, qry);

    if (!bRet)
    {
        ROS_ERROR("insertTaskAtBeginning failed: qrySql:%s, err:%s", str.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::updateSingleTask(taskType task)
{
    ROS_INFO("updateSingleTask, taskSsid:%s", task.task_ssid.toStdString().c_str());
    QString str = QString("UPDATE task SET task_end_time = '%1', task_duration = %2, task_total_devices = %3, task_total_bugs = %4 WHERE task_ssid = '%6'")
        .arg(task.task_end_time.toString("yyyy-MM-dd hh:mm:ss")).arg(task.task_duration).arg(task.task_total_devices).arg(task.task_total_bugs).arg(task.task_ssid);

    QSqlQuery qry;
    bool bRet = querySqlString(str, qry);

    if (!bRet)
    {
        ROS_ERROR("updateSingleTask failed: qrySql:%s, err:%s", str.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }

    return bRet;
}

bool LibDLHangRailRobotDBOperation::getEnvironmentTypeData(QList<QString> &m_enviTypeName)
{
	QString sql = "select * from environment_type";
	QSqlQuery query;
    bool bRet = querySqlString(sql, query);

	while (query.next())
	{
		m_enviTypeName.append(query.value(1).toString());
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getStationCfgStationName(QList<QString> &m_stationName)
{
	QString sql = "select * from station_cfg";
	QSqlQuery query;
    bool bRet = querySqlString(sql, query);

	while (query.next())
	{
		m_stationName.append(query.value(1).toString());
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getAllTaskTemplatesSortByTaskTemplateName(QList<taskTemplateType> &tasks)
{
    QString sql = "SELECT * FROM task_template ORDER BY task_template_name ASC";
    QSqlQuery qry;
    bool bRet = querySqlString(sql, qry);

    while (qry.next())
    {
        taskTemplateType task;
        int i = 0;
        task.task_template_ssid = qry.value(i++).toString();
        task.task_template_name = qry.value(i++).toString();
        task.task_template_device_list = qry.value(i++).toString().split(" ");
        task.task_end_action_id = qry.value(i++).toInt();
        task.task_type_id = qry.value(i++).toInt();
        task.task_start_date = qry.value(i++).toDateTime();
        task.task_repeat_duration = qry.value(i++).toInt();

        tasks.push_back(task);
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getAllTaskTemplate(QList<taskTemplateType> &tasks)
{
    return true;
}

bool LibDLHangRailRobotDBOperation::getTodayTimeTask(QList<taskTemplateType> &tasks)
{
    QString str; 
    str.sprintf("SELECT * FROM task_template WHERE task_start_date BETWEEN '%s' AND '3000-01-01 00:00:00'", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString().c_str());

    QSqlQuery qry;
    bool bRet = querySqlString(str, qry);

    while (qry.next())
    {
        taskTemplateType task;
        int i = 0;
        task.task_template_ssid = qry.value(i++).toString();
        task.task_template_device_list = qry.value(i++).toString().split(" ");
        task.task_end_action_id = qry.value(i++).toInt();
        task.task_type_id = qry.value(i++).toInt();
        task.task_start_date = qry.value(i++).toDateTime();
        task.task_repeat_duration = qry.value(i++).toInt();
        task.task_template_name = qry.value(i++).toString();
        tasks.push_back(task);
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getTaskExecType(std::map<int, taskExecuteType> &types)
{
    QSqlQuery qry;
    bool bRet = querySqlString("SELECT * FROM task_type", qry);
    while (qry.next())
    {
        taskExecuteType t;
        int i = 0;
        t.task_type_id = qry.value(i++).toInt();
        t.task_type_name = qry.value(i++).toString();
        types.insert(std::make_pair(t.task_type_id, t));
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::insertSingleTaskTemplate(taskTemplateType taskTemplate)
{
    QString str;
 //   str.sprintf("INSERT INTO task_template VALUE ('%s', '%s', '%s', %d, %d, '%s', '%d')",
//        taskTemplate.task_template_ssid, taskTemplate.task_template_name, taskTemplate.task_template_device_list, taskTemplate.task_end_action_id,
 //       taskTemplate.task_type_id, taskTemplate.task_start_date.toString("yyyy-MM-dd hh:mm:ss"), taskTemplate.task_repeat_duration);

	str = QString("insert into task_template values('%1','%2','%3','%4','%5','%6','%7');")
        .arg(taskTemplate.task_template_ssid)
        .arg(taskTemplate.task_template_device_list.join(" "))
        .arg(taskTemplate.task_end_action_id)
        .arg(taskTemplate.task_type_id)
        .arg(taskTemplate.task_start_date.toString("yyyy-MM-dd hh:mm:ss"))
        .arg(taskTemplate.task_repeat_duration)
        .arg(taskTemplate.task_template_name);
    QSqlQuery qry;
    bool bRet = querySqlString(str, qry);
    return bRet;
}

bool LibDLHangRailRobotDBOperation::insertSingleInpectResult(inspectResultType inspectResult)
{
	int tr1, tr2;
	if (inspectResult.inspect_status)
		tr1 = 1;
	else
		tr1 = 0;
	if (inspectResult.is_virtual)
		tr2 = 1;
	else
		tr2 = 0;
	QString str = QString("insert into inpect_result(task_ssid, device_ssid, inspect_time, inspect_result, inspect_status, visible_file_path, "
		"thermo_file_path, is_dealed, dealed_info, deal_user, is_virtual) values('%1', '%2', '%3', '%4', '%5', '%6', '%7', '%8', '%9', '%10', '%11');")
		.arg(inspectResult.task_ssid).arg(inspectResult.device_ssid)
		.arg(inspectResult.inspect_time.toString("yyyy-MM-dd hh:mm:ss")).arg(inspectResult.inspect_result).arg(tr1)
		.arg(inspectResult.visible_file_path).arg(inspectResult.thermo_file_path).arg(inspectResult.is_dealed)
		.arg(inspectResult.dealed_info).arg(inspectResult.deal_user).arg(tr2);
	
	//str.sprintf("INSERT INTO inpect_result VALUE ('%d', '%s', '%s', '%s', '%s', '%d', '%s', '%s', '%d', '%s', '%s' , '%d')",
	//	taskTemplate.task_template_ssid, taskTemplate.task_template_name, taskTemplate.task_template_device_list, taskTemplate.task_end_action_id,
	//	taskTemplate.task_type_id, taskTemplate.task_start_data.toString("yyyy-MM-dd hh:mm:ss"), taskTemplate.task_repeat_duration, taskTemplate.task_repeat_times);
	QSqlQuery qry;
	bool bRet = querySqlString(str, qry);

    if (!bRet)
    {
        ROS_ERROR("insertSingleInpectResult failed: qrySql:%s, err:%s", str.toStdString().c_str(), qry.lastError().text().toStdString().c_str());
    }

	return bRet;
}


bool LibDLHangRailRobotDBOperation::deleteSingleTaskBySsid(QString ssid)
{
    QString str;
    str.sprintf("DELETE FROM task_template WHERE task_template_ssid = %s", ssid);
    QSqlQuery qry;
    bool bRet = querySqlString(str, qry);
    return bRet;
}

bool LibDLHangRailRobotDBOperation::openDb(QString hostName, QString dbName, QString userName, QString passwd, int port)
{
    m_myDb = QSqlDatabase::addDatabase("QMYSQL");
    m_myDb.setHostName(hostName);
    m_myDb.setDatabaseName(dbName);
    m_myDb.setUserName(userName);
    m_myDb.setPassword(passwd);
    m_myDb.setPort(port);
	m_myDb.setConnectOptions("MYSQL_OPT_RECONNECT=1;");
    if (m_myDb.open())
    {
        ROS_INFO("database is established!");
        return true;
    }
    else
    {
		ROS_ERROR("build error! err:%s;", m_myDb.lastError().text().toStdString().c_str());
		ROS_ERROR("DB ERROR:(hostName:%s, dnName:%s, userName:%s, passwd:%s, port:%d)", hostName.toStdString().c_str(), dbName.toStdString().c_str(), userName.toStdString().c_str(), passwd.toStdString().c_str(), port);
        return false;
    }
}

void LibDLHangRailRobotDBOperation::closeDb()
{
    m_myDb.close();
}

bool LibDLHangRailRobotDBOperation::execSqlString(QSqlQuery &qry)
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

bool LibDLHangRailRobotDBOperation::querySqlString(QString sqlString, QSqlQuery &qry)
{
	boost::mutex::scoped_lock lock(m_lock);
// 	QSqlQuery qurey;
 	bool bRet = false;
	int count = 1;
// 	bool bRetcry = qurey.exec("show processlist;");
// 	if (!qurey.isActive())
// 	{
// 		closeDb();
// 		DBInfoData_T c_dbInfoData = DLHangRailCommonTools::getDBInfoData();
// 		if (openDb(c_dbInfoData.hostName, c_dbInfoData.dbName, c_dbInfoData.userName, c_dbInfoData.password))
// 		{
// 			Sleep(100);
// 			ROS_INFO("querySqlString success again;");
// 		}
// 		else
// 		{
// 			ROS_INFO("querySqlString failed again;");
// 		}
// 	}
// 	else
// 	{
// 		bRet = qry.exec(sqlString);
// 		if (!bRet)
// 		{
// 			ROS_ERROR("querySqlString failed; sqlString: %s, qry execed: %s ,qry error : %s", sqlString.toStdString().c_str(), qry.executedQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
// 		}
// 		else
// 		{
// 			ROS_INFO("querySqlString success; sqlString: %s, qry execed: %s", sqlString.toStdString().c_str(), qry.executedQuery().toStdString().c_str());
// 		}
//	}
	bRet = qry.exec(sqlString);
	while (!bRet)
	{
		ROS_ERROR("querySqlString failed; Count: %d, sqlString: %s, qry execed: %s ,qry error : %s", count, sqlString.toStdString().c_str(), qry.executedQuery().toStdString().c_str(), qry.lastError().text().toStdString().c_str());
		bRet = qry.exec(sqlString);
		count++;
		if (count > 4)
		{
			ROS_ERROR("querySqlString failed; Datebase mistake for question is not appear linking!");
			break;
		}
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

bool LibDLHangRailRobotDBOperation::ReadDatabaseStorageFirstList(QString task_ssid, FirstListPortion &m_FiListData)
{
	QSqlQuery query;

	QString tr1 = QString("SELECT station_name FROM station_cfg;").arg(task_ssid);
//	query.exec(tr1);

    bool bRet = querySqlString(tr1, query);

	while (query.next())
	{
		m_FiListData.m_station_name = query.value(0).toString();
	}
	QString tr2 = QString("select task_name,task_start_time,task_end_time,task_total_devices,task_total_bugs from task where task_ssid='%1'").arg(task_ssid);
    bRet = querySqlString(tr2, query);
	while (query.next())
	{
		m_FiListData.m_task_name = query.value(0).toString();
		m_FiListData.m_start_time = query.value(1).toString().replace("T"," ");
		m_FiListData.m_end_time = query.value(2).toString().replace("T"," ");
		m_FiListData.m_total_devices = QString("%1个").arg(query.value(3).toInt());
		m_FiListData.m_total_breakdown = QString("%1个").arg(query.value(4).toInt());
	}
	return true;
}

bool LibDLHangRailRobotDBOperation::ReadDatabaseStorageSecondList(QString task_ssid, QList<SecondListPortion> &m_SecondData)
{
	QSqlQuery query;
 	int k = 0;
	bool bRet = false;// = querySqlString("select count(*) from device_type", query);
// 	while (query.next())
// 	{
// 		secondListRow = query.value(0).toInt();
// 	}
	bRet = querySqlString("select device_type_id,device_type_name from device_type ORDER BY device_type_id;", query);
	while (query.next())
	{
		SecondListPortion data;
		data.m_id = query.value(0).toString();
		data.m_category = query.value(1).toString();
		m_SecondData.append(data);
		k++;
	}

	for (int i = 0; i < m_SecondData.size(); i++)
	{
		QString str = QString("SELECT inpect_result.inspect_status,count(*) FROM inpect_result,devices,device_name WHERE inpect_result.device_ssid=devices.device_ssid AND devices.device_name_id=device_name.device_detail_id AND inpect_result.task_ssid='%1' AND device_name.device_type_id='%2' GROUP BY inpect_result.inspect_status;")
			.arg(task_ssid)
			.arg(m_SecondData[i].m_id);
		bRet = querySqlString(str, query);
		int sum = 0;
		while (query.next())
		{
			if (query.value(0).toInt() == 0)
			{
				m_SecondData[i].m_equipment_bad = query.value(1).toString();
			}
			sum = sum + query.value(1).toInt();
		}
		m_SecondData[i].m_equipment = QString("%1").arg(sum);
// 		QString str1 = QString("select count(*) from inpect_result_device_type where (device_type_name="
// 			"(select device_type_name from device_type where device_type_id='%1') and task_ssid='%2')").arg(i).arg(task_ssid);
// 		bRet = querySqlString(str1, query);
// 		while (query.next())
// 		{
// 			m_SecondData[i - 1].m_equipment = QString("%1个").arg(query.value(0).toString());
// 		}
// 		QString str2 = QString("select count(*) from inpect_result_device_type where (device_type_name="
// 			"(select device_type_name from device_type where device_type_id='%1')and inspect_status='0' and task_ssid='%2')").arg(i).arg(task_ssid);
// 		bRet = querySqlString(str2, query);
// 		while (query.next())
// 		{
// 			m_SecondData[i - 1].m_equipment_bad = QString("%1个").arg(query.value(0).toString());
// 		}
	}
	return true;
}
bool LibDLHangRailRobotDBOperation::ReadDatabaseStorageForthList(QString task_ssid, QString & strRet)
{
	QSqlQuery query;
	strRet = ("");
    bool bRet = false;
	bool bRect;
//	QString str1 = QString("select device_area_name,device_alternate_name,inspect_result,device_unit_name from inpect_result_device_type where task_ssid='%1' and inspect_status='0';").arg(task_ssid);
	QString str1 = QString("SELECT devices.device_area_name,device_name.device_name,inpect_result.inspect_result,device_unit.device_unit_name FROM devices,inpect_result,device_name,device_unit,task WHERE devices.device_name_id=device_name.device_detail_id AND inpect_result.device_ssid=devices.device_ssid AND device_unit.device_unit_id=device_name.device_unit_id AND inpect_result.task_ssid=task.task_ssid AND inpect_result.inspect_status='0' AND inpect_result.task_ssid='%1' ORDER BY device_name.device_name;")
		.arg(task_ssid);
	bRet = querySqlString(str1, query);
	while (query.next())
	{
		strRet = strRet + query.value(0).toString() + QString(" ") + query.value(1).toString() +
			QString(" ") + query.value(2).toString() + QString("(%1)").arg(query.value(3).toString()) +
			QString("异常") + QString::fromLocal8Bit("\r\n");
	}

	return bRet;
}

int LibDLHangRailRobotDBOperation::ReadDatabaseStoFifth_AllSumList(QString task_ssid)
{
	QSqlQuery query;
	int listCount = 0;
    
//	QString sqlStr = QString("SELECT count(DISTINCT (device_area_name)) FROM inpect_result_device_type where task_ssid='%1'").arg(task_ssid);
	QString sqlStr = QString("SELECT DISTINCT (device_area_name) FROM inpect_result,task,devices WHERE inpect_result.task_ssid=task.task_ssid AND inpect_result.device_ssid=devices.device_ssid AND inpect_result.task_ssid='%1';")
		.arg(task_ssid);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		listCount = query.value(0).toInt();
	}

	return listCount;
}
bool LibDLHangRailRobotDBOperation::ReadDatabaseStoFifth_DevAreName(QString task_ssid, QStringList &deviceAreaName)
{
	QSqlQuery query;
	QString sqlStr = QString("SELECT DISTINCT(device_area_name) FROM inpect_result,task,devices WHERE inpect_result.task_ssid=task.task_ssid AND inpect_result.device_ssid=devices.device_ssid AND inpect_result.task_ssid='%1';")
		.arg(task_ssid);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		deviceAreaName.append(query.value(0).toString());
	}
	return true;
}

int LibDLHangRailRobotDBOperation::ReadDatabaseStoFifth_AllCountDevAreName(QString task_ssid, QString area_name)
{
	QSqlQuery query;
	int listCount_s;
	QString sqlStr2 = QString("SELECT count(*) FROM inpect_result_device_type where device_area_name='%1' and task_ssid='%2';").arg(area_name).arg(task_ssid);
    bool bRet = querySqlString(sqlStr2, query);
	while (query.next())
	{
		listCount_s = query.value(0).toInt();
	}
	QString sqlStr3 = QString("select count(*) from virtual_inpect_result where task_ssid='%1' and vir_dev_area_name='%2';").arg(task_ssid).arg(area_name);
	bRet = querySqlString(sqlStr3, query);
	while (query.next())
	{
		listCount_s = listCount_s + query.value(0).toInt();
	}
	return listCount_s;
}

bool LibDLHangRailRobotDBOperation::ReadDatabaseStoFifth_AllData(QList<FifthListPortion> &m_FifthData,QString m_task_ssid, QString m_device_area_name)
{
	QSqlQuery query;
	int notNullList;
	int is_virtual;
	int inspect_status;
	QString area_name;
	QString detail_name;
	QString inspect_result;
	QString unit_name;
	QString relative_dev;
	int i = 0;
// 	QString sqlStr2 = QString("select count(*) from inpect_result_device_type where relative_dev !='' and device_area_name='%1' and task_ssid='%2';").arg(m_device_area_name).arg(m_task_ssid);
//     bool bRet = querySqlString(sqlStr2, query);
// 	while (query.next())
// 	{
// 		notNullList = query.value(0).toInt();
// 	}
	bool bRet = false;
// 	QString sqlStr = QString("SELECT * from inpect_result_device_type where device_area_name='%1' and task_ssid='%2';")
// 		.arg(m_device_area_name)
// 		.arg(m_task_ssid);
	QList<FifthListPortion> fifData;
	QStringList deviceList;
	QString sqlStr = QString("SELECT devices.device_ssid,devices.device_area_name,device_alternate_name.device_alternate_name,inpect_result.inspect_result,device_unit.device_unit_name,inpect_result.inspect_status,devices.relative_dev,inpect_result.is_virtual FROM devices,device_alternate_name,inpect_result,task,device_name,device_unit WHERE devices.device_alternate_name_id = device_alternate_name.device_alternate_name_id AND inpect_result.device_ssid=devices.device_ssid AND inpect_result.task_ssid=task.task_ssid AND devices.device_name_id=device_name.device_detail_id AND device_name.device_unit_id=device_unit.device_unit_id AND inpect_result.task_ssid='%1' AND devices.device_area_name='%2' ORDER BY devices.relative_dev DESC;")
		.arg(m_task_ssid)
		.arg(m_device_area_name);
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		int i = 0;
		FifthListPortion data;
	//	deviceList.append(query.value(i++).toString());
		data.m_device_ssid = query.value(i++).toString();
		data.m_area_name = query.value(i++).toString();
		data.m_detail_name = query.value(i++).toString();
		inspect_result = query.value(i++).toString();
		unit_name = query.value(i++).toString();
		data.m_inspect_result_unit_name = QString("%1(%2)").arg(inspect_result).arg(unit_name);
		inspect_status = query.value(i++).toInt();
		if (inspect_status == 0)
			data.m_statue = QString("异常");
		else
			data.m_statue = QString("正常");
		data.m_relative_dev = query.value(i++).toString();
		data.m_judge = 0;
		fifData.append(data);
	}
	int num = 1;
	for (int i = 0; i < fifData.size(); i++)
	{
		deviceList.append(fifData[i].m_device_ssid);
		fifData[i].m_serial_num = QString("%1").arg(num);
		m_FifthData.append(fifData[i]);
		num++;
		if (fifData[i].m_relative_dev.isEmpty())
		{
		}
		else
		{
			QStringList virDeviceSSid;
			sqlStr = QString("SELECT relative_dev FROM virtualdevices WHERE vir_dev_ssid = '%1' AND vir_dev_area_name='%2';")
				.arg(fifData[i].m_relative_dev).arg(m_device_area_name);
			bRet = querySqlString(sqlStr, query);
			while (query.next())
			{
				virDeviceSSid = query.value(0).toString().split(" ");
			}
			int count = 0;
			for (int k = 0; k < deviceList.size(); k++)
			{
				if (virDeviceSSid.size() <= 1)
				{
					break;
				}
				if (virDeviceSSid.size() == 2)
				{
					if (deviceList[k] == virDeviceSSid[0] || deviceList[k] == virDeviceSSid[1])
					{
						count++;
					}
					if (count == 2)
					{
						break;
					}
				}
				if (virDeviceSSid.size() == 3)
				{
					if (deviceList[k] == virDeviceSSid[0] || deviceList[k] == virDeviceSSid[1] || deviceList[k] == virDeviceSSid[2])
					{
						count++;
					}
					if (count == 3)
					{
						break;
					}
				}
			}
			if (count == virDeviceSSid.size())
			{
				sqlStr = QString("SELECT virtualdevices.vir_dev_area_name,device_alternate_name.device_alternate_name,inpect_result.inspect_result,device_unit.device_unit_name,inpect_result.inspect_status FROM inpect_result,virtualdevices,device_unit ,device_alternate_name,task,device_name WHERE inpect_result.device_ssid=virtualdevices.vir_dev_ssid AND inpect_result.task_ssid = task.task_ssid AND virtualdevices.vir_dev_name_id=device_alternate_name.device_alternate_name_id AND virtualdevices.vir_dev_name_id=device_name.device_detail_id AND device_name.device_unit_id=device_unit.device_unit_id AND virtualdevices.vir_dev_area_name='%1' AND inpect_result.task_ssid='%2' AND virtualdevices.vir_dev_ssid='%3';")
					.arg(m_device_area_name)
					.arg(m_task_ssid)
					.arg(fifData[i].m_relative_dev);
				bRet = querySqlString(sqlStr, query);
				while (query.next())
				{
					FifthListPortion data;
					data.m_judge = 1;
					data.m_serial_num = QString("%1").arg(num);
					num++;
					data.m_area_name = query.value(0).toString();
					data.m_detail_name = query.value(1).toString();
					inspect_result = query.value(2).toString();
					unit_name = query.value(3).toString();
					data.m_inspect_result_unit_name = QString("%1(%2)").arg(inspect_result).arg(unit_name);
					inspect_status = query.value(4).toInt();
					if (inspect_status == 0)
						data.m_statue = QString("异常");
					else
						data.m_statue = QString("正常");
					m_FifthData.append(data);
// 
// 					area_name = query.value(2).toString();
// 					detail_name = query.value(8).toString();
// 					inspect_result = query.value(5).toString();
// 					unit_name = query.value(6).toString();
// 					inspect_status = query.value(4).toInt();
// 					// 		relative_dev = query.value(9).toString();
// 					// 		is_virtual = query.value(10).toInt();
// 
// 					m_FifthData[i].m_judge = 1;
// 					m_FifthData[i].m_serial_num = QString("%1").arg(i + 1);
// 					m_FifthData[i].m_area_name = area_name;
// 					m_FifthData[i].m_detail_name = detail_name;
// 					m_FifthData[i].m_inspect_result_unit_name = QString("%1(%2)").arg(inspect_result).arg(unit_name);
// 
// 					if (inspect_status == 0)
// 						m_FifthData[i].m_statue = QString("异常");
// 					else
// 						m_FifthData[i].m_statue = QString("正常");
// 					i++;
				}
			}
		}
	}


// 		int m_judge;
// 		QString m_serial_num;
// 		QString m_area_name;
// 		QString m_detail_name;
// 		QString m_inspect_result_unit_name;
// 		QString m_statue;

// 		area_name = query.value(2).toString();
// 		detail_name = query.value(8).toString();
// 		inspect_result = query.value(5).toString();
// 		unit_name = query.value(6).toString();
// 		inspect_status = query.value(4).toInt();
// 		relative_dev = query.value(9).toString();
// 		is_virtual = query.value(10).toInt();
		

// 		m_FifthData[i].m_judge = 0;
// 		m_FifthData[i].m_serial_num = QString("%1").arg(i+1);
// 		m_FifthData[i].m_area_name = area_name;
// 		m_FifthData[i].m_detail_name = detail_name;
// 		m_FifthData[i].m_inspect_result_unit_name = QString("%1(%2)").arg(inspect_result).arg(unit_name);
// 
// 		if (inspect_status == 0)
// 			m_FifthData[i].m_statue = QString("异常");
// 		else
// 			m_FifthData[i].m_statue = QString("正常");
// 		i++;
//	}
// 
// 	sqlStr = QString("SELECT virtualdevices.vir_dev_area_name,device_alternate_name.device_alternate_name,inpect_result.inspect_result,device_unit.device_unit_name,inpect_result.inspect_status,device_type.device_type_name FROM inpect_result,virtualdevices,device_type,device_unit ,device_alternate_name,task,device_name WHERE inpect_result.device_ssid=virtualdevices.vir_dev_ssid AND inpect_result.task_ssid = task.task_ssid AND virtualdevices.vir_dev_name_id=device_alternate_name.device_alternate_name_id AND virtualdevices.vir_dev_name_id=device_name.device_detail_id AND device_name.device_type_id=device_type.device_type_id AND device_name.device_unit_id=device_unit.device_unit_id AND virtualdevices.vir_dev_area_name='%1' AND inpect_result.task_ssid='%2' AND virtualdevices.vir_dev_ssid='%3';")
// 		.arg(m_device_area_name)
// 		.arg(m_task_ssid)
// 		.arg(fifData[i].m_relative_dev);
// 	bRet = querySqlString(sqlStr, query);
// 	while (query.next())
// 	{
// 		area_name = query.value(2).toString();
// 		detail_name = query.value(8).toString();
// 		inspect_result = query.value(5).toString();
// 		unit_name = query.value(6).toString();
// 		inspect_status = query.value(4).toInt();
// // 		relative_dev = query.value(9).toString();
// // 		is_virtual = query.value(10).toInt();
// 
// 		m_FifthData[i].m_judge = 1;
// 		m_FifthData[i].m_serial_num = QString("%1").arg(i + 1);
// 		m_FifthData[i].m_area_name = area_name;
// 		m_FifthData[i].m_detail_name = detail_name;
// 		m_FifthData[i].m_inspect_result_unit_name = QString("%1(%2)").arg(inspect_result).arg(unit_name);
// 
// 		if (inspect_status == 0)
// 			m_FifthData[i].m_statue = QString("异常");
// 		else
// 			m_FifthData[i].m_statue = QString("正常");
// 		i++;
// 	}
	return true;
}

bool LibDLHangRailRobotDBOperation::ReadDatabaseStorageSeventhList(QString task_ssid, SeventhListPortion &m_SeventhData)
{
	QSqlQuery query;
	QString m_station_name;
	QString m_start_time;
	QString tr1 = QString("select station_name from station_cfg where station_ssid="
		"(select station_ssid from task where task_ssid='%1');").arg(task_ssid);
    bool bRet = querySqlString(tr1, query);

	while (query.next())
	{
		m_station_name = query.value(0).toString();
	}
	QString tr2 = QString("select task_start_time from task where task_ssid='%1';").arg(task_ssid);
    bRet = querySqlString(tr2, query);
	while (query.next())
	{
		m_start_time = query.value(0).toString();
	}
// 	QString sqlStr1 = QString("select DISTINCT (device_area_name) from inpect_result_device_type "
// 		"where(task_ssid='%1' and inspect_status='0');").arg(task_ssid);
	QString sqlStr1 = QString("SELECT DISTINCT (devices.device_area_name) FROM devices,inpect_result,task WHERE inpect_result.task_ssid=task.task_ssid AND inpect_result.device_ssid=devices.device_ssid AND inpect_result.inspect_status='0' AND task.task_ssid='%1';")
		.arg(task_ssid);
    bRet = querySqlString(sqlStr1, query);
	QString strc = "";
	QString strRet = "";
	while (query.next())
	{
		strc = strc + query.value(0).toString() + QString(",");
	}

// 	QString sqlStr2 = QString("select device_area_name,device_alternate_name,inspect_result,device_unit_name from "
// 		"inpect_result_device_type where(task_ssid='%1' and inspect_status='0');").arg(task_ssid);
// 
//     bRet = querySqlString(sqlStr2, query);
// 
// 	while (query.next())
// 	{
// 		strRet = strRet + query.value(0).toString() + QString(" ") + query.value(1).toString() +
// 			QString(" ") + query.value(2).toString() + QString("(%1)").arg(query.value(3).toString()) +
// 			QString("异常") + QString::fromLocal8Bit("\r\n");
// 	}

	QString sqlStr2 = QString("SELECT devices.device_area_name,device_name.device_name,inpect_result.inspect_result,device_unit.device_unit_name FROM devices,inpect_result,device_name,device_unit,task WHERE devices.device_name_id=device_name.device_detail_id AND inpect_result.device_ssid=devices.device_ssid AND device_unit.device_unit_id=device_name.device_unit_id AND inpect_result.task_ssid=task.task_ssid AND inpect_result.inspect_status='0' AND inpect_result.task_ssid='%1' ORDER BY device_name.device_name;")
		.arg(task_ssid);
	bRet = querySqlString(sqlStr2, query);
	while (query.next())
	{
		strRet = strRet + query.value(0).toString() + QString(" ") + query.value(1).toString() +
			QString(" ") + query.value(2).toString() + QString("(%1)").arg(query.value(3).toString()) +
			QString("异常") + QString::fromLocal8Bit("\r\n");
	}

	m_SeventhData.m_seventh_station_name = m_station_name;
	m_SeventhData.m_seventh_start_time = m_start_time.mid(0, 10).replace("T"," ");
	m_SeventhData.m_seventh_bad_devices = strc;
	m_SeventhData.m_seventh_flaw_describe = strRet;

	return true;
}

bool LibDLHangRailRobotDBOperation::getUserRole(QString username, QString password, hangRobotUserLoginRetVal &ret)
{
    QSqlQuery query;
    bool bReturn;

    if (username.isNull() || password.isNull())
    {
        ret.role = USER_NONE;
        ret.retCode = HANG_RAIL_LOGIN_USERNAME_NOT_EXIST;
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
            ret.role = (UserType)query.value(query.record().indexOf("user_role")).toInt();
            ret.retCode = HANG_RAIL_LOGIN_SUCCESS;
            ret.errMsg = "成功了！";
            return true;
        }
        else
        {
            ret.role = USER_NONE;
            ret.retCode = HANG_RAIL_LOGIN_PASSWORD_INCORRECT;
            ret.errMsg = "密码错误！";
            return false;
        }
    }
    else
    {
        ret.role = USER_NONE;
        ret.retCode = HANG_RAIL_LOGIN_USERNAME_NOT_EXIST;
        ret.errMsg = "用户不存在";
        ROS_ERROR("user does not exists");
        return false;
    }
}

bool LibDLHangRailRobotDBOperation::SearchTaskTableViewData(QStandardItemModel *m_SearchTaskDataModel)
{
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "任务名" << "任务类型" << "时间";
	m_SearchTaskDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	QSqlQuery query;
	int x = 0;
	QString sqlStr = QString("select task_template.task_template_name,task_type.task_type_name,task_template.task_start_date,task_template.task_template_ssid from task_template inner join task_type where task_template.task_type_id=task_type.task_type_id order by task_template.task_start_date desc;");
	
    bool bRet = querySqlString(sqlStr, query);

    while (query.next())
	{
		QString tr1 = query.value(0).toString();
		QString tr2 = query.value(1).toString();
		QString tr3 = query.value(2).toString().replace("T"," ");
		QString tr4 = query.value(3).toString();

		QStandardItem *item1 = new QStandardItem(tr1);
		m_SearchTaskDataModel->setItem(x, 0, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		m_SearchTaskDataModel->setItem(x, 1, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		m_SearchTaskDataModel->setItem(x, 2, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		m_SearchTaskDataModel->setItem(x, 3, item4);
		x++;
	}
	return true;
}

bool LibDLHangRailRobotDBOperation::DeleteDatalistTaskTemplate(QString taskTemplateSsid)
{
	QSqlQuery query;
	QString sqlStr = QString("delete from task_template where task_template_ssid='%1';").arg(taskTemplateSsid);
    bool bRet = querySqlString(sqlStr, query);
	return bRet;
}

int LibDLHangRailRobotDBOperation::TaskTableViewDataCount(bool b_Select, QString m_StartTime, QString m_StopTime, QString m_taskName)
{
	QSqlQuery query;
    bool bRet = false;
	if (b_Select)
	{
		QString sqlStr = QString("select count(*) from task_show_list where "
			"(task_start_time>='%1' and task_end_time<='%2' and task_name like '%%3%');").arg(m_StartTime).arg(m_StopTime).arg(m_taskName);
        bRet = querySqlString(sqlStr, query);
		while (query.next())
		{
			return(query.value(0).toInt());
		}
	}
	else
	{
        bRet = querySqlString("select count(*) from task_show_list", query);
		while (query.next())
		{
			return(query.value(0).toInt());
		}
	}
	return 1;
}

void LibDLHangRailRobotDBOperation::TaskTableViewData(int row,int trow,QStandardItemModel *m_taskDataModel)
{
	QSqlQuery query;
//	task_tableModel = new QStandardItemModel();
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "任务名" << "任务类型" << "总设备数" << "异常设备数" << "开始时间" << "结束时间";
	m_taskDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	int x = 0;
	QString sqlStr = QString("select * from task_show_list limit %1,%2;").arg(row).arg(trow);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		for (int y = 0; y<7; y++)
		{			
			if (y == 4 || y == 5)
			{
				QStandardItem *item = new QStandardItem(query.value(y).toString().replace("T"," "));
				m_taskDataModel->setItem(x, y, item);
			}
			else
			{
				QStandardItem *item = new QStandardItem(query.value(y).toString());
				m_taskDataModel->setItem(x, y, item);
			}
			
		}
		x++;
	}
}
void LibDLHangRailRobotDBOperation::TaskSelectTableViewData(int row,int trow, QString m_StartTime, QString m_StopTime ,QString m_taskName, QStandardItemModel *m_taskSearchDataModel)
{
	QSqlQuery query;
//	taskSelect_tableModel = new QStandardItemModel();
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "任务名" << "任务类型" << "总设备数" << "异常设备数" << "开始时间" << "结束时间";
	m_taskSearchDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	int x = 0;

	QString sqlStr = QString("select * from task_show_list where task_start_time>='%1' and task_end_time<='%2' and task_name like '%%3%' limit %4,%5;")
		.arg(m_StartTime).arg(m_StopTime).arg(m_taskName).arg(row).arg(trow);

    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		for (int y = 0; y<7; y++)
		{
			if (y == 4 || y == 5)
			{
				QStandardItem *item = new QStandardItem(query.value(y).toString().replace("T", " "));
				m_taskSearchDataModel->setItem(x, y, item);
			}
			else
			{
				QStandardItem *item = new QStandardItem(query.value(y).toString());
				m_taskSearchDataModel->setItem(x, y, item);
			}
		}
		x++;
	}
}

int LibDLHangRailRobotDBOperation::TaskDeviceTableViewDataCount(QString m_task_ssid)
{
	QSqlQuery query;
	QString sqlStr = QString("select count(*) from device_show_list where task_ssid='%1'").arg(m_task_ssid);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		return(query.value(0).toInt());
	}
}

void LibDLHangRailRobotDBOperation::TaskDeviceTableViewData(int row, int trow, QString m_task_ssid, QStandardItemModel *m_taskDoubleClickDataModel)
{
	QSqlQuery query;
//	task_device_tableModel = new QStandardItemModel();
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "站所名" <<"任务名"<< "区域名" << "设备名" << "设备类型" << "结果" << "时间" <<"状态";
	m_taskDoubleClickDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	int x = 0;
	QString sqlStr = QString("select * from device_show_list where task_ssid='%1' limit %2,%3;")
		.arg(m_task_ssid).arg(row).arg(trow);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		QString tr = query.value(0).toString();
		QString tr1 = query.value(1).toString();
		QString tr2 = query.value(2).toString();
		QString tr3 = query.value(3).toString();
		QString tr4 = query.value(4).toString();
		QString tr5 = query.value(5).toString();
		QString tr6 = query.value(6).toString() + QString("(%1)").arg(query.value(7).toString());
		QString tr7 = query.value(8).toString().replace("T"," ");
		QString tr8;
		if (query.value(9).toInt() == 0)
			tr8 = "异常";
		else
			tr8 = "正常";
		QString tr_2 = query.value(10).toString();

		QStandardItem *item1 = new QStandardItem(tr1);
		m_taskDoubleClickDataModel->setItem(x, 0, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		m_taskDoubleClickDataModel->setItem(x, 1, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		m_taskDoubleClickDataModel->setItem(x, 2, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		m_taskDoubleClickDataModel->setItem(x, 3, item4);
		QStandardItem *item5 = new QStandardItem(tr5);
		m_taskDoubleClickDataModel->setItem(x, 4, item5);
		QStandardItem *item6 = new QStandardItem(tr6);
		m_taskDoubleClickDataModel->setItem(x, 5, item6);
		QStandardItem *item7 = new QStandardItem(tr7);
		m_taskDoubleClickDataModel->setItem(x, 6, item7);
		QStandardItem *item8 = new QStandardItem(tr8);
		m_taskDoubleClickDataModel->setItem(x, 7, item8);
		QStandardItem *item9 = new QStandardItem(tr_2);
		m_taskDoubleClickDataModel->setItem(x, 8, item9);
		QStandardItem *item10 = new QStandardItem(tr);
		m_taskDoubleClickDataModel->setItem(x, 9, item10);
		x++;
	}
}

int LibDLHangRailRobotDBOperation::DevicesTableViewDataCount(bool b_Select, QString m_StartTime, QString m_StopTime, QString m_deviceName)
{
	QSqlQuery query;
    QString sqlStr;
    bool bRet = false;
	if (b_Select)
	{
		sqlStr = QString("SELECT count(*) FROM(SELECT count(*) FROM devices, inpect_result, task, station_cfg, device_name, device_type, device_unit, device_alternate_name WHERE devices.device_ssid = inpect_result.device_ssid AND inpect_result.task_ssid = task.task_ssid AND task.station_ssid = station_cfg.station_ssid AND devices.device_alternate_name_id = device_alternate_name.device_alternate_name_id AND device_name.device_type_id = device_type.device_type_id AND device_name.device_unit_id = device_unit.device_unit_id AND devices.device_name_id = device_name.device_detail_id and inspect_time>='%1' and inspect_time<='%2' and device_type_name like '%%3%' GROUP BY devices.device_ssid) t;").arg(m_StartTime).arg(m_StopTime).arg(m_deviceName);
    //    sqlStr = QString("select count(*) from select_device_showlist_dec where "
	//		"(inspect_time>='%1' and inspect_time<='%2' and device_type_name like '%%3%');").arg(m_StartTime).arg(m_StopTime).arg(m_deviceName);
        bRet = querySqlString(sqlStr, query);
		while (query.next())
		{
			return(query.value(0).toInt());
		}
	}
	else
	{
        sqlStr = QString("SELECT count(*) FROM (SELECT count(*) FROM devices,inpect_result,task,station_cfg,device_name,device_type,device_unit,device_alternate_name WHERE devices.device_ssid=inpect_result.device_ssid AND inpect_result.task_ssid = task.task_ssid AND task.station_ssid = station_cfg.station_ssid AND devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id AND device_name.device_type_id = device_type.device_type_id AND device_name.device_unit_id = device_unit.device_unit_id AND devices.device_name_id = device_name.device_detail_id GROUP BY devices.device_ssid) t;");
        bRet = querySqlString(sqlStr, query);
		while (query.next())
		{
			return(query.value(0).toInt());
		}
	}
}

void LibDLHangRailRobotDBOperation::DevicesTableViewData(int row,int trow,QStandardItemModel *m_devicesDataModel)
{
	QSqlQuery query;
//	device_tableModel = new QStandardItemModel();
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "站所名" << "任务名" << "区域名" << "设备名" << "设备类型" << "结果" << "时间" << "状态";
	m_devicesDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	int x = 0;
	QString sqlStr = QString("SELECT devices.device_ssid,inpect_result.task_ssid,station_cfg.station_name,task.task_name,devices.device_area_name,device_alternate_name.device_alternate_name,device_type.device_type_name,inpect_result.inspect_result,device_unit.device_unit_name,max(inpect_result.inspect_time),inpect_result.inspect_status,inpect_result.is_dealed FROM devices,inpect_result,task,station_cfg,device_name,device_type,device_unit,device_alternate_name WHERE devices.device_ssid=inpect_result.device_ssid AND inpect_result.task_ssid = task.task_ssid AND task.station_ssid = station_cfg.station_ssid AND devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id AND device_name.device_type_id = device_type.device_type_id AND device_name.device_unit_id = device_unit.device_unit_id AND devices.device_name_id = device_name.device_detail_id GROUP BY devices.device_ssid limit %1,%2;").arg(row).arg(trow);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		QString tr = query.value(0).toString();
		QString tr_2 = query.value(1).toString();
		QString tr1 = query.value(2).toString();
		QString tr2 = query.value(3).toString();
		QString tr3 = query.value(4).toString();
		QString tr4 = query.value(5).toString();
		QString tr5 = query.value(6).toString();
		QString tr6 = query.value(7).toString() + QString("(%1)").arg(query.value(8).toString());
		QString tr7 = query.value(9).toString().replace("T"," ");
		QString tr8;
		if (query.value(10).toInt() == 0)
			tr8 = "异常";
		else
			tr8 = "正常";
		QStandardItem *item1 = new QStandardItem(tr1);
		m_devicesDataModel->setItem(x, 0, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		m_devicesDataModel->setItem(x, 1, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		m_devicesDataModel->setItem(x, 2, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		m_devicesDataModel->setItem(x, 3, item4);
		QStandardItem *item5 = new QStandardItem(tr5);
		m_devicesDataModel->setItem(x, 4, item5);
		QStandardItem *item6 = new QStandardItem(tr6);
		m_devicesDataModel->setItem(x, 5, item6);
		QStandardItem *item7 = new QStandardItem(tr7);
		m_devicesDataModel->setItem(x, 6, item7);
		QStandardItem *item8 = new QStandardItem(tr8);
		m_devicesDataModel->setItem(x, 7, item8);
		QStandardItem *item9 = new QStandardItem(tr);
		m_devicesDataModel->setItem(x, 8, item9);
		QStandardItem *item10 = new QStandardItem(tr_2);
		m_devicesDataModel->setItem(x, 9, item10);
		x++;
	}
}
void LibDLHangRailRobotDBOperation::DevicesSelectTableViewData(int row, int trow, QString m_StartTime, QString m_StopTime, QString m_deviceName, QStandardItemModel *m_devicesSearchDataModel)
{
	QSqlQuery query;
//	deviceSelect_tableModel = new QStandardItemModel();
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "站所名" << "任务名" << "区域名" << "设备名" << "设备类型" << "结果" << "时间" << "状态";
	m_devicesSearchDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	int x = 0;
	QString sqlStr = QString("SELECT devices.device_ssid,inpect_result.task_ssid,station_cfg.station_name,task.task_name,devices.device_area_name,device_alternate_name.device_alternate_name,device_type.device_type_name,inpect_result.inspect_result,device_unit.device_unit_name,max(inpect_result.inspect_time),inpect_result.inspect_status,inpect_result.is_dealed FROM devices,inpect_result,task,station_cfg,device_name,device_type,device_unit,device_alternate_name WHERE devices.device_ssid=inpect_result.device_ssid AND inpect_result.task_ssid = task.task_ssid AND task.station_ssid = station_cfg.station_ssid AND devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id AND device_name.device_type_id = device_type.device_type_id AND device_name.device_unit_id = device_unit.device_unit_id AND devices.device_name_id = device_name.device_detail_id and inspect_time>='%1' and inspect_time<='%2' and device_type_name like '%%3%' GROUP BY devices.device_ssid limit %4,%5;")
		.arg(m_StartTime).arg(m_StopTime).arg(m_deviceName).arg(row).arg(trow);
// 	QString sqlStr = QString("select * from select_device_showlist_dec where"
// 		"(inspect_time>='%1') and (inspect_time<='%2') and device_type_name like '%%3%' limit %4,%5;")
// 		.arg(m_StartTime).arg(m_StopTime).arg(m_deviceName).arg(row).arg(trow);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		QString tr = query.value(0).toString();
		QString tr_2 = query.value(1).toString();
		QString tr1 = query.value(2).toString();
		QString tr2 = query.value(3).toString();
		QString tr3 = query.value(4).toString();
		QString tr4 = query.value(5).toString();
		QString tr5 = query.value(6).toString();
		QString tr6 = query.value(7).toString() + QString("(%1)").arg(query.value(8).toString());
		QString tr7 = query.value(9).toString().replace("T"," ");
		QString tr8;
		if (query.value(10).toInt() == 0)
			tr8 = "异常";
		else
			tr8 = "正常";
		QStandardItem *item1 = new QStandardItem(tr1);
		m_devicesSearchDataModel->setItem(x, 0, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		m_devicesSearchDataModel->setItem(x, 1, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		m_devicesSearchDataModel->setItem(x, 2, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		m_devicesSearchDataModel->setItem(x, 3, item4);
		QStandardItem *item5 = new QStandardItem(tr5);
		m_devicesSearchDataModel->setItem(x, 4, item5);
		QStandardItem *item6 = new QStandardItem(tr6);
		m_devicesSearchDataModel->setItem(x, 5, item6);
		QStandardItem *item7 = new QStandardItem(tr7);
		m_devicesSearchDataModel->setItem(x, 6, item7);
		QStandardItem *item8 = new QStandardItem(tr8);
		m_devicesSearchDataModel->setItem(x, 7, item8);
		QStandardItem *item9 = new QStandardItem(tr);
		m_devicesSearchDataModel->setItem(x, 8, item9);
		QStandardItem *item10 = new QStandardItem(tr_2);
		m_devicesSearchDataModel->setItem(x, 9, item10);
		x++;
	}
}

int LibDLHangRailRobotDBOperation::DeviceClickTableViewDataCount(QString m_device_ssid)
{
	QSqlQuery query;
	QString sqlStr = QString("select count(*) from select_device_showlist where device_ssid='%1'").arg(m_device_ssid);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		return(query.value(0).toInt());
	}
}

void LibDLHangRailRobotDBOperation::DeviceClickTableViewData(int row, int trow, QString m_device_ssid, QStandardItemModel *m_devicesDoubleClickDataModel)
{
	QSqlQuery query;
//	deviceClick_tableModel = new QStandardItemModel();
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "站所名" << "任务名" << "区域名" << "设备名" << "设备类型" << "结果" << "时间" << "状态";
	m_devicesDoubleClickDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	int x = 0;
	QString sqlStr = QString("select * from select_device_showlist where device_ssid='%1' limit %2,%3;")
		.arg(m_device_ssid).arg(row).arg(trow);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		QString tr = query.value(0).toString();
		QString tr_2 = query.value(1).toString();
		QString tr1 = query.value(2).toString();
		QString tr2 = query.value(3).toString();
		QString tr3 = query.value(4).toString();
		QString tr4 = query.value(5).toString();
		QString tr5 = query.value(6).toString();
		QString tr6 = query.value(7).toString() + QString("(%1)").arg(query.value(8).toString());
		QString tr7 = query.value(9).toString().replace("T"," ");
		QString tr8;
		if (query.value(10).toInt() == 0)
			tr8 = "异常";
		else
			tr8 = "正常";

		QStandardItem *item1 = new QStandardItem(tr1);
		m_devicesDoubleClickDataModel->setItem(x, 0, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		m_devicesDoubleClickDataModel->setItem(x, 1, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		m_devicesDoubleClickDataModel->setItem(x, 2, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		m_devicesDoubleClickDataModel->setItem(x, 3, item4);
		QStandardItem *item5 = new QStandardItem(tr5);
		m_devicesDoubleClickDataModel->setItem(x, 4, item5);
		QStandardItem *item6 = new QStandardItem(tr6);
		m_devicesDoubleClickDataModel->setItem(x, 5, item6);
		QStandardItem *item7 = new QStandardItem(tr7);
		m_devicesDoubleClickDataModel->setItem(x, 6, item7);
		QStandardItem *item8 = new QStandardItem(tr8);
		m_devicesDoubleClickDataModel->setItem(x, 7, item8);
		QStandardItem *item9 = new QStandardItem(tr);
		m_devicesDoubleClickDataModel->setItem(x, 8, item9);
		QStandardItem *item10 = new QStandardItem(tr_2);
		m_devicesDoubleClickDataModel->setItem(x, 9, item10);
		x++;
	}
}

int LibDLHangRailRobotDBOperation::DeviceAlarmTableViewDataCount(bool b_Select, QString m_StartTime, QString m_StopTime, QString m_AreaName)
{
	QSqlQuery query;
    QString sqlStr;
    bool bRet = false;
	if (b_Select)
	{
		if (m_AreaName.isEmpty())
		{
            sqlStr = QString("select count(*) from select_device_showlist where"
				"(inspect_status='0' and is_dealed='0' and inspect_time>='%1' and inspect_time<='%2')")
				.arg(m_StartTime).arg(m_StopTime);
            bRet = querySqlString(sqlStr, query);
			while (query.next())
			{
				return(query.value(0).toInt());
			}
		}
		else
		{
            sqlStr = QString("select count(*) from select_device_showlist where"
				"(inspect_status='0' and is_dealed='0' and inspect_time>='%1' and inspect_time<='%2' and device_area_name like '%%3%')")
				.arg(m_StartTime).arg(m_StopTime).arg(m_AreaName);
            bRet = querySqlString(sqlStr, query);
			while (query.next())
			{
				return(query.value(0).toInt());
			}
		}
		
	}
	else
	{
        sqlStr = QString("select count(*) from select_device_showlist where (inspect_status='0' and is_dealed='0')");
        bRet = querySqlString(sqlStr, query);
		while (query.next())
		{
			return(query.value(0).toInt());
		}
	}
}

int LibDLHangRailRobotDBOperation::DeviceAlarmSearchAreaNameDataCount(QString m_AreaName)
{
	QSqlQuery query;
	bool bRet;
    QString sqlStr = QString("SELECT  count(*) FROM select_device_showlist WHERE inspect_status='0' and is_dealed='0' and device_area_name='%1';").arg(m_AreaName);
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		return query.value(0).toInt();
	}
}

void LibDLHangRailRobotDBOperation::DeviceAlarmTableViewData(int row, int trow, QStandardItemModel *DeviceAlarm_tableModel)
{
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "" << "站所名" << "任务名" << "区域名" << "设备名" << "设备类型" << "结果" << "时间" << "状态";
	DeviceAlarm_tableModel->setHorizontalHeaderLabels(strHeaderLabels);

	QSqlQuery query;
	int x = 0;
	QString sqlStr = QString("select * from select_device_showlist where (inspect_status='0' and is_dealed='0') limit %1,%2;").arg(row).arg(trow);
    bool bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		int i = 0;
		QString tr = query.value(0).toString();
		QString tr_2 = query.value(1).toString();
		QString tr1 = query.value(2).toString();
		QString tr2 = query.value(3).toString();
		QString tr3 = query.value(4).toString();
		QString tr4 = query.value(5).toString();
		QString tr5 = query.value(6).toString();
		QString tr6 = query.value(7).toString() + QString("(%1)").arg(query.value(8).toString());
		QString tr7 = query.value(9).toString().replace("T"," ");
		QString tr8;
		if (query.value(10).toInt() == 0)
			tr8 = "异常";
		else
			tr8 = "正常";
		
		// 插入一个空的item给checkBox使用;
		QStandardItem *item = new QStandardItem;
		DeviceAlarm_tableModel->setItem(x, i++, item);

		QStandardItem *item1 = new QStandardItem(tr1);
		DeviceAlarm_tableModel->setItem(x, i++, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		DeviceAlarm_tableModel->setItem(x, i++, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		DeviceAlarm_tableModel->setItem(x, i++, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		DeviceAlarm_tableModel->setItem(x, i++, item4);
		QStandardItem *item5 = new QStandardItem(tr5);
		DeviceAlarm_tableModel->setItem(x, i++, item5);
		QStandardItem *item6 = new QStandardItem(tr6);
		DeviceAlarm_tableModel->setItem(x, i++, item6);
		QStandardItem *item7 = new QStandardItem(tr7);
		DeviceAlarm_tableModel->setItem(x, i++, item7);
		QStandardItem *item8 = new QStandardItem(tr8);
		DeviceAlarm_tableModel->setItem(x, i++, item8);
		QStandardItem *item9 = new QStandardItem(tr);
		DeviceAlarm_tableModel->setItem(x, i++, item9);
		QStandardItem *item10 = new QStandardItem(tr_2);
		DeviceAlarm_tableModel->setItem(x, i++, item10);
		x++;
	}
}

void LibDLHangRailRobotDBOperation::DeviceAlarmSearchTableViewData(int row, int trow, QString m_StartTime, QString m_StopTime, QString m_AreaName, QStandardItemModel *DeviceAlarmSearch_tableModel)
{
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << " "<<"站所名" << "任务名" << "区域名" << "设备名" << "设备类型" << "结果" << "时间" << "状态";
	DeviceAlarmSearch_tableModel->setHorizontalHeaderLabels(strHeaderLabels);
	int x = 0;

	QSqlQuery query;
	QString sql;
	sql = QString("select * from select_device_showlist where"
			"(inspect_time>='%1' and inspect_time<='%2'and inspect_status='0' and is_dealed='0' and device_area_name like '%%3%') limit %4,%5;")
			.arg(m_StartTime).arg(m_StopTime).arg(m_AreaName).arg(row).arg(trow);
	
    bool bRet = querySqlString(sql, query);
	while (query.next())
	{
		int i = 0;
		QString tr = query.value(0).toString();
		QString tr_2 = query.value(1).toString();
		QString tr1 = query.value(2).toString();
		QString tr2 = query.value(3).toString();
		QString tr3 = query.value(4).toString();
		QString tr4 = query.value(5).toString();
		QString tr5 = query.value(6).toString();
		QString tr6 = query.value(7).toString() + QString("(%1)").arg(query.value(8).toString());
		QString tr7 = query.value(9).toString().replace("T"," ");
		QString tr8;
		if (query.value(10).toInt() == 0)
			tr8 = "异常";
		else
			tr8 = "正常";

		// 插入一个空的item给checkBox使用;
		QStandardItem *item = new QStandardItem;
		DeviceAlarmSearch_tableModel->setItem(x, i++, item);

		QStandardItem *item1 = new QStandardItem(tr1);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item4);
		QStandardItem *item5 = new QStandardItem(tr5);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item5);
		QStandardItem *item6 = new QStandardItem(tr6);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item6);
		QStandardItem *item7 = new QStandardItem(tr7);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item7);
		QStandardItem *item8 = new QStandardItem(tr8);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item8);
		QStandardItem *item9 = new QStandardItem(tr);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item9);
		QStandardItem *item10 = new QStandardItem(tr_2);
		DeviceAlarmSearch_tableModel->setItem(x, i++, item10);
		x++;
	}
}

void LibDLHangRailRobotDBOperation::DeviceAlarmUpDateViewData(QString task_ssid, QString device_ssid, QString dealed_info)
{
	QSqlQuery query;

	QString sqlStr = QString("update inpect_result set is_dealed='1',dealed_info='%1' where task_ssid='%3' and device_ssid='%4';")
		.arg(dealed_info).arg(task_ssid).arg(device_ssid);

    bool bRet = querySqlString(sqlStr, query);
}

bool LibDLHangRailRobotDBOperation::DeviceListAlarmUpDateViewData(QVector<updateAlarmInfoStruct> infoList, QString dealed_info, QString &errMsg)
{
    QSqlQuery query;
    QString sqlStr = QString("UPDATE inpect_result SET is_dealed = '1', dealed_info = '%1' WHERE").arg(dealed_info);
    for (int i = 0; i < infoList.size(); i++, sqlStr += QString(" OR "))
    {
        sqlStr += QString("(task_ssid = '%1' AND device_ssid = '%2')").arg(infoList[i].task_uuid).arg(infoList[i].device_uuid);
    }
	sqlStr += QString(";");
    sqlStr.chop(4);

    bool bRet = querySqlString(sqlStr, query);
    errMsg = query.lastError().text();
    return bRet;
}


void LibDLHangRailRobotDBOperation::TreeViewConfirmPatrolModel(QStandardItemModel *m_ConfirmPatrolModel)
{
	QSqlQuery query;
	QString device_area_name;
	m_ConfirmPatrolModel->setHorizontalHeaderLabels(QStringList() << QString("定向巡检"));
	QStandardItem* DeviceAreaName;
	QStandardItem* DeviceName;
	bool bRet = false;
    QString sqlStr;
    sqlStr = QString("select devices.device_area_name,device_alternate_name.device_alternate_name,"
		"devices.device_ssid from devices inner join device_alternate_name "
		"where devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id "
		"order by devices.device_area_name limit 0,1;");
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		device_area_name = query.value(0).toString();
		DeviceAreaName = new QStandardItem(QString("%1")
			.arg(query.value(0).toString()));
		DeviceAreaName->setEditable(false);
		DeviceAreaName->setCheckable(true);
		m_ConfirmPatrolModel->appendRow(DeviceAreaName);
	}
    sqlStr = QString("select devices.device_area_name,device_alternate_name.device_alternate_name,devices.device_ssid,device_alternate_name.device_alternate_name_id "
		"from devices inner join device_alternate_name where devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id order by devices.device_area_name;");
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		if (query.value(3).toInt() == 900006)
		{

		}
		else
		{
			if (device_area_name == query.value(0).toString())
			{
				DeviceName = new QStandardItem(QString("%1")
					.arg(query.value(1).toString()));
				DeviceName->setData(QString("%1").arg(query.value(2).toString()), Qt::UserRole);
				DeviceName->setEditable(false);
				DeviceName->setCheckable(true);
				DeviceAreaName->appendRow(DeviceName);
			}
			else
			{
				DeviceAreaName = new QStandardItem(QString("%1")
					.arg(query.value(0).toString()));
				DeviceAreaName->setEditable(false);
				DeviceAreaName->setCheckable(true);
				m_ConfirmPatrolModel->appendRow(DeviceAreaName);

				DeviceName = new QStandardItem(QString("%1")
					.arg(query.value(1).toString()));
				DeviceName->setData(QString("%1").arg(query.value(2).toString()), Qt::UserRole);
				DeviceName->setEditable(false);
				DeviceName->setCheckable(true);
				DeviceAreaName->appendRow(DeviceName);
				device_area_name = query.value(0).toString();
			}
		}
	}
}

void LibDLHangRailRobotDBOperation::TreeViewParticularPatrolModel(QStandardItemModel *m_ParticularPatrolModel)
{
	QSqlQuery query;
	bool bRet;
	QString device_patrol_type_name;
	m_ParticularPatrolModel->setHorizontalHeaderLabels(QStringList() << QString("特制巡检"));
	QStandardItem* PatrolTypeName;
	QStandardItem* AreaNameDeviceName;
	QString sqlStr = QString("select device_type.device_type_name,devices.device_area_name,"
		"device_alternate_name.device_alternate_name,devices.device_ssid from device_type inner join devices "
		"inner join device_name inner join device_alternate_name where device_name.device_type_id=device_type.device_type_id "
		"and devices.device_name_id = device_name.device_detail_id and devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id order by device_type_name desc limit 0,1;");
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		device_patrol_type_name = query.value(0).toString();
		PatrolTypeName = new QStandardItem(QString("%1")
			.arg(query.value(0).toString()));
		PatrolTypeName->setEditable(false);
		PatrolTypeName->setCheckable(true);/////
		m_ParticularPatrolModel->appendRow(PatrolTypeName);
	}
    sqlStr = QString("select device_type.device_type_name,devices.device_area_name,device_alternate_name.device_alternate_name,devices.device_ssid,device_alternate_name.device_alternate_name_id from device_type inner join devices inner join device_name inner join device_alternate_name where device_name.device_type_id=device_type.device_type_id and devices.device_name_id = device_name.device_detail_id and devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id order by device_type_name desc;");
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		if (query.value(4).toInt()==900006)
		{

		}
		else
		{
			if (device_patrol_type_name == query.value(0).toString())
			{
				AreaNameDeviceName = new QStandardItem(QString("%1_%2")
					.arg(query.value(1).toString()).arg(query.value(2).toString()));
				AreaNameDeviceName->setData(QString("%1").arg(query.value(3).toString()), Qt::UserRole);
				AreaNameDeviceName->setEditable(false);
				AreaNameDeviceName->setCheckable(true);
				PatrolTypeName->appendRow(AreaNameDeviceName);
			}
			else
			{
				PatrolTypeName = new QStandardItem(QString("%1")
					.arg(query.value(0).toString()));
				PatrolTypeName->setEditable(false);
				PatrolTypeName->setCheckable(true);////
				m_ParticularPatrolModel->appendRow(PatrolTypeName);

				AreaNameDeviceName = new QStandardItem(QString("%1_%2")
					.arg(query.value(1).toString()).arg(query.value(2).toString()));
				AreaNameDeviceName->setData(QString("%1").arg(query.value(3).toString()), Qt::UserRole);
				AreaNameDeviceName->setEditable(false);
				AreaNameDeviceName->setCheckable(true);
				PatrolTypeName->appendRow(AreaNameDeviceName);
				device_patrol_type_name = query.value(0).toString();
			}
		}
	}
}

void LibDLHangRailRobotDBOperation::getDeviceCurveData(int &x,int &y,QString device_ssid,QStandardItemModel *m_DeviceCurveModel)
{
	QSqlQuery query;
    QString sqlStr;
	QList<QString> DeviceSsidData;
	QList<QString> TaskSsidData;
	QList<QString> ListNameData;
	bool bRet;
	int deviceCount = 0;
	int taskCount = 0;
	int listNameCount = 0;
	int virtualDevice = 0;
	int virtuaJudge = 0;
	x = 0;
	y = 0;
	QString judgeVirtualDevice = "";

    sqlStr = QString("select relative_dev,device_alternate_name_id from devices where device_ssid = '%1';").arg(device_ssid);
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		judgeVirtualDevice = query.value(0).toString();
		virtuaJudge = query.value(1).toInt();
		if (virtuaJudge == VideoDevice || virtuaJudge == VoiceDevice)
		{
			x = 99;
			return;
		}
	}
	if (!judgeVirtualDevice.isEmpty())
	{
        sqlStr = QString("select count(*) from devices where relative_dev = '%1';").arg(judgeVirtualDevice);
        bRet = querySqlString(sqlStr, query);
		while (query.next())
		{
			virtualDevice = query.value(0).toInt();
		}
	}
	if (judgeVirtualDevice.isEmpty() || virtualDevice == 1)
	{
        sqlStr = QString("select inpect_result.inspect_result,device_alternate_name.device_alternate_name,inpect_result.inspect_status,device_name.device_detail_id,device_type.device_type_id,threshold_patrol.patrol_alarm_up,threshold_patrol.patrol_alarm_down from devices inner join inpect_result inner join device_name inner join device_type inner join threshold_patrol inner join device_alternate_name where devices.device_ssid=inpect_result.device_ssid and devices.device_name_id=device_name.device_detail_id and devices.device_ssid='%1' and device_name.device_type_id=device_type.device_type_id and device_name.device_type_id=threshold_patrol.threshold_patrol_id and device_name.device_detail_id=devices.device_name_id and devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id order by inpect_result.inspect_time desc limit 0,10;").arg(device_ssid);
        bRet = querySqlString(sqlStr, query);
		while (query.next())
		{
			virtuaJudge = query.value(3).toInt();
			
			taskCount++;
			if (taskCount == 1)
			{
				if (virtuaJudge < NumberDevice || (virtuaJudge >= StringDevice && virtuaJudge<= NumberDeviceNine) || virtuaJudge == LoadDeviceNine || virtuaJudge == ThreeTempDeviceNine)
				{
					QStandardItem *item3 = new QStandardItem(QString("1"));
					m_DeviceCurveModel->setItem(0, 0, item3);
					QStandardItem *item4 = new QStandardItem(query.value(1).toString());
					m_DeviceCurveModel->setItem(0, 1, item4);
				}
				else
				{
					QStandardItem *item3 = new QStandardItem(QString("2"));
					m_DeviceCurveModel->setItem(0, 0, item3);
					QStandardItem *item4 = new QStandardItem(query.value(1).toString());
					m_DeviceCurveModel->setItem(0, 1, item4);
				}
			}
	//		QString a = QString("%1 %2 %3").arg(query.value(4).toString()).arg(query.value(5).toString()).arg(query.value(6).toString());
			QStandardItem *item5 = new QStandardItem(QString("%1 %2 %3").arg(query.value(4).toString()).arg(query.value(5).toString()).arg(query.value(6).toString()));
			m_DeviceCurveModel->setItem(taskCount, 0, item5);

			if (virtuaJudge < NumberDevice || (virtuaJudge >= StringDevice && virtuaJudge <= NumberDeviceNine) || virtuaJudge == LoadDeviceNine || virtuaJudge == ThreeTempDeviceNine)
			{
				QStandardItem *item6 = new QStandardItem(query.value(0).toString());
				m_DeviceCurveModel->setItem(taskCount, 1, item6);
			}
			else
			{
				QStandardItem *item6 = new QStandardItem(query.value(2).toString());
				m_DeviceCurveModel->setItem(taskCount, 1, item6);
			}
			x++;
		}
		x++;
		y = 2;
	}
	if(!judgeVirtualDevice.isEmpty() && virtualDevice!=1)
	{
		sqlStr = QString("select devices.device_ssid,device_alternate_name.device_alternate_name,device_name.device_detail_id from devices inner join device_name inner join device_alternate_name where devices.device_name_id=device_name.device_detail_id and devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id and devices.relative_dev=(select relative_dev from devices where device_ssid='%1');").arg(device_ssid);
        bRet = querySqlString(sqlStr, query);

		while (query.next())
		{
			virtuaJudge = query.value(2).toInt();
			DeviceSsidData.append(query.value(0).toString());
			ListNameData.append(query.value(1).toString());
		}
		if (virtuaJudge < NumberDevice || (virtuaJudge >= StringDevice && virtuaJudge <= NumberDeviceNine) || virtuaJudge == LoadDeviceNine || virtuaJudge == ThreeTempDeviceNine)
		{
			ListNameData.prepend("1");
		}
		else
		{
			ListNameData.prepend("2");
		}
		deviceCount = DeviceSsidData.size();
		//////////
		if (deviceCount == 2)
		{
            sqlStr = QString("select task_ssid,inspect_time from inpect_result where device_ssid='%1' or device_ssid='%2' group by task_ssid order by inspect_time desc limit 0,10;")
				.arg(DeviceSsidData[0]).arg(DeviceSsidData[1]);
            bRet = querySqlString(sqlStr, query);
			while (query.next())
			{
				TaskSsidData.prepend(query.value(0).toString());
			}
		}
		if (deviceCount == 3)
		{
			sqlStr = QString("select task_ssid,inspect_time from inpect_result where device_ssid='%1' or device_ssid='%2' or device_ssid='%3' group by task_ssid order by inspect_time desc limit 0,10;")
				.arg(DeviceSsidData[0]).arg(DeviceSsidData[1]).arg(DeviceSsidData[2]);
            bRet = querySqlString(sqlStr, query);
			while (query.next())
			{
				TaskSsidData.prepend(query.value(0).toString());
			}
		}

		taskCount = TaskSsidData.size();

		for (int k = 0; k < ListNameData.size(); k++)
		{
			QStandardItem *item = new QStandardItem(ListNameData.at(k));
			m_DeviceCurveModel->setItem(0, k, item);
		}

		for (int i = 0; i < taskCount; i++)
		{
			if (i == 0) {
				sqlStr = QString("select device_type.device_type_id, threshold_patrol.patrol_alarm_up, threshold_patrol.patrol_alarm_down from devices inner join device_name inner join device_type inner join threshold_patrol where devices.device_name_id = device_name.device_detail_id and devices.device_ssid = '%1' and device_name.device_type_id = device_type.device_type_id and device_name.device_type_id = threshold_patrol.threshold_patrol_id;")
					.arg(DeviceSsidData[i]);
				bRet = querySqlString(sqlStr, query);
				while (query.next())
				{
					QStandardItem *item1 = new QStandardItem(QString("%1 %2 %3").arg(query.value(0).toString()).arg(query.value(1).toString()).arg(query.value(2).toString()));
					m_DeviceCurveModel->setItem(i + 1, 0, item1);
				}
			}
			for (int j = 0; j < deviceCount; j++)
			{
				sqlStr = QString("select inspect_result,inspect_status from inpect_result where task_ssid='%1' and device_ssid='%2';")
					.arg(TaskSsidData.at(i)).arg(DeviceSsidData.at(j));
				bRet = querySqlString(sqlStr, query);
// 				bRet = query.exec(QString("select inspect_result,inspect_status from inpect_result where task_ssid='%1' and device_ssid='%2';")
// 					.arg(TaskSsidData.at(i)).arg(DeviceSsidData.at(j)));
				while (query.next())
				{
					if (virtuaJudge < NumberDevice || (virtuaJudge >= StringDevice && virtuaJudge <= NumberDeviceNine) || virtuaJudge == LoadDeviceNine || virtuaJudge == ThreeTempDeviceNine)
					{
						QStandardItem *item2 = new QStandardItem(query.value(0).toString());
						m_DeviceCurveModel->setItem(i + 1, j + 1, item2);
					}
					else
					{
						QStandardItem *item2 = new QStandardItem(query.value(1).toString());
						m_DeviceCurveModel->setItem(i + 1, j + 1, item2);
					}
					
				}
			}
		}
		x = taskCount + 1;
		y = deviceCount + 1;
		int g;
	}	
}

void LibDLHangRailRobotDBOperation::ThresholdEnvironmentViewData(QStandardItemModel *m_ThresholdEnviDataModel)
{
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "站所名称" << "环境设备类型" << "告警值上限" << "告警值下限";
	m_ThresholdEnviDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	QSqlQuery query;
	int x = 0;

    bool bRet = querySqlString("select * from threshold_envi", query);

	while (query.next())
	{
		int i = 0;
		QString tr1 = query.value(1).toString();
		QString tr2 = query.value(2).toString();
		QString tr3 = query.value(3).toString();
		QString tr4 = query.value(4).toString();

		QStandardItem *item1 = new QStandardItem(tr1);
		m_ThresholdEnviDataModel->setItem(x, i++, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		m_ThresholdEnviDataModel->setItem(x, i++, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		m_ThresholdEnviDataModel->setItem(x, i++, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		m_ThresholdEnviDataModel->setItem(x, i++, item4);
		x++;
	}
}

void LibDLHangRailRobotDBOperation::ThresholdPatrolViewData(QStandardItemModel *m_ThresholdPatrolDataModel)
{
	// 设置表头;
	QStringList strHeaderLabels;
	strHeaderLabels << "站所名称" << "环境设备类型" << "告警值上限" << "告警值下限";
	m_ThresholdPatrolDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	QSqlQuery query;
	int x = 0;
    bool bRet = querySqlString("select * from threshold_patrol", query);
	while (query.next())
	{
		int i = 0;
		QString tr1 = query.value(1).toString();
		QString tr2 = query.value(2).toString();
		QString tr3 = query.value(3).toString();
		QString tr4 = query.value(4).toString();

		QStandardItem *item1 = new QStandardItem(tr1);
		m_ThresholdPatrolDataModel->setItem(x, i++, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		m_ThresholdPatrolDataModel->setItem(x, i++, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		m_ThresholdPatrolDataModel->setItem(x, i++, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		m_ThresholdPatrolDataModel->setItem(x, i++, item4);
		x++;
	}
}

bool LibDLHangRailRobotDBOperation::insertAndUpdateMapData(QList<MapItemData> MapDataStruct, QString station_id)
{
	QSqlQuery query;
	bool bRet;
	QString insertData;
	QString deleteData = QString("delete from map_data where station_id='%1';").arg(station_id);
    bRet = querySqlString(deleteData, query);
	for (int i = 0; i < MapDataStruct.size(); i++)
	{
		insertData = QString("insert into map_data (coordinate_x1,coordinate_y1,coordinate_x2,coordinate_y2,start_skewing,end_skewing,insertion_type,device_name,station_id) values ('%1','%2','%3','%4','%5','%6','%7','%8','%9');")
			.arg(MapDataStruct[i].startPointX).arg(MapDataStruct[i].startPointY)
			.arg(MapDataStruct[i].endPointX).arg(MapDataStruct[i].endPointY)
			.arg(MapDataStruct[i].startOffset).arg(MapDataStruct[i].endOffset)
			.arg(MapDataStruct[i].itemType).arg(MapDataStruct[i].deviceName)
			.arg(MapDataStruct[i].staionId);
        bRet = querySqlString(insertData, query);
	}
	return bRet;
}
bool LibDLHangRailRobotDBOperation::getMapDataList(QList<MapItemData> &m_MapDataList, QString station_id)
{
	QSqlQuery query;
	bool bRet;
	MapItemData MapDataStruct;
	QString getData = QString("select * from map_data where station_id='%1';").arg(station_id);
    bRet = querySqlString(getData, query);
	while (query.next())
	{
		MapDataStruct.startPointX = query.value(1).toInt();
		MapDataStruct.startPointY = query.value(2).toInt();
		MapDataStruct.endPointX = query.value(3).toInt();
		MapDataStruct.endPointY = query.value(4).toInt();
		MapDataStruct.startOffset = query.value(5).toInt();
		MapDataStruct.endOffset = query.value(6).toInt();
		MapDataStruct.itemType = query.value(7).toInt();
		MapDataStruct.deviceName = query.value(8).toString();
		MapDataStruct.staionId = query.value(9).toString();
		m_MapDataList.append(MapDataStruct);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getMapCornerItem(QList<MapItemData>& m_MapDataList, QString station_id)
{
    QSqlQuery query;
    bool bRet;
    MapItemData MapDataStruct;
    QString getData = QString("select * from map_data where station_id='%1' and (insertion_type=2 or insertion_type=3);").arg(station_id);
    bRet = querySqlString(getData, query);
    while (query.next())
    {
        MapDataStruct.startPointX = query.value(1).toInt();
        MapDataStruct.startPointY = query.value(2).toInt();
        MapDataStruct.endPointX = query.value(3).toInt();
        MapDataStruct.endPointY = query.value(4).toInt();
        MapDataStruct.startOffset = query.value(5).toInt();
        MapDataStruct.endOffset = query.value(6).toInt();
        MapDataStruct.itemType = query.value(7).toInt();
        MapDataStruct.deviceName = query.value(8).toString();
        MapDataStruct.staionId = query.value(9).toString();
        m_MapDataList.append(MapDataStruct);
    }
    return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceAreaNameList(QList<QString> &m_deviceAreaName)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("select  DISTINCT device_area_name from devices;");
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		m_deviceAreaName.append(query.value(0).toString());
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceAlarmSearchAreaNameData(int m_Page, int trow, QString m_AreaName, QStandardItemModel *DeviceAlarmAreaName_tableModel)
{
	QSqlQuery query;
	bool bRet;
	int x = 0;
	QStringList strHeaderLabels;
	strHeaderLabels << "" << "站所名" << "任务名" << "区域名" << "设备名" << "设备类型" << "结果" << "时间" << "状态";
	DeviceAlarmAreaName_tableModel->setHorizontalHeaderLabels(strHeaderLabels);

	QString sqlStr = QString("SELECT  * FROM select_device_showlist WHERE inspect_status='0' and is_dealed='0' and device_area_name='%1' limit %2,%3;").arg(m_AreaName).arg(m_Page).arg(trow);
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		int i = 0;
		QString tr = query.value(0).toString();
		QString tr_2 = query.value(1).toString();
		QString tr1 = query.value(2).toString();
		QString tr2 = query.value(3).toString();
		QString tr3 = query.value(4).toString();
		QString tr4 = query.value(5).toString();
		QString tr5 = query.value(6).toString();
		QString tr6 = query.value(7).toString() + QString("(%1)").arg(query.value(8).toString());
		QString tr7 = query.value(9).toString().replace("T"," ");
		QString tr8;
		if (query.value(10).toInt() == 0)
			tr8 = "异常";
		else
			tr8 = "正常";

		// 插入一个空的item给checkBox使用;
		QStandardItem *item = new QStandardItem;
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item);
		QStandardItem *item1 = new QStandardItem(tr1);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item4);
		QStandardItem *item5 = new QStandardItem(tr5);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item5);
		QStandardItem *item6 = new QStandardItem(tr6);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item6);
		QStandardItem *item7 = new QStandardItem(tr7);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item7);
		QStandardItem *item8 = new QStandardItem(tr8);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item8);
		QStandardItem *item9 = new QStandardItem(tr);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item9);
		QStandardItem *item10 = new QStandardItem(tr_2);
		DeviceAlarmAreaName_tableModel->setItem(x, i++, item10);
		x++;
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::DoubleClickIntPatrolTableViewData(QString m_taskSsid, QString m_deviceSsid, QStandardItemModel *m_doubleClickIntPatrolDataModel)
{
	QSqlQuery query;
	bool bRet;
	QStringList strHeaderLabels;
	strHeaderLabels << "站所名" << "任务名" << "区域名" << "设备名" << "设备类型" << "结果" << "时间" << "状态";
	m_doubleClickIntPatrolDataModel->setHorizontalHeaderLabels(strHeaderLabels);

	int x = 0;
	QString sqlStr = QString("select * from select_device_showlist where task_ssid='%1' and device_ssid='%2';")
		.arg(m_taskSsid).arg(m_deviceSsid);

    bRet = querySqlString(sqlStr, query);

	while (query.next())
	{
		QString tr = query.value(0).toString();
		QString tr_2 = query.value(1).toString();
		QString tr1 = query.value(2).toString();
		QString tr2 = query.value(3).toString();
		QString tr3 = query.value(4).toString();
		QString tr4 = query.value(5).toString();
		QString tr5 = query.value(6).toString();
		QString tr6 = query.value(7).toString() + QString("(%1)").arg(query.value(8).toString());
		QString tr7 = query.value(9).toString().replace("T"," ");
		QString tr8;
		if (query.value(10).toInt() == 0)
			tr8 = "异常";
		else
			tr8 = "正常";

		QStandardItem *item1 = new QStandardItem(tr1);
		m_doubleClickIntPatrolDataModel->setItem(x, 0, item1);
		QStandardItem *item2 = new QStandardItem(tr2);
		m_doubleClickIntPatrolDataModel->setItem(x, 1, item2);
		QStandardItem *item3 = new QStandardItem(tr3);
		m_doubleClickIntPatrolDataModel->setItem(x, 2, item3);
		QStandardItem *item4 = new QStandardItem(tr4);
		m_doubleClickIntPatrolDataModel->setItem(x, 3, item4);
		QStandardItem *item5 = new QStandardItem(tr5);
		m_doubleClickIntPatrolDataModel->setItem(x, 4, item5);
		QStandardItem *item6 = new QStandardItem(tr6);
		m_doubleClickIntPatrolDataModel->setItem(x, 5, item6);
		QStandardItem *item7 = new QStandardItem(tr7);
		m_doubleClickIntPatrolDataModel->setItem(x, 6, item7);
		QStandardItem *item8 = new QStandardItem(tr8);
		m_doubleClickIntPatrolDataModel->setItem(x, 7, item8);
		QStandardItem *item9 = new QStandardItem(tr);
		m_doubleClickIntPatrolDataModel->setItem(x, 8, item9);
		QStandardItem *item10 = new QStandardItem(tr_2);
		m_doubleClickIntPatrolDataModel->setItem(x, 9, item10);
		x++;
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::insertEnvironmentResultData(EnvironmentResult m_enviResult)
{
	QSqlQuery query;
	bool bRet;
	QString sql = QString("insert into environment_result (envi_result_datetime,envi_result_temp,envi_result_humi,envi_result_sf6,envi_result_o3) values ('%1','%2','%3','%4','%5');")
			.arg(m_enviResult.dateTime)
			.arg(m_enviResult.envi_result_temp)
			.arg(m_enviResult.envi_result_humi)
			.arg(m_enviResult.envi_result_sf6)
			.arg(m_enviResult.envi_result_o3);
    bRet = querySqlString(sql, query);
	return bRet;
}

bool LibDLHangRailRobotDBOperation::EnvironmentResultEnviViewData(QStandardItemModel *m_enviResultModel)
{
	QSqlQuery query;
	bool bRet;
	int x = 0;
	QStringList strHeaderLabels;
	strHeaderLabels << "时间" << "温度(℃)" << "湿度(%rh)" << "SF6(ppm)" << "O3(ppm)";
	m_enviResultModel->setHorizontalHeaderLabels(strHeaderLabels);
	QString sql = QString("select * from environment_result order by envi_result_datetime desc limit 0,50");
    bRet = querySqlString(sql, query);
	while (query.next())
	{
		QString strTime = query.value(1).toString();
		strTime.replace("T", " ");
		QStandardItem *item1 = new QStandardItem(strTime);
		m_enviResultModel->setItem(x, 0, item1);
		QStandardItem *item2 = new QStandardItem(query.value(2).toString());
		m_enviResultModel->setItem(x, 1, item2);
		QStandardItem *item3 = new QStandardItem(query.value(3).toString());
		m_enviResultModel->setItem(x, 2, item3);
		QStandardItem *item4 = new QStandardItem(query.value(4).toString());
		m_enviResultModel->setItem(x, 3, item4);
		QStandardItem *item5 = new QStandardItem(query.value(5).toString());
		m_enviResultModel->setItem(x, 4, item5);
		x++;
	}
	return bRet;
}
bool LibDLHangRailRobotDBOperation::deleteEnvironmentResult(QString DeletedateTime)
{
	QSqlQuery query;
	bool bRet;
	QString sql = QString("delete from environment_result where envi_result_datetime<'%1';").arg(DeletedateTime);
    bRet = querySqlString(sql, query);
	return bRet;
}

bool LibDLHangRailRobotDBOperation::EnvironmentResultEnviListData(QString m_enviKind, QList<EnvironmentResult> &m_enviData)
{
	QSqlQuery query;
	bool bRet;
	QString sql = QString("SELECT envi_result_datetime,%1 FROM environment_result ORDER BY envi_result_datetime DESC LIMIT 0,24;").arg(m_enviKind);
    bRet = querySqlString(sql, query);
	EnvironmentResult enviData;
	while (query.next())
	{
		QString strTime = query.value(0).toString();
		strTime.replace("T", "\n");

		enviData.dateTime = strTime;
		enviData.envi_value = query.value(1).toDouble();
		m_enviData.prepend(enviData);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getRestorationValueForDB(std::map<int, RestorationValue> &m_restorationValue)
{
	QSqlQuery query;
	bool bRet;
	RestorationValue restValue;
    QString sqlStr = QString("select * from restoration_value;");
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		int station_ssid = query.value(1).toInt();
		restValue.rest_id = query.value(0).toInt();
		restValue.rest_station_id = query.value(1).toInt();
		restValue.rest_move_location = query.value(2).toInt();
		restValue.rest_move_speed = query.value(3).toInt();
		restValue.rest_lift_location = query.value(4).toInt();
		restValue.rest_bady_rotate = query.value(5).toInt();
		restValue.rest_camera_rotate = query.value(6).toInt();
		restValue.rest_camera_zoom = query.value(7).toInt();
		restValue.rest_camera_focal = query.value(8).toInt();
		m_restorationValue.insert(std::make_pair(station_ssid,restValue));
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::insertRestorationValue(RestorationValue m_restorationValue)
{
	QSqlQuery query;
	bool bRet;
	bool bRect = false;
	QString sqlStr = QString("select * from restoration_value where rest_station_id = '%1'").arg(m_restorationValue.rest_station_id);
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		bRect = true;
	}
	if (bRect)
	{
        sqlStr = QString("update restoration_value set rest_move_location='%1',rest_move_speed='%2',rest_lift_location='%3', rest_bady_rotate='%4', rest_camera_rotate='%6',rest_camera_zoom='%7',rest_camera_focal='%8' where rest_id='%9';")
			.arg(m_restorationValue.rest_move_location)
			.arg(m_restorationValue.rest_move_speed)
			.arg(m_restorationValue.rest_lift_location)
			.arg(m_restorationValue.rest_bady_rotate)
			.arg(m_restorationValue.rest_camera_rotate)
			.arg(m_restorationValue.rest_camera_zoom)
			.arg(m_restorationValue.rest_camera_focal)
			.arg(m_restorationValue.rest_station_id);

        bRet = querySqlString(sqlStr, query);

	}
	else
	{
        sqlStr = QString("insert into restoration_value (rest_station_id,rest_move_location,rest_move_speed,rest_lift_location,rest_bady_rotate,rest_camera_rotate,rest_camera_zoom,rest_camera_focal) values ('%1','%2','%3','%4','%5','%6','%7','%8');")
			.arg(m_restorationValue.rest_station_id)
			.arg(m_restorationValue.rest_move_location)
			.arg(m_restorationValue.rest_move_speed)
			.arg(m_restorationValue.rest_lift_location)
			.arg(m_restorationValue.rest_bady_rotate)
			.arg(m_restorationValue.rest_camera_rotate)
			.arg(m_restorationValue.rest_camera_zoom)
			.arg(m_restorationValue.rest_camera_focal);

        bRet = querySqlString(sqlStr, query);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getVirtualDeviceList(QList<RelevanceDevice> &m_virDeviceInitializeList)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("SELECT * FROM virtualdevices ORDER BY vir_dev_area_name;");
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		RelevanceDevice releDevice;
		releDevice.m_area_name = query.value(2).toString();
		releDevice.m_virtual_name_id = query.value(1).toInt();
		releDevice.m_virtualDevice_id = query.value(0).toString();
		if (query.value(4).toString().isEmpty())
		{
			releDevice.m_device_id.append(NULL);
		}
		else
		{
			releDevice.m_device_id = query.value(4).toString().split(" ");
		}
		m_virDeviceInitializeList.append(releDevice);
	}

	return bRet;
}

bool LibDLHangRailRobotDBOperation::getVirtualDeviceNameMap(std::map<int ,QString> &m_virDeviceNameMap)
{
	QSqlQuery query;
	bool bRet;
    QString sqlStr = QString("select device_detail_id,device_name from device_name where device_detail_id>='990101';");
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		m_virDeviceNameMap.insert(std::make_pair(query.value(0).toInt(), query.value(1).toString()));
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getSearchAreaDeviceList(QString m_areaName, QList<QString> &m_searchAreaDeviceList)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("select device_ssid,device_alternate_name_id,relative_dev from devices where device_area_name='%1';").arg(m_areaName);
    bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		if (query.value(2).toString().isEmpty())
		{
			m_searchAreaDeviceList.append(QString("%1 %2").arg(query.value(0).toString()).arg(query.value(1).toString()));
		}
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::upDateDevicesForRelative(RelevanceDevice m_releDevice)
{
	QSqlQuery query;
	bool bRet;
	for (int i = 0; i < m_releDevice.m_device_id.size(); i++)
	{
		QString sqlStr = QString("update devices set relative_dev = '%1' where device_ssid = '%2';")
			.arg(m_releDevice.m_virtualDevice_id)
			.arg(m_releDevice.m_device_id[i]);
        bRet = querySqlString(sqlStr, query);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getTaskHistoryDataFromDB(TaskSearchCondition condi, QList<TaskShowItemInfo>& data)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr;
	if (condi.startTime.isEmpty())
	{
		sqlStr = QString("SELECT task.task_name,task.task_total_devices,task.task_total_bugs,task.task_start_time,task.task_ssid FROM task ORDER BY task.task_start_time DESC LIMIT %1,%2;")
			.arg(condi.nowCount).arg(condi.showCount);
	}
	else
	{
		sqlStr = QString("SELECT task.task_name,task.task_total_devices,task.task_total_bugs,task.task_start_time,task.task_ssid FROM task WHERE task.task_start_time>'%1' AND task.task_end_time<'%2' ORDER BY task.task_start_time DESC LIMIT %3,%4;")
			.arg(condi.startTime).arg(condi.stopTime).arg(condi.nowCount).arg(condi.showCount);
	}
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		TaskShowItemInfo stru;
		stru.strTitle = query.value(0).toString();
		stru.totalCount = query.value(1).toInt();
		stru.abnormalCount = query.value(2).toInt();
		stru.strTime = query.value(3).toString().replace("T"," ");
		stru.strTaskId = query.value(4).toString();
		data.append(stru);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceInspectResultData(QString task_ssid, QList<DeviceInspectResult> &data, TaskSearchCondition condi)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("SELECT device_name.device_name,inpect_result.inspect_result FROM task,devices,device_name,inpect_result WHERE task.task_ssid=inpect_result.task_ssid AND inpect_result.device_ssid=devices.device_ssid AND devices.device_name_id=device_name.device_detail_id AND task.task_ssid='%1' limit %2,%3;")
		.arg(task_ssid)
		.arg(condi.nowCount)
		.arg(condi.showCount);
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		DeviceInspectResult stru;
		stru.device_name = query.value(0).toString();
		stru.inspect_result = query.value(1).toString();
		data.append(stru);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getTreeConfirmPatrolForPadDB(QMap<QString, QList<TreeTaskIsuData>> &map)
{
	QSqlQuery query;
	QString iniData;
	QString nextData = "";
	int i = 0;
	bool bRet = false;
	QList<TreeTaskIsuData> listData;
	TreeTaskIsuData struData;
	QString sqlStr;
	sqlStr = QString("select devices.device_area_name,device_alternate_name.device_alternate_name,devices.device_ssid,device_alternate_name.device_alternate_name_id from devices inner join device_alternate_name where devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id order by devices.device_area_name;");
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		if (query.value(3).toInt() == 900006)
		{}
		else
		{
			iniData = query.value(0).toString();
			if (i == 0)
			{
				struData.about_name = query.value(1).toString();
				struData.device_ssid = query.value(2).toString();
				listData.append(struData);
			}
			else
			{
				if (iniData == nextData)
				{
					struData.about_name = query.value(1).toString();
					struData.device_ssid = query.value(2).toString();
					listData.append(struData);
				}
				else
				{
					map.insert(nextData, listData);
					listData.clear();
					struData.about_name = query.value(1).toString();
					struData.device_ssid = query.value(2).toString();
					listData.append(struData);
				}
			}
			nextData = iniData;
			i++;
		}
	}
	if (iniData.isEmpty())
	{
	}
	else
	{
		map.insert(nextData, listData);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getTreeParticularPatrolForPadDB(QMap<QString, QList<TreeTaskIsuData>> &map)
{
	QSqlQuery query;
	QString iniData;
	QString nextData = "";
	int i = 0;
	bool bRet = false;
	QList<TreeTaskIsuData> listData;
	TreeTaskIsuData struData;
	QString sqlStr;
	sqlStr = QString("select device_type.device_type_name,devices.device_area_name,device_alternate_name.device_alternate_name,devices.device_ssid,device_alternate_name.device_alternate_name_id from device_type inner join devices inner join device_name inner join device_alternate_name where device_name.device_type_id=device_type.device_type_id and devices.device_name_id = device_name.device_detail_id and devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id order by device_type_name desc;");
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		if (query.value(4).toInt() == 900006)
		{
		}
		else
		{
			iniData = query.value(0).toString();
			if (i == 0)
			{
				struData.about_name = query.value(1).toString() + "_" + query.value(2).toString();
				struData.device_ssid = query.value(3).toString();
				listData.append(struData);
			}
			else
			{
				if (iniData == nextData)
				{
					struData.about_name = query.value(1).toString() + "_" + query.value(2).toString();
					struData.device_ssid = query.value(3).toString();
					listData.append(struData);
				}
				else
				{
					map.insert(nextData, listData);
					listData.clear();
					struData.about_name = query.value(1).toString() + "_" + query.value(2).toString();
					struData.device_ssid = query.value(3).toString();
					listData.append(struData);
				}
			}
			nextData = iniData;
			i++;
		}
	}
	if (iniData.isEmpty())
	{
	}
	else
	{
		map.insert(nextData, listData);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getDeviceHistoryInspectResultDataDB(TaskSearchCondition condi, QStringList deviceAreaName, QStringList deviceTypeId, QList<DeviceRecordInfo> &data)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr;
	if (deviceAreaName.size() != 0 && deviceTypeId.size() != 0)
	{
		QString strArea = QString("devices.device_area_name='%1'").arg(deviceAreaName[0]);
		for (int i = 0; i < deviceAreaName.size(); i++)
		{
			if (i == 0)
			{
			}
			else
			{
				strArea = strArea + QString(" or ") + QString("devices.device_area_name='%1'").arg(deviceAreaName[i]);
			}
		}

		QString strType = QString("device_type.device_type_id='%1'").arg(deviceTypeId[0]);
		for (int j = 0; j < deviceTypeId.size(); j++)
		{
			if (j == 0)
			{
			}
			else
			{
				strType = strType + QString(" or ") + QString("device_type.device_type_id='%1'").arg(deviceTypeId[j]);
			}
		}
		sqlStr = QString("SELECT devices.device_ssid,device_name.device_name,devices.device_area_name,device_type.device_type_name,inpect_result.inspect_result,inpect_result.inspect_time FROM inpect_result,devices,device_type,device_name,task WHERE inpect_result.device_ssid=devices.device_ssid AND devices.device_name_id=device_name.device_detail_id AND device_name.device_type_id=device_type.device_type_id AND inpect_result.task_ssid=task.task_ssid AND (%1) AND (%2) ORDER BY inpect_result.inspect_time DESC limit %3,%4;")
			.arg(strArea)
			.arg(strType)
			.arg(condi.nowCount)
			.arg(condi.showCount);
	}
	else
	{
		if (deviceAreaName.size() != 0)
		{
			QString strArea = QString("devices.device_area_name='%1'").arg(deviceAreaName[0]);
			for (int i = 0; i < deviceAreaName.size(); i++)
			{
				if (i == 0)
				{
				}
				else
				{
					strArea = strArea + QString(" or ") + QString("devices.device_area_name='%1'").arg(deviceAreaName[i]);
				}
			}
			sqlStr = QString("SELECT devices.device_ssid,device_name.device_name,devices.device_area_name,device_type.device_type_name,inpect_result.inspect_result,inpect_result.inspect_time FROM inpect_result,devices,device_type,device_name,task WHERE inpect_result.device_ssid=devices.device_ssid AND devices.device_name_id=device_name.device_detail_id AND device_name.device_type_id=device_type.device_type_id AND inpect_result.task_ssid=task.task_ssid AND (%1) ORDER BY inpect_result.inspect_time DESC limit %2,%3;")
				.arg(strArea)
				.arg(condi.nowCount)
				.arg(condi.showCount);
		}
		if (deviceTypeId.size() != 0)
		{
			QString strType = QString("device_type.device_type_id='%1'").arg(deviceTypeId[0]);
			for (int i = 0; i < deviceTypeId.size(); i++)
			{
				if (i == 0)
				{
				}
				else
				{
					strType = strType + QString(" or ") + QString("device_type.device_type_id='%1'").arg(deviceTypeId[i]);
				}
			}
			sqlStr = QString("SELECT devices.device_ssid,device_name.device_name,devices.device_area_name,device_type.device_type_name,inpect_result.inspect_result,inpect_result.inspect_time FROM inpect_result,devices,device_type,device_name,task WHERE inpect_result.device_ssid=devices.device_ssid AND devices.device_name_id=device_name.device_detail_id AND device_name.device_type_id=device_type.device_type_id AND inpect_result.task_ssid=task.task_ssid AND (%1) ORDER BY inpect_result.inspect_time DESC limit %2,%3;")
				.arg(strType)
				.arg(condi.nowCount)
				.arg(condi.showCount);
		}
		if (deviceAreaName.size() == 0 && deviceTypeId.size() == 0)
		{
			sqlStr = QString("SELECT devices.device_ssid,device_name.device_name,devices.device_area_name,device_type.device_type_name,inpect_result.inspect_result,inpect_result.inspect_time FROM inpect_result,devices,device_type,device_name,task WHERE inpect_result.device_ssid=devices.device_ssid AND devices.device_name_id=device_name.device_detail_id AND device_name.device_type_id=device_type.device_type_id AND inpect_result.task_ssid=task.task_ssid ORDER BY inpect_result.inspect_time DESC limit %1,%2;")
				.arg(condi.nowCount)
				.arg(condi.showCount);
		}
	}
	
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		DeviceRecordInfo stru;
		int i = 0;
		stru.strDeviceSsid = query.value(i++).toString();
		stru.strDeviceName = query.value(i++).toString();
		stru.strDeviceArea = query.value(i++).toString();
		stru.strDeviceType = query.value(i++).toString();
		stru.strPatrolResult = query.value(i++).toString();
		stru.strPatrolTime = query.value(i++).toString().replace("T", " ");
		data.append(stru);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getHistoryDeviceByDeviceSsidDB(TaskSearchCondition condi, QString device_ssid, DeviceHistoryRecord &data)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr;
	sqlStr = QString("SELECT device_name.device_name,devices.device_area_name,device_type.device_type_name,count(if(inpect_result.inspect_status='1',TRUE ,NULL )),count(if(inpect_result.inspect_status='0',TRUE ,NULL)),count(*) FROM task,inpect_result,devices,device_name,device_type WHERE task.task_ssid=inpect_result.task_ssid AND devices.device_ssid=inpect_result.device_ssid AND devices.device_name_id=device_name.device_detail_id AND device_name.device_type_id=device_type.device_type_id AND devices.device_ssid='%1';")
		.arg(device_ssid);
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		int i = 0;
		data.strDeviceName = query.value(i++).toString();
		data.strDeviceArea = query.value(i++).toString();
		data.strDeviceType = query.value(i++).toString();
		data.iNormalCount = query.value(i++).toInt();
		data.iAbnormalCount = query.value(i++).toInt();
		data.iTotalCount = query.value(i++).toInt();
	}

	sqlStr = QString("SELECT task.task_name,inpect_result.inspect_time,inpect_result.inspect_result FROM task,inpect_result,devices,device_name,device_type WHERE task.task_ssid=inpect_result.task_ssid AND devices.device_ssid=inpect_result.device_ssid AND devices.device_name_id=device_name.device_detail_id AND device_name.device_type_id=device_type.device_type_id AND devices.device_ssid='%1' ORDER BY inpect_result.inspect_time desc limit %2,%3;")
		.arg(device_ssid)
		.arg(condi.nowCount)
		.arg(condi.showCount);
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		int i = 0;
		TaskResultForDevice stru;
		stru.strTaskName = query.value(i++).toString();
		stru.strPatrolTime = query.value(i++).toString().replace("T"," ");
		stru.strPatrolResult = query.value(i++).toString();
		data.taskResult.append(stru);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getHistoryDeviceAlarmDataDB(TaskSearchCondition condi, QList<DeviceAlarmHistory> &data)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr;
	if (condi.startTime.isEmpty())
	{
		sqlStr = QString("SELECT task.task_ssid,devices.device_ssid,device_name.device_name,inpect_result.inspect_time,inpect_result.inspect_result FROM task,inpect_result,devices,device_name WHERE inpect_result.task_ssid=task.task_ssid AND inpect_result.device_ssid=devices.device_ssid AND device_name.device_detail_id=devices.device_name_id AND inpect_result.inspect_status='0' AND inpect_result.is_dealed='0'ORDER BY inpect_result.inspect_time DESC LIMIT %1,%2;")
			.arg(condi.nowCount)
			.arg(condi.showCount);
	}
	else
	{
		sqlStr = QString("SELECT task.task_ssid,devices.device_ssid,device_name.device_name,inpect_result.inspect_time,inpect_result.inspect_result FROM task,inpect_result,devices,device_name WHERE inpect_result.task_ssid=task.task_ssid AND inpect_result.device_ssid=devices.device_ssid AND device_name.device_detail_id=devices.device_name_id AND inpect_result.inspect_status='0' AND inpect_result.is_dealed='0' AND inpect_result.inspect_time>'%1' AND inpect_result.inspect_time<'%2' ORDER BY inpect_result.inspect_time DESC LIMIT %3,%4;")
			.arg(condi.startTime)
			.arg(condi.stopTime)
			.arg(condi.nowCount)
			.arg(condi.showCount);
	}
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		int i = 0;
		DeviceAlarmHistory stru;
		stru.strTaskSsid = query.value(i++).toString();
		stru.strDeviceSsid = query.value(i++).toString();
		stru.strDeviceName = query.value(i++).toString();
		stru.strInspectTime = query.value(i++).toString().replace("T"," ");
		stru.strInspectResult = query.value(i++).toString();
		data.append(stru);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getAlarmDeviceCountDB(int &iCount)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("SELECT COUNT(*) FROM task,inpect_result,devices,device_name WHERE inpect_result.task_ssid=task.task_ssid AND inpect_result.device_ssid=devices.device_ssid AND device_name.device_detail_id=devices.device_name_id AND inpect_result.inspect_status='0' AND inpect_result.is_dealed='0';");
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		iCount = query.value(0).toInt();
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::getReturnInsertDeviceDB(QString deviceSsid, InsertDeviceReturn & rdev)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("SELECT devices.device_ssid,devices.device_area_name,device_alternate_name.device_alternate_name FROM devices,device_alternate_name WHERE devices.device_alternate_name_id=device_alternate_name.device_alternate_name_id AND devices.device_ssid='%1';")
		.arg(deviceSsid);
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		rdev.deviceSsid = query.value(0).toString();
		rdev.areaName = query.value(1).toString();
		rdev.deviceName = query.value(2).toString();
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::updateWalkThresholdToStationCfg(int start_value, int terminus_value)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("UPDATE station_cfg SET station_cfg.walk_start_value='%1',station_cfg.walk_terminus_value='%2';")
		.arg(start_value).arg(terminus_value);
	bRet = querySqlString(sqlStr, query);
	return bRet;
}


bool LibDLHangRailRobotDBOperation::updateDeviceNameIdForDevices(QList<updateDeviceNameId> data)
{
	QSqlQuery query;
	bool bRet;
	for (int i = 0; i < data.size(); i++)
	{
		QString sqlStr = QString("UPDATE devices SET device_name_id='%1' WHERE device_ssid='%2';")
			.arg(data[i].deviceNameId)
			.arg(data[i].deviceSSid);
		bRet = querySqlString(sqlStr, query);
	}
	return bRet;
}

bool LibDLHangRailRobotDBOperation::selectDeviceNameIdWithDeviceSSid(QString deviceSSid, QString &deviceNameId)
{
	QSqlQuery query;
	bool bRet;
	QString sqlStr = QString("SELECT device_name_id FROM devices WHERE device_ssid='%1';")
		.arg(deviceSSid);
	bRet = querySqlString(sqlStr, query);
	while (query.next())
	{
		deviceNameId = query.value(0).toString();
	}
	return bRet;
}