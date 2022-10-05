#include "DLHangRailStationCfgData.h"
#include "LibDLHangRailRobotDBOperation/LibDLHangRailRobotDBOperation.h"
#include <QVariant>
DLHangRailStationCfgData::DLHangRailStationCfgData()
{
    loadStationCfg();
	loadStationCfgStationName();
	loadRestorationValue();
}

DLHangRailStationCfgData::~DLHangRailStationCfgData()
{
    saveStationCfg();
}

void DLHangRailStationCfgData::updateStationCfg(stationCfg cfg)
{
    //bool bRet;
    //QSqlQuery qry;

    //qry.prepare("UPDATE station_cfg SET \
    //                    station_ssid = :station_ssid, \
    //                    station_name = :station_name, \
    //                    visibleCameraIpAddr = :visibleCameraIpAddr, \
    //                    visibleCameraPort = :visibleCameraPort, \
    //                    visibleUsername = :visibleUsername, \
    //                    visiblePassword = :visiblePassword, \
    //                    infraredIpAddr = :infraredIpAddr, \
    //                    infraredPort = :infraredPort, \
    //                    robotSensorIpAddr = :robotSensorIpAddr, \
    //                    robotSensorBoxCtrlPort = :robotSensorBoxCtrlPort, \
    //                    robotSensorBoxQueryPort = :robotSensorBoxQueryPort, \
    //                    robotWalkBoxIpAddr = :robotWalkBoxIpAddr, \
    //                    robotWalkBoxCtrlPort = :robotWalkBoxCtrlPort, \
    //                    robotWalkBoxQueryPort = :robotWalkBoxQueryPort, \
    //                    calibFilePath = :calibFilePath, \
    //                    reportFilePath = :reportFilePath, \
    //                    ");

    //qry.bindValue(":stationSsid", QString::fromStdString(cfg.stationSsid));
    //qry.bindValue(":stationName", QString::fromStdString(cfg.stationName));
    //qry.bindValue(":visibleCameraIpAddr", QString::fromStdString(cfg.visibleCameraIpAddr));
    //qry.bindValue(":visibleCameraPort", cfg.visibleCameraPort);
    //qry.bindValue(":visibleUsername", QString::fromStdString(cfg.visibleUsername));
    //qry.bindValue(":visiblePassword", QString::fromStdString(cfg.visiblePassword));
    //qry.bindValue(":infraredIpAddr", QString::fromStdString(cfg.infraredIpAddr));
    //qry.bindValue(":infraredPort", cfg.infraredPort);
    //qry.bindValue(":robotSensorIpAddr", QString::fromStdString(cfg.robotSensorIpAddr));
    //qry.bindValue(":robotSensorBoxCtrlPort", cfg.robotSensorBoxCtrlPort);
    //qry.bindValue(":robotSensorBoxQueryPort", cfg.robotSensorBoxQueryPort);
    //qry.bindValue(":robotWalkBoxIpAddr", QString::fromStdString(cfg.robotWalkBoxIpAddr));
    //qry.bindValue(":robotWalkBoxCtrlPort", cfg.robotWalkBoxCtrlPort);
    //qry.bindValue(":robotWalkBoxQueryPort", cfg.robotWalkBoxQueryPort);
    //qry.bindValue(":calibFilePath", QString::fromStdString(cfg.calibFilePath));
    //qry.bindValue(":reportFilePath", QString::fromStdString(cfg.reportFilePath));

    //bRet = ROBOTDB_EXEC_SQL_STRING(qry);

    //if (!bRet)
    //{
    //    ROS_INFO("updateStationCfg error");
    //}
}

void DLHangRailStationCfgData::saveStationCfg()
{
    bool bRet = false;
    QSqlQuery qry;
    QString qryStr = "SELECT * FROM station_cfg";

//    bRet = ROBOTDB_QUERY_SQL_STRING(qryStr, qry);

    if (!bRet)
    {
        ROS_ERROR("query station failed");
        return;
    }
    else if (qry.size() == 0)
    {
        insertStationCfg(m_stationCfg);
    }
    else
    {
        updateStationCfg(m_stationCfg);
    }

//    bRet = ROBOTDB_EXEC_SQL_STRING(qry);

    if (!bRet)
    {
        ROS_INFO("save station error");
    }
}

bool DLHangRailStationCfgData::loadStationCfg()
{
    return ROBOTDB_DB.getStationCfg(m_stationCfg);
}

void DLHangRailStationCfgData::insertStationCfg(stationCfg cfg)
{
    //bool bRet;
    //QSqlQuery qry;

    //qry.prepare("INSERT INTO station_cfg VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");

    //qry.addBindValue(QString::fromStdString(cfg.stationSsid));
    //qry.addBindValue(QString::fromStdString(cfg.stationName));
    //qry.addBindValue(QString::fromStdString(cfg.visibleCameraIpAddr));
    //qry.addBindValue(cfg.visibleCameraPort);
    //qry.addBindValue(QString::fromStdString(cfg.visibleUsername));
    //qry.addBindValue(QString::fromStdString(cfg.visiblePassword));
    //qry.addBindValue(QString::fromStdString(cfg.infraredIpAddr));
    //qry.addBindValue(cfg.infraredPort);
    //qry.addBindValue(QString::fromStdString(cfg.robotSensorIpAddr));
    //qry.addBindValue(cfg.robotSensorBoxCtrlPort);
    //qry.addBindValue(cfg.robotSensorBoxQueryPort);
    //qry.addBindValue(QString::fromStdString(cfg.robotWalkBoxIpAddr));
    //qry.addBindValue(cfg.robotWalkBoxCtrlPort);
    //qry.addBindValue(cfg.robotWalkBoxQueryPort);
    //qry.addBindValue(QString::fromStdString(cfg.calibFilePath));
    //qry.addBindValue(QString::fromStdString(cfg.reportFilePath));

    //bRet = ROBOTDB_EXEC_SQL_STRING(qry);

    //if (!bRet)
    //{
    //    ROS_INFO("insertStationCfg error");
    //}
}

stationCfg DLHangRailStationCfgData::getStationCfg()
{
    return m_stationCfg;
}

void DLHangRailStationCfgData::setStationId(int id)
{
    m_stationCfg.station_ssid = id;
}

void DLHangRailStationCfgData::setStationName(QString name)
{
    m_stationCfg.station_name = name;
}

void DLHangRailStationCfgData::setVisibleCameraIpAddr(QString addr)
{
//    m_stationCfg.visibleCameraIpAddr = addr;
}

void DLHangRailStationCfgData::setVisibleCameraPort(int port)
{
//    m_stationCfg.visibleCameraPort = port;
}

void DLHangRailStationCfgData::setVisibleUsername(QString name)
{
//    m_stationCfg.visibleUsername = name;
}

void DLHangRailStationCfgData::setVisiblePassword(QString passwd)
{
//    m_stationCfg.visiblePassword = passwd;
}

void DLHangRailStationCfgData::setInfraredIpAddr(QString addr)
{
 //   m_stationCfg.infraredIpAddr = addr;
}

void DLHangRailStationCfgData::setInfraredPort(int port)
{
//    m_stationCfg.infraredPort = port;
}

void DLHangRailStationCfgData::setRobotSensorIpAddr(QString addr)
{
 //   m_stationCfg.robotSensorIpAddr = addr;
}

void DLHangRailStationCfgData::setRobotSensorBoxCtrlPort(int port)
{
 //   m_stationCfg.robotSensorBoxCtrlPort = port;
}

void DLHangRailStationCfgData::setRobotSensorBoxQueryPort(int port)
{
//    m_stationCfg.robotSensorBoxQueryPort = port;
}

void DLHangRailStationCfgData::setRobotWalkBoxIpAddr(QString addr)
{
 //   m_stationCfg.robotWalkBoxIpAddr = addr;
}

void DLHangRailStationCfgData::setRobotWalkBoxCtrlPort(int port)
{
//    m_stationCfg.robotWalkBoxCtrlPort = port;
}

void DLHangRailStationCfgData::setRobotWalkBoxQueryPort(int port)
{
//    m_stationCfg.robotWalkBoxQueryPort = port;
}

void DLHangRailStationCfgData::setCalibFilePath(QString path)
{
//    m_stationCfg.calibFilePath = path;
}

void DLHangRailStationCfgData::setReportFilePath(QString path)
{
 //   m_stationCfg.reportFilePath = path;
}

void DLHangRailStationCfgData::setAll(stationCfg cfg)
{
    m_stationCfg = cfg;
}

QList<QString> DLHangRailStationCfgData::getStationCfgStationName()
{
	return m_stationName;
}

void DLHangRailStationCfgData::loadStationCfgStationName()
{
	m_stationName.clear();
	ROBOTDB_DB.getStationCfgStationName(m_stationName);
}

void DLHangRailStationCfgData::loadRestorationValue()
{
	m_restorationValue.clear();
	ROBOTDB_DB.getRestorationValueForDB(m_restorationValue);
}

std::map<int, RestorationValue> DLHangRailStationCfgData::getRestorationValue()
{
	return m_restorationValue;
}

RestorationValue DLHangRailStationCfgData::getRestorationValueStruct()
{
	RestorationValue rest;
	int key = ROBOTSTATIONCFG.getStationCfg().station_ssid.toInt();
	if (m_restorationValue.size() > 0)
	{
		rest.rest_id = m_restorationValue.at(key).rest_id;
		rest.rest_station_id = m_restorationValue.at(key).rest_station_id;
		rest.rest_move_location = m_restorationValue.at(key).rest_move_location;
		rest.rest_move_speed = m_restorationValue.at(key).rest_move_speed;
		rest.rest_lift_location = m_restorationValue.at(key).rest_lift_location;
		rest.rest_bady_rotate = m_restorationValue.at(key).rest_bady_rotate;
		rest.rest_camera_rotate = m_restorationValue.at(key).rest_camera_rotate;
		rest.rest_camera_zoom = m_restorationValue.at(key).rest_camera_zoom;
		rest.rest_camera_focal = m_restorationValue.at(key).rest_camera_focal;
	}
	return rest;
}