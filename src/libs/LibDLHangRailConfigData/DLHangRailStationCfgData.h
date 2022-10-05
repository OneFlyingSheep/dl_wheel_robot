#ifndef __DL_HANG_ROBOT_STATION_DATA_H__
#define __DL_HANG_ROBOT_STATION_DATA_H__

#include "common/DLHangRailRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <QList>
class DLHangRailStationCfgData : public Singleton<DLHangRailStationCfgData>
{
public:
    DLHangRailStationCfgData();
    ~DLHangRailStationCfgData();

public:
    void saveStationCfg();
    bool loadStationCfg();
    void updateStationCfg(stationCfg cfg);

    stationCfg getStationCfg();

    void setStationId(int id);
    void setStationName(QString name);

    void setVisibleCameraIpAddr(QString addr);
    void setVisibleCameraPort(int port);
    void setVisibleUsername(QString name);
    void setVisiblePassword(QString passwd);
    void setInfraredIpAddr(QString addr);
    void setInfraredPort(int port);
    void setRobotSensorIpAddr(QString addr);
    void setRobotSensorBoxCtrlPort(int port);
    void setRobotSensorBoxQueryPort(int port);
    void setRobotWalkBoxIpAddr(QString addr);
    void setRobotWalkBoxCtrlPort(int port);
    void setRobotWalkBoxQueryPort(int port);
    void setCalibFilePath(QString path);
    void setReportFilePath(QString path);
    void setAll(stationCfg cfg);

	QList<QString> getStationCfgStationName();
	void loadStationCfgStationName();

	void loadRestorationValue();

	std::map<int, RestorationValue> getRestorationValue();

	RestorationValue getRestorationValueStruct();

private:
    void insertStationCfg(stationCfg cfg);
    stationCfg m_stationCfg;

	QList<QString> m_stationName;
	std::map<int, RestorationValue> m_restorationValue;
};

#define ROBOTSTATIONCFG DLHangRailStationCfgData::GetSingleton()

#define ROBOTSTATIONCFG_SAVE ROBOTSTATIONCFG.saveStationCfg()
#define ROBOTSTATIONCFG_REALOAD ROBOTSTATIONCFG.loadStationCfg()

#define ROBOTSTATIONCFG_STATUS ROBOTSTATIONCFG.getStationCfg()

#define ROBOTSTATIONCFG_SETALL(val) ROBOTSTATIONCFG.setAll(val);

#define ROBOTSTATIONCFG_SET_STATION_ID(val) ROBOTSTATIONCFG.setStationId(val)
#define ROBOTSTATIONCFG_SET_STATION_NAME(val) ROBOTSTATIONCFG.setStationName(val)
#define ROBOTSTATIONCFG_SET_VISIBLE_IP_ADDR(val) ROBOTSTATIONCFG.setVisibleCameraIpAddr(val)
#define ROBOTSTATIONCFG_SET_VISIBLE_PORT(val) ROBOTSTATIONCFG.setVisibleCameraPort(val)
#define ROBOTSTATIONCFG_SET_VISIBLE_USERNAME(val) ROBOTSTATIONCFG.setVisibleUsername(val)
#define ROBOTSTATIONCFG_SET_VISIBLE_PASSWORD(val) ROBOTSTATIONCFG.setVisiblePassword(val)
#define ROBOTSTATIONCFG_SET_INFRARED_IP_ADDR(val) ROBOTSTATIONCFG.setInfraredIpAddr(val)
#define ROBOTSTATIONCFG_SET_INFRARED_PORT(val) ROBOTSTATIONCFG.setInfraredPort(val)
#define ROBOTSTATIONCFG_SET_ROBOT_SENSOR_IP_ADDR(val) ROBOTSTATIONCFG.setRobotSensorIpAddr(val)
#define ROBOTSTATIONCFG_SET_ROBOT_SENSOR_CTRL_PORT(val) ROBOTSTATIONCFG.setRobotSensorBoxCtrlPort(val)
#define ROBOTSTATIONCFG_SET_ROBOT_SENSOR_QUERY_PORT(val) ROBOTSTATIONCFG.setRobotSensorBoxQueryPort(val)
#define ROBOTSTATIONCFG_SET_ROBOT_WALK_IP_ADDR(val) ROBOTSTATIONCFG.setRobotWalkBoxIpAddr(val)
#define ROBOTSTATIONCFG_SET_ROBOT_WALK_CTRL_PORT(val) ROBOTSTATIONCFG.setRobotWalkBoxCtrlPort(val)
#define ROBOTSTATIONCFG_SET_ROBOT_WALK_QUERY_PORT(val) ROBOTSTATIONCFG.setRobotWalkBoxQueryPort(val)
#define ROBOTSTATIONCFG_SET_CALIB_FILE_PATH(val) ROBOTSTATIONCFG.setCalibFilePath(val)
#define ROBOTSTATIONCFG_SET_REPORT_FILE_PATH(val) ROBOTSTATIONCFG.setReportFilePath(val)

#endif


