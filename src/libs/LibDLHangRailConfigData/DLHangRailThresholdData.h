#ifndef __DL_HANG_ROBOT_THRESHOLD_DATA_H__
#define __DL_HANG_ROBOT_THRESHOLD_DATA_H__

#include "common/DLHangRailRobotGlobalDef.hpp"
#include "LibDLHangRailRobotDBOperation/LibDLHangRailRobotDBOperation.h"
#include <boost/serialization/singleton.hpp>
#include <QString>
#include <QVector>
#include <map>
#include <QList>
class DLHangRailThresholdData : public Singleton<DLHangRailThresholdData>
{
public:
	DLHangRailThresholdData();
	~DLHangRailThresholdData();

	std::map<int, thresholdEnvi> getThresholdEnvironmentList();
	std::map<int, thresholdPatrol> getThresholdPatrolList();
	std::map<int, thresholdEnvi> getThresholdEnvironmentID();
	std::map<int, thresholdPatrol> getThresholdPatrolID();
	QList<QString> getEnvironmentTypeName();
	void loadThresholdEnvironment();
	void loadThresholdPatrol();
	void loadThresholdEnvironmentID();
	void loadThresholdPatrolID();
	void loadEnvironmentType();
	void refresh();
private:
	std::map<int, thresholdEnvi> m_thresholdEnviName;
	std::map<int, thresholdPatrol> m_thresholdPatrolName;
	std::map<int, thresholdEnvi> m_thresholdEnviID;
	std::map<int, thresholdPatrol> m_thresholdPatrolID;
	QList<QString> m_enviTypeName;
};

#define ROBOT_THRESHOLD DLHangRailThresholdData::GetSingleton()

#endif