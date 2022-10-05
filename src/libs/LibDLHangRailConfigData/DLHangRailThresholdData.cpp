#include "DLHangRailThresholdData.h"

DLHangRailThresholdData::DLHangRailThresholdData()
{
	refresh();
}


DLHangRailThresholdData::~DLHangRailThresholdData()
{
}

void DLHangRailThresholdData::refresh()
{
	loadThresholdEnvironment();
	loadThresholdPatrol();
	loadThresholdEnvironmentID();
	loadThresholdPatrolID();
	loadEnvironmentType();
}

std::map<int, thresholdEnvi> DLHangRailThresholdData::getThresholdEnvironmentList()
{
	return m_thresholdEnviName;
}

std::map<int, thresholdPatrol> DLHangRailThresholdData::getThresholdPatrolList()
{
	return m_thresholdPatrolName;
}

std::map<int, thresholdEnvi> DLHangRailThresholdData::getThresholdEnvironmentID()
{
	return m_thresholdEnviID;
}

std::map<int, thresholdPatrol> DLHangRailThresholdData::getThresholdPatrolID()
{
	return m_thresholdPatrolID;
}
QList<QString> DLHangRailThresholdData::getEnvironmentTypeName()
{
	return m_enviTypeName;
}

void DLHangRailThresholdData::loadThresholdEnvironment()
{
	m_thresholdEnviName.clear();
	ROBOTDB_DB.getThresholdEnvironment(m_thresholdEnviName);
}

void DLHangRailThresholdData::loadThresholdPatrol()
{
	m_thresholdPatrolName.clear();
	ROBOTDB_DB.getThresholdPatrol(m_thresholdPatrolName);
}

void DLHangRailThresholdData::loadThresholdEnvironmentID()
{
	m_thresholdEnviID.clear();
	ROBOTDB_DB.getThresholdEnvironmentID(m_thresholdEnviID);
}

void DLHangRailThresholdData::loadThresholdPatrolID()
{
	m_thresholdPatrolID.clear();
	ROBOTDB_DB.getThresholdPatrolID(m_thresholdPatrolID);
}

void DLHangRailThresholdData::loadEnvironmentType()
{
	m_enviTypeName.clear();
	ROBOTDB_DB.getEnvironmentTypeData(m_enviTypeName);
}
