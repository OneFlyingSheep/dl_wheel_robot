#pragma once

#include "qword.h"
#include "common/Singleton.hpp"
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"

class PatrolTaskReportCreate
{
public:

	PatrolTaskReportCreate();
	~PatrolTaskReportCreate();

public:

	bool CreateNewWordWithPath(QString m_path, QString task_ssid);
	void CreateWordFromTaskEnd(QString m_SavePath, QString m_TaskUUid);
	void WordReportHeadMessage(WheelReportHeadMessage m_ReportHeadMessage);
	void AlarmPointReportWrite(int row, int column, QList<AlarmUnusualNormalPoint> m_AlarmPoint);
	void UnusualPointReportWrite(int row, int column, QList<AlarmUnusualNormalPoint> m_UnusualPoint);
	void NormalPointReportWrite(int row, int column, QList<AlarmUnusualNormalPoint> m_NormalPoint);
	void PatrolLineMapReportWrite(QString m_PatrolMapPath);
	void CloseReportRedact();
	QString DateCalculate(QString StartTime, QString StopTime);
	
private:
	QWord * word;
	int AtPresentTable;
};

