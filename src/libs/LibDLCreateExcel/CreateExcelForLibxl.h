#pragma once
#if _MSC_VER >= 1600  
#pragma execution_character_set("utf-8")  
#endif 
#include <QWidget>
#include <QList>
#include <QDebug>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "libxl.h"
#include "ChooseExcelClass.hpp"

class CreateExcelForLibxl :public ChooseExcelVersion
{
public:
	CreateExcelForLibxl();
	~CreateExcelForLibxl();

	bool createReportForTask(QString task_uuid, QString SavePath);

	void SetReportHeadMessage(QString _taskUUid, libxl::Book *_book, libxl::Sheet *_sheet);

	void SetAlarmPointMessage(QString task_uuid, libxl::Book *book, libxl::Sheet *sheet);

	void SetUnusualPointMessage(QString task_uuid, libxl::Book *_book, libxl::Sheet *_sheet);

	void SetNormalPointMessage(QString task_uuid, libxl::Book *_book, libxl::Sheet *_sheet);

	QString DateCalculate(QString StartTime, QString StopTime);
private:
	int m_row;
};
