#pragma once
#if _MSC_VER >= 1600  
#pragma execution_character_set("utf-8")  
#endif 
#include <QWidget>
#include <QList>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "ChooseExcelClass.hpp"
#include "qxlsx.h"

class CreateExcelForQt5 :public ChooseExcelVersion
{
public:
	CreateExcelForQt5() :
		m_row(1) {};
	~CreateExcelForQt5() {};
	bool createReportForTask(QString task_uuid, QString SavePath);

	QString getMergeCellsQString(int x1, int y1, int x2, int y2);

	void SetReportHeadMessage(QString _taskUUid, QXlsx::Document &book);

	void SetAlarmPointMessage(QString task_uuid, QXlsx::Document &book);

	void SetUnusualPointMessage(QString task_uuid, QXlsx::Document &book);

	void SetNormalPointMessage(QString task_uuid, QXlsx::Document &book);

	QString DateCalculate(QString StartTime, QString StopTime);

    bool import_excel_report(QList<QStringList> data, QString filePath);

    bool export_excel_result(QString taskUUid, QString strPath);
private:
	int m_row;
};
