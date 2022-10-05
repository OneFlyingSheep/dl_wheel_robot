#pragma once
#if _MSC_VER >= 1600  
#pragma execution_character_set("utf-8")  
#endif 
class ChooseExcelVersion
{
public:
	ChooseExcelVersion() {}
	~ChooseExcelVersion() {}

	virtual bool createReportForTask(QString task_uuid, QString SavePath) = 0;
private:

};