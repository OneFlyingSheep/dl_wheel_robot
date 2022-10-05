#pragma once
#if _MSC_VER >= 1600  
#pragma execution_character_set("utf-8")  
#endif 
#include "ChooseExcelClass.hpp"
#include "CreateExcelForLibxl.h"
#include "CreateExcelForQt5.h"

namespace Excel
{
	enum ChooseExcelEnum
	{
		USELIBXL = 0,
		USEQT5,
	};
}

namespace Excel
{
	bool createExcelReport(ChooseExcelEnum choose,QString task_uuid, QString task_name)
	{
		bool bRet = false;
		if (choose == USELIBXL)
		{
			CreateExcelForLibxl ls;
			bRet = ls.createReportForTask(task_uuid, task_name);
		}
		if (choose == USEQT5)
		{
			CreateExcelForQt5 ls;
			bRet = ls.createReportForTask(task_uuid, task_name);
		}
		return bRet;
	}
}