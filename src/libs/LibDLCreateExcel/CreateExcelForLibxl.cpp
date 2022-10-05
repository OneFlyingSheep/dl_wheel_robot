#include "CreateExcelForLibxl.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include <QDir>
#include "windows.h"
#include <shellapi.h>
#include <QDateTime>

using namespace libxl;

CreateExcelForLibxl::CreateExcelForLibxl() :
	m_row(1)
{
}

CreateExcelForLibxl::~CreateExcelForLibxl()
{
}

bool CreateExcelForLibxl::createReportForTask(QString task_uuid, QString SavePath)
{
	QString fileName = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/report";
	QDir dir;
	if (!dir.exists(fileName))
	{
		dir.mkpath(fileName);
	}
	QString path = QString("%1/%2.xls").arg(fileName).arg(SavePath);

	Book* book = xlCreateBook();
	if (book)
	{
		Sheet* sheet = book->addSheet("Sheet1");
		if (sheet)
		{
			sheet->setCol(0, 0, 15);
			sheet->setCol(1, 1, 20);
			sheet->setCol(2, 2, 40);
			sheet->setCol(3, 4, 15);
			sheet->setCol(5, 5, 30);
			sheet->setCol(6, 7, 13);

			//报告头
			SetReportHeadMessage(task_uuid, book, sheet);
			//告警点位
			SetAlarmPointMessage(task_uuid, book, sheet);
			//异常点位
			SetUnusualPointMessage(task_uuid, book, sheet);
			//正常点位
			SetNormalPointMessage(task_uuid, book, sheet);
		}
	}

	if (book->save(path.toLocal8Bit().constData()))
	{
		//	::ShellExecute(NULL, "open", path.toStdString().c_str(), NULL, NULL, SW_SHOW);
	}
	else
	{
		//	std::cout << book->errorMessage() << std::endl;
		book->release();
		return false;
	}
	book->release();
	return true;
}

void CreateExcelForLibxl::SetReportHeadMessage(QString _taskUUid, libxl::Book *_book, libxl::Sheet *_sheet)
{
	WheelReportHeadMessage m_ReportHeadMessage;
	WHEEL_ROBOT_DB.getWheelReportHeadMessageDB(_taskUUid, m_ReportHeadMessage);
	QString StartTime = m_ReportHeadMessage.TaskStartTime.replace("T", " ");
	QString StopTime = m_ReportHeadMessage.TaskStopTime.replace("T", " ");
	QString OverallTime = DateCalculate(StartTime, StopTime);
	QString TaskRunTime = QString("开始时间:%1 结束时间:%2 总时常:%3").arg(StartTime).arg(StopTime).arg(OverallTime);
	QString TaskParrolPoint = QString("本次任务共巡检%1点位，正常点位%2个，告警点位%3个，识别异常点位%4个")
		.arg(m_ReportHeadMessage.PatrolPointNum.AllPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.NormalPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.AlarmPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.UnusualPatrolPointNum);
	QString TaskEnvironment = QString("环境温度:%1 环境湿度:%2 环境PM2.5%3 环境风向:%4 环境风速%5")
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_temperature)
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_humidity)
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_pm_2_5)
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_wind_direct)
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_wind_speed);

	_sheet->setMerge(m_row, m_row, 0, 7);
	_sheet->writeStr(m_row, 0, QString("巡检报告").toLocal8Bit().data());
	Format *_f = _book->addFormat();
	Font *_ft = _book->addFont();
	_ft->setSize(24);
	_f->setFont(_ft);
	_f->setAlignH(AlignH::ALIGNH_CENTER);
	_sheet->setCellFormat(m_row, 0, _f);
	m_row++;

	_sheet->setMerge(m_row, m_row, 1, 7);
	_sheet->writeStr(m_row, 0, QString("任务名称").toLocal8Bit().data());
	_sheet->writeStr(m_row, 1, m_ReportHeadMessage.TaskName.toLocal8Bit().data());
	m_row++;

	_sheet->setMerge(m_row, m_row, 1, 7);
	_sheet->writeStr(m_row, 0, QString("任务时间").toLocal8Bit().data());
	_sheet->writeStr(m_row, 1, TaskRunTime.toLocal8Bit().data());
	m_row++;

	_sheet->setMerge(m_row, m_row, 1, 7);
	_sheet->writeStr(m_row, 0, QString("任务状态").toLocal8Bit().data());
	_sheet->writeStr(m_row, 1, m_ReportHeadMessage.TaskStatus.toLocal8Bit().data());
	m_row++;

	_sheet->setMerge(m_row, m_row, 1, 7);
	_sheet->writeStr(m_row, 0, QString("巡检点").toLocal8Bit().data());
	_sheet->writeStr(m_row, 1, TaskParrolPoint.toLocal8Bit().data());
	m_row++;

	_sheet->setMerge(m_row, m_row, 1, 7);
	_sheet->writeStr(m_row, 0, QString("环境信息").toLocal8Bit().data());
	_sheet->writeStr(m_row, 1, TaskEnvironment.toLocal8Bit().data());
	m_row++;
}

void CreateExcelForLibxl::SetAlarmPointMessage(QString task_uuid, libxl::Book *_book, libxl::Sheet *_sheet)
{
	_sheet->setMerge(m_row, m_row, 0, 7);
	_sheet->writeStr(m_row, 0, QString("告警点位").toLocal8Bit().data());
	Format *_f = _book->addFormat();
	Font *_ft = _book->addFont();
	_ft->setSize(16);
	_f->setFont(_ft);
	_f->setAlignH(AlignH::ALIGNH_CENTER);
	_sheet->setCellFormat(m_row, 0, _f);
	m_row++;

	_sheet->writeStr(m_row, 0, QString("序号").toLocal8Bit().data());
	_sheet->writeStr(m_row, 1, QString("识别类型").toLocal8Bit().data());
	_sheet->writeStr(m_row, 2, QString("点位名称").toLocal8Bit().data());
	_sheet->writeStr(m_row, 3, QString("识别结果").toLocal8Bit().data());
	_sheet->writeStr(m_row, 4, QString("告警等级").toLocal8Bit().data());
	_sheet->writeStr(m_row, 5, QString("识别时间").toLocal8Bit().data());
	_sheet->writeStr(m_row, 6, QString("采集信息").toLocal8Bit().data());
	_sheet->writeStr(m_row, 7, QString("采集信息").toLocal8Bit().data());
	m_row++;

	QList<AlarmUnusualNormalPoint> m_AlarmPoint;
	WHEEL_ROBOT_DB.getWheelAlarmPointDB(task_uuid, m_AlarmPoint);

	for (int i = 0; i < m_AlarmPoint.size(); i++)
	{
		_sheet->writeNum(m_row, 0, i + 1);
		_sheet->writeStr(m_row, 1, m_AlarmPoint[i].DiscernType.toLocal8Bit().data());
		_sheet->writeStr(m_row, 2, m_AlarmPoint[i].PointName.toLocal8Bit().data());
		_sheet->writeStr(m_row, 3, m_AlarmPoint[i].DiscernResult.toLocal8Bit().data());
		_sheet->writeStr(m_row, 4, m_AlarmPoint[i].AlarmGradeOrAuditResult.toLocal8Bit().data());
		_sheet->writeStr(m_row, 5, m_AlarmPoint[i].DiscernTime.toLocal8Bit().data());
		if (m_AlarmPoint[i].RecognitionType == 1 || m_AlarmPoint[i].RecognitionType == 2 || m_AlarmPoint[i].RecognitionType == 3)
		{
			if (!m_AlarmPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_AlarmPoint[i].CollectMessage.VisibleLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 6, index, 96, 54, 0.5);
			}
		}
		else if (m_AlarmPoint[i].RecognitionType == 4 || m_AlarmPoint[i].RecognitionType == 6)
		{
			if (!m_AlarmPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_AlarmPoint[i].CollectMessage.VisibleLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 6, index, 96, 54, 0.5);
			}
			if (!m_AlarmPoint[i].CollectMessage.InfraredLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_AlarmPoint[i].CollectMessage.InfraredLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 7, index, 96, 54, 0.5);
			}
		}
		else {}
		m_row++;
	}

}

void CreateExcelForLibxl::SetUnusualPointMessage(QString task_uuid, libxl::Book *_book, libxl::Sheet *_sheet)
{
	_sheet->setMerge(m_row, m_row, 0, 7);
	_sheet->writeStr(m_row, 0, QString("异常点位").toLocal8Bit().data());
	Format *_f = _book->addFormat();
	Font *_ft = _book->addFont();
	_ft->setSize(16);
	_f->setFont(_ft);
	_f->setAlignH(AlignH::ALIGNH_CENTER);
	_sheet->setCellFormat(m_row, 0, _f);
	m_row++;

	_sheet->writeStr(m_row, 0, QString("序号").toLocal8Bit().data());
	_sheet->writeStr(m_row, 1, QString("识别类型").toLocal8Bit().data());
	_sheet->writeStr(m_row, 2, QString("点位名称").toLocal8Bit().data());
	_sheet->writeStr(m_row, 3, QString("识别结果").toLocal8Bit().data());
	_sheet->writeStr(m_row, 4, QString("审核结果").toLocal8Bit().data());
	_sheet->writeStr(m_row, 5, QString("识别时间").toLocal8Bit().data());
	_sheet->writeStr(m_row, 6, QString("采集信息").toLocal8Bit().data());
	_sheet->writeStr(m_row, 7, QString("采集信息").toLocal8Bit().data());
	m_row++;

	QList<AlarmUnusualNormalPoint> m_UnusualPoint;
	WHEEL_ROBOT_DB.getUnusualPointDB(task_uuid, m_UnusualPoint);

	for (int i = 0; i < m_UnusualPoint.size(); i++)
	{
		_sheet->writeNum(m_row, 0, i + 1);
		_sheet->writeStr(m_row, 1, m_UnusualPoint[i].DiscernType.toLocal8Bit().data());
		_sheet->writeStr(m_row, 2, m_UnusualPoint[i].PointName.toLocal8Bit().data());
		_sheet->writeStr(m_row, 3, m_UnusualPoint[i].DiscernResult.toLocal8Bit().data());
		_sheet->writeStr(m_row, 4, m_UnusualPoint[i].AlarmGradeOrAuditResult.toLocal8Bit().data());
		_sheet->writeStr(m_row, 5, m_UnusualPoint[i].DiscernTime.toLocal8Bit().data());
		if (m_UnusualPoint[i].RecognitionType == 1 || m_UnusualPoint[i].RecognitionType == 2 || m_UnusualPoint[i].RecognitionType == 3)
		{
			if (!m_UnusualPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_UnusualPoint[i].CollectMessage.VisibleLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 6, index, 96, 54, 0.5);
			}
		}
		else if (m_UnusualPoint[i].RecognitionType == 4 || m_UnusualPoint[i].RecognitionType == 6)
		{
			if (!m_UnusualPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_UnusualPoint[i].CollectMessage.VisibleLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 6, index, 96, 54, 0.5);
			}
			if (!m_UnusualPoint[i].CollectMessage.InfraredLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_UnusualPoint[i].CollectMessage.InfraredLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 7, index, 96, 54, 0.5);
			}
		}
		else {}
		m_row++;
	}
}

void CreateExcelForLibxl::SetNormalPointMessage(QString task_uuid, libxl::Book *_book, libxl::Sheet *_sheet)
{
	_sheet->setMerge(m_row, m_row, 0, 7);
	_sheet->writeStr(m_row, 0, QString("正常点位").toLocal8Bit().data());
	Format *_f = _book->addFormat();
	Font *_ft = _book->addFont();
	_ft->setSize(16);
	_f->setFont(_ft);
	_f->setAlignH(AlignH::ALIGNH_CENTER);
	_sheet->setCellFormat(m_row, 0, _f);
	m_row++;

	_sheet->writeStr(m_row, 0, QString("序号").toLocal8Bit().data());
	_sheet->writeStr(m_row, 1, QString("识别类型").toLocal8Bit().data());
	_sheet->writeStr(m_row, 2, QString("点位名称").toLocal8Bit().data());
	_sheet->writeStr(m_row, 3, QString("识别结果").toLocal8Bit().data());
	_sheet->writeStr(m_row, 4, QString("告警等级").toLocal8Bit().data());
	_sheet->writeStr(m_row, 5, QString("识别时间").toLocal8Bit().data());
	_sheet->writeStr(m_row, 6, QString("采集信息").toLocal8Bit().data());
	_sheet->writeStr(m_row, 7, QString("采集信息").toLocal8Bit().data());
	m_row++;

	QList<AlarmUnusualNormalPoint> m_NormalPoint;
	WHEEL_ROBOT_DB.getWheelNormalPointDB(task_uuid, m_NormalPoint);

	for (int i = 0; i < m_NormalPoint.size(); i++)
	{
		_sheet->writeNum(m_row, 0, i + 1);
		_sheet->writeStr(m_row, 1, m_NormalPoint[i].DiscernType.toLocal8Bit().data());
		_sheet->writeStr(m_row, 2, m_NormalPoint[i].PointName.toLocal8Bit().data());
		_sheet->writeStr(m_row, 3, m_NormalPoint[i].DiscernResult.toLocal8Bit().data());
		_sheet->writeStr(m_row, 4, QString("正常").toLocal8Bit().data());
		_sheet->writeStr(m_row, 5, m_NormalPoint[i].DiscernTime.toLocal8Bit().data());
		if (m_NormalPoint[i].RecognitionType == 1 || m_NormalPoint[i].RecognitionType == 2 || m_NormalPoint[i].RecognitionType == 3)
		{
			if (!m_NormalPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_NormalPoint[i].CollectMessage.VisibleLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 6, index, 96, 54, 0.5);
			}
		}
		else if (m_NormalPoint[i].RecognitionType == 4 || m_NormalPoint[i].RecognitionType == 6)
		{
			if (!m_NormalPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_NormalPoint[i].CollectMessage.VisibleLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 6, index, 96, 54, 0.5);
			}
			if (!m_NormalPoint[i].CollectMessage.InfraredLightPath.isEmpty())
			{
				_sheet->setRow(m_row, 41);
				int index = _book->addPicture((WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_NormalPoint[i].CollectMessage.InfraredLightPath).toLocal8Bit().data());
				_sheet->setPicture2(m_row, 7, index, 96, 54, 0.5);
			}
		}
		else {}
		m_row++;
	}
}

QString CreateExcelForLibxl::DateCalculate(QString StartTime, QString StopTime)
{
	int startYear = StartTime.mid(0, 4).toInt();
	int startMonth = StartTime.mid(5, 2).toInt();
	int startDay = StartTime.mid(8, 2).toInt();
	int startHour = StartTime.mid(11, 2).toInt();
	int startMinute = StartTime.mid(14, 2).toInt();
	int startSecond = StartTime.mid(17, 2).toInt();

	int stopYear = StopTime.mid(0, 4).toInt();
	int stopMonth = StopTime.mid(5, 2).toInt();
	int stopDay = StopTime.mid(8, 2).toInt();
	int stopHour = StopTime.mid(11, 2).toInt();
	int stopMinute = StopTime.mid(14, 2).toInt();
	int stopSecond = StopTime.mid(17, 2).toInt();

	int OverallSecond = stopSecond - startSecond;
	int OverallMinute = stopMinute - startMinute;
	int OverallHour = stopHour - startHour;
	int OverallDay = stopDay - startDay;
	int OverallMonth = stopMonth - startMonth;
	int OverallYear = stopYear - startYear;

	if (OverallSecond < 0)
	{
		OverallMinute = OverallMinute - 1;
		OverallSecond = OverallSecond + 60;
	}
	if (OverallMinute < 0)
	{
		OverallHour = OverallHour - 1;
		OverallMinute = OverallMinute + 60;
	}
	if (OverallHour < 0)
	{
		OverallDay = OverallDay - 1;
		OverallHour = OverallHour + 24;
	}
	if (OverallDay < 0)
	{
		OverallMonth = OverallMonth - 1;
		OverallDay = OverallDay + 30;
	}
	if (OverallMonth < 0)
	{
		OverallYear = OverallYear - 1;
		OverallMonth = OverallMonth + 12;
	}
	QString OverallTime = QString("%1年%2月%3日%4时%5分%6秒")
		.arg(OverallYear).arg(OverallMonth).arg(OverallDay)
		.arg(OverallHour).arg(OverallMinute).arg(OverallSecond);
	return OverallTime;
}