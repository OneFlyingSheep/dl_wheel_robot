#include "PatrolTaskReportCreate.h"
#include <QDir>

PatrolTaskReportCreate::PatrolTaskReportCreate()
{
	word = new QWord();
	AtPresentTable = 0;
}

PatrolTaskReportCreate::~PatrolTaskReportCreate()
{
	delete word;
}

void PatrolTaskReportCreate::CreateWordFromTaskEnd(QString m_SavePath, QString m_TaskUUid)
{
	CreateNewWordWithPath(m_SavePath, NULL);

	WheelReportHeadMessage m_ReportHeadMessage;
	WHEEL_ROBOT_DB.getWheelReportHeadMessageDB(m_TaskUUid, m_ReportHeadMessage);
	WordReportHeadMessage(m_ReportHeadMessage);

	QList<AlarmUnusualNormalPoint> m_AlarmPoint;
	WHEEL_ROBOT_DB.getWheelAlarmPointDB(m_TaskUUid, m_AlarmPoint);
	AlarmPointReportWrite(2 + m_AlarmPoint.size(), 6, m_AlarmPoint);

	QList<AlarmUnusualNormalPoint> m_UnusualPoint;
	AlarmUnusualNormalPoint unusualPoint;
	unusualPoint.DiscernType = "表计读取";
	unusualPoint.PointName = "#1主变220kV套管A相";
	unusualPoint.DiscernResult = "#1主变XXXXXX点温度XXX度";
	unusualPoint.AlarmGradeOrAuditResult = "审核数据";
	unusualPoint.DiscernTime = "XX年XX月XX日XX点XX分";
	unusualPoint.CollectMessage.VisibleLightPath = "D:\\test\\cc.jpg";
	unusualPoint.CollectMessage.InfraredLightPath = "D:\\test\\aa.jpg";
	for (int k = 0; k < 100; k++)
	{
		m_UnusualPoint.append(unusualPoint);
	}
	//WHEEL_ROBOT_DB.getWheelUnusualPointDB(m_TaskUUid, m_AlarmPoint);
	UnusualPointReportWrite(2 + m_UnusualPoint.size(), 6, m_UnusualPoint);

	QList<AlarmUnusualNormalPoint> m_NormalPoint;
	AlarmUnusualNormalPoint normalPoint;
	normalPoint.DiscernType = "表计读取";
	normalPoint.PointName = "#1主变220kV套管A相";
	normalPoint.DiscernResult = "#1主变XXXXXX点温度XXX度";
	normalPoint.AlarmGradeOrAuditResult = "（对应判断结果字段，取非正常）";
	normalPoint.DiscernTime = "XX年XX月XX日XX点XX分";
	normalPoint.CollectMessage.VisibleLightPath = "D:\\test\\dd.jpg";
	normalPoint.CollectMessage.InfraredLightPath = "D:\\test\\bb.jpg";
	for (int j = 0; j < 100; j++)
	{
		m_NormalPoint.append(normalPoint);
	}
	NormalPointReportWrite(2 + m_NormalPoint.size(), 6, m_NormalPoint);

	PatrolLineMapReportWrite("D:\\test\\cc.jpg");

	CloseReportRedact();
}

bool PatrolTaskReportCreate::CreateNewWordWithPath(QString m_path, QString task_ssid)
{
	QString fileName = "巡检报告";
	QString filePath = m_path;

	QDir dirReportPath(filePath);

	if (!dirReportPath.exists())
	{
		if (dirReportPath.mkpath(filePath))
		{
			filePath += fileName + QString(".doc");
		}
	}
	else
	{
		filePath += fileName + QString(".doc");
	}

	if (!word->createNewWord(filePath))
	{
		return false;
	}
	return true;
}

void PatrolTaskReportCreate::WordReportHeadMessage(WheelReportHeadMessage m_ReportHeadMessage)
{
	AtPresentTable++;
	word->setProperty("Visible", true);
	word->setPageOrientation(0);					//页面横向
	word->setWordPageView(3);					//页面视图

	word->setFontSize(26);						//字体大小
	word->setFontBold(true);						//字体加粗
	word->setParagraphAlignment(0);
	word->insertText(QString("智能巡检报告"));
	word->insertMoveDown();
	////////////////////////////////////////////////////////////////////////////////////////
	//
	////////////////////////////////////////////////////////////////////////////////////////
	word->setFontBold(false);
	word->setFontSize(12);
	word->setParagraphAlignment(0);				//下面文字置中
	word->intsertTable(5, 6);					//几行几列table
	word->setTableAutoFitBehavior(0);			//是否可以拉伸

	word->MergeCells(AtPresentTable, 1, 2, 1, 6);
	word->MergeCells(AtPresentTable, 2, 2, 2, 6);
	word->MergeCells(AtPresentTable, 3, 2, 3, 6);
	word->MergeCells(AtPresentTable, 4, 2, 4, 6);
	word->MergeCells(AtPresentTable, 5, 2, 5, 6);

	word->setCellString(AtPresentTable, 1, 1, QString("任务名称"));
	word->setCellString(AtPresentTable, 2, 1, QString("任务时间"));
	word->setCellString(AtPresentTable, 3, 1, QString("任务状态"));
	word->setCellString(AtPresentTable, 4, 1, QString("巡检点"));
	word->setCellString(AtPresentTable, 5, 1, QString("环境信息"));

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

	word->setCellString(AtPresentTable, 1, 2, m_ReportHeadMessage.TaskName);
	word->setCellString(AtPresentTable, 2, 2, TaskRunTime);
	word->setCellString(AtPresentTable, 3, 2, m_ReportHeadMessage.TaskStatus);
	word->setCellString(AtPresentTable, 4, 2, TaskParrolPoint);
	word->setCellString(AtPresentTable, 5, 2, TaskEnvironment);

	word->moveForEnd();
	word->insertMoveDown();
}

void PatrolTaskReportCreate::AlarmPointReportWrite(int row, int column, QList<AlarmUnusualNormalPoint> m_AlarmPoint)
{
	AtPresentTable++;
	word->setFontSize(12);
	word->setParagraphAlignment(0);				//下面文字置中
	word->intsertTable(row, column);			//几行几列table

	word->setColumnWidth(AtPresentTable, 1, 60);
	word->setColumnWidth(AtPresentTable, 2, 60);
	word->setColumnWidth(AtPresentTable, 3, 60);
	word->setColumnWidth(AtPresentTable, 4, 60);
	word->setColumnWidth(AtPresentTable, 5, 60);
	word->setColumnWidth(AtPresentTable, 6, 126);

	word->MergeCells(AtPresentTable, 1, 1, 1, 6);
	word->setCellString(AtPresentTable, 1, 1, QString("告警点位"));
	word->setCellString(AtPresentTable, 2, 1, QString("识别类型"));
	word->setCellString(AtPresentTable, 2, 2, QString("点位名称"));
	word->setCellString(AtPresentTable, 2, 3, QString("识别结果"));
	word->setCellString(AtPresentTable, 2, 4, QString("告警等级"));
	word->setCellString(AtPresentTable, 2, 5, QString("识别时间"));
	word->setCellString(AtPresentTable, 2, 6, QString("采集信息"));

	for (int i = 0; i < m_AlarmPoint.size(); i++)
	{
		word->setCellString(AtPresentTable, i + 3, 1, QString(m_AlarmPoint[i].DiscernType));
		word->setCellString(AtPresentTable, i + 3, 2, QString(m_AlarmPoint[i].PointName));
		word->setCellString(AtPresentTable, i + 3, 3, QString(m_AlarmPoint[i].DiscernResult));
		word->setCellString(AtPresentTable, i + 3, 4, QString(m_AlarmPoint[i].AlarmGradeOrAuditResult));
		word->setCellString(AtPresentTable, i + 3, 5, QString(m_AlarmPoint[i].DiscernTime));
		if (!m_AlarmPoint[i].CollectMessage.InfraredLightPath.isEmpty())
		{
			word->insertCellPic(AtPresentTable, i + 3, 6, m_AlarmPoint[i].CollectMessage.InfraredLightPath);
		}
		if (!m_AlarmPoint[i].CollectMessage.VisibleLightPath.isEmpty())
		{
			word->insertCellPic(AtPresentTable, i + 3, 6, m_AlarmPoint[i].CollectMessage.VisibleLightPath);
		}
	}
	word->moveForEnd();
	word->insertMoveDown();
}

void PatrolTaskReportCreate::UnusualPointReportWrite(int row, int column, QList<AlarmUnusualNormalPoint> m_UnusualPoint)
{
	AtPresentTable++;
	word->setFontSize(12);
	word->setParagraphAlignment(0);				//下面文字置中
	word->intsertTable(row, column);			//几行几列table

	word->setColumnWidth(AtPresentTable, 1, 60);
	word->setColumnWidth(AtPresentTable, 2, 60);
	word->setColumnWidth(AtPresentTable, 3, 60);
	word->setColumnWidth(AtPresentTable, 4, 60);
	word->setColumnWidth(AtPresentTable, 5, 60);
	word->setColumnWidth(AtPresentTable, 6, 126);

	word->MergeCells(AtPresentTable, 1, 1, 1, 6);
	word->setCellString(AtPresentTable, 1, 1, QString("异常点位"));
	word->setCellString(AtPresentTable, 2, 1, QString("识别类型"));
	word->setCellString(AtPresentTable, 2, 2, QString("点位名称"));
	word->setCellString(AtPresentTable, 2, 3, QString("识别结果"));
	word->setCellString(AtPresentTable, 2, 4, QString("审核结果"));
	word->setCellString(AtPresentTable, 2, 5, QString("识别时间"));
	word->setCellString(AtPresentTable, 2, 6, QString("采集信息"));

	for (int i = 0; i < m_UnusualPoint.size(); i++)
	{
		word->setCellString(AtPresentTable, i + 3, 1, QString(m_UnusualPoint[i].DiscernType));
		word->setCellString(AtPresentTable, i + 3, 2, QString(m_UnusualPoint[i].PointName));
		word->setCellString(AtPresentTable, i + 3, 3, QString(m_UnusualPoint[i].DiscernResult));
		word->setCellString(AtPresentTable, i + 3, 4, QString(m_UnusualPoint[i].AlarmGradeOrAuditResult));
		word->setCellString(AtPresentTable, i + 3, 5, QString(m_UnusualPoint[i].DiscernTime));
		if (!m_UnusualPoint[i].CollectMessage.InfraredLightPath.isEmpty())
		{
			word->insertCellPic(AtPresentTable, i + 3, 6, m_UnusualPoint[i].CollectMessage.InfraredLightPath);
		}
		if (!m_UnusualPoint[i].CollectMessage.VisibleLightPath.isEmpty())
		{
			word->insertCellPic(AtPresentTable, i + 3, 6, m_UnusualPoint[i].CollectMessage.VisibleLightPath);
		}
	}
	word->moveForEnd();
	word->insertMoveDown();
}

void PatrolTaskReportCreate::NormalPointReportWrite(int row, int column, QList<AlarmUnusualNormalPoint> m_NormalPoint)
{
	AtPresentTable++;
	word->setFontSize(12);
	word->setParagraphAlignment(0);				//下面文字置中
	word->intsertTable(row, column);			//几行几列table

	word->setColumnWidth(AtPresentTable, 1, 60);
	word->setColumnWidth(AtPresentTable, 2, 60);
	word->setColumnWidth(AtPresentTable, 3, 60);
	word->setColumnWidth(AtPresentTable, 4, 60);
	word->setColumnWidth(AtPresentTable, 5, 60);
	word->setColumnWidth(AtPresentTable, 6, 126);

	word->MergeCells(AtPresentTable, 1, 1, 1, 6);
	word->setCellString(AtPresentTable, 1, 1, QString("正常点位"));
	word->setCellString(AtPresentTable, 2, 1, QString("识别类型"));
	word->setCellString(AtPresentTable, 2, 2, QString("点位名称"));
	word->setCellString(AtPresentTable, 2, 3, QString("识别结果"));
	word->setCellString(AtPresentTable, 2, 4, QString("告警等级"));
	word->setCellString(AtPresentTable, 2, 5, QString("识别时间"));
	word->setCellString(AtPresentTable, 2, 6, QString("采集信息"));

	for (int i = 0; i < m_NormalPoint.size(); i++)
	{
		word->setCellString(AtPresentTable, i + 3, 1, QString(m_NormalPoint[i].DiscernType));
		word->setCellString(AtPresentTable, i + 3, 2, QString(m_NormalPoint[i].PointName));
		word->setCellString(AtPresentTable, i + 3, 3, QString(m_NormalPoint[i].DiscernResult));
		word->setCellString(AtPresentTable, i + 3, 4, QString(m_NormalPoint[i].AlarmGradeOrAuditResult));
		word->setCellString(AtPresentTable, i + 3, 5, QString(m_NormalPoint[i].DiscernTime));
		if (!m_NormalPoint[i].CollectMessage.InfraredLightPath.isEmpty())
		{
			word->insertCellPic(AtPresentTable, i + 3, 6, m_NormalPoint[i].CollectMessage.InfraredLightPath);
		}
		if (!m_NormalPoint[i].CollectMessage.VisibleLightPath.isEmpty())
		{
			word->insertCellPic(AtPresentTable, i + 3, 6, m_NormalPoint[i].CollectMessage.VisibleLightPath);
		}
	}
	word->moveForEnd();
	word->insertMoveDown();
}

void PatrolTaskReportCreate::PatrolLineMapReportWrite(QString m_PatrolMapPath)
{
	AtPresentTable++;
	word->intsertTable(1, 1);
	word->setCellString(AtPresentTable, 1, 1, QString("巡视路线图"));
	word->insertCellPic(AtPresentTable, 1, 1, m_PatrolMapPath);

	word->moveForEnd();
	word->insertMoveDown();
}

void PatrolTaskReportCreate::CloseReportRedact()
{
	word->close();
}

QString PatrolTaskReportCreate::DateCalculate(QString StartTime, QString StopTime)
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