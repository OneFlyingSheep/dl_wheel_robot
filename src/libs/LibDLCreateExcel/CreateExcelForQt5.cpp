#include "CreateExcelForQt5.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"


bool CreateExcelForQt5::createReportForTask(QString task_uuid, QString SavePath)
{
	QString fileName = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/report";
	QDir dir;
	if (!dir.exists(fileName))
	{
		dir.mkpath(fileName);
	}
	QString path = QString("%1/%2.xlsx").arg(fileName).arg(SavePath);

	QXlsx::Document book;

	book.selectSheet("Sheet1");

	book.setColumnWidth(1, 15);book.setColumnWidth(2, 20);book.setColumnWidth(3, 40);
	book.setColumnWidth(4, 15);book.setColumnWidth(5, 15);book.setColumnWidth(6, 30);
	book.setColumnWidth(7, 13);book.setColumnWidth(8, 13);
	book.setRowHeight(1, 30);
	book.mergeCells(getMergeCellsQString(1, 1, 1, 8));

	QXlsx::Format format;
	format.setFontSize(24);
	format.setFontBold(true);
	//文字居中
	format.setHorizontalAlignment(QXlsx::Format::AlignHCenter);

	QXlsx::RichString rich;
	rich.addFragment("巡检报告", format);

	book.write(m_row, 1, rich);
	m_row++;
	//报告表头
	SetReportHeadMessage(task_uuid, book);
	//告警点位
	SetAlarmPointMessage(task_uuid, book);
	//异常点位
	SetUnusualPointMessage(task_uuid, book);
	//正常点位
	SetNormalPointMessage(task_uuid, book);

	bool bRet = book.saveAs(path);
	return bRet;
}

QString CreateExcelForQt5::getMergeCellsQString(int x1, int y1, int x2, int y2)
{
	QString cell;
	cell.append(QChar(y1 - 1 + 'A'));
	cell.append(QString::number(x1));
	cell.append(":");
	cell.append(QChar(y2 - 1 + 'A'));
	cell.append(QString::number(x2));
	return cell;
}

void CreateExcelForQt5::SetReportHeadMessage(QString _taskUUid, QXlsx::Document &book)
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

	book.mergeCells(getMergeCellsQString(m_row, 2, m_row, 8));
	book.write(m_row, 1, QString("任务名称"));
	book.write(m_row, 2, m_ReportHeadMessage.TaskName);
	m_row++;

	book.mergeCells(getMergeCellsQString(m_row, 2, m_row, 8));
	book.write(m_row, 1, QString("任务时间"));
	book.write(m_row, 2, TaskRunTime);
	m_row++;

	book.mergeCells(getMergeCellsQString(m_row, 2, m_row, 8));
	book.write(m_row, 1, QString("任务状态"));
	book.write(m_row, 2, m_ReportHeadMessage.TaskStatus);
	m_row++;

	book.mergeCells(getMergeCellsQString(m_row, 2, m_row, 8));
	book.write(m_row, 1, QString("巡检点"));
	book.write(m_row, 2, TaskParrolPoint);
	m_row++;

	book.mergeCells(getMergeCellsQString(m_row, 2, m_row, 8));
	book.write(m_row, 1, QString("环境信息"));
	book.write(m_row, 2, TaskEnvironment);
	m_row++;
}

void CreateExcelForQt5::SetAlarmPointMessage(QString task_uuid, QXlsx::Document &book)
{
	book.mergeCells(getMergeCellsQString(m_row, 1, m_row, 8));

	QXlsx::Format format;
	format.setFontSize(16);
	format.setFontBold(true);
	format.setHorizontalAlignment(QXlsx::Format::AlignHCenter);
	QXlsx::RichString rich;
	rich.addFragment("告警点位", format);
	book.write(m_row, 1, rich);
	m_row++;

	book.write(m_row, 1, QString("序号"));
	book.write(m_row, 2, QString("识别类型"));
	book.write(m_row, 3, QString("点位名称"));
	book.write(m_row, 4, QString("识别结果"));
	book.write(m_row, 5, QString("告警等级"));
	book.write(m_row, 6, QString("识别时间"));
	book.write(m_row, 7, QString("采集信息"));
	book.write(m_row, 8, QString("采集信息"));
	m_row++;

	QList<AlarmUnusualNormalPoint> m_AlarmPoint;
	WHEEL_ROBOT_DB.getWheelAlarmPointDB(task_uuid, m_AlarmPoint);

	for (int i = 0; i < m_AlarmPoint.size(); i++)
	{
		book.setRowHeight(m_row, 40.5);
		book.write(m_row, 1, i + 1);
		book.write(m_row, 2, m_AlarmPoint[i].DiscernType);
		book.write(m_row, 3, m_AlarmPoint[i].PointName);
		book.write(m_row, 4, m_AlarmPoint[i].DiscernResult);
		book.write(m_row, 5, m_AlarmPoint[i].AlarmGradeOrAuditResult);
		book.write(m_row, 6, m_AlarmPoint[i].DiscernTime);
		
		if (m_AlarmPoint[i].RecognitionType == 1 || m_AlarmPoint[i].RecognitionType == 2 || m_AlarmPoint[i].RecognitionType == 3)
		{
			if (!m_AlarmPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				QImage image;
		 		image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_AlarmPoint[i].CollectMessage.VisibleLightPath);
				book.insertImage(m_row-1, 6, image);
			}
		}
		else if (m_AlarmPoint[i].RecognitionType == 4 || m_AlarmPoint[i].RecognitionType == 6)
		{
			if (!m_AlarmPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				QImage image;
				image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_AlarmPoint[i].CollectMessage.VisibleLightPath);
				book.insertImage(m_row - 1, 6, image);
			}
			if (!m_AlarmPoint[i].CollectMessage.InfraredLightPath.isEmpty())
			{
				QImage image;
				image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_AlarmPoint[i].CollectMessage.InfraredLightPath);
				book.insertImage(m_row - 1, 6, image);
			}
		}
		else {}
		
		m_row++;
	}
}

void CreateExcelForQt5::SetUnusualPointMessage(QString task_uuid, QXlsx::Document &book)
{
	book.mergeCells(getMergeCellsQString(m_row, 1, m_row, 8));

	QXlsx::Format format;
	format.setFontSize(16);
	format.setFontBold(true);
	format.setHorizontalAlignment(QXlsx::Format::AlignHCenter);
	QXlsx::RichString rich;
	rich.addFragment("异常点位", format);
	book.write(m_row, 1, rich);
	m_row++;

	book.write(m_row, 1, QString("序号"));
	book.write(m_row, 2, QString("识别类型"));
	book.write(m_row, 3, QString("点位名称"));
	book.write(m_row, 4, QString("识别结果"));
	book.write(m_row, 5, QString("审核结果"));
	book.write(m_row, 6, QString("识别时间"));
	book.write(m_row, 7, QString("采集信息"));
	book.write(m_row, 8, QString("采集信息"));
	m_row++;

	QList<AlarmUnusualNormalPoint> m_UnusualPoint;
	WHEEL_ROBOT_DB.getUnusualPointDB(task_uuid, m_UnusualPoint);

	for (int i = 0; i < m_UnusualPoint.size(); i++)
	{
		book.setRowHeight(m_row, 40.5);
		book.write(m_row, 1, i + 1);
		book.write(m_row, 2, m_UnusualPoint[i].DiscernType);
		book.write(m_row, 3, m_UnusualPoint[i].PointName);
		book.write(m_row, 4, m_UnusualPoint[i].DiscernResult);
		book.write(m_row, 5, m_UnusualPoint[i].AlarmGradeOrAuditResult);
		book.write(m_row, 6, m_UnusualPoint[i].DiscernTime);
		if (m_UnusualPoint[i].RecognitionType == 1 || m_UnusualPoint[i].RecognitionType == 2 || m_UnusualPoint[i].RecognitionType == 3)
		{
			if (!m_UnusualPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				QImage image;
				image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_UnusualPoint[i].CollectMessage.VisibleLightPath);
				book.insertImage(m_row - 1, 6, image);
			}
		}
		else if (m_UnusualPoint[i].RecognitionType == 4 || m_UnusualPoint[i].RecognitionType == 6)
		{
			if (!m_UnusualPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				QImage image;
				image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_UnusualPoint[i].CollectMessage.VisibleLightPath);
				book.insertImage(m_row - 1, 6, image);
			}
			if (!m_UnusualPoint[i].CollectMessage.InfraredLightPath.isEmpty())
			{
				QImage image;
				image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_UnusualPoint[i].CollectMessage.InfraredLightPath);
				book.insertImage(m_row - 1, 6, image);
			}
		}
		else {}
		m_row++;
	}
}

void CreateExcelForQt5::SetNormalPointMessage(QString task_uuid, QXlsx::Document &book)
{
	book.mergeCells(getMergeCellsQString(m_row, 1, m_row, 8));

	QXlsx::Format format;
	format.setFontSize(16);
	format.setFontBold(true);
	format.setHorizontalAlignment(QXlsx::Format::AlignHCenter);
	QXlsx::RichString rich;
	rich.addFragment("正常点位", format);
	book.write(m_row, 1, rich);
	m_row++;

	book.write(m_row, 1, QString("序号"));
	book.write(m_row, 2, QString("识别类型"));
	book.write(m_row, 3, QString("点位名称"));
	book.write(m_row, 4, QString("识别结果"));
	book.write(m_row, 5, QString("告警等级"));
	book.write(m_row, 6, QString("识别时间"));
	book.write(m_row, 7, QString("采集信息"));
	book.write(m_row, 8, QString("采集信息"));
	m_row++;

	QList<AlarmUnusualNormalPoint> m_NormalPoint;
	WHEEL_ROBOT_DB.getWheelNormalPointDB(task_uuid, m_NormalPoint);

	for (int i = 0; i < m_NormalPoint.size(); i++)
	{
		book.setRowHeight(m_row, 40.5);
		book.write(m_row, 1, i + 1);
		book.write(m_row, 2, m_NormalPoint[i].DiscernType);
		book.write(m_row, 3, m_NormalPoint[i].PointName);
		book.write(m_row, 4, m_NormalPoint[i].DiscernResult);
		book.write(m_row, 5, QString("正常"));
		book.write(m_row, 6, m_NormalPoint[i].DiscernTime);
		if (m_NormalPoint[i].RecognitionType == 1 || m_NormalPoint[i].RecognitionType == 2 || m_NormalPoint[i].RecognitionType == 3)
		{
			if (!m_NormalPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				QImage image;
				image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_NormalPoint[i].CollectMessage.VisibleLightPath);
				book.insertImage(m_row - 1, 6, image);
			}
		}
		else if (m_NormalPoint[i].RecognitionType == 4 || m_NormalPoint[i].RecognitionType == 6)
		{
			if (!m_NormalPoint[i].CollectMessage.VisibleLightPath.isEmpty())
			{
				QImage image;
				image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_NormalPoint[i].CollectMessage.VisibleLightPath);
				book.insertImage(m_row - 1, 6, image);
			}
			if (!m_NormalPoint[i].CollectMessage.InfraredLightPath.isEmpty())
			{
				QImage image;
				image.load(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + m_NormalPoint[i].CollectMessage.InfraredLightPath);
				book.insertImage(m_row - 1, 6, image);
			}
		}
		else {}
		m_row++;
	}
}

QString CreateExcelForQt5::DateCalculate(QString StartTime, QString StopTime)
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

bool CreateExcelForQt5::import_excel_report(QList<QStringList> data, QString filePath)
{
    QString path = filePath;

    QXlsx::Document book;

    book.selectSheet("Sheet1");

    int row = 0;
    for (int row = 0; row < data.size(); row++)
    {
        if (row == 0)
        {
            book.write(row + 1, 1, QString("序号"));
        }
        else
        {
            book.write(row, 1, row + 1);
        }

        for (int column = 0; column < data[row].size(); column++)
        {
            book.write(row + 1, column + 2, data[row][column]);
        }
    }

    bool bRet = book.saveAs(path);
    return bRet;
}

bool CreateExcelForQt5::export_excel_result(QString taskUUid, QString strPath)
{
    QString path = strPath + "/" + taskUUid + ".xlsx";

    QXlsx::Document book;

    book.selectSheet("Sheet1");

    QList<QStringList> data;
    WHEEL_ROBOT_DB.getCheTaskDeviceNameResult(taskUUid, data);
    int row = 1;
    book.write(1, 1, QString("设备名称"));
    book.write(1, 2, QString("识别结果"));
    for (int i = 0; i < data.size(); i++)
    {
        book.write(i + 2, 1, data[i][0]);
        book.write(i + 2, 2, data[i][1]);
    }
    bool bRet = book.saveAs(path);
    return bRet;
}
