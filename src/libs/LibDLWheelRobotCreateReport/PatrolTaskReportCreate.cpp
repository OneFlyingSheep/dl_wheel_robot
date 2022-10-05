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
	unusualPoint.DiscernType = "��ƶ�ȡ";
	unusualPoint.PointName = "#1����220kV�׹�A��";
	unusualPoint.DiscernResult = "#1����XXXXXX���¶�XXX��";
	unusualPoint.AlarmGradeOrAuditResult = "�������";
	unusualPoint.DiscernTime = "XX��XX��XX��XX��XX��";
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
	normalPoint.DiscernType = "��ƶ�ȡ";
	normalPoint.PointName = "#1����220kV�׹�A��";
	normalPoint.DiscernResult = "#1����XXXXXX���¶�XXX��";
	normalPoint.AlarmGradeOrAuditResult = "����Ӧ�жϽ���ֶΣ�ȡ��������";
	normalPoint.DiscernTime = "XX��XX��XX��XX��XX��";
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
	QString fileName = "Ѳ�챨��";
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
	word->setPageOrientation(0);					//ҳ�����
	word->setWordPageView(3);					//ҳ����ͼ

	word->setFontSize(26);						//�����С
	word->setFontBold(true);						//����Ӵ�
	word->setParagraphAlignment(0);
	word->insertText(QString("����Ѳ�챨��"));
	word->insertMoveDown();
	////////////////////////////////////////////////////////////////////////////////////////
	//
	////////////////////////////////////////////////////////////////////////////////////////
	word->setFontBold(false);
	word->setFontSize(12);
	word->setParagraphAlignment(0);				//������������
	word->intsertTable(5, 6);					//���м���table
	word->setTableAutoFitBehavior(0);			//�Ƿ��������

	word->MergeCells(AtPresentTable, 1, 2, 1, 6);
	word->MergeCells(AtPresentTable, 2, 2, 2, 6);
	word->MergeCells(AtPresentTable, 3, 2, 3, 6);
	word->MergeCells(AtPresentTable, 4, 2, 4, 6);
	word->MergeCells(AtPresentTable, 5, 2, 5, 6);

	word->setCellString(AtPresentTable, 1, 1, QString("��������"));
	word->setCellString(AtPresentTable, 2, 1, QString("����ʱ��"));
	word->setCellString(AtPresentTable, 3, 1, QString("����״̬"));
	word->setCellString(AtPresentTable, 4, 1, QString("Ѳ���"));
	word->setCellString(AtPresentTable, 5, 1, QString("������Ϣ"));

	QString StartTime = m_ReportHeadMessage.TaskStartTime.replace("T", " ");
	QString StopTime = m_ReportHeadMessage.TaskStopTime.replace("T", " ");
	QString OverallTime = DateCalculate(StartTime, StopTime);
	QString TaskRunTime = QString("��ʼʱ��:%1 ����ʱ��:%2 ��ʱ��:%3").arg(StartTime).arg(StopTime).arg(OverallTime);
	QString TaskParrolPoint = QString("��������Ѳ��%1��λ��������λ%2�����澯��λ%3����ʶ���쳣��λ%4��")
		.arg(m_ReportHeadMessage.PatrolPointNum.AllPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.NormalPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.AlarmPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.UnusualPatrolPointNum);
	QString TaskEnvironment = QString("�����¶�:%1 ����ʪ��:%2 ����PM2.5%3 ��������:%4 ��������%5")
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
	word->setParagraphAlignment(0);				//������������
	word->intsertTable(row, column);			//���м���table

	word->setColumnWidth(AtPresentTable, 1, 60);
	word->setColumnWidth(AtPresentTable, 2, 60);
	word->setColumnWidth(AtPresentTable, 3, 60);
	word->setColumnWidth(AtPresentTable, 4, 60);
	word->setColumnWidth(AtPresentTable, 5, 60);
	word->setColumnWidth(AtPresentTable, 6, 126);

	word->MergeCells(AtPresentTable, 1, 1, 1, 6);
	word->setCellString(AtPresentTable, 1, 1, QString("�澯��λ"));
	word->setCellString(AtPresentTable, 2, 1, QString("ʶ������"));
	word->setCellString(AtPresentTable, 2, 2, QString("��λ����"));
	word->setCellString(AtPresentTable, 2, 3, QString("ʶ����"));
	word->setCellString(AtPresentTable, 2, 4, QString("�澯�ȼ�"));
	word->setCellString(AtPresentTable, 2, 5, QString("ʶ��ʱ��"));
	word->setCellString(AtPresentTable, 2, 6, QString("�ɼ���Ϣ"));

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
	word->setParagraphAlignment(0);				//������������
	word->intsertTable(row, column);			//���м���table

	word->setColumnWidth(AtPresentTable, 1, 60);
	word->setColumnWidth(AtPresentTable, 2, 60);
	word->setColumnWidth(AtPresentTable, 3, 60);
	word->setColumnWidth(AtPresentTable, 4, 60);
	word->setColumnWidth(AtPresentTable, 5, 60);
	word->setColumnWidth(AtPresentTable, 6, 126);

	word->MergeCells(AtPresentTable, 1, 1, 1, 6);
	word->setCellString(AtPresentTable, 1, 1, QString("�쳣��λ"));
	word->setCellString(AtPresentTable, 2, 1, QString("ʶ������"));
	word->setCellString(AtPresentTable, 2, 2, QString("��λ����"));
	word->setCellString(AtPresentTable, 2, 3, QString("ʶ����"));
	word->setCellString(AtPresentTable, 2, 4, QString("��˽��"));
	word->setCellString(AtPresentTable, 2, 5, QString("ʶ��ʱ��"));
	word->setCellString(AtPresentTable, 2, 6, QString("�ɼ���Ϣ"));

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
	word->setParagraphAlignment(0);				//������������
	word->intsertTable(row, column);			//���м���table

	word->setColumnWidth(AtPresentTable, 1, 60);
	word->setColumnWidth(AtPresentTable, 2, 60);
	word->setColumnWidth(AtPresentTable, 3, 60);
	word->setColumnWidth(AtPresentTable, 4, 60);
	word->setColumnWidth(AtPresentTable, 5, 60);
	word->setColumnWidth(AtPresentTable, 6, 126);

	word->MergeCells(AtPresentTable, 1, 1, 1, 6);
	word->setCellString(AtPresentTable, 1, 1, QString("������λ"));
	word->setCellString(AtPresentTable, 2, 1, QString("ʶ������"));
	word->setCellString(AtPresentTable, 2, 2, QString("��λ����"));
	word->setCellString(AtPresentTable, 2, 3, QString("ʶ����"));
	word->setCellString(AtPresentTable, 2, 4, QString("�澯�ȼ�"));
	word->setCellString(AtPresentTable, 2, 5, QString("ʶ��ʱ��"));
	word->setCellString(AtPresentTable, 2, 6, QString("�ɼ���Ϣ"));

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
	word->setCellString(AtPresentTable, 1, 1, QString("Ѳ��·��ͼ"));
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
	QString OverallTime = QString("%1��%2��%3��%4ʱ%5��%6��")
		.arg(OverallYear).arg(OverallMonth).arg(OverallDay)
		.arg(OverallHour).arg(OverallMinute).arg(OverallSecond);
	return OverallTime;
}