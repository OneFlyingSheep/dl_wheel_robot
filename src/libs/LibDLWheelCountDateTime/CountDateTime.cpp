#include "CountDateTime.h"
#include<QDebug>
#include<QRegExp>
CountDateTime::CountDateTime()
{
}

CountDateTime::~CountDateTime()
{
}

QStringList CountDateTime::getCountTimingTaskChoose(QDate startDate, QDate endDate, QString dateDay, QString dateTime)
{
	setCountTimeMess();
	QDate dateMid = QDate::fromString(dateDay, "yyyy-MM-dd");
	QStringList returnDateList;

	if ((dateMid < startDate) || (dateMid > endDate))
	{
		return returnDateList;
	}
	else
	{
		if (nowTime < startDate)
		{
			returnDateList.append(QString("%1 %2").arg(dateDay).arg(dateTime));
		}
		else if(nowTime > endDate)
		{
			return returnDateList;
		}
		else
		{
			if (nowTime < dateMid)
			{
				returnDateList.append(QString("%1 %2").arg(dateDay).arg(dateTime));
			}
			else if (nowTime > dateMid)
			{
				return returnDateList;
			}
			else
			{
			//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
				{
					returnDateList.append(QString("%1 %2").arg(dateDay).arg(dateTime));
				}
			}
		}
	}
	return returnDateList;
}

QStringList CountDateTime::getCountTwoDateForAddOneDay(QDate startDate, QDate endDate, QString dateDay, QString dateTime)
{
	setCountTimeMess();
	QDate dateMid = QDate::fromString(dateDay, "yyyy-MM-dd");
	QStringList returnDateList;
	int i = 0;
	if (dateMid < startDate)
	{

	}
	else if (dateMid >= startDate && dateMid <= endDate)
	{
		startDate = dateMid;
	}
	else if (dateMid > endDate)
	{
		return returnDateList;
	}
	if (startDate == nowTime)
	{
	//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
		{
			returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
		}
	}
	else
	{
		returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
	}
	if (startDate == endDate)
	{
	}
	else
	{
		while (1)
		{
			startDate = startDate.addDays(1);
			if (startDate == nowTime)
			{
			//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
				{
					returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
				}
			}
			else
			{
				returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
			}
			if (startDate == endDate)
				break;
			i++;
			if (i > 300)
				break;
		}
	}
	return returnDateList;
}

QStringList CountDateTime::getCountChooseWeekList(QDate startDate, QDate endDate, QString dateWeek, QString dateTime)
{
	setCountTimeMess();
	QStringList returnDateList;
	int i = 0;
	int iWeek = startDate.dayOfWeek();
	QStringList sections = dateWeek.split(QRegExp("[,]"));
	for (int k = 0; k < sections.size(); k++)
	{
		if (sections[k].toInt() == iWeek)
		{
			if (startDate == nowTime)
			{
			//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
				{
					returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
				}
			}
			else
			{
				returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
			}
		}
	}
	if (startDate == endDate)
		return returnDateList;
	while (1)
	{
		startDate = startDate.addDays(1);
		iWeek = startDate.dayOfWeek();
		for (int k = 0; k < sections.size(); k++)
		{
			if (sections[k].toInt() == iWeek)
			{
				if (startDate == nowTime)
				{
				//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
					{
						returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
					}
				}
				else
				{
					returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
				}
			}
		}
 		if (startDate == endDate)
 			break;
		i++;
		if (i > 300)
			break;
	}
	return returnDateList;
}

QStringList CountDateTime::getCountMonthlyOneDayList(QDate startDate, QDate endDate, QString dateDay, QString dateTime)
{
	setCountTimeMess();
	QStringList returnDateList;
	int i = 0;

	int iday = startDate.toString("yyyy-MM-dd").mid(8, 2).toInt();
	int sections = dateDay.toInt();
	
	if (sections == iday)
	{
		if (startDate == nowTime)
		{
		//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
			{
				returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
			}
		}
		else
		{
			returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
		}
	}
	if (startDate == endDate)
		return returnDateList;
	while (1)
	{
		startDate = startDate.addDays(1);
		iday = startDate.toString("yyyy-MM-dd").mid(8, 2).toInt();

		if (sections == iday)
		{
			if (startDate == nowTime)
			{
			//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
				{
					returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
				}
			}
			else
			{
				returnDateList.append(QString("%1 %2").arg(startDate.toString("yyyy-MM-dd")).arg(dateTime));
			}
		}
		
		if (startDate == endDate)
			break;
		i++;
		if (i > 300)
			break;
	}
	return returnDateList;
}

QStringList CountDateTime::getCountFixationIntervalTimeList(QDate initialDate, QDate startDate, QDate endDate, int fixationDay, QString dateTime)
{
	setCountTimeMess();
	QStringList returnDateList;
	QStringList dateList;
	bool bRect = false;
	int i = 0;
	int sections = fixationDay;
	if (initialDate >= startDate&& initialDate<= endDate)
	{
		if (startDate == nowTime)
		{
		//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
			{
				returnDateList.append(QString("%1 %2").arg(initialDate.toString("yyyy-MM-dd")).arg(dateTime));
			}
			bRect = true;
		}
		else
		{
			returnDateList.append(QString("%1 %2").arg(initialDate.toString("yyyy-MM-dd")).arg(dateTime));
		}
	}
	if (startDate == endDate || initialDate > endDate)
		return returnDateList;
	while (1)
	{
		i++;
		initialDate = initialDate.addDays(sections + 1);
		if (initialDate >= startDate && initialDate <= endDate)
		{
			if (bRect)
			{
				returnDateList.append(QString("%1 %2").arg(initialDate.toString("yyyy-MM-dd")).arg(dateTime));
			}
			else
			{
				if (startDate == nowTime)
				{
				//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
					{
						returnDateList.append(QString("%1 %2").arg(initialDate.toString("yyyy-MM-dd")).arg(dateTime));
					}
				}
				else
				{
					returnDateList.append(QString("%1 %2").arg(initialDate.toString("yyyy-MM-dd")).arg(dateTime));
				}
			}
		}
		if (initialDate >= endDate)
			break;
		
		if (i > 300)
			break;
	}

	return returnDateList;
}

QStringList CountDateTime::getCountFixationDateTimeList(QDate startDate, QDate endDate, QString fixationDay, QString dateTime)
{
	setCountTimeMess();
	QStringList returnDateList;
	QStringList sections = fixationDay.split(QRegExp("[,]"));
	for (int i = 0; i < sections.size(); i++)
	{
		QDate date = QDate::fromString(sections[i], "yyyy-MM-dd");
		if (date >= startDate && date <= endDate)
		{
			if (date == nowTime)
			{
			//	if (QTime::fromString(dateTime, "hh:mm:ss") > curr_hms_now_time)
				{
					returnDateList.append(QString("%1 %2").arg(date.toString("yyyy-MM-dd")).arg(dateTime));
				}
			}
			else
			{
				returnDateList.append(QString("%1 %2").arg(date.toString("yyyy-MM-dd")).arg(dateTime));
			}
		}
	}
	return returnDateList;
}

void CountDateTime::setCountTimeMess()
{
	QDateTime current_date_time = QDateTime::currentDateTime();
	QString current_date = current_date_time.toString("yyyy-MM-dd hh:mm:ss");
	QString curr_hms_now_string = current_date.mid(11, 8);
	curr_hms_now_time = QTime::fromString(curr_hms_now_string, "hh:mm:ss");
	nowTime = QDate::fromString(current_date.mid(0, 10), "yyyy-MM-dd");
}
