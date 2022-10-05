#pragma once
#include <QWidget>
#include <QDate>
#include "common/Singleton.hpp"
class CountDateTime : public Singleton<CountDateTime>
{
public:
	CountDateTime();
	~CountDateTime();

	

public:
	//定时任务
	QStringList getCountTimingTaskChoose(QDate startDate, QDate endDate, QString dateDay, QString dateTime);
	//获取两个日期之间 所有天数列表
	QStringList getCountTwoDateForAddOneDay(QDate startDate, QDate endDate, QString dateDay, QString dateTime);
	//每周选择几天列表
	QStringList getCountChooseWeekList(QDate startDate, QDate endDate, QString dateWeek, QString dateTime);
	//每月一天列表
	QStringList getCountMonthlyOneDayList(QDate startDate, QDate endDate, QString dateDay, QString dateTime);
	//间隔固定天数
	QStringList getCountFixationIntervalTimeList(QDate initialDate, QDate startDate, QDate endDate, int fixationDay, QString dateTime);
	//固定时间
	QStringList getCountFixationDateTimeList(QDate startDate, QDate endDate, QString fixationDay, QString dateTime);
	void setCountTimeMess();
private:
	QTime curr_hms_now_time;
	QDate nowTime;
};
#define WHEEL_COUNT_DATE CountDateTime::GetSingleton()
