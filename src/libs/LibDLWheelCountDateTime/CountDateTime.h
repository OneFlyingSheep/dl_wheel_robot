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
	//��ʱ����
	QStringList getCountTimingTaskChoose(QDate startDate, QDate endDate, QString dateDay, QString dateTime);
	//��ȡ��������֮�� ���������б�
	QStringList getCountTwoDateForAddOneDay(QDate startDate, QDate endDate, QString dateDay, QString dateTime);
	//ÿ��ѡ�����б�
	QStringList getCountChooseWeekList(QDate startDate, QDate endDate, QString dateWeek, QString dateTime);
	//ÿ��һ���б�
	QStringList getCountMonthlyOneDayList(QDate startDate, QDate endDate, QString dateDay, QString dateTime);
	//����̶�����
	QStringList getCountFixationIntervalTimeList(QDate initialDate, QDate startDate, QDate endDate, int fixationDay, QString dateTime);
	//�̶�ʱ��
	QStringList getCountFixationDateTimeList(QDate startDate, QDate endDate, QString fixationDay, QString dateTime);
	void setCountTimeMess();
private:
	QTime curr_hms_now_time;
	QDate nowTime;
};
#define WHEEL_COUNT_DATE CountDateTime::GetSingleton()
