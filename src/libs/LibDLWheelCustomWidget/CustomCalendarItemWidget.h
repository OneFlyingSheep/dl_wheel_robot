#pragma once

#include <QWidget>
#include <QDate>
#include <QLabel>
#include <QListWidget>
#include "common/DLWheelRobotGlobalDef.hpp"

// ����Item����;
enum DayType {
	DayType_MonthPre = 0,       // ����ʣ������;
	DayType_MonthNext = 1,      // �¸��µ�����;
	DayType_MonthCurrent = 2,   // ��������;
	DayType_WeekEnd = 3         // ��ĩ;
};

// ����ִ�н������;
enum TaskExecuteResultType
{
	ExecuteDone,				// ִ�����;
	MidWayAbort,				// ��;��ֹ;
	Executing,					// ����ִ��;
	WaitForExecute,				// �ȴ�ִ��;
	TaskOverTime,				// ������;
};

class CustomCalendarItemWidget : public QWidget
{
	Q_OBJECT

public:
	CustomCalendarItemWidget(QWidget *parent = NULL);
	~CustomCalendarItemWidget();

	// ��item�������ڣ�������;
	void setDate(QDate itemDate, DayType dayType, QList<WheelCalendarData> itemDataList = QList<WheelCalendarData>());

private:
	// ��ʼ��Item;
	void initWidget();
	// ����Item;
	void updateItem(bool isToday);

	void addTaskRecordItem(QString taskName, QString strSaskStatus);

	// �����¼�;
	void paintEvent(QPaintEvent *event);
    // ���˫���¼�;
    void mouseDoubleClickEvent(QMouseEvent *event);

signals:
    // ���˫��ĳ������Item;
    void signalMouseDoubleClicked(QDate);

private:
	QDate m_itemDate;
	DayType m_dayType;

	QLabel* m_dayLabel;
	QListWidget* m_taskListWidget;
	QColor m_backgroundColor;
};
