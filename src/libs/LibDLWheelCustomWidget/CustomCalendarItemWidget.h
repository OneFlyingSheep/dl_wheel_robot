#pragma once

#include <QWidget>
#include <QDate>
#include <QLabel>
#include <QListWidget>
#include "common/DLWheelRobotGlobalDef.hpp"

// 日历Item类型;
enum DayType {
	DayType_MonthPre = 0,       // 上月剩余天数;
	DayType_MonthNext = 1,      // 下个月的天数;
	DayType_MonthCurrent = 2,   // 当月天数;
	DayType_WeekEnd = 3         // 周末;
};

// 任务执行结果类型;
enum TaskExecuteResultType
{
	ExecuteDone,				// 执行完成;
	MidWayAbort,				// 中途终止;
	Executing,					// 正在执行;
	WaitForExecute,				// 等待执行;
	TaskOverTime,				// 任务超期;
};

class CustomCalendarItemWidget : public QWidget
{
	Q_OBJECT

public:
	CustomCalendarItemWidget(QWidget *parent = NULL);
	~CustomCalendarItemWidget();

	// 对item设置日期，及类型;
	void setDate(QDate itemDate, DayType dayType, QList<WheelCalendarData> itemDataList = QList<WheelCalendarData>());

private:
	// 初始化Item;
	void initWidget();
	// 更新Item;
	void updateItem(bool isToday);

	void addTaskRecordItem(QString taskName, QString strSaskStatus);

	// 绘制事件;
	void paintEvent(QPaintEvent *event);
    // 鼠标双击事件;
    void mouseDoubleClickEvent(QMouseEvent *event);

signals:
    // 鼠标双击某个日期Item;
    void signalMouseDoubleClicked(QDate);

private:
	QDate m_itemDate;
	DayType m_dayType;

	QLabel* m_dayLabel;
	QListWidget* m_taskListWidget;
	QColor m_backgroundColor;
};
