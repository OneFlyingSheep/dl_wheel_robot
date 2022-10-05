#ifndef DL_WHEEL_ROBOT_ALARM_SEARCH_H
#define DL_WHEEL_ROBOT_ALARM_SEARCH_H

#include <QWidget>
#include <QLabel>
#include <QDateEdit>
#include <QCalendarWidget>
#include <QToolButton>
#include <QDate>

class CustomTableWidget;

/*********机器人告警查询页面**********/

class DLWheelRobotAlarmSearch : public QWidget
{
	Q_OBJECT

public:
	DLWheelRobotAlarmSearch(QWidget* parent = NULL);

    // 初始化控件;
	void initWidget();

private:
    // 初始化各个控件;
	void initTimeSearchWidget();
	void initTabelWidget();
    void initTableData();

private:
	QWidget* m_topBackWidget;
	QDateEdit* m_startTimeWidget;
	QDateEdit* m_endTimeWidget;
	QToolButton* m_pButtonSearch;

	CustomTableWidget* m_alarmInfoTableWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;
    // 当前table页数;
    int m_currentPageIndex;

    QDate m_startDate;
    QDate m_endData;
};

#endif
