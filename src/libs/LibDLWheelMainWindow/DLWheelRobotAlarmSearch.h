#ifndef DL_WHEEL_ROBOT_ALARM_SEARCH_H
#define DL_WHEEL_ROBOT_ALARM_SEARCH_H

#include <QWidget>
#include <QLabel>
#include <QDateEdit>
#include <QCalendarWidget>
#include <QToolButton>
#include <QDate>

class CustomTableWidget;

/*********�����˸澯��ѯҳ��**********/

class DLWheelRobotAlarmSearch : public QWidget
{
	Q_OBJECT

public:
	DLWheelRobotAlarmSearch(QWidget* parent = NULL);

    // ��ʼ���ؼ�;
	void initWidget();

private:
    // ��ʼ�������ؼ�;
	void initTimeSearchWidget();
	void initTabelWidget();
    void initTableData();

private:
	QWidget* m_topBackWidget;
	QDateEdit* m_startTimeWidget;
	QDateEdit* m_endTimeWidget;
	QToolButton* m_pButtonSearch;

	CustomTableWidget* m_alarmInfoTableWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;
    // ��ǰtableҳ��;
    int m_currentPageIndex;

    QDate m_startDate;
    QDate m_endData;
};

#endif
