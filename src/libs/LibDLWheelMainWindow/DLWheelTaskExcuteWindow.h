#ifndef DL_WHEEL_TASK_EXCUTE_H
#define DL_WHEEL_TASK_EXCUTE_H

#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QDateEdit>
#include <QTimeEdit>
#include <QCalendarWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QComboBox>
#include <QSpinBox>
#include <QListWidget>
#include "common/DLWheelRobotGlobalDef.hpp"

class DLWheelFixedTaskExcuteWindow : public BaseWidget
{
	Q_OBJECT

public:
	DLWheelFixedTaskExcuteWindow(QWidget* parent = NULL);
	~DLWheelFixedTaskExcuteWindow();

private:
	void initWidget();

signals:
	void signalOKButtonClikced(QString strDate, QTime strTime);

private:
	QDateEdit* m_dateEdit;
	QTimeEdit* m_timeEdit;
};

enum PeriodTaskType
{
	EveryDay = 0,
	EveryWeek,
	EveryMonth,
	FixedInterval,
	MultiDate,
};

class DLWheelPeriodTaskExcuteWindow : public BaseWidget
{
	Q_OBJECT

public:
	DLWheelPeriodTaskExcuteWindow(QWidget* parent = NULL);
	~DLWheelPeriodTaskExcuteWindow();

    // ��ȡÿ�������ʱ��;
    QTime getEveryDayTime();

    // ��ȡÿ����������ں�ʱ��;
    void getEveryWeekTime(QString& chooseWeeks, QTime& strTime);
    
    // ��ȡÿ������ľ���������ʱ��;
    void getEveryMonthTime(QString& day, QTime& strTime);

    // �̶��������Ŀ�ʼ���ڡ����������ʱ��;
    void getFixTime(QString& strDate, int& intervalDay, QTime& strTime);

    // ������ѡ��;
    void getMultiDateChoose(QString& strDateList, QTime& strTime);

private:
	void initEveryDayTask();
	void initEveryWeek();
	void initEveryMonth();
	void initFixedInterval();
	void initMultiDate();

	void initWidget();

signals:
	void signalOKButtonClikced(PeriodTaskType taskType);

private:
	// ÿ��;
	QTimeEdit* m_everyDayTimeEdit;

	// ÿ��;
	QTimeEdit* m_everyWeekTimeEdit;
	QList<QCheckBox*> m_weekChooseCheckBoxList;

	// ÿ��;
	QTimeEdit* m_everyMonthTimeEdit;
	QSpinBox* m_dayChooseSpinBox;

	// �̶����;
	QTimeEdit* m_fixedIntervalTimeEdit;
	QDateEdit* m_startDateEdit;
	QSpinBox* m_intervalDaysSpinBox;

	// ������;
	QTimeEdit* m_multiDateTimeEdit;
	QCalendarWidget* m_calendarWidget;
	QListWidget* m_multiDayChooseListWidget;
	QMap<QString, QString> m_dateChooseMap;

	// ��������ѡ��;
	QComboBox* m_taskTypeComboBox;

	QStackedWidget* m_stackedWidget;
};

// ����ִ����������;
class DLWheelImmediatelyExcuteWindow : public BaseWidget
{
    Q_OBJECT

public:
    DLWheelImmediatelyExcuteWindow(QWidget* parent = NULL);
    ~DLWheelImmediatelyExcuteWindow();

    void setImmediatelyTaskInfo(WheelRobotAssignTask task);

    void getImmediatelyExcuteInfo();

private:
    void initWidget();

signals:
    void signalOKButtonClikced(WheelRobotAssignTask);

private:
    QCheckBox* m_isOptimizeDeviceCheckBox;
    QCheckBox* m_isBreakCurrentTaskCheckBox;
    QComboBox* m_taskPriorityComboBox;
    QComboBox* m_taskOverActionComboBox;

    WheelRobotAssignTask m_task;
};

#endif
