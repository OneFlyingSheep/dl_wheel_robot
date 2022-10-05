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

    // 获取每天任务的时间;
    QTime getEveryDayTime();

    // 获取每周任务的星期和时间;
    void getEveryWeekTime(QString& chooseWeeks, QTime& strTime);
    
    // 获取每月任务的具体天数，时间;
    void getEveryMonthTime(QString& day, QTime& strTime);

    // 固定间隔任务的开始日期、间隔天数，时间;
    void getFixTime(QString& strDate, int& intervalDay, QTime& strTime);

    // 多日期选择;
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
	// 每天;
	QTimeEdit* m_everyDayTimeEdit;

	// 每周;
	QTimeEdit* m_everyWeekTimeEdit;
	QList<QCheckBox*> m_weekChooseCheckBoxList;

	// 每月;
	QTimeEdit* m_everyMonthTimeEdit;
	QSpinBox* m_dayChooseSpinBox;

	// 固定间隔;
	QTimeEdit* m_fixedIntervalTimeEdit;
	QDateEdit* m_startDateEdit;
	QSpinBox* m_intervalDaysSpinBox;

	// 多日期;
	QTimeEdit* m_multiDateTimeEdit;
	QCalendarWidget* m_calendarWidget;
	QListWidget* m_multiDayChooseListWidget;
	QMap<QString, QString> m_dateChooseMap;

	// 任务类型选择;
	QComboBox* m_taskTypeComboBox;

	QStackedWidget* m_stackedWidget;
};

// 立即执行条件窗口;
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
