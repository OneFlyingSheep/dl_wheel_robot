#ifndef CUSTOM_CALENDAR_WIDGET_H
#define CUSTOM_CALENDAR_WIDGET_H

#include <QWidget>
#include <QDate>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "CustomCalendarInfo.h"
#include "CustomCalendarItemWidget.h"

class QLabel;
class QComboBox;

class CustomCalendarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CustomCalendarWidget(QWidget *parent = 0);
    ~CustomCalendarWidget();

	// 刷新当前日历;
	void refreshData(QMap<int, QList<WheelCalendarData>> dataMap);

	// 获取当前日期;
	QDate getCurrentDate();

private:
	// 顶部星期名称;
    QList<QLabel *> labWeeks;				
	// 日期元素;
    QList<CustomCalendarItemWidget *> dayItems;
	// 当前日期;
    QDate m_date;                        

private slots:
    void initWidget();
    void initDate();
    void dayChanged(const QDate &date);
    void dateChanged(int year, int month, int day);

public:
    QDate getDate()                     const;

public Q_SLOTS:
    //设置星期名称格式;
    void setWeekNameFormat();

    //设置日期;
    void setDate(const QDate &date, QMap<int, QList<WheelCalendarData>> dataMap);

Q_SIGNALS:
    void clicked(const QDate &date);
    void selectionChanged();

    // 鼠标双击日历某个日期Item;
    void signalMouseDoubleClickeDateItem(QDate itemDate);

private:
	QMap<int, QList<WheelCalendarData>> m_dataMap;
};

#endif // CUSTOM_CALENDAR_WIDGET_H
