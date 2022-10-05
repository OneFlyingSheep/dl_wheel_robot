#pragma execution_character_set("utf-8")

#include "CustomCalendarWidget.h"
#include "qdatetime.h"
#include "qlayout.h"
#include "qlabel.h"
#include "qpushbutton.h"
#include "qtoolbutton.h"
#include "qcombobox.h"
#include "qdebug.h"

CustomCalendarWidget::CustomCalendarWidget(QWidget *parent) : QWidget(parent)
{
    initWidget();
	this->setStyleSheet(QString("QLabel#WeekLabel{background:rgb(86,119,252);color:white;}\
						QWidget#widgetBody{border:1px solid rgb(180, 180, 180);}"));
}

CustomCalendarWidget::~CustomCalendarWidget()
{
}

void CustomCalendarWidget::refreshData(QMap<int, QList<WheelCalendarData>> dataMap)
{
	m_dataMap = dataMap;
	initDate();
}

QDate CustomCalendarWidget::getCurrentDate()
{
	return m_date;
}

void CustomCalendarWidget::initWidget()
{
    setObjectName("CustomCalendarWidget");

    //星期widget;
    QWidget *widgetWeek = new QWidget;
	widgetWeek->setFixedHeight(30);
    widgetWeek->setObjectName("widgetWeek");
    widgetWeek->setMinimumHeight(30);

    //星期布局;
    QHBoxLayout *layoutWeek = new QHBoxLayout(widgetWeek);
    layoutWeek->setMargin(0);
    layoutWeek->setSpacing(0);

    for (int i = 0; i < 7; i++) {
        QLabel *lab = new QLabel;
		lab->setObjectName("WeekLabel");
        lab->setAlignment(Qt::AlignCenter);
        layoutWeek->addWidget(lab);
        labWeeks.append(lab);
    }

    setWeekNameFormat();

    //日期标签widget;
    QWidget *widgetBody = new QWidget;
    widgetBody->setObjectName("widgetBody");

    //日期标签布局;
    QGridLayout *layoutBody = new QGridLayout(widgetBody);
    layoutBody->setMargin(1);
    layoutBody->setHorizontalSpacing(0);
    layoutBody->setVerticalSpacing(0);

    //逐个添加日标签;
    for (int i = 0; i < 42; i++) {
		CustomCalendarItemWidget *lab = new CustomCalendarItemWidget;
        connect(lab, &CustomCalendarItemWidget::signalMouseDoubleClicked, this, &CustomCalendarWidget::signalMouseDoubleClickeDateItem);
		layoutBody->addWidget(lab, i / 7, i % 7);
        dayItems.append(lab);
    }

    //主布局;
    QVBoxLayout *verLayoutCalendar = new QVBoxLayout(this);
    verLayoutCalendar->setMargin(0);
    verLayoutCalendar->setSpacing(0);
    verLayoutCalendar->addWidget(widgetWeek);
    verLayoutCalendar->addWidget(widgetBody);
}

//初始化日期面板;
void CustomCalendarWidget::initDate()
{
    int year = m_date.year();
    int month = m_date.month();
    int day = m_date.day();

    //首先判断当前月的第一天是星期几;
    int week = CustomCalendarInfo::Instance()->getFirstDayOfWeek(year, month);
    //当前月天数;
    int countDay = CustomCalendarInfo::Instance()->getMonthDays(year, month);
    //上月天数;
    int countDayPre = CustomCalendarInfo::Instance()->getMonthDays(1 == month ? year - 1 : year, 1 == month ? 12 : month - 1);

    //如果上月天数上月刚好一周则另外处理;
    int startPre, endPre, startNext, endNext, index, tempYear, tempMonth, tempDay;
    if (0 == week) {
        startPre = 0;
        endPre = 7;
        startNext = 0;
        endNext = 42 - (countDay + 7);
    } else {
        startPre = 0;
        endPre = week;
        startNext = week + countDay;
        endNext = 42;
    }

    //纠正1月份前面部分偏差,1月份前面部分是上一年12月份;
    tempYear = year;
    tempMonth = month - 1;
    if (tempMonth < 1) {
        tempYear--;
        tempMonth = 12;
    }

    //显示上月天数;
    for (int i = startPre; i < endPre; i++) {
        index = i;
        tempDay = countDayPre - endPre + i + 1;

        QDate date(tempYear, tempMonth, tempDay);
        QString lunar = CustomCalendarInfo::Instance()->getLunarDay(tempYear, tempMonth, tempDay);
        dayItems.at(index)->setDate(date, DayType::DayType_MonthPre);
    }

    //纠正12月份后面部分偏差,12月份后面部分是下一年1月份;
    tempYear = year;
    tempMonth = month + 1;
    if (tempMonth > 12) {
        tempYear++;
        tempMonth = 1;
    }

    //显示下月天数;
    for (int i = startNext; i < endNext; i++) {
        index = 42 - endNext + i;
        tempDay = i - startNext + 1;

        QDate date(tempYear, tempMonth, tempDay);
        QString lunar = CustomCalendarInfo::Instance()->getLunarDay(tempYear, tempMonth, tempDay);
        dayItems.at(index)->setDate(date, DayType::DayType_MonthNext);
    }

    //重新置为当前年月;
    tempYear = year;
    tempMonth = month;

    //显示当前月;
    for (int i = week; i < (countDay + week); i++) {
        index = (0 == week ? (i + 7) : i);
        tempDay = i - week + 1;

        QDate date(tempYear, tempMonth, tempDay);
        QString lunar = CustomCalendarInfo::Instance()->getLunarDay(tempYear, tempMonth, tempDay);

		QList<WheelCalendarData> itemDataList;
		if (m_dataMap.contains(tempDay))
		{
			itemDataList = m_dataMap.value(tempDay);
		}

        if (0 == (i % 7) || 6 == (i % 7)) 
		{
            dayItems.at(index)->setDate(date, DayType::DayType_WeekEnd, itemDataList);
        } 
		else 
		{
            dayItems.at(index)->setDate(date, DayType::DayType_MonthCurrent, itemDataList);
        }
    }

    dayChanged(m_date);
}

void CustomCalendarWidget::dayChanged(const QDate &date)
{
    //计算星期几,当前天对应标签索引=日期+星期几-1;
    int year = date.year();
    int month = date.month();
    int day = date.day();
    int week = CustomCalendarInfo::Instance()->getFirstDayOfWeek(year, month);
    //qDebug() << QString("%1-%2-%3").arg(year).arg(month).arg(day);

    //选中当前日期,其他日期恢复,这里还有优化空间,比方说类似单选框机制;
    for (int i = 0; i < 42; i++) {
        //当月第一天是星期天要另外计算;
        int index = day + week - 1;
        if (week == 0) {
            index = day + 6;
        }

    //    dayItems.at(i)->setSelect(i == index);
    }

    //发送日期单击信号;
    emit clicked(date);
    //发送日期更新信号;
    emit selectionChanged();
}

void CustomCalendarWidget::dateChanged(int year, int month, int day)
{
    //如果原有天大于28则设置为1,防止出错;
	m_date.setDate(year, month, day > 28 ? 1 : day);
    initDate();
}

QDate CustomCalendarWidget::getDate() const
{
    return m_date;
}

void CustomCalendarWidget::setWeekNameFormat()
{
    QStringList listWeek;
	listWeek << "星期天" << "星期一" << "星期二" << "星期三" << "星期四" << "星期五" << "星期六";
// 		listWeek << "日" << "一" << "二" << "三" << "四" << "五" << "六";
// 		listWeek << "周日" << "周一" << "周二" << "周三" << "周四" << "周五" << "周六";
// 		listWeek << "Sun" << "Mon" << "Tue" << "Wed" << "Thu" << "Fri" << "Sat";

    //逐个添加日期文字;
    for (int i = 0; i < 7; i++) 
	{
        labWeeks.at(i)->setText(listWeek.at(i));
    }
}

void CustomCalendarWidget::setDate(const QDate &date, QMap<int, QList<WheelCalendarData>> dataMap)
{
	m_date = date;
	m_dataMap = dataMap;
    initDate();
}
