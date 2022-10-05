#include "CustomCalendarItemWidget.h"
#include <QVBoxLayout>
#include <QPainter>
#include <QMouseEvent>

#pragma execution_character_set("utf-8")

CustomCalendarItemWidget::CustomCalendarItemWidget(QWidget *parent)
	: QWidget(parent)
{
	initWidget();
	this->setStyleSheet("QLabel{font-size:14px;}\
						QScrollBar:vertical{background:white;width:6px;margin:0px;}\
						QScrollBar::handle:vertical{border-radius:3px;background:lightgray;border-width:2 0 2 0;}\
						QScrollBar::sub-line:vertical{height:0px;}\
						QScrollBar::add-line:vertical{height:0px;}");
}

CustomCalendarItemWidget::~CustomCalendarItemWidget()
{
}

void CustomCalendarItemWidget::initWidget()
{
	m_dayLabel = new QLabel;
	m_dayLabel->setAlignment(Qt::AlignRight);

	m_taskListWidget = new QListWidget;
	m_taskListWidget->setStyleSheet("QListWidget{border:none;background:transparent;}\
										 QListView::item:selected:!active{background:transparent;}");

	QVBoxLayout* vItemLayout = new QVBoxLayout(this);
	vItemLayout->addWidget(m_dayLabel);
	vItemLayout->addWidget(m_taskListWidget);
	vItemLayout->setSpacing(10);
	vItemLayout->setMargin(10);
}

void CustomCalendarItemWidget::setDate(QDate itemDate, DayType dayType, QList<WheelCalendarData> itemDataList)
{
	m_itemDate = itemDate;
	m_dayType = dayType;

	bool isToday = m_itemDate == QDate::currentDate();
	if (isToday)
	{
		m_dayLabel->setText("Today");
	}
	else
	{
		m_dayLabel->setText(QString::number(itemDate.day()));
	}

	if (dayType == DayType_MonthCurrent || dayType == DayType_WeekEnd)
	{
		m_taskListWidget->clear();
		// 如果是本月需要获取当前是否有任务记录;
		if (!itemDataList.isEmpty())
		{
			for (int i = 0; i < itemDataList.count(); i++)
			{
				addTaskRecordItem(itemDataList.at(i).task_name, itemDataList.at(i).task_status);
			}
		}
	}
	else
	{
		m_taskListWidget->clear();
	}

	updateItem(isToday);
}

void CustomCalendarItemWidget::updateItem(bool isToday)
{
	if (isToday)
	{
		m_dayLabel->setStyleSheet("color:rgb(220, 185, 100);");
		m_backgroundColor = QColor(255, 243, 190);
	}
	else
	{
		switch (m_dayType)
		{
		case DayType_MonthPre:
		case DayType_MonthNext:
		{
			m_dayLabel->setStyleSheet("color:gray;");
			m_backgroundColor = QColor(240, 240, 240);
		}
		break;
		case DayType_WeekEnd:
		{
			m_dayLabel->setStyleSheet("color:rgb(180, 205, 255);");
			m_backgroundColor = QColor(240, 250, 250);
		}
		break;
		case DayType_MonthCurrent:
		{
			m_dayLabel->setStyleSheet("color:rgb(180, 205, 255);");
			m_backgroundColor = Qt::white;
		}
		break;
		default:
			break;
		}
	}
	
	update();
}

void CustomCalendarItemWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), m_backgroundColor);
	painter.setPen(QColor(225, 235, 255));
	painter.drawRect(QRect(0, 0, this->width() - 1, this->height() - 1));
}

void CustomCalendarItemWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        if (DayType_MonthCurrent == m_dayType || DayType_WeekEnd == m_dayType)
        {
            emit signalMouseDoubleClicked(m_itemDate);
        }
    }
    
    return __super::mouseDoubleClickEvent(event);
}

void CustomCalendarItemWidget::addTaskRecordItem(QString taskName, QString strSaskStatus)
{
	QLabel* itemLabel = new QLabel(taskName);
	itemLabel->setToolTip(taskName);
	QListWidgetItem* item = new QListWidgetItem(m_taskListWidget);
	item->setToolTip(taskName);
	m_taskListWidget->addItem(item);
	m_taskListWidget->setItemWidget(item, itemLabel);

	if (!strSaskStatus.compare("已执行"))
	{
		itemLabel->setStyleSheet("QLabel{color:green;}");
	}
	else if (!strSaskStatus.compare("中止"))
	{
		itemLabel->setStyleSheet("QLabel{color:rgb(140,70,20);}");
	}
	else if (!strSaskStatus.compare("正在执行"))
	{
		itemLabel->setStyleSheet("QLabel{color:red;}");
	}
	else if (!strSaskStatus.compare("等待执行"))
	{
		itemLabel->setStyleSheet("QLabel{color:blue;}");
	}
	else if (!strSaskStatus.compare("任务超期"))
	{
		itemLabel->setStyleSheet("QLabel{color:yellow;}");
	}
}