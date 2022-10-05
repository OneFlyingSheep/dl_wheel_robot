#ifndef DL_WHEELINTERVAL_SHOW_H
#define DL_WHEELINTERVAL_SHOW_H

#include <QWidget>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QListWidget>
#include <QPainter>
#include "DLWheelMainLineShow.h"
#include "LibDLWheelCustomWidget/TurnPageWidget.h"
#include "common/DLWheelRobotGlobalDef.hpp"

#pragma execution_character_set("utf-8")

#define TABLE_PER_PAGE_COUNT 84

// 间隔信息;
struct IntervalInfo
{
	QString strIntervalName;
	QString strIntervalId;
	DeviceAlarmLevel eAlarmLevel;
	IntervalInfo(QString intervalName, QString intervalId, DeviceAlarmLevel level)
	{
		this->strIntervalName = intervalName;
		this->strIntervalId = intervalId;
		this->eAlarmLevel = level;
	}
};

class CustomButtonListWidget;

/***********间隔按钮控件************/

class IntervalButton : public QToolButton
{
public:
	IntervalButton(WheelRobortEquiInterAlarmStruct intervalInfo)
	{
		this->setText(intervalInfo.equipment_interval_name);
		m_intervalId = intervalInfo.equipment_interval_uuid;
		m_deviceAlarmLevel = intervalInfo.IntervalAlarmLevel;
		setIntervalButtonLevel();
	}

    // 设置按钮的报警等级;
	void setIntervalButtonLevel()
	{
		switch (m_deviceAlarmLevel)
		{
		case Alarm_Normal:
			this->setStyleSheet("background:rgb(0, 128, 0);border-radius:3px;color:white;font-size:14px;");
			break;
		case Alarm_Waring:
			this->setStyleSheet("background:rgb(0, 0, 255);border-radius:3px;color:white;font-size:14px;");
			break;
		case Alarm_Common:
			this->setStyleSheet("background:rgb(255, 255, 0);border-radius:3px;color:white;font-size:14px;");
			break;
		case Alarm_Serious:
			this->setStyleSheet("background:rgb(255, 128, 20);border-radius:3px;color:white;font-size:14px;");
			break;
		case Alarm_Dangerous:
			this->setStyleSheet("background:rgb(255, 0, 0);border-radius:3px;color:white;font-size:14px;");
			break;
		case Alarm_NoIdentifyAbnormal:
			this->setStyleSheet("background:gray;border-radius:3px;color:white;font-size:14px;");
			break;
		default:
			break;
		}
	}

    // 获取间隔id;
	QString getIntervalId()
	{
		return m_intervalId;
	}

    // 获取间隔名称;
    QString getIntervalName()
    {
        return this->text();
    }

private:
	DeviceAlarmLevel m_deviceAlarmLevel;
	QString m_intervalId;
};

/*********电压等级控件(包含每一个电压等级后面的间隔)*********/

class IntervalWidget : public QWidget
{
	Q_OBJECT

public:
	IntervalWidget();
	// 添加按钮;
	void addIntervalButton(QString widgetText, QString strVoltageId, QList<WheelRobortEquiInterAlarmStruct> intervalInfoList);
	// 获取当前widget的高度;
	int getCurrentWidgetHeight();

signals:
    // 间隔按钮点击;
	void signalButtonClicked(QString buttonId, QString buttonName);

private:
	QLabel * m_deviceAreaWidget;
	QWidget* m_buttonBackWidget;
	// 保存当前项所有的checkBox;
	QList<QToolButton*> m_toolButtonList;
	// 保存当前项所有的checkBox的信息;
	QList<WheelRobortEquiInterAlarmStruct> m_intervalInfoList;
	// 保存当前widget的高度;
	int m_widgetHeight;
    // 当前电压等级Id;
    QString m_strVoltageId;
};

class TurnPageWidget;

/*******间隔所包含的设备展示窗口**********/

class DeviceShowWidget : public QWidget
{
public:
	DeviceShowWidget()
	{
		initWidget();
		this->setFixedSize(QSize(900, 500));
		this->setStyleSheet("QWidget#TopBackWidget{background:transparent;}\
								QWidget#CenterWidget{background:white;}\
								QWidget#ColorWidget{border:1px solid lightgray;}");

		this->setAttribute(Qt::WA_DeleteOnClose);
		this->setAttribute(Qt::WA_TranslucentBackground);
		this->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
		this->setWindowModality(Qt::ApplicationModal);
	}

    // 获取当前间隔的报警颜色;
    QColor getAlarmLevelColor(DeviceAlarmLevel alarmLevel)
    {
        QColor itemColor;
        switch (alarmLevel)
        {
        case Alarm_Normal:
            itemColor = QColor(0, 128, 0);
            break;
        case Alarm_Waring:
            itemColor = QColor(0, 0, 255);
            break;
        case Alarm_Common:
            itemColor = QColor(255, 255, 0);
            break;
        case Alarm_Serious:
            itemColor = QColor(255, 128, 20);
            break;
        case Alarm_Dangerous:
            itemColor = QColor(255, 0, 0);
            break;
        case Alarm_NoIdentifyAbnormal:
            itemColor = Qt::gray;
            break;
        default:
            break;
        }

        return itemColor;
    }

	// 设置窗口标题文字;
	void setTitleTest(const QString& text)
	{
		m_titleLabel->setText(text + "间隔告警一览");
		m_titleLabel->setScaledContents(true);
	}

    // 添加当前间隔设备列表;
	void addIntervalDeviceWidget(QList<WheelRobortDeviceFromIntervalStruct> deviceInfoList)
	{
		int intervalLineCount = 1;
		// 确定当前总行数;
		qreal count = 1.0 * deviceInfoList.count() / 6;
		if (count > int(count))
		{
			intervalLineCount = count + 1;
		}
		else
		{
			intervalLineCount = count;
		}
		QGridLayout* gToolButtonLayout = new QGridLayout(m_centerWidget);
		int currentIndex = 0;
		int i = 0;
		for (; i < intervalLineCount; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				if (currentIndex < deviceInfoList.count())
				{
					QWidget* colorWidget = new QWidget;
					colorWidget->setObjectName("ColorWidget");
					colorWidget->setFixedSize(QSize(147, 30));
					ColorLabel* colorLabel = new ColorLabel;
					colorLabel->setText(deviceInfoList[currentIndex].device_name);
					colorLabel->setColor(getAlarmLevelColor(deviceInfoList[currentIndex].DeviceAlarmLevel));
					QHBoxLayout* hLayout = new QHBoxLayout(colorWidget);
					hLayout->addStretch();
					hLayout->addWidget(colorLabel);
					hLayout->addStretch();
					hLayout->setMargin(0);
					gToolButtonLayout->addWidget(colorWidget, i, j);
				}
				else
				{
					gToolButtonLayout->addItem(new QSpacerItem(40, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), i, j);
					break;
				}
				currentIndex++;
			}
		}
		gToolButtonLayout->addItem(new QSpacerItem(0, 40, QSizePolicy::Minimum, QSizePolicy::Expanding), i, 0);
		gToolButtonLayout->setSpacing(0);
		gToolButtonLayout->setMargin(0);

        int totalCount = deviceInfoList.count();
        int totalPage = totalCount / TABLE_PER_PAGE_COUNT;
        if (totalCount > totalPage * TABLE_PER_PAGE_COUNT)
        {
            totalPage++;
        }
        m_turnPageWidget->setTurnPageInfo(1, totalPage, totalCount, TABLE_PER_PAGE_COUNT);
	}

private:
    // 初始化顶部输入框控件;
	void initTopWidget()
	{
		m_titleLabel = new QLabel;
		m_titleLabel->setStyleSheet("color:rgb(63,81,181);font-weight:bold;font-size:14px;");

		QIcon icon = style()->standardIcon(QStyle::SP_TitleBarCloseButton);
		QToolButton* pButtonClose = new QToolButton;
		pButtonClose->setIcon(icon);
		pButtonClose->setIconSize(QSize(12, 12));
		pButtonClose->setFixedSize(QSize(14, 14));
		pButtonClose->setStyleSheet("border:none;background:rgb(220, 235, 235);");
		connect(pButtonClose, &QToolButton::clicked, this, [=] {
			close();
		});

		m_topBackWidget = new QWidget;
		m_topBackWidget->setObjectName("TopBackWidget");
		m_topBackWidget->setFixedHeight(25);
		QHBoxLayout* hTopLayout = new QHBoxLayout(m_topBackWidget);
		hTopLayout->addStretch();
		hTopLayout->addWidget(m_titleLabel);
		hTopLayout->addStretch();
		hTopLayout->addWidget(pButtonClose);
		hTopLayout->setMargin(0);
	}

    // 初始化控件;
	void initWidget()
	{
		initTopWidget();
		m_centerWidget = new QWidget;
		m_centerWidget->setObjectName("CenterWidget");
		m_turnPageWidget = new TurnPageWidget;

		QVBoxLayout* vMainLayout = new QVBoxLayout(this);
		vMainLayout->addWidget(m_topBackWidget);
		vMainLayout->addWidget(m_centerWidget);
		vMainLayout->addWidget(m_turnPageWidget);
		vMainLayout->setSpacing(0);
		vMainLayout->setContentsMargins(10, 0, 10, 10);
	}

    // 绘制边框;
	void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);
		painter.setPen(Qt::lightGray);
		painter.setBrush(QColor(200, 224, 221));
		painter.drawRoundedRect(QRect(0, 0, this->width() - 1, this->height() - 1), 3, 3);
	}

private:
	QLabel* m_titleLabel;
	QWidget* m_topBackWidget;
	QWidget* m_centerWidget;
	TurnPageWidget* m_turnPageWidget;
};

/*********间隔展示页面s**********/

class DLWheelIntervalShow : public QWidget
{
	Q_OBJECT

public:
	DLWheelIntervalShow();

    // 添加电压等级中的间隔;
	void addIntervalWidget(QString widgetText, QString strVoltageId, QList<WheelRobortEquiInterAlarmStruct> intervalInfoList);

	// 窗口控件初始化;
	void initWidget();

private:
    // 初始化控件;
	void initTopWidget();
	void initCenterWidget();
	void initBottomWidget();

private:
	CustomButtonListWidget* m_topWidget;
	QLineEdit* m_searchLineEdit;
	QWidget* m_bottomBackWidget;
	QListWidget* m_centerBackwidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;
};

#endif
