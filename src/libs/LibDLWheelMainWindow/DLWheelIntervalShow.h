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

// �����Ϣ;
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

/***********�����ť�ؼ�************/

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

    // ���ð�ť�ı����ȼ�;
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

    // ��ȡ���id;
	QString getIntervalId()
	{
		return m_intervalId;
	}

    // ��ȡ�������;
    QString getIntervalName()
    {
        return this->text();
    }

private:
	DeviceAlarmLevel m_deviceAlarmLevel;
	QString m_intervalId;
};

/*********��ѹ�ȼ��ؼ�(����ÿһ����ѹ�ȼ�����ļ��)*********/

class IntervalWidget : public QWidget
{
	Q_OBJECT

public:
	IntervalWidget();
	// ��Ӱ�ť;
	void addIntervalButton(QString widgetText, QString strVoltageId, QList<WheelRobortEquiInterAlarmStruct> intervalInfoList);
	// ��ȡ��ǰwidget�ĸ߶�;
	int getCurrentWidgetHeight();

signals:
    // �����ť���;
	void signalButtonClicked(QString buttonId, QString buttonName);

private:
	QLabel * m_deviceAreaWidget;
	QWidget* m_buttonBackWidget;
	// ���浱ǰ�����е�checkBox;
	QList<QToolButton*> m_toolButtonList;
	// ���浱ǰ�����е�checkBox����Ϣ;
	QList<WheelRobortEquiInterAlarmStruct> m_intervalInfoList;
	// ���浱ǰwidget�ĸ߶�;
	int m_widgetHeight;
    // ��ǰ��ѹ�ȼ�Id;
    QString m_strVoltageId;
};

class TurnPageWidget;

/*******������������豸չʾ����**********/

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

    // ��ȡ��ǰ����ı�����ɫ;
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

	// ���ô��ڱ�������;
	void setTitleTest(const QString& text)
	{
		m_titleLabel->setText(text + "����澯һ��");
		m_titleLabel->setScaledContents(true);
	}

    // ��ӵ�ǰ����豸�б�;
	void addIntervalDeviceWidget(QList<WheelRobortDeviceFromIntervalStruct> deviceInfoList)
	{
		int intervalLineCount = 1;
		// ȷ����ǰ������;
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
    // ��ʼ�����������ؼ�;
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

    // ��ʼ���ؼ�;
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

    // ���Ʊ߿�;
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

/*********���չʾҳ��s**********/

class DLWheelIntervalShow : public QWidget
{
	Q_OBJECT

public:
	DLWheelIntervalShow();

    // ��ӵ�ѹ�ȼ��еļ��;
	void addIntervalWidget(QString widgetText, QString strVoltageId, QList<WheelRobortEquiInterAlarmStruct> intervalInfoList);

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
    // ��ʼ���ؼ�;
	void initTopWidget();
	void initCenterWidget();
	void initBottomWidget();

private:
	CustomButtonListWidget* m_topWidget;
	QLineEdit* m_searchLineEdit;
	QWidget* m_bottomBackWidget;
	QListWidget* m_centerBackwidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;
};

#endif
