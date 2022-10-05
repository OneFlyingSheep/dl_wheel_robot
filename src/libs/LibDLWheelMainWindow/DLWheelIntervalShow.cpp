#include "DLWheelIntervalShow.h"
#include <QDateTime>
#include <QApplication>
#include <QDesktopWidget>
#include <QButtonGroup>
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"

#define TIP_LABEL_WIDTH 150						// TipLable的宽度;
#define INTERVAL_COUNT_EACH_LINE 8				// 每行ToolButton数目;
#define TOOLBUTTON_WIDTH 130					// ToolButton宽度;
#define TOOLBUTTON_HEIGHT 30					// ToolButton高度;
#define INTERVAM_WIDGET_MARGIN 5				// 间隔控件的边距;
#define INTERVAM_WIDGET_SPACE 5					// 间隔控件的间距;

IntervalWidget::IntervalWidget()
{
	m_buttonBackWidget = new QWidget;
	m_buttonBackWidget->setObjectName("ButtonBackWidget");
	this->setStyleSheet("QLabel{background:rgb(159,168,218);color:white;font-size:24px;}\
							QWidget#ButtonBackWidget{background:rgb(159,168,218);}");
}

void IntervalWidget::addIntervalButton(QString widgetText, QString strVoltageId, QList<WheelRobortEquiInterAlarmStruct> intervalInfoList)
{
    m_strVoltageId = strVoltageId;

	QDesktopWidget* desktopWidget = QApplication::desktop();
	QRect screenRect = desktopWidget->screenGeometry();

	int widgetSpace = (screenRect.width() - TIP_LABEL_WIDTH - TOOLBUTTON_WIDTH * INTERVAL_COUNT_EACH_LINE - 100) / (INTERVAL_COUNT_EACH_LINE - 1);

	m_intervalInfoList = intervalInfoList;
	// 标题;
	m_deviceAreaWidget = new QLabel;
	m_deviceAreaWidget->setText(widgetText + ":");
	m_deviceAreaWidget->setAlignment(Qt::AlignCenter);

	int intervalLineCount = 1;
	// 确定当前总行数;
	qreal count = 1.0 * intervalInfoList.count() / INTERVAL_COUNT_EACH_LINE;
	if (count > int(count))
	{
		intervalLineCount = count + 1;
	}
	else
	{
		intervalLineCount = count;
	}
	
	QButtonGroup* toolButtonGroup = new QButtonGroup(this);
	QGridLayout* gToolButtonLayout = new QGridLayout(m_buttonBackWidget);
	int currentIndex = 0;
	for (int i = 0; i < intervalLineCount; i++)
	{
		for (int j = 0; j < INTERVAL_COUNT_EACH_LINE; j++)
		{
			if (currentIndex < intervalInfoList.count())
			{
				IntervalButton* pButton = new IntervalButton(intervalInfoList[currentIndex]);
				pButton->setFixedSize(QSize(TOOLBUTTON_WIDTH, TOOLBUTTON_HEIGHT));
				toolButtonGroup->addButton(pButton, currentIndex);
				gToolButtonLayout->addWidget(pButton, i, j);
			}
			else
			{
				gToolButtonLayout->addItem(new QSpacerItem(TOOLBUTTON_WIDTH, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), i, j);
				break;
			}
			currentIndex++;
		}
	}

	connect(toolButtonGroup, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked), this, [=](QAbstractButton* button) {
		IntervalButton* intervalButton = static_cast<IntervalButton*>(button);
		signalButtonClicked(intervalButton->getIntervalId(), intervalButton->getIntervalName());
	});

	gToolButtonLayout->setVerticalSpacing(INTERVAM_WIDGET_SPACE);
	gToolButtonLayout->setHorizontalSpacing(widgetSpace);
	gToolButtonLayout->setContentsMargins(5, INTERVAM_WIDGET_MARGIN, 45, INTERVAM_WIDGET_MARGIN);

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_deviceAreaWidget);
	hLayout->addWidget(m_buttonBackWidget);
	hLayout->setSpacing(10);
	hLayout->setContentsMargins(5, INTERVAM_WIDGET_MARGIN, 45, INTERVAM_WIDGET_MARGIN);

	m_widgetHeight = TOOLBUTTON_HEIGHT * intervalLineCount + INTERVAM_WIDGET_SPACE * (intervalLineCount - 1) + 2 * INTERVAM_WIDGET_MARGIN + 10;
	m_deviceAreaWidget->setFixedWidth(TIP_LABEL_WIDTH);
	this->setFixedHeight(m_widgetHeight);
}

int IntervalWidget::getCurrentWidgetHeight()
{
	return m_widgetHeight;
}

DLWheelIntervalShow::DLWheelIntervalShow()
	: m_isInitWidget(false)
{
	this->setStyleSheet("QListView::item{background:transparent;}\
							QWidget#BottomBackWidget{background:rgb(159,168,218);}");
}

void DLWheelIntervalShow::initTopWidget()
{
	m_topWidget = new CustomButtonListWidget;
	m_searchLineEdit = new QLineEdit;
	m_searchLineEdit->setFixedSize(QSize(200, 22));

	m_topWidget->addWidget(m_searchLineEdit);
	m_topWidget->addToolButton(0, "查询", ":/Resources/Common/image/Search.png");
	m_topWidget->addWidgetFinished();
}

void DLWheelIntervalShow::addIntervalWidget(QString widgetText, QString strVoltageId, QList<WheelRobortEquiInterAlarmStruct> intervalInfoList)
{
	IntervalWidget* intervalWidget = new IntervalWidget;
	intervalWidget->addIntervalButton(widgetText, strVoltageId, intervalInfoList);
	int widgetHeight = intervalWidget->getCurrentWidgetHeight();
	connect(intervalWidget, &IntervalWidget::signalButtonClicked, this, [=](QString intervalId, QString intervalName) {
        QList<WheelRobortDeviceFromIntervalStruct> deviceDataList = WHEEL_PATROL_RESULT.getWheelDeviceFromIntervalShow(intervalId);
		DeviceShowWidget* deviceShowWidget = new DeviceShowWidget;
        deviceShowWidget->setTitleTest(intervalName);
        deviceShowWidget->addIntervalDeviceWidget(deviceDataList);
		deviceShowWidget->show();
        deviceShowWidget->activateWindow();
	});

	QDesktopWidget* desktopWidget = QApplication::desktop();
	QRect screenRect = desktopWidget->screenGeometry();

	QListWidgetItem* item = new QListWidgetItem;
	item->setSizeHint(QSize(screenRect.width(), widgetHeight));
	m_centerBackwidget->addItem(item);
	m_centerBackwidget->setItemWidget(item, intervalWidget);
}

void DLWheelIntervalShow::initCenterWidget()
{
	m_centerBackwidget = new QListWidget;

    QList<WheelRobortIntervalShowStruct> intervalDataList = WHEEL_PATROL_RESULT.getWheelIntervalShow();

    for (int i = 0; i < intervalDataList.count(); i++)
    {
        addIntervalWidget(intervalDataList[i].voltage_level_name, intervalDataList[i].voltage_level_id, intervalDataList[i].equipmentIntervalStru);
    }
}

void DLWheelIntervalShow::initBottomWidget()
{
	m_bottomBackWidget = new QWidget;
	m_bottomBackWidget->setObjectName("BottomBackWidget");
	m_bottomBackWidget->setFixedHeight(30);

	QLabel* tipLabel = new QLabel;
	tipLabel->setText("提示:");

	QHBoxLayout* hBottomLayout = new QHBoxLayout(m_bottomBackWidget);
	hBottomLayout->addWidget(tipLabel);
	ColorLabel* colorLabel[6];
	for (int i = 0; i < 6; i++)
	{
		colorLabel[i] = new ColorLabel;
		hBottomLayout->addWidget(colorLabel[i]);
	}
	colorLabel[0]->setText("正常状态");
	colorLabel[0]->setColor(QColor(0, 128, 0));
	colorLabel[1]->setText("预警");
	colorLabel[1]->setColor(Qt::blue);
	colorLabel[2]->setText("一般告警");
	colorLabel[2]->setColor(QColor(255, 255, 0));
	colorLabel[3]->setText("严重警告");
	colorLabel[3]->setColor(QColor(255, 128, 10));
	colorLabel[4]->setText("危急告警");
	colorLabel[4]->setColor(Qt::red);
	colorLabel[5]->setText("未识别异常");
	colorLabel[5]->setColor(Qt::gray);
	hBottomLayout->addStretch();
	hBottomLayout->setSpacing(5);
	hBottomLayout->setContentsMargins(5, 0, 0, 0);
}

void DLWheelIntervalShow::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initTopWidget();
	initCenterWidget();
	initBottomWidget();

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_topWidget);
	vMainLayout->addWidget(m_centerBackwidget);
	vMainLayout->addWidget(m_bottomBackWidget);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(0);
}