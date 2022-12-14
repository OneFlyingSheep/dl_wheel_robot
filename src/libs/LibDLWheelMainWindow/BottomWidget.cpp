#include "BottomWidget.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QScrollBar>

#define ITEM_WIDTH 150				// 底部item宽度;
#define ITEM_SPACE 5				// 底部item之间间隔;

BottomWidgetItem::BottomWidgetItem(bool isMainItem /* = false */)
	: m_isEnter(false)
	, m_isChecked(false)
	, m_isMainItem(isMainItem)
{
	m_labelIcon = new QLabel;
	m_labelIcon->setFixedSize(QSize(20, 20));

	m_labelText = new QLabel;
	m_labelText->setAlignment(Qt::AlignCenter);

	QIcon icon = style()->standardIcon(QStyle::SP_TitleBarCloseButton);
	m_pButtonClose = new QToolButton;
	m_pButtonClose->setIcon(icon);
	m_pButtonClose->setIconSize(QSize(12, 12));
	m_pButtonClose->setFixedSize(QSize(14, 14));
	m_pButtonClose->setStyleSheet("border:none;");

	connect(m_pButtonClose, &QToolButton::clicked, this, [=] {
		emit signalItemClose(m_itemType);
	});

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_labelIcon);
	hLayout->addWidget(m_labelText);
	hLayout->addWidget(m_pButtonClose);
	hLayout->setSpacing(3);
	hLayout->setMargin(3);

	// 如果是MianItem;
	if (m_isMainItem)
	{
		QIcon icon(":/Resources/DLWheelMainWindow/Image/HomePageButton.png");
		m_labelIcon->setPixmap(icon.pixmap(m_labelIcon->size()));
		m_pButtonClose->setVisible(false);
	}
	else
	{
		QIcon icon(":/Resources/DLWheelMainWindow/Image/BottomItemIcon.png");
		m_labelIcon->setPixmap(icon.pixmap(m_labelIcon->size()));
	}

	this->setFixedSize(QSize(150, 32));
}

void BottomWidgetItem::setItemType(SystemGuideMenuItemType itemType)
{
	m_itemType = itemType;
	m_labelText->setText(getItemText(itemType));
}

QString BottomWidgetItem::getItemText(SystemGuideMenuItemType itemType)
{
	QString itemText;
	switch (itemType)
	{
	case Menu_RobotManage:
		itemText = QString("机器人管理");
		break;
	case Menu_RobotControl:
		itemText = QString("机器人控制");
		break;
	case Menu_PatrolManage:
		itemText = QString("巡检监控");
		break;
	case Menu_All_Patrol:
		itemText = QString("全面巡检");
		break;
	case Menu_Routine_Patrol:
		itemText = QString("例行巡检");
		break;
	case Menu_InfraredCalculateTmp:
		itemText = QString("红外测温");
		break;
	case Menu_OilTableRecord:
		itemText = QString("油位、油温表抄录");
		break;
	case Menu_ArresterTableRead:
		itemText = QString("避雷器表计读取");
		break;
	case Menu_SF6PressureRecord:
		itemText = QString("SF6压力抄录");
		break;
	case Menu_HydraumaticTableRecord:
		itemText = QString("液压表抄录");
		break;
	case Menu_PosStateRecognition:
		itemText = QString("位置状态识别");
		break;
	case Menu_BadWeatherPatrol:
		itemText = QString("恶劣天气特巡");
		break;
	case Menu_DefectTrack:
		itemText = QString("缺陷跟踪");
		break;
	case Menu_DistanceStateAffirm:
		itemText = QString("远方状态确认");
		break;
	case Menu_DistanceAbnormalAlarmAffirm:
		itemText = QString("远方异常感觉确认");
		break;
	case Menu_SecurityLinkage:
		itemText = QString("安防联动");
		break;
	case Menu_AssistAccidentDeal:
		itemText = QString("协助应急事故处理");
		break;
	case Menu_CustomTask:
		itemText = QString("自定义任务");
		break;
	case Menu_MapChoosePoint:
		itemText = QString("地图选点");
		break;
	case Menu_TaskShow:
		itemText = QString("任务展示");
		break;
	case Menu_DeviceAlarmSearchAffirm:
		itemText = QString("设备告警查询确认");
		break;
	case Menu_MainLineShow:
		itemText = QString("主接线展示");
		break;
	case Menu_IntervalShow:
		itemText = QString("间隔展示");
		break;
	case Menu_PatrolResultBrowse:
		itemText = QString("巡检结果浏览");
		break;
	case Menu_PatrolReportCreate:
		itemText = QString("巡检报告生成");
		break;
	case Menu_CompareAnalysis:
		itemText = QString("对比分析");
		break;
	case Menu_CreateReportForm:
		itemText = QString("生成报表");
		break;
	case Menu_AlarmThresholdSet:
		itemText = QString("告警阈值设置");
		break;
	case Menu_AlarmInfoSubscribeSet:
		itemText = QString("告警信息订阅设置");
		break;
	case Menu_RightManage:
		itemText = QString("权限管理");
		break;
	case Menu_TypicalPatrolPointMaintain:
		itemText = QString("典型巡检点库维护");
		break;
	case Menu_PatrolPointPosSet:
		itemText = QString("巡检点位设置");
		break;
	case Menu_ServiceAreaSet:
		itemText = QString("检修区域设置");
		break;
	case Menu_PatrolMapMaintain:
		itemText = QString("巡检地图维护");
		break;
	case Menu_SoftSet:
		itemText = QString("软件设置");
		break;
	case Menu_RobotSet:
		itemText = QString("机器人设置");
		break;
	case Menu_RobotStateShow:
		itemText = QString("机器人状态显示");
		break;
	case Menu_RobotAlarmSearch:
		itemText = QString("机器人告警查询");
		break;
	case Menu_RecognizeAbnormalPointSearch:
		itemText = QString("识别异常点位查询");
		break;
	case Menu_MainPage:
		itemText = QString("变电站智能机器人巡检");
		break;
    case Menu_CompareDetection:
        itemText = QString("异物检测");
        break;
	default:
		break;
	}

	return itemText;
}

void BottomWidgetItem::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	QColor fillColor = QColor(55, 143, 254);
	if (m_isChecked)
	{
		// 反走样;
		painter.setRenderHint(QPainter::Antialiasing, true);
		// 设置渐变色;
		QLinearGradient linear(QPointF(this->width() / 2, 0), QPointF(this->width() / 2, this->height()));
		linear.setColorAt(0, QColor(55, 143, 254));
		linear.setColorAt(0.8, QColor(175, 191, 255));
		linear.setColorAt(1, Qt::white);
		// 设置显示模式;
		linear.setSpread(QGradient::PadSpread);

		painter.fillRect(this->rect(), linear);
		return;
	}
	else if (m_isEnter)
	{
		fillColor = QColor(175, 191, 255);
	}
	painter.fillRect(this->rect(), fillColor);
	painter.setPen(QPen(QColor(86, 119, 252)));
	painter.drawRect(0, 0, this->width() - 1, this->height() - 1);
}

void BottomWidgetItem::enterEvent(QEvent *event)
{
    m_isEnter = true;
    update();
}

void BottomWidgetItem::leaveEvent(QEvent *event)
{
	m_isEnter = false;
	update();
}

void BottomWidgetItem::mouseReleaseEvent(QMouseEvent *event)
{
    m_isChecked = true;
    emit signalItemClicked(m_itemType);
    update();
}

BottomWidget::BottomWidget(QWidget *parent)
	: QWidget(parent)
{
	initWidget();

	this->setAttribute(Qt::WA_TranslucentBackground);

	this->setStyleSheet("QToolButton{border:1px solid rgb(166, 233, 210);background:rgb(159,168,218);}\
						QWidget#ItemBackWidget{background:rgb(0, 175, 200);}");
	this->setFixedHeight(40);
}

void BottomWidget::setScreenWidth(int width)
{
    m_screenWidth = width;
}

void BottomWidget::initWidget()
{
	m_bottomItemArea = new QScrollArea;
	m_bottomItemArea->setObjectName("ItemBackWidget");
	m_bottomItemArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	m_bottomItemArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	
	m_pButtonMoveLeft = new QToolButton;
	m_pButtonMoveLeft->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/Bottom_LeftArrow_Hover.png"));
	m_pButtonMoveLeft->setIconSize(QSize(25, 25));
	m_pButtonMoveLeft->setFixedWidth(30);
	m_pButtonMoveLeft->setFixedSize(QSize(35, 40));
	m_pButtonMoveLeft->setVisible(false);
	connect(m_pButtonMoveLeft, &QToolButton::clicked, this, [=] {
		QScrollBar* scrollBar = m_bottomItemArea->horizontalScrollBar();
		scrollBar->setValue(scrollBar->value() - ITEM_WIDTH);
	});

	m_pButtonMoveRight = new QToolButton;
	m_pButtonMoveRight->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/Bottom_RightArrow_Hover.png"));
	m_pButtonMoveRight->setIconSize(QSize(25, 25));
	m_pButtonMoveRight->setFixedSize(QSize(35, 40));
	m_pButtonMoveRight->setVisible(false);
	connect(m_pButtonMoveRight, &QToolButton::clicked, this, [=] {
		QScrollBar* scrollBar = m_bottomItemArea->horizontalScrollBar();
		scrollBar->setValue(scrollBar->value() + ITEM_WIDTH);
	});

	m_itemBackWidget = new QWidget;
	m_itemBackWidget->setFixedHeight(40);
	m_itemBackWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_itemBackWidget->setObjectName("ItemBackWidget");

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_pButtonMoveLeft);
	hMainLayout->addWidget(m_bottomItemArea);
	hMainLayout->addWidget(m_pButtonMoveRight);
	hMainLayout->setSpacing(0);
	hMainLayout->setMargin(0);

	BottomWidgetItem* mainItemWidget = new BottomWidgetItem(true);
	mainItemWidget->setItemType(Menu_MainPage);
	m_lstBottomItems.append(mainItemWidget);
    connect(mainItemWidget, &BottomWidgetItem::signalItemClicked, this, &BottomWidget::onUpdateItemState);

	QHBoxLayout* hLayout = new QHBoxLayout(m_itemBackWidget);
	hLayout->addWidget(mainItemWidget);
	hLayout->setSpacing(ITEM_SPACE);
	hLayout->setMargin(3);

	m_bottomItemArea->setWidget(m_itemBackWidget);
}

bool BottomWidget::isExistItem(SystemGuideMenuItemType itemType)
{
	// 先遍历一遍当前当前类型的item是否已经加入，如果加入则不需要再次加入bottomWidget;
	bool isExist = false;
	for (int i = 0; i < m_lstBottomItems.count(); i++)
	{
		if (m_lstBottomItems[i]->getItemType() == itemType)
		{
			m_lstBottomItems[i]->setChecked(true);
			isExist = true;
		}
		else
		{
			m_lstBottomItems[i]->setChecked(false);
		}
	}

	return isExist;
}

void BottomWidget::addBottomItem(SystemGuideMenuItemType itemType)
{
	// 如果已经存在则不需要再次添加;
	if (isExistItem(itemType))
	{
		return;
	}

	QHBoxLayout* hLayout = static_cast<QHBoxLayout*>(m_itemBackWidget->layout());
	
	BottomWidgetItem* itemWidget = new BottomWidgetItem;
	itemWidget->setItemType(itemType);
	itemWidget->setChecked(true);
	hLayout->addWidget(itemWidget);
	m_lstBottomItems.append(itemWidget);

	connect(itemWidget, &BottomWidgetItem::signalItemClose, this, [=](SystemGuideMenuItemType itemType) {
		m_lstBottomItems.removeOne(itemWidget);
		hLayout->removeWidget(itemWidget);
		delete itemWidget;
		int itemCount = m_lstBottomItems.count();
		int currentBottomWidth = ITEM_WIDTH * itemCount + ITEM_SPACE * (itemCount - 1);
		// 如果减少一个item需要判断当前是否还需要显示左右箭头;
		if (currentBottomWidth < m_screenWidth)
		{
			m_pButtonMoveLeft->setVisible(false);
			m_pButtonMoveRight->setVisible(false);
		}
		m_itemBackWidget->setFixedWidth(currentBottomWidth);

		BottomWidgetItem *pLastButton = m_lstBottomItems.at(m_lstBottomItems.size() - 1);
		if (nullptr != pLastButton)
		{
			pLastButton->setChecked(true);
			emit pLastButton->signalItemClicked(pLastButton->getItemType());
		}
		emit CloseItemSignal(itemType);

	});

	connect(itemWidget, &BottomWidgetItem::signalItemClicked, this, &BottomWidget::onUpdateItemState);
	

	int itemCount = m_lstBottomItems.count();
	int currentBottomWidth = ITEM_WIDTH * itemCount + ITEM_SPACE * (itemCount - 1);
	// 如果新增一个item超出了边界，则显示左右箭头，用于调节item显示;
	if (currentBottomWidth > m_screenWidth)
	{
		m_pButtonMoveLeft->setVisible(true);
		m_pButtonMoveRight->setVisible(true);
	}
	m_itemBackWidget->setFixedWidth(currentBottomWidth);
	// 添加一个新的item，将滚动条移至新的item;
	QScrollBar* scrollBar = m_bottomItemArea->horizontalScrollBar();
	scrollBar->setValue(scrollBar->maximum());
}

void BottomWidget::onUpdateItemState(SystemGuideMenuItemType itemType)
{
	for (int i = 0; i < m_lstBottomItems.count(); i++)
	{
		if (m_lstBottomItems[i]->getItemType() != itemType)
		{
			m_lstBottomItems[i]->setChecked(false);
		}
	}

	emit signalItemClicked(itemType);
}

void BottomWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), QColor(0, 175, 200));
}