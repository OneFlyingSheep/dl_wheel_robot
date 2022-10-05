#include "BottomWidget.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QScrollBar>

#define ITEM_WIDTH 150				// �ײ�item����;
#define ITEM_SPACE 5				// �ײ�item֮����;

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

	// �����MianItem;
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
		itemText = QString("�����˹���");
		break;
	case Menu_RobotControl:
		itemText = QString("�����˿���");
		break;
	case Menu_PatrolManage:
		itemText = QString("Ѳ����");
		break;
	case Menu_All_Patrol:
		itemText = QString("ȫ��Ѳ��");
		break;
	case Menu_Routine_Patrol:
		itemText = QString("����Ѳ��");
		break;
	case Menu_InfraredCalculateTmp:
		itemText = QString("�������");
		break;
	case Menu_OilTableRecord:
		itemText = QString("��λ�����±���¼");
		break;
	case Menu_ArresterTableRead:
		itemText = QString("���������ƶ�ȡ");
		break;
	case Menu_SF6PressureRecord:
		itemText = QString("SF6ѹ����¼");
		break;
	case Menu_HydraumaticTableRecord:
		itemText = QString("Һѹ����¼");
		break;
	case Menu_PosStateRecognition:
		itemText = QString("λ��״̬ʶ��");
		break;
	case Menu_BadWeatherPatrol:
		itemText = QString("����������Ѳ");
		break;
	case Menu_DefectTrack:
		itemText = QString("ȱ�ݸ���");
		break;
	case Menu_DistanceStateAffirm:
		itemText = QString("Զ��״̬ȷ��");
		break;
	case Menu_DistanceAbnormalAlarmAffirm:
		itemText = QString("Զ���쳣�о�ȷ��");
		break;
	case Menu_SecurityLinkage:
		itemText = QString("��������");
		break;
	case Menu_AssistAccidentDeal:
		itemText = QString("Э��Ӧ���¹ʴ���");
		break;
	case Menu_CustomTask:
		itemText = QString("�Զ�������");
		break;
	case Menu_MapChoosePoint:
		itemText = QString("��ͼѡ��");
		break;
	case Menu_TaskShow:
		itemText = QString("����չʾ");
		break;
	case Menu_DeviceAlarmSearchAffirm:
		itemText = QString("�豸�澯��ѯȷ��");
		break;
	case Menu_MainLineShow:
		itemText = QString("������չʾ");
		break;
	case Menu_IntervalShow:
		itemText = QString("���չʾ");
		break;
	case Menu_PatrolResultBrowse:
		itemText = QString("Ѳ�������");
		break;
	case Menu_PatrolReportCreate:
		itemText = QString("Ѳ�챨������");
		break;
	case Menu_CompareAnalysis:
		itemText = QString("�Աȷ���");
		break;
	case Menu_CreateReportForm:
		itemText = QString("���ɱ���");
		break;
	case Menu_AlarmThresholdSet:
		itemText = QString("�澯��ֵ����");
		break;
	case Menu_AlarmInfoSubscribeSet:
		itemText = QString("�澯��Ϣ��������");
		break;
	case Menu_RightManage:
		itemText = QString("Ȩ�޹���");
		break;
	case Menu_TypicalPatrolPointMaintain:
		itemText = QString("����Ѳ����ά��");
		break;
	case Menu_PatrolPointPosSet:
		itemText = QString("Ѳ���λ����");
		break;
	case Menu_ServiceAreaSet:
		itemText = QString("������������");
		break;
	case Menu_PatrolMapMaintain:
		itemText = QString("Ѳ���ͼά��");
		break;
	case Menu_SoftSet:
		itemText = QString("��������");
		break;
	case Menu_RobotSet:
		itemText = QString("����������");
		break;
	case Menu_RobotStateShow:
		itemText = QString("������״̬��ʾ");
		break;
	case Menu_RobotAlarmSearch:
		itemText = QString("�����˸澯��ѯ");
		break;
	case Menu_RecognizeAbnormalPointSearch:
		itemText = QString("ʶ���쳣��λ��ѯ");
		break;
	case Menu_MainPage:
		itemText = QString("���վ���ܻ�����Ѳ��");
		break;
    case Menu_CompareDetection:
        itemText = QString("������");
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
		// ������;
		painter.setRenderHint(QPainter::Antialiasing, true);
		// ���ý���ɫ;
		QLinearGradient linear(QPointF(this->width() / 2, 0), QPointF(this->width() / 2, this->height()));
		linear.setColorAt(0, QColor(55, 143, 254));
		linear.setColorAt(0.8, QColor(175, 191, 255));
		linear.setColorAt(1, Qt::white);
		// ������ʾģʽ;
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
	// �ȱ���һ�鵱ǰ��ǰ���͵�item�Ƿ��Ѿ����룬�����������Ҫ�ٴμ���bottomWidget;
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
	// ����Ѿ���������Ҫ�ٴ�����;
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
		// �������һ��item��Ҫ�жϵ�ǰ�Ƿ���Ҫ��ʾ���Ҽ�ͷ;
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
	// �������һ��item�����˱߽磬����ʾ���Ҽ�ͷ�����ڵ���item��ʾ;
	if (currentBottomWidth > m_screenWidth)
	{
		m_pButtonMoveLeft->setVisible(true);
		m_pButtonMoveRight->setVisible(true);
	}
	m_itemBackWidget->setFixedWidth(currentBottomWidth);
	// ����һ���µ�item���������������µ�item;
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