#include "SystemGuideWidget.h"
#include <QEvent>
#include "SystemGuideMenu.h"

#pragma execution_character_set("utf-8")

SystemGuideWidget::SystemGuideWidget(QWidget *parent)
	: QWidget(parent)
	, m_systemGuidMenu(NULL)
{
	initWidget();
	this->setFixedSize(QSize(200, 35));
	this->setWindowFlags(Qt::FramelessWindowHint);
	m_customListWidget->installEventFilter(this);
	this->installEventFilter(this);
}

SystemGuideWidget::~SystemGuideWidget()
{
}

void SystemGuideWidget::initWidget()
{
	initListWidget();

	m_labelText = new QLabel;
	m_labelText->setText("系统导航");
	m_labelText->setStyleSheet("color:white;font-size:16px;font-weight:bold");

	m_labelArrow = new QLabel;
	m_labelArrow->setStyleSheet("border:none;margin-top:3px;");
	m_labelArrow->setFixedSize(QSize(20, 16));
	m_labelArrow->setPixmap(QIcon(":/Resources/DLWheelMainWindow/Image/DownTriangle.png").pixmap(m_labelArrow->size()));
	
	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addStretch();
	hLayout->addWidget(m_labelText);
	hLayout->addWidget(m_labelArrow);
	hLayout->addStretch();
	hLayout->setSpacing(15);
	hLayout->setContentsMargins(20, 0, 0, 0);
}

void SystemGuideWidget::initListWidget()
{
	m_customListWidget = new QWidget();
	m_customListWidget->setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);
	m_customListWidget->setObjectName("CustomListWidget");
	m_customListWidget->setStyleSheet("QWidget#CustomListWidget{background:white;border:1px solid rgb(124,77,255);}");
	m_customListWidget->setFixedSize(QSize(200, 458));				//导航栏的尺寸
	m_customListWidget->setHidden(true);

	m_pButtonUpArrow = new QPushButton;
	m_pButtonUpArrow->setStyleSheet("background:rgb(57,73,171);border:none;");
	m_pButtonUpArrow->setFixedHeight(30);
	m_pButtonUpArrow->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/upArrow.png"));
	m_pButtonUpArrow->setIconSize(QSize(35, 16));

	m_pButtonDownArrow = new QPushButton;
	m_pButtonDownArrow->setStyleSheet("background:rgb(57,73,171);border:none;");
	m_pButtonDownArrow->setFixedHeight(30);
	m_pButtonDownArrow->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/downArrow.png"));
	m_pButtonDownArrow->setIconSize(QSize(35, 16));

	QVBoxLayout* vItemLayout = new QVBoxLayout;
	for (int i = 0; i < 7; i ++)
	{
		SystemGuideItem* item = new SystemGuideItem;
		item->setIcon(QIcon(":/Resources/DLWheelMainWindow/Image/TitleIcon.png"));
		m_customItemList.append(item);
		vItemLayout->addWidget(item);

		connect(item, &SystemGuideItem::signalEnterItem, this, [=](SystemGuideMenuType menuType) {
			if (m_systemGuidMenu != NULL)
			{
				delete m_systemGuidMenu;
			}
			m_systemGuidMenu = new SystemGuideMenu(item->getMenuType());
			connect(m_systemGuidMenu, &SystemGuideMenu::signalMenuItemClicked, this, &SystemGuideWidget::signalMenuItemClicked);
			QPoint posPoint = item->mapToGlobal(QPoint(item->width() + 5, 5));
			m_systemGuidMenu->move(posPoint);
			m_systemGuidMenu->startHideMenu(false);
			m_systemGuidMenu->show();
		});

		connect(item, &SystemGuideItem::signalLeaveItem, this, [=] {
			m_systemGuidMenu->startHideMenu(true);
		});
	}	
	vItemLayout->setContentsMargins(4, 0, 4, 0);
    vItemLayout->addStretch();
    vItemLayout->setSpacing(10);

	m_customItemList[0]->setText("机器人管理");
	m_customItemList[0]->setMenuType(SystemGuideMenuType::RobotManage);
	m_customItemList[1]->setText("任务管理");
	m_customItemList[1]->setMenuType(SystemGuideMenuType::TaskManager);
	m_customItemList[2]->setText("实时监控");
	m_customItemList[2]->setMenuType(SystemGuideMenuType::RealTimeMonitor);
	m_customItemList[3]->setText("巡检结果确认");
	m_customItemList[3]->setMenuType(SystemGuideMenuType::PatrolResultAffirm);
	m_customItemList[4]->setText("巡检结果分析");
	m_customItemList[4]->setMenuType(SystemGuideMenuType::PatrolResultAnalysis);
	m_customItemList[5]->setText("用户设置");
	m_customItemList[5]->setMenuType(SystemGuideMenuType::UserSet);
	m_customItemList[6]->setText("机器人系统调试维护");
	m_customItemList[6]->setMenuType(SystemGuideMenuType::RobotSystemMaintain);

//     QPushButton* pButtonMinWindow = new QPushButton("最小化");
//     pButtonMinWindow->setStyleSheet("border:2px solid rgb(175,191,255);margin:3px;color:rgb(63,81,181);");
//     pButtonMinWindow->setFixedHeight(40);
//     connect(pButtonMinWindow, &QPushButton::clicked, this, &SystemGuideWidget::signalWindowMinsize);

    //QPushButton* pButtonCompareDetection = new QPushButton("异物检测");
    //pButtonCompareDetection->setStyleSheet("border:2px solid rgb(175,191,255);margin:3px;color:rgb(63,81,181);");
    //pButtonCompareDetection->setFixedHeight(40);
    //connect(pButtonCompareDetection, &QPushButton::clicked, this, &SystemGuideWidget::signalCompareDetection);

	QVBoxLayout* vListWidgetLayout = new QVBoxLayout(m_customListWidget);
	vListWidgetLayout->addWidget(m_pButtonUpArrow);
    //vListWidgetLayout->addWidget(pButtonMinWindow);
	vListWidgetLayout->addLayout(vItemLayout);
    //vListWidgetLayout->addWidget(pButtonCompareDetection);
	vListWidgetLayout->addWidget(m_pButtonDownArrow);
	vListWidgetLayout->setSpacing(10);
	vListWidgetLayout->setMargin(1);
}

void SystemGuideWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(QRect(0, 0, this->width(), this->height() / 2), QColor(90, 177, 202));
	painter.fillRect(QRect(0, this->height() / 2, this->width(), this->height() / 2), QColor(60, 170, 205));

	painter.setPen(QPen(Qt::white));
	painter.drawRect(QRect(0, 0, this->width() - 1, this->height() - 1));
}

void SystemGuideWidget::mouseReleaseEvent(QMouseEvent *event)
{
	if (!m_customListWidget->isActiveWindow())
	{
		QPoint globalPos = this->mapToGlobal(this->pos());
		m_customListWidget->move(globalPos + QPoint(-10, 38));
		m_customListWidget->show();
		m_customListWidget->activateWindow();
	}
}

void SystemGuideWidget::hideMenuWidget()
{
	m_customListWidget->hide();
	if (m_systemGuidMenu != NULL)
	{
		m_systemGuidMenu->hide();
	}
}

void SystemGuideWidget::updateSystemMenu(WheelUserType currentLoginRole)
{
    switch (currentLoginRole)
    {
    case WHEEL_USER_NONE:
        break;
    case WHEEL_USER_NORMAL:
    {
		/// zhw test
        m_customItemList[UserSet]->setVisible(false);
        m_customItemList[RobotSystemMaintain]->setVisible(false);
        m_customListWidget->setFixedHeight(400);
    }
        break;
    case WHEEL_USER_MANAGER:
    {
		/// zhw test
        m_customItemList[RobotSystemMaintain]->setVisible(false);
        m_customListWidget->setFixedHeight(445);
    }
        break;
    case WHEEL_USER_SUPER_MANAGER:
        break;
    case WHEEL_USER_ROOT:
        break;
    case WHEEL_USER_ENGINEER:
        break;
    default:
        break;
    }
}

bool SystemGuideWidget::eventFilter(QObject *watched, QEvent *event)
{
	if (watched == m_customListWidget)
	{
		if (event->type() == QEvent::WindowDeactivate)
		{
			hideMenuWidget();
		}
	}

	return __super::eventFilter(watched, event);
}