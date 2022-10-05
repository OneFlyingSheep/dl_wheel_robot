#include "SystemGuideMenu.h"

#pragma execution_character_set("utf-8")

#define MENU_ITEM_INTERVAL 3			// 菜单项之间间隔;
#define MENU_ITEM_HEIGHT 30				// 每行菜单项的高度;
#define MENU_ITEM_WIDTH 600				// 每行菜单项的宽度;

SystemGuideMenu::SystemGuideMenu(SystemGuideMenuType itemType)
	: m_menuType(itemType)
{
	this->setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);
	
	initMenu();

	m_hideMenuTimer.setInterval(300);
	connect(&m_hideMenuTimer, &QTimer::timeout, this, [=] {
		this->hide();
		m_hideMenuTimer.stop();
	});
}

void SystemGuideMenu::initMenu()
{
	switch (m_menuType)
	{
	case RobotManage:
		initRobotManageMenu();
		break;
	case RealTimeMonitor:
		initRealTimeMonitorMenu();
		break;
	case TaskManager:
		initTaskManageMenu();
		break;
	case RobotSystemMaintain:
		initRobotSystemMaintain();
		break;
	case UserSet:
		initUserSetMenu();
		break;
	case PatrolResultAffirm:
		initPatrolResultAffirmMenu();
		break;
	case PatrolResultAnalysis:
		initPatrolResultAnalysisMenu();
		break;
	default:
		break;
	}
}

void SystemGuideMenu::initRobotManageMenu()
{
	SystemGuideMenuItem* mainMenuItem = new SystemGuideMenuItem(true);
	mainMenuItem->setText("机器人管理");
	
	SystemGuideMenuItem* childMenuItem = new SystemGuideMenuItem;
	childMenuItem->setText("机器人管理");
	childMenuItem->setMemuItemType(SystemGuideMenuItemType::Menu_RobotManage);
	connect(childMenuItem, &SystemGuideMenuItem::signalMenuItemClicked, this, &SystemGuideMenu::signalMenuItemClicked);

	QHBoxLayout* hLayout = addMenuLineItem("机器人管理", SystemGuideMenuItemType::Menu_RobotManage, 1);
	hLayout->setMargin(5);
	this->setLayout(hLayout);

	this->setFixedSize(QSize(MENU_ITEM_WIDTH, MENU_ITEM_HEIGHT));
}

void SystemGuideMenu::initRealTimeMonitorMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("巡检监控", Menu_PatrolManage, 1, true));
	menuLyoutList.append(addMenuLineItem("机器人遥控", Menu_RobotControl, 1, true));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initTaskManageMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("全面巡检", Menu_All_Patrol, 1, true));
	//menuLyoutList.append(addMenuLineItem("例行巡检", Menu_Routine_Patrol, 1, true));
	//menuLyoutList.append(addMenuLineItem("专项巡检", Menu_InfraredCalculateTmp, 4, true));
	//menuLyoutList.append(addMenuLineItem("专项巡检", Menu_HydraumaticTableRecord, 2, false));
	//menuLyoutList.append(addMenuLineItem("特殊巡检", Menu_BadWeatherPatrol, 4, true));
	//menuLyoutList.append(addMenuLineItem("特殊巡检", Menu_SecurityLinkage, 2, false));
	menuLyoutList.append(addMenuLineItem("自定义任务", Menu_CustomTask, 1, true));
	//menuLyoutList.append(addMenuLineItem("地图选点", Menu_MapChoosePoint, 1, true));
	//menuLyoutList.append(addMenuLineItem("任务展示", Menu_TaskShow, 1, true));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initUserSetMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("告警设置", Menu_AlarmThresholdSet, 2, true, 95));
	//menuLyoutList.append(addMenuLineItem("组织权限设置", Menu_RightManage, 1, true, 95));
	//menuLyoutList.append(addMenuLineItem("点位设置", Menu_TypicalPatrolPointMaintain, 2, true, 95));
	//menuLyoutList.append(addMenuLineItem("检修区域设置", Menu_ServiceAreaSet, 1, true, 95));
	
	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initRobotSystemMaintain()
{
	QList<QLayout*> menuLyoutList;
	//menuLyoutList.append(addMenuLineItem("巡检地图设置", Menu_PatrolMapMaintain, 1, true, 120));
	//menuLyoutList.append(addMenuLineItem("软件设置", Menu_SoftSet, 1, true, 120));
	menuLyoutList.append(addMenuLineItem("机器人设置", Menu_RobotSet, 1, true, 120));
	menuLyoutList.append(addMenuLineItem("机器人信息查询", Menu_RobotStateShow, 2, true, 120));
	menuLyoutList.append(addMenuLineItem("识别异常点位查询", Menu_RecognizeAbnormalPointSearch, 1, true, 120));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initPatrolResultAffirmMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("设备告警信息确认", Menu_DeviceAlarmSearchAffirm, 3, true, 120));
	menuLyoutList.append(addMenuLineItem("巡检结果浏览", Menu_PatrolResultBrowse, 1, true, 120));
	menuLyoutList.append(addMenuLineItem("巡检报告生成", Menu_PatrolReportCreate, 1, true, 120));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initPatrolResultAnalysisMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("对比分析", Menu_CompareAnalysis, 1, true));
	menuLyoutList.append(addMenuLineItem("生成报表", Menu_CreateReportForm, 1, true));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), Qt::white);
	painter.setPen(Qt::gray);
	painter.drawRect(QRect(0, 0, this->width() - 1, this->height() - 1));
}

void SystemGuideMenu::startHideMenu(bool isStart)
{
	if (isStart)
	{
		m_hideMenuTimer.start();
	}
	else
	{
		m_hideMenuTimer.stop();
	}
}

void SystemGuideMenu::onStartHideMenu()
{

}

void SystemGuideMenu::enterEvent(QEvent *event)
{
	m_hideMenuTimer.stop();
}

void SystemGuideMenu::leaveEvent(QEvent *event)
{
	startHideMenu(true);
}

QHBoxLayout* SystemGuideMenu::addMenuLineItem(QString titleMenuText, SystemGuideMenuItemType startItemType, int menuItemcount, bool isFirstLine, int firstItemWidth)
{
	QHBoxLayout* hFirstLayout = new QHBoxLayout;
	if (isFirstLine)
	{
		SystemGuideMenuItem* mainMenuItem = new SystemGuideMenuItem(true, firstItemWidth);
		mainMenuItem->setText(titleMenuText);
		// 先添加标题菜单项;
		hFirstLayout->addWidget(mainMenuItem);
	}
	else
	{
		hFirstLayout->addItem(new QSpacerItem(firstItemWidth, 0, QSizePolicy::Fixed, QSizePolicy::Minimum));
	}
	// 再添加子菜单项;
	SystemGuideMenuItem* childMenuItem;
	int startItemIndex = startItemType;
	for (int i = 0; i < menuItemcount; i++)
	{
		SystemGuideMenuItem* childMenuItem = new SystemGuideMenuItem;
		childMenuItem->setMemuItemType(SystemGuideMenuItemType(startItemIndex + i));
		hFirstLayout->addWidget(childMenuItem);
		connect(childMenuItem, &SystemGuideMenuItem::signalMenuItemClicked, this, &SystemGuideMenu::signalMenuItemClicked);
	}
	hFirstLayout->addStretch();
	hFirstLayout->setSpacing(MENU_ITEM_INTERVAL);
	hFirstLayout->setMargin(0);

	return hFirstLayout;
}

void SystemGuideMenu::autoMenuLayout(QList<QLayout *> layoutList)
{
	QVBoxLayout* vLayout = new QVBoxLayout(this);
	int i = 0;
	for (; i < layoutList.count() - 1; i++)
	{
		vLayout->addLayout(layoutList[i]);
		QLabel* labelSplite = new QLabel;
		labelSplite->setFixedHeight(1);
		labelSplite->setStyleSheet("border:1px solid gray");
		vLayout->addWidget(labelSplite);
	}
	vLayout->addLayout(layoutList[i]);
	vLayout->setSpacing(MENU_ITEM_INTERVAL);
	vLayout->setMargin(5);

	this->setFixedSize(QSize(MENU_ITEM_WIDTH, MENU_ITEM_HEIGHT * layoutList.count()));
}