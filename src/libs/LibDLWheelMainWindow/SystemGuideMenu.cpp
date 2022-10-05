#include "SystemGuideMenu.h"

#pragma execution_character_set("utf-8")

#define MENU_ITEM_INTERVAL 3			// �˵���֮����;
#define MENU_ITEM_HEIGHT 30				// ÿ�в˵���ĸ߶�;
#define MENU_ITEM_WIDTH 600				// ÿ�в˵���Ŀ��;

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
	mainMenuItem->setText("�����˹���");
	
	SystemGuideMenuItem* childMenuItem = new SystemGuideMenuItem;
	childMenuItem->setText("�����˹���");
	childMenuItem->setMemuItemType(SystemGuideMenuItemType::Menu_RobotManage);
	connect(childMenuItem, &SystemGuideMenuItem::signalMenuItemClicked, this, &SystemGuideMenu::signalMenuItemClicked);

	QHBoxLayout* hLayout = addMenuLineItem("�����˹���", SystemGuideMenuItemType::Menu_RobotManage, 1);
	hLayout->setMargin(5);
	this->setLayout(hLayout);

	this->setFixedSize(QSize(MENU_ITEM_WIDTH, MENU_ITEM_HEIGHT));
}

void SystemGuideMenu::initRealTimeMonitorMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("Ѳ����", Menu_PatrolManage, 1, true));
	menuLyoutList.append(addMenuLineItem("������ң��", Menu_RobotControl, 1, true));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initTaskManageMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("ȫ��Ѳ��", Menu_All_Patrol, 1, true));
	//menuLyoutList.append(addMenuLineItem("����Ѳ��", Menu_Routine_Patrol, 1, true));
	//menuLyoutList.append(addMenuLineItem("ר��Ѳ��", Menu_InfraredCalculateTmp, 4, true));
	//menuLyoutList.append(addMenuLineItem("ר��Ѳ��", Menu_HydraumaticTableRecord, 2, false));
	//menuLyoutList.append(addMenuLineItem("����Ѳ��", Menu_BadWeatherPatrol, 4, true));
	//menuLyoutList.append(addMenuLineItem("����Ѳ��", Menu_SecurityLinkage, 2, false));
	menuLyoutList.append(addMenuLineItem("�Զ�������", Menu_CustomTask, 1, true));
	//menuLyoutList.append(addMenuLineItem("��ͼѡ��", Menu_MapChoosePoint, 1, true));
	//menuLyoutList.append(addMenuLineItem("����չʾ", Menu_TaskShow, 1, true));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initUserSetMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("�澯����", Menu_AlarmThresholdSet, 2, true, 95));
	//menuLyoutList.append(addMenuLineItem("��֯Ȩ������", Menu_RightManage, 1, true, 95));
	//menuLyoutList.append(addMenuLineItem("��λ����", Menu_TypicalPatrolPointMaintain, 2, true, 95));
	//menuLyoutList.append(addMenuLineItem("������������", Menu_ServiceAreaSet, 1, true, 95));
	
	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initRobotSystemMaintain()
{
	QList<QLayout*> menuLyoutList;
	//menuLyoutList.append(addMenuLineItem("Ѳ���ͼ����", Menu_PatrolMapMaintain, 1, true, 120));
	//menuLyoutList.append(addMenuLineItem("�������", Menu_SoftSet, 1, true, 120));
	menuLyoutList.append(addMenuLineItem("����������", Menu_RobotSet, 1, true, 120));
	menuLyoutList.append(addMenuLineItem("��������Ϣ��ѯ", Menu_RobotStateShow, 2, true, 120));
	menuLyoutList.append(addMenuLineItem("ʶ���쳣��λ��ѯ", Menu_RecognizeAbnormalPointSearch, 1, true, 120));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initPatrolResultAffirmMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("�豸�澯��Ϣȷ��", Menu_DeviceAlarmSearchAffirm, 3, true, 120));
	menuLyoutList.append(addMenuLineItem("Ѳ�������", Menu_PatrolResultBrowse, 1, true, 120));
	menuLyoutList.append(addMenuLineItem("Ѳ�챨������", Menu_PatrolReportCreate, 1, true, 120));

	autoMenuLayout(menuLyoutList);
}

void SystemGuideMenu::initPatrolResultAnalysisMenu()
{
	QList<QLayout*> menuLyoutList;
	menuLyoutList.append(addMenuLineItem("�Աȷ���", Menu_CompareAnalysis, 1, true));
	menuLyoutList.append(addMenuLineItem("���ɱ���", Menu_CreateReportForm, 1, true));

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
		// ����ӱ���˵���;
		hFirstLayout->addWidget(mainMenuItem);
	}
	else
	{
		hFirstLayout->addItem(new QSpacerItem(firstItemWidth, 0, QSizePolicy::Fixed, QSizePolicy::Minimum));
	}
	// ������Ӳ˵���;
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