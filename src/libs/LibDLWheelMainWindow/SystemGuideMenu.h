#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QPainter>
#include <QDebug>
#include <QTimer>
#include "common/DLWheelRobotGlobalDef.hpp"

/******主窗口菜单项item********/

class SystemGuideMenuItem : public QLabel
{
	Q_OBJECT

public:
	SystemGuideMenuItem(bool isMainItem = false, int mainItemWidth = 80)
		: m_isMainItem(isMainItem)
	{
		if (m_isMainItem)
		{
			this->setStyleSheet("font-weight:bold;color:rgb(51, 163, 195);\
								border:none;border-right:1px solid rgb(51, 163, 195);margin:4px;");
			this->setFixedWidth(mainItemWidth);
		}
		else
		{
			this->setStyleSheet("QLabel{color:rgb(102, 102, 102);\
								border:none;border-right:1px solid rgb(102, 102, 102);margin:4px;}\
								QLabel:pressed{;}");
		}

		this->setAlignment(Qt::AlignLeft);
	}

    // 设置文字;
	void setText(const QString& itemText)
	{
		__super::setText(itemText + "  ");
		this->setScaledContents(true);
	}

    // 设置菜单item类型;s
	void setMemuItemType(SystemGuideMenuItemType itemType)
	{
		m_itemType = itemType;
		QString itemText = getItemText(itemType);
		__super::setText(itemText + "  ");
		this->setScaledContents(true);
	}

    // 根据菜单类型获取对应文字;
	QString getItemText(SystemGuideMenuItemType itemType)
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
			itemText = QString("远方异常告警确认");
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
		case Menu_None:
			itemText = QString("变电站智能机器人巡检");
			break;
		default:
			break;
		}
		return itemText;
	}

signals:
    // 菜单item点击;
	void signalMenuItemClicked(SystemGuideMenuItemType itemType);

private:
    // 鼠标点击事件;
	void mouseReleaseEvent(QMouseEvent *ev)
	{
		if (!m_isMainItem)
		{
			emit signalMenuItemClicked(m_itemType);
		}
	}

private:
	bool m_isMainItem;
	SystemGuideMenuItemType m_itemType;
};

/**********主窗口菜单***********/

class SystemGuideMenu : public QWidget
{
	Q_OBJECT

public:
	SystemGuideMenu(SystemGuideMenuType itemType);

	// 是否开始隐藏菜单;
	void startHideMenu(bool isStart);

private slots:
    // 开始隐藏菜单;
	void onStartHideMenu();

signals:
    // 菜单项点击;
	void signalMenuItemClicked(SystemGuideMenuItemType itemType);

private:
    // 鼠标进入/进出事件;
	void enterEvent(QEvent *event);
	void leaveEvent(QEvent *event);

private:
	void initMenu();
	// 初始化机器人管理菜单;
	void initRobotManageMenu();
	// 初始化实时监控菜单;
	void initRealTimeMonitorMenu();
	// 初始化任务管理菜单;
	void initTaskManageMenu();
	// 初始化用户设置菜单;
	void initUserSetMenu();
	// 初始化机器人系统调试维护菜单;
	void initRobotSystemMaintain();
	// 巡检结果确认菜单;
	void initPatrolResultAffirmMenu();
	// 巡检结果分析菜单;
	void initPatrolResultAnalysisMenu();

	/*!
	 * \fn	QHBoxLayout* SystemGuideMenu::addMenuLineItem(QString titleMenuText,SystemGuideMenuItemType startItemType, int menuItemcount, bool isFirstLine = true);
	 *
	 * \brief	Adds a menu line item
	 *
	 * \author	Zhou
	 * \date	2018/4/25
	 *
	 * \param	titleMenuText	The title menu text.
	 * \param	startItemType	Type of the start item.
	 * \param	menuItemcount	The menu itemcount.
	 * \param	isFirstLine  	(Optional) True if is first line, false if not.
	 *
	 * \return	Null if it fails, else a pointer to a QHBoxLayout.
	 */
    // 添加菜单item;
	QHBoxLayout* addMenuLineItem(QString titleMenuText,SystemGuideMenuItemType startItemType, int menuItemcount, bool isFirstLine = true, int firstItemWidth = 80);
    
    // 自动布局;
	void autoMenuLayout(QList<QLayout*> layoutList);
    
    // 背景绘制;
	void paintEvent(QPaintEvent *event);

private:
    // 菜单类型;
	SystemGuideMenuType m_menuType;
	QTimer m_hideMenuTimer;
};