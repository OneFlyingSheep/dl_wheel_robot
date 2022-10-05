#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QPainter>
#include <QDebug>
#include <QTimer>
#include "common/DLWheelRobotGlobalDef.hpp"

/******�����ڲ˵���item********/

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

    // ��������;
	void setText(const QString& itemText)
	{
		__super::setText(itemText + "  ");
		this->setScaledContents(true);
	}

    // ���ò˵�item����;s
	void setMemuItemType(SystemGuideMenuItemType itemType)
	{
		m_itemType = itemType;
		QString itemText = getItemText(itemType);
		__super::setText(itemText + "  ");
		this->setScaledContents(true);
	}

    // ���ݲ˵����ͻ�ȡ��Ӧ����;
	QString getItemText(SystemGuideMenuItemType itemType)
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
			itemText = QString("��λ�����±�¼");
			break;
		case Menu_ArresterTableRead:
			itemText = QString("��������ƶ�ȡ");
			break;
		case Menu_SF6PressureRecord:
			itemText = QString("SF6ѹ����¼");
			break;
		case Menu_HydraumaticTableRecord:
			itemText = QString("Һѹ��¼");
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
			itemText = QString("Զ���쳣�澯ȷ��");
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
			itemText = QString("�������");
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
		case Menu_None:
			itemText = QString("���վ���ܻ�����Ѳ��");
			break;
		default:
			break;
		}
		return itemText;
	}

signals:
    // �˵�item���;
	void signalMenuItemClicked(SystemGuideMenuItemType itemType);

private:
    // ������¼�;
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

/**********�����ڲ˵�***********/

class SystemGuideMenu : public QWidget
{
	Q_OBJECT

public:
	SystemGuideMenu(SystemGuideMenuType itemType);

	// �Ƿ�ʼ���ز˵�;
	void startHideMenu(bool isStart);

private slots:
    // ��ʼ���ز˵�;
	void onStartHideMenu();

signals:
    // �˵�����;
	void signalMenuItemClicked(SystemGuideMenuItemType itemType);

private:
    // ������/�����¼�;
	void enterEvent(QEvent *event);
	void leaveEvent(QEvent *event);

private:
	void initMenu();
	// ��ʼ�������˹���˵�;
	void initRobotManageMenu();
	// ��ʼ��ʵʱ��ز˵�;
	void initRealTimeMonitorMenu();
	// ��ʼ���������˵�;
	void initTaskManageMenu();
	// ��ʼ���û����ò˵�;
	void initUserSetMenu();
	// ��ʼ��������ϵͳ����ά���˵�;
	void initRobotSystemMaintain();
	// Ѳ����ȷ�ϲ˵�;
	void initPatrolResultAffirmMenu();
	// Ѳ���������˵�;
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
    // ��Ӳ˵�item;
	QHBoxLayout* addMenuLineItem(QString titleMenuText,SystemGuideMenuItemType startItemType, int menuItemcount, bool isFirstLine = true, int firstItemWidth = 80);
    
    // �Զ�����;
	void autoMenuLayout(QList<QLayout*> layoutList);
    
    // ��������;
	void paintEvent(QPaintEvent *event);

private:
    // �˵�����;
	SystemGuideMenuType m_menuType;
	QTimer m_hideMenuTimer;
};