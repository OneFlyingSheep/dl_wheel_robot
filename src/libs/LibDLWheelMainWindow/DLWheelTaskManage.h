#ifndef DL_WHEEL_TASK_MANAGE_H
#define DL_WHEEL_TASK_MANAGE_H

#include <QWidget>
#include <QLabel>
#include <QButtonGroup>
#include <QToolButton>
#include <QPushButton>
#include <QLineEdit>
#include <QTreeWidget>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QHeaderView>
#include <QDateTimeEdit>
#include <QListWidget>
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLWheelCustomWidget/CheckBoxWidget.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelTaskAdministrationData.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include <QCalendarWidget>
#include <QTimer>

#pragma execution_character_set("utf-8")

// Ѳ������;
enum PatrolType
{
	Routine_Patrol = 0,				// ����Ѳ��;
	All_Patrol,						// ȫ��Ѳ��;
	Single_Patrol,					// ר��Ѳ��;
	Special_Patrol,					// ����Ѳ��;
};

// ����ģʽ����;
enum SpecialModeType
{
	ClimbSummer_Mode = 0,			// ӭ�������Ѳ;
	ThunderStorm_Mode,				// �ױ�������Ѳ;
	FloodPrevention_Mode,			// ��Ѵ��̨��Ѳ;
	SleetFrozen_Mode,				// ��ѩ������Ѳ;
	HazeWeahter_Mode,				// ����������Ѳ;
	GaleWeather_Mode,				// ���������Ѳ;
};

// ����ִ������;
enum TaskExcuteType
{
	ImmediatelyExcute = 0,
	FixedTimeExcute,
	PeriodExcute,
};

class InputWidget;

/**********��ʷ�����봰��**********/

class HistoryTaskImportWindow : public BaseWidget
{
    Q_OBJECT
public:
    HistoryTaskImportWindow(QWidget* parent = NULL)
        : BaseWidget(parent, PopupWindow)
        , m_currentRow(-1)
    {
        initWidget();
        m_dataList = WHEEL_TASK_ADMINISTRATION.getWheelTaskHistoryStru("", "", "");
        initListWidget();
        initTaskNameWidget();
        initOverTimeTimer();

        this->setWindowModality(Qt::ApplicationModal);
        this->setTitleContent("������");
        this->setFixedSize(QSize(350, 350));
        this->setStyleSheet("QPushButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(145, 167, 255);border-radius:3px;}\
					    	QPushButton:pressed{padding-left:2px;padding-top:2px;}");

        WHEEL_BACK_TO_CORE_SOCKET.wheelRobotTaskEditImportStatus.connect(boost::bind(&HistoryTaskImportWindow::signalImportCallBack, this, _1, _2));

        connect(this, &HistoryTaskImportWindow::signalImportCallBack, this, [=](bool isSuccess, QString strMsg) {
            m_overtimeTimer.stop();
            if (isSuccess)
            {
                emit signalImportSuccess();
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "����", "����ʧ��: " + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
            this->setDisabled(false);
        }, Qt::QueuedConnection);
    }

    // ���ô���;
    void resetWidget()
    {
        m_dataList = WHEEL_TASK_ADMINISTRATION.getWheelTaskHistoryStru("", "", "");
        initListWidget();
    }

private:

    // ��ʼ�������б�;
    void initListWidget()
    {
        m_taskNameListWidget->clear();
        for (int i = 0; i < m_dataList.count(); i ++)
        {
            m_taskNameListWidget->addItem(m_dataList[i].task_edit_name);
        }
    }

    // ��ʼ���������ƿؼ�;
    void initTaskNameWidget()
    {
        m_taskNameModifyWidget = new BaseWidget(this, BaseWidgetType::PopupWindow);
        m_taskNameModifyWidget->setTitleContent("��������");
        m_taskNameModifyWidget->setWindowModality(Qt::ApplicationModal);

        m_modifyTaskNameLineEdit = new QLineEdit;

        // ����;
        QPushButton* pButtonOK = new QPushButton("ȷ��");
        pButtonOK->setFixedSize(QSize(60, 25));
        connect(pButtonOK, &QPushButton::clicked, this, [=] {
            WheelTaskEditStruct wheelTaskEditStruct;
            wheelTaskEditStruct.task_edit_name = m_modifyTaskNameLineEdit->text();
            wheelTaskEditStruct.task_edit_uuid = m_dataList[m_currentRow].task_edit_uuid;
            wheelTaskEditStruct.task_edit_date = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
            WHEEL_BACK_TO_CORE_SOCKET.robot_task_edit_import_req(wheelTaskEditStruct);
            m_taskNameModifyWidget->hide();
            this->setDisabled(true);
            m_overtimeTimer.start();
        });

        QPushButton* pButtonCancel = new QPushButton("ȡ��");
        pButtonCancel->setFixedSize(QSize(60, 25));
        connect(pButtonCancel, &QPushButton::clicked, this, [=] {
            m_taskNameModifyWidget->hide();
        });

        QHBoxLayout* hButtonLayout = new QHBoxLayout;
        hButtonLayout->addStretch();
        hButtonLayout->addWidget(pButtonOK);
        hButtonLayout->addWidget(pButtonCancel);
        hButtonLayout->setSpacing(15);
        hButtonLayout->setMargin(0);

        QVBoxLayout* hTaskNameLayout = new QVBoxLayout(m_taskNameModifyWidget->getCenterWidget());
        hTaskNameLayout->addWidget(m_modifyTaskNameLineEdit);
        hTaskNameLayout->addLayout(hButtonLayout);
        hButtonLayout->setSpacing(15);
        hButtonLayout->setMargin(10);
    }

    // ��ʼ����ʱ��Ϣ��;
    void initOverTimeTimer()
    {
        m_overtimeTimer.setInterval(5000);
        connect(&m_overtimeTimer, &QTimer::timeout, this, [=] {
            DLMessageBox::showDLMessageBox(NULL, "����", "������ʱ", MessageButtonType::BUTTON_OK, true, QSize(250, 180));

            this->setDisabled(false);
        });
    }

    // ��ʼ���ؼ�;
    void initWidget()
    {
        m_taskNameLineEdit = new InputWidget(InputWidgetType::WheelLineEdit);
        m_taskNameLineEdit->setTipText("��������");

        QPushButton* pButtonSearch = new QPushButton("��ѯ");
        pButtonSearch->setFixedSize(QSize(60, 25));
        connect(pButtonSearch, &QPushButton::clicked, this, [=] {
            m_dataList = WHEEL_TASK_ADMINISTRATION.getWheelTaskHistoryStru(m_taskFinishStartDateEdit->date().toString("yyyy-MM-dd"),
                                                                            m_taskFinishEndDateEdit->date().toString("yyyy-MM-dd"),
                                                                            m_taskNameLineEdit->getLineEditText());
            initListWidget();
        });

        QHBoxLayout* hSearchLayout = new QHBoxLayout;
        hSearchLayout->addWidget(m_taskNameLineEdit);
        hSearchLayout->addWidget(pButtonSearch);
        hSearchLayout->addStretch();
        hSearchLayout->setSpacing(10);
        hSearchLayout->setMargin(0);

        QLabel* startTimeLabel = new QLabel("��ʼʱ��:");

        m_taskFinishStartDateEdit = new QDateTimeEdit;
        m_taskFinishStartDateEdit->setDisplayFormat("yyyy-MM-dd");
        m_taskFinishStartDateEdit->setDate(QDate::currentDate());
        m_taskFinishStartDateEdit->setCalendarPopup(true);
        m_taskFinishStartDateEdit->setFixedWidth(90);
        QCalendarWidget* calendarWidget = new QCalendarWidget(m_taskFinishStartDateEdit);
        m_taskFinishStartDateEdit->setCalendarWidget(calendarWidget);

        QLabel* endTimeLabel = new QLabel("����ʱ��:");
        
        m_taskFinishEndDateEdit = new QDateTimeEdit;
        m_taskFinishEndDateEdit->setDisplayFormat("yyyy-MM-dd");
        m_taskFinishEndDateEdit->setDate(QDate::currentDate());
        m_taskFinishEndDateEdit->setCalendarPopup(true);
        m_taskFinishEndDateEdit->setFixedWidth(90);
        calendarWidget = new QCalendarWidget(m_taskFinishEndDateEdit);
        m_taskFinishEndDateEdit->setCalendarWidget(calendarWidget);

        QHBoxLayout* hEndTimeLayout = new QHBoxLayout;
        hEndTimeLayout->addWidget(startTimeLabel);
        hEndTimeLayout->addWidget(m_taskFinishStartDateEdit);
        hEndTimeLayout->addWidget(endTimeLabel);
        hEndTimeLayout->addWidget(m_taskFinishEndDateEdit);
        hEndTimeLayout->setSpacing(5);
        hEndTimeLayout->setMargin(0);

        m_taskNameListWidget = new QListWidget;

        // ����;
        QPushButton* pButtonImport = new QPushButton("����");
        pButtonImport->setFixedSize(QSize(60, 25));
        connect(pButtonImport, &QPushButton::clicked, this, &HistoryTaskImportWindow::onImportTask);

        QPushButton* pButtonCancel = new QPushButton("ȡ��");
        pButtonCancel->setFixedSize(QSize(60, 25));
        connect(pButtonCancel, &QPushButton::clicked, this, &HistoryTaskImportWindow::signalWindowClose);

        QHBoxLayout* hButtonLayout = new QHBoxLayout;
        hButtonLayout->addStretch();
        hButtonLayout->addWidget(pButtonImport);
        hButtonLayout->addWidget(pButtonCancel);
        hButtonLayout->setSpacing(15);
        hButtonLayout->setMargin(0);

        QVBoxLayout* vLayout = new QVBoxLayout(this->getCenterWidget());
        vLayout->addLayout(hSearchLayout);
        vLayout->addLayout(hEndTimeLayout);
        vLayout->addWidget(m_taskNameListWidget);
        vLayout->addLayout(hButtonLayout);
        vLayout->setSpacing(15);
        vLayout->setContentsMargins(15, 10, 15, 15);
    }

    // ��������;
    void onImportTask()
    {
        QListWidgetItem* item = m_taskNameListWidget->currentItem();
        if (item != NULL)
        {
            m_currentRow = m_taskNameListWidget->row(item);
            QString taskName = item->text();
            m_modifyTaskNameLineEdit->setText(taskName);
            m_modifyTaskNameLineEdit->setFocus(Qt::TabFocusReason);
            m_taskNameModifyWidget->show();
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "��ʾ", "δѡ������", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    }

signals:
    // ���ڹر�;
    void signalWindowClose();
    // ���뷵��;
    void signalImportCallBack(bool, QString);
    // ����ɹ�;
    void signalImportSuccess();

private:
    // ����ģ���б�;
    QList<WheelTaskEditStruct> m_dataList;

    InputWidget * m_taskNameLineEdit;
    QDateTimeEdit* m_taskFinishStartDateEdit;
    QDateTimeEdit* m_taskFinishEndDateEdit;
    QListWidget* m_taskNameListWidget;

    BaseWidget* m_taskNameModifyWidget;
    QLineEdit* m_modifyTaskNameLineEdit;

    int m_currentRow;

    QTimer m_overtimeTimer;
};

/*******�Զ��尴ť(table������ִ��/����ִ��/����ִ��)********/

class ButtonWidget : public QWidget
{
	Q_OBJECT
public:
	ButtonWidget(WheelTaskEditStruct data, QWidget* parent = NULL)
		: QWidget(parent)
		, m_rowIndex(0)
		, m_buttonData(data)
	{
		QButtonGroup* buttonGroup = new QButtonGroup(this);
		m_pButtonImmediately = new QPushButton("����ִ��");
		m_pButtonImmediately->setFixedSize(QSize(70, 25));
		buttonGroup->addButton(m_pButtonImmediately, ImmediatelyExcute);

		m_pButtonFixedTime = new QPushButton("����ִ��");
		m_pButtonFixedTime->setFixedSize(QSize(70, 25));
		buttonGroup->addButton(m_pButtonFixedTime, FixedTimeExcute);

		m_pButtonPeriod = new QPushButton("����ִ��");
		m_pButtonPeriod->setFixedSize(QSize(70, 25));
		buttonGroup->addButton(m_pButtonPeriod, PeriodExcute);

		connect(buttonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, [=](int buttonId) {
			emit signalButtonClicked(buttonId, m_buttonData);
		});

		QHBoxLayout* hButtonLayout = new QHBoxLayout(this);
		hButtonLayout->addStretch();
		hButtonLayout->addWidget(m_pButtonImmediately);
		hButtonLayout->addWidget(m_pButtonFixedTime);
		hButtonLayout->addWidget(m_pButtonPeriod);
		hButtonLayout->addStretch();
		hButtonLayout->setSpacing(0);
		hButtonLayout->setMargin(0);

		this->setAttribute(Qt::WA_TranslucentBackground);
		this->setStyleSheet("QPushButton{border:none;}QPushButton:hover{background:rgb(210, 255, 255);}QPushButton:pressed{padding-left:2px;padding-top:2px;}");
	}

    // ������table�еڼ���;
	void setRowIndex(int rowIndex)
	{
		m_rowIndex = rowIndex;
	}

    // ��ȡ��table�еڼ���;
	int getRowIndex()
	{
		return m_rowIndex;
	}

signals:
    // ��ť����ź�;
	void signalButtonClicked(int buttonId, WheelTaskEditStruct data);

private:
	QPushButton* m_pButtonImmediately;
	QPushButton* m_pButtonFixedTime;
	QPushButton* m_pButtonPeriod;
	int m_rowIndex;

	WheelTaskEditStruct m_buttonData;
};

class CustomTableWidget;

/*********�������˵�������ҳ��;************/

class DLWheelTaskManage : public QWidget
{
	Q_OBJECT

public:
	DLWheelTaskManage(SystemGuideMenuItemType itemType = Menu_All_Patrol);
	~DLWheelTaskManage();

	int GetMenuItemType();

	// ���õ�ǰ����ҳ������;
	void setTaskShowType(SystemGuideMenuItemType itemType);

	// �Ӳ˵�����֮���ٽ��г�ʼ��;
	void initWidget();

    // �޸��������;
    void onModifyTaskTable(QString task_edit_uuid);

signals:
    // ������ذ�ť��ת������չʾҳ��;
    void signalGoBackToTaskShow();

	//�������ڵ��level��Ϣ
	void signalUpdateTreeLevel(QString strUUid, DeviceAlarmLevel iLevel);

public slots:
	// ���ص�ǰ����������;
	void onGetSaveOperationResult(bool isSuccess, QString strMsg);

	// ����ִ�����Ͱ�ť���;
	void onTaskOperateTypeButtonClicked(int buttonId, WheelTaskEditStruct data);

    // Core����ɾ���������;
    void onDeleteTaskCallBack(bool isSuccess, QString strMsg);

private:

	/************************************************************************/
	/*  ��ʼ���ϰ벿��widget;                                               */
	/************************************************************************/
	// ��ʼ�������Widget;
	void initLineEditWidget();
	// ��ʼ��RadioButtonWidget;
	void initPatrolTypeWidget();
	void initSpecialModeWidget();
	// ˢ��checkBox;
	void refreshCheckBox(WheelTaskAdminType taskPageType);
	// ��ʼ����ѯwidget;
	void initSearchWidget();
	void initTopWidget();


	/************************************************************************/
	/*  ��ʼ���°벿��widget;                                               */
	/************************************************************************/
	// ��ʼ����λ��;
	void initPointPosTreeWidget();

	// ��ʼ����������б�;
	void initTaskFormulateTable();

	// ��ʼ���������;
	void initTableData();
    // ��ʼ���²��ؼ�;
	void initBottomWidget();
    // �����¼�;
	void paintEvent(QPaintEvent* event);

	// ��ȡ��ǰ��������;
	QString getCurrentTaskTypeName();

private slots:
	// ��ѯ;
	void onSearchButtonClicked();
	// ����;
	void onSaveButtonClicked();
    // ����;
    void onResetButtonClicked();

    // table˫��ʱ����豸����ѡ��ǰ������ѡ����豸;
    void onTaskTableDoubleClicked(int row);

private:
	// ��������Ϊ����������;

	// �����ϰ벿��widget;
	QWidget* m_topBackWidget;
	// �����backWidget;
	QWidget* m_lineEditBackWidget;
	// ��������;
	InputWidget* m_taskNameInputWidget;
	// ��������;
	InputWidget* m_taskDescribeInputWidget;

	// Ѳ������backWidget;
	QWidget* m_patrolTypeBackWidget;
	// Ѳ������buttonGroup;
	QButtonGroup*  m_patrolTypeButtonGroup;
	
	// ��ѲģʽbackWidget;
	QWidget* m_specialModeBackWidget;
	// ��ѲģʽbuttonGroup;
	QButtonGroup*  m_specialModeButtonGroup;

	// ��ѯwidget;
	QWidget* m_searchBackWidget;
	QLineEdit* m_deviceSearchLineEdit;
	QToolButton* m_pButtonSearch;
	QToolButton* m_pButtonSave;
	QToolButton* m_pButtonReset;
	QPushButton* m_pButtonImportTask;
	QPushButton* m_pButtonGoBack;

	// �����°벿��widget;
	QWidget* m_bottomBackWidget;

	// ��λ��;
	CustomTreeWidget* m_pointPosTreeWidget;

	// ��������б�;
	QWidget* m_taskFormulateTableBackWidget;
	CustomTableWidget* m_taskFormulateTable;

	// radioButton֮��ļ��;
	int m_radioButtonInterval;

	// ��ǰҳ������;
	WheelTaskAdminType m_taskPageType;

	QWidget* m_checkBoxBackWidget;

	QList<CheckBoxWidget*> m_checkBoxWidgetList;

	// ��ǰҳ��Ĳ˵�����;
	SystemGuideMenuItemType m_itemType;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰtable��ҳ��;
    int m_currentPageIndex;

    // ��ǰ�Ƿ����޸�״̬;
    // ��Ϊ����޸�ʱ��չ�����ڵ�Ҫ�Ե�ǰ�����Ѿ���ѡ���豸���д�;
    bool m_isModifyStatus;

    // ���浱ǰ�޸�ʱ���ص�����;
    WheelTaskEditStruct m_modifyTaskEditStruct;

    // Table��ǰ����;
    QList<WheelTaskEditStruct> m_tableDataList;

    // ��ʷ������;
    HistoryTaskImportWindow* m_historyTaskImportWindow;

    // ���ؼ����ݲ�ѯ����;
    DLWheelPointTreeData* m_wheelPointTreeData;
};


#endif // !DL_WHEEL_TASK_MANAGE_H
