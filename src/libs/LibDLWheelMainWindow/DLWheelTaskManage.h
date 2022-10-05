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

// 巡检类型;
enum PatrolType
{
	Routine_Patrol = 0,				// 例行巡视;
	All_Patrol,						// 全面巡视;
	Single_Patrol,					// 专项巡视;
	Special_Patrol,					// 特殊巡视;
};

// 特殊模式类型;
enum SpecialModeType
{
	ClimbSummer_Mode = 0,			// 迎峰度夏特巡;
	ThunderStorm_Mode,				// 雷暴天气特巡;
	FloodPrevention_Mode,			// 防汛坑台特巡;
	SleetFrozen_Mode,				// 雨雪冰冻特巡;
	HazeWeahter_Mode,				// 雾霾天气特巡;
	GaleWeather_Mode,				// 大风天气特巡;
};

// 任务执行类型;
enum TaskExcuteType
{
	ImmediatelyExcute = 0,
	FixedTimeExcute,
	PeriodExcute,
};

class InputWidget;

/**********历史任务导入窗口**********/

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
        this->setTitleContent("任务导入");
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
                DLMessageBox::showDLMessageBox(NULL, "错误", "导入失败: " + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
            this->setDisabled(false);
        }, Qt::QueuedConnection);
    }

    // 重置窗口;
    void resetWidget()
    {
        m_dataList = WHEEL_TASK_ADMINISTRATION.getWheelTaskHistoryStru("", "", "");
        initListWidget();
    }

private:

    // 初始化任务列表;
    void initListWidget()
    {
        m_taskNameListWidget->clear();
        for (int i = 0; i < m_dataList.count(); i ++)
        {
            m_taskNameListWidget->addItem(m_dataList[i].task_edit_name);
        }
    }

    // 初始化任务名称控件;
    void initTaskNameWidget()
    {
        m_taskNameModifyWidget = new BaseWidget(this, BaseWidgetType::PopupWindow);
        m_taskNameModifyWidget->setTitleContent("任务名称");
        m_taskNameModifyWidget->setWindowModality(Qt::ApplicationModal);

        m_modifyTaskNameLineEdit = new QLineEdit;

        // 导入;
        QPushButton* pButtonOK = new QPushButton("确定");
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

        QPushButton* pButtonCancel = new QPushButton("取消");
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

    // 初始化超时消息框;
    void initOverTimeTimer()
    {
        m_overtimeTimer.setInterval(5000);
        connect(&m_overtimeTimer, &QTimer::timeout, this, [=] {
            DLMessageBox::showDLMessageBox(NULL, "错误", "操作超时", MessageButtonType::BUTTON_OK, true, QSize(250, 180));

            this->setDisabled(false);
        });
    }

    // 初始化控件;
    void initWidget()
    {
        m_taskNameLineEdit = new InputWidget(InputWidgetType::WheelLineEdit);
        m_taskNameLineEdit->setTipText("任务名称");

        QPushButton* pButtonSearch = new QPushButton("查询");
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

        QLabel* startTimeLabel = new QLabel("开始时间:");

        m_taskFinishStartDateEdit = new QDateTimeEdit;
        m_taskFinishStartDateEdit->setDisplayFormat("yyyy-MM-dd");
        m_taskFinishStartDateEdit->setDate(QDate::currentDate());
        m_taskFinishStartDateEdit->setCalendarPopup(true);
        m_taskFinishStartDateEdit->setFixedWidth(90);
        QCalendarWidget* calendarWidget = new QCalendarWidget(m_taskFinishStartDateEdit);
        m_taskFinishStartDateEdit->setCalendarWidget(calendarWidget);

        QLabel* endTimeLabel = new QLabel("结束时间:");
        
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

        // 导入;
        QPushButton* pButtonImport = new QPushButton("导入");
        pButtonImport->setFixedSize(QSize(60, 25));
        connect(pButtonImport, &QPushButton::clicked, this, &HistoryTaskImportWindow::onImportTask);

        QPushButton* pButtonCancel = new QPushButton("取消");
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

    // 导入任务;
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
            DLMessageBox::showDLMessageBox(NULL, "提示", "未选择任务", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    }

signals:
    // 窗口关闭;
    void signalWindowClose();
    // 导入返回;
    void signalImportCallBack(bool, QString);
    // 导入成功;
    void signalImportSuccess();

private:
    // 任务模板列表;
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

/*******自定义按钮(table中立即执行/定期执行/周期执行)********/

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
		m_pButtonImmediately = new QPushButton("立即执行");
		m_pButtonImmediately->setFixedSize(QSize(70, 25));
		buttonGroup->addButton(m_pButtonImmediately, ImmediatelyExcute);

		m_pButtonFixedTime = new QPushButton("定期执行");
		m_pButtonFixedTime->setFixedSize(QSize(70, 25));
		buttonGroup->addButton(m_pButtonFixedTime, FixedTimeExcute);

		m_pButtonPeriod = new QPushButton("周期执行");
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

    // 设置在table中第几行;
	void setRowIndex(int rowIndex)
	{
		m_rowIndex = rowIndex;
	}

    // 获取在table中第几行;
	int getRowIndex()
	{
		return m_rowIndex;
	}

signals:
    // 按钮点击信号;
	void signalButtonClicked(int buttonId, WheelTaskEditStruct data);

private:
	QPushButton* m_pButtonImmediately;
	QPushButton* m_pButtonFixedTime;
	QPushButton* m_pButtonPeriod;
	int m_rowIndex;

	WheelTaskEditStruct m_buttonData;
};

class CustomTableWidget;

/*********任务管理菜单中所有页面;************/

class DLWheelTaskManage : public QWidget
{
	Q_OBJECT

public:
	DLWheelTaskManage(SystemGuideMenuItemType itemType = Menu_All_Patrol);
	~DLWheelTaskManage();

	int GetMenuItemType();

	// 设置当前任务页面类型;
	void setTaskShowType(SystemGuideMenuItemType itemType);

	// 从菜单点入之后再进行初始化;
	void initWidget();

    // 修改任务编制;
    void onModifyTaskTable(QString task_edit_uuid);

signals:
    // 点击返回按钮跳转回任务展示页面;
    void signalGoBackToTaskShow();

	//更新树节点的level信息
	void signalUpdateTreeLevel(QString strUUid, DeviceAlarmLevel iLevel);

public slots:
	// 返回当前保存操作结果;
	void onGetSaveOperationResult(bool isSuccess, QString strMsg);

	// 任务执行类型按钮点击;
	void onTaskOperateTypeButtonClicked(int buttonId, WheelTaskEditStruct data);

    // Core返回删除任务编制;
    void onDeleteTaskCallBack(bool isSuccess, QString strMsg);

private:

	/************************************************************************/
	/*  初始化上半部分widget;                                               */
	/************************************************************************/
	// 初始化输入框Widget;
	void initLineEditWidget();
	// 初始化RadioButtonWidget;
	void initPatrolTypeWidget();
	void initSpecialModeWidget();
	// 刷新checkBox;
	void refreshCheckBox(WheelTaskAdminType taskPageType);
	// 初始化查询widget;
	void initSearchWidget();
	void initTopWidget();


	/************************************************************************/
	/*  初始化下半部分widget;                                               */
	/************************************************************************/
	// 初始化点位树;
	void initPointPosTreeWidget();

	// 初始化任务编制列表;
	void initTaskFormulateTable();

	// 初始化表格数据;
	void initTableData();
    // 初始化下部控件;
	void initBottomWidget();
    // 绘制事件;
	void paintEvent(QPaintEvent* event);

	// 获取当前任务类型;
	QString getCurrentTaskTypeName();

private slots:
	// 查询;
	void onSearchButtonClicked();
	// 保存;
	void onSaveButtonClicked();
    // 重置;
    void onResetButtonClicked();

    // table双击时左边设备树勾选当前任务所选择的设备;
    void onTaskTableDoubleClicked(int row);

private:
	// 任务管理分为上下两部分;

	// 窗口上半部分widget;
	QWidget* m_topBackWidget;
	// 输入框backWidget;
	QWidget* m_lineEditBackWidget;
	// 任务名称;
	InputWidget* m_taskNameInputWidget;
	// 任务描述;
	InputWidget* m_taskDescribeInputWidget;

	// 巡检类型backWidget;
	QWidget* m_patrolTypeBackWidget;
	// 巡检类型buttonGroup;
	QButtonGroup*  m_patrolTypeButtonGroup;
	
	// 特巡模式backWidget;
	QWidget* m_specialModeBackWidget;
	// 特巡模式buttonGroup;
	QButtonGroup*  m_specialModeButtonGroup;

	// 查询widget;
	QWidget* m_searchBackWidget;
	QLineEdit* m_deviceSearchLineEdit;
	QToolButton* m_pButtonSearch;
	QToolButton* m_pButtonSave;
	QToolButton* m_pButtonReset;
	QPushButton* m_pButtonImportTask;
	QPushButton* m_pButtonGoBack;

	// 窗口下半部分widget;
	QWidget* m_bottomBackWidget;

	// 点位树;
	CustomTreeWidget* m_pointPosTreeWidget;

	// 任务编制列表;
	QWidget* m_taskFormulateTableBackWidget;
	CustomTableWidget* m_taskFormulateTable;

	// radioButton之间的间距;
	int m_radioButtonInterval;

	// 当前页面类型;
	WheelTaskAdminType m_taskPageType;

	QWidget* m_checkBoxBackWidget;

	QList<CheckBoxWidget*> m_checkBoxWidgetList;

	// 当前页面的菜单类型;
	SystemGuideMenuItemType m_itemType;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 当前table的页数;
    int m_currentPageIndex;

    // 当前是否处于修改状态;
    // 因为点击修改时，展开树节点要对当前任务已经勾选的设备进行打钩;
    bool m_isModifyStatus;

    // 保存当前修改时返回的数据;
    WheelTaskEditStruct m_modifyTaskEditStruct;

    // Table当前数据;
    QList<WheelTaskEditStruct> m_tableDataList;

    // 历史任务导入;
    HistoryTaskImportWindow* m_historyTaskImportWindow;

    // 树控件数据查询对象;
    DLWheelPointTreeData* m_wheelPointTreeData;
};


#endif // !DL_WHEEL_TASK_MANAGE_H
