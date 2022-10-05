#include "DLWheelTaskShow.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelCustomWidget/CustomCalendarWidget.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QCalendarWidget>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QMenu>

#pragma execution_character_set("utf-8")

#define TABLE_PER_PAGE_COUNT 20							// table每页总条数;

DLWheelTaskShow::DLWheelTaskShow(QWidget* parent)
	: QWidget(parent)
	, m_isInitWidget(false)
    , m_currentPageIndex(1)
{
	this->setStyleSheet("QToolButton{border:none;}\
							QWidget#CommonBackWidget{background:rgb(175, 191, 255);border:1px solid rgb(166, 233, 210);}\
							QTableWidget{border:none;}");

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotUpdateTaskStatusStatus.connect(boost::bind(&DLWheelTaskShow::signalTaskStatusModifyCallBack, this, _1, _2));

    connect(this, &DLWheelTaskShow::signalTaskStatusModifyCallBack, this, [=](bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            initTableData(m_currentSeachCondition);
            DLMessageBox::showDLMessageBox(NULL, "提示", "任务状态修改成功", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "错误", "任务状态修改失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });    
}

void DLWheelTaskShow::initTaskExecuteWidget()
{
	m_taskExecuteTimeWidget = new QWidget;
	m_taskExecuteTimeWidget->setObjectName("CommonBackWidget");
	m_taskExecuteTimeWidget->setFixedHeight(30);

	QLabel* labelTaskExecuteTime = new QLabel("任务执行时间:");

	m_pButtonSearchLeft = new QToolButton;
	m_pButtonSearchLeft->setText("查询");
	m_pButtonSearchLeft->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonSearchLeft->setFixedSize(QSize(60, 20));
	m_pButtonSearchLeft->setIcon(QIcon(":/Resources/Common/image/Search.png"));
	m_pButtonSearchLeft->setIconSize(QSize(16, 16));
	connect(m_pButtonSearchLeft, &QToolButton::clicked, this, [=] {
		QDate searchData = m_executeTimeEdit->date();
		QMap<int, QList<WheelCalendarData>> dataMap = WHEEL_TASK_ADMINISTRATION.getCalendarTaskMap(searchData.toString("yyyy-MM"), searchData.addMonths(1).toString("yyyy-MM"));
		m_taskDateChooseBackWidget->setDate(searchData, dataMap);
	});

	QCalendarWidget* dateWidget = new QCalendarWidget;
	m_executeTimeEdit = new QDateEdit;
	m_executeTimeEdit->setFixedSize(QSize(100, 25));
	m_executeTimeEdit->setCalendarPopup(true);
	m_executeTimeEdit->setCalendarWidget(dateWidget);
	m_executeTimeEdit->setDate(QDate::currentDate());
	m_executeTimeEdit->setDisplayFormat("yyyy年MM月");

	QHBoxLayout* hExecuteTimeLayout = new QHBoxLayout(m_taskExecuteTimeWidget);
	hExecuteTimeLayout->addWidget(labelTaskExecuteTime);
	hExecuteTimeLayout->addWidget(m_executeTimeEdit);
	hExecuteTimeLayout->addWidget(m_pButtonSearchLeft);
	hExecuteTimeLayout->addStretch();
	hExecuteTimeLayout->setSpacing(15);
	hExecuteTimeLayout->setContentsMargins(10, 0, 0, 0);
}

void DLWheelTaskShow::initLeftWidget()
{
	initTaskExecuteWidget();

	QDesktopWidget* desktopWidget = QApplication::desktop();
	QRect screenRect = desktopWidget->screenGeometry();

	m_leftBackWidget = new QWidget;
	m_leftBackWidget->setFixedWidth(screenRect.width() / 2);

	m_taskDateChooseBackWidget = new CustomCalendarWidget;
	QDate searchData = QDate::currentDate();
	QMap<int, QList<WheelCalendarData>> dataMap = WHEEL_TASK_ADMINISTRATION.getCalendarTaskMap(searchData.toString("yyyy-MM"), searchData.addMonths(1).toString("yyyy-MM"));
	m_taskDateChooseBackWidget->setDate(searchData, dataMap);
	m_taskDateChooseBackWidget->setObjectName("TaskDateChooseBackWidget");
    connect(m_taskDateChooseBackWidget, &CustomCalendarWidget::signalMouseDoubleClickeDateItem, this, [=] (QDate itemDate){
        m_startDateEdit->setDate(itemDate);
        m_endDateEdit->setDate(itemDate.addDays(1));
        QString strDate = itemDate.toString("yyyy-MM-dd");
        m_pButtonSearchRight->clicked();
    });


	m_colorRemarkBackWidget = new QWidget;
	m_colorRemarkBackWidget->setFixedHeight(30);
	m_colorRemarkBackWidget->setObjectName("CommonBackWidget");
	
	QLabel* colorRemarkLabel = new QLabel;
	colorRemarkLabel->setTextFormat(Qt::RichText);
	colorRemarkLabel->setText("备注: <a style=\"color:green\">执行完成: 绿色&nbsp;&nbsp;</a>\
									<a style=\"color:rgb(140,70,20)\">中途中止: 褐色&nbsp;&nbsp;</a>\
									<a style=\"color:red\">正在执行: 红色&nbsp;&nbsp;</a>\
									<a style=\"color:blue\">等待执行: 蓝色&nbsp;&nbsp;</a>\
									<a style=\"color:yellow\">任务超期: 黄色</a>");

	QHBoxLayout* hColorRemarkLayout = new QHBoxLayout(m_colorRemarkBackWidget);
	hColorRemarkLayout->addWidget(colorRemarkLabel);
	hColorRemarkLayout->addStretch();
	hColorRemarkLayout->setContentsMargins(5, 0, 0, 0);

	QVBoxLayout* vLeftLayout = new QVBoxLayout(m_leftBackWidget);
	vLeftLayout->addWidget(m_taskExecuteTimeWidget);
	vLeftLayout->addWidget(m_taskDateChooseBackWidget);
	vLeftLayout->addWidget(m_colorRemarkBackWidget);
	vLeftLayout->setSpacing(0);
	vLeftLayout->setMargin(0);
}

void DLWheelTaskShow::initTaskSelectWidget()
{
	m_taskSelectBackWidget = new QWidget;
	m_taskSelectBackWidget->setObjectName("TaskSelectBackWidget");

	QLabel* labelTaskState = new QLabel("任务状态:");
	labelTaskState->setFixedWidth(70);
	labelTaskState->setAlignment(Qt::AlignTop);

	m_taskStateButtonGrouop = new QButtonGroup(this);
    // 设置checkBox选择不唯一;
    m_taskStateButtonGrouop->setExclusive(false);
	for (int i = 0; i < 5; i++)
	{
		QCheckBox* checkBox = new QCheckBox;
		m_taskStateButtonGrouop->addButton(checkBox, i);
	}

	m_taskStateButtonGrouop->button(TaskState_WaitForExecute)->setText("等待执行");
	m_taskStateButtonGrouop->button(TaskState_ExecuteFinished)->setText("已执行");
	m_taskStateButtonGrouop->button(TaskState_Executing)->setText("正在执行");
	m_taskStateButtonGrouop->button(TaskState_Abort)->setText("中止");
	m_taskStateButtonGrouop->button(TaskState_TaskOutOfDate)->setText("任务超期");
	connect(m_taskStateButtonGrouop, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &DLWheelTaskShow::onTaskStateChooseChanged);

	QLabel* labelStartTime = new QLabel("开始时间:");
	labelStartTime->setFixedWidth(70);

	QLabel* labelEndTime = new QLabel("结束时间:");
	labelEndTime->setFixedWidth(70);

	QCalendarWidget* startDateWidget = new QCalendarWidget;
	m_startDateEdit = new QDateEdit;
	m_startDateEdit->setFixedSize(QSize(100, 25));
	m_startDateEdit->setCalendarPopup(true);
	m_startDateEdit->setCalendarWidget(startDateWidget);
	m_startDateEdit->setDate(QDate::currentDate().addMonths(-1));
	m_startDateEdit->setDisplayFormat("yyyy-MM-dd");

	QCalendarWidget* endDateWidget = new QCalendarWidget;
	m_endDateEdit = new QDateEdit;
	m_endDateEdit->setFixedSize(QSize(100, 25));
	m_endDateEdit->setCalendarPopup(true);
	m_endDateEdit->setCalendarWidget(endDateWidget);
	m_endDateEdit->setDate(QDate::currentDate().addMonths(1));
	m_endDateEdit->setDisplayFormat("yyyy-MM-dd");

	QLabel* labelTaskName = new QLabel("任务名称:");
	labelTaskName->setFixedWidth(70);

	m_taskNameLineEdit = new QLineEdit;
	m_taskNameLineEdit->setFixedSize(QSize(100, 25));

	/********* 开始布局 **********/
	// 任务状态布局;
	QGridLayout* gCheckBoxLayout = new QGridLayout;
	gCheckBoxLayout->addWidget(m_taskStateButtonGrouop->button(TaskState_WaitForExecute), 0, 0);
	gCheckBoxLayout->addItem(new QSpacerItem(40, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 0, 1);
	gCheckBoxLayout->addWidget(m_taskStateButtonGrouop->button(TaskState_ExecuteFinished), 0, 2);
	gCheckBoxLayout->addItem(new QSpacerItem(40, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 0, 3);
	gCheckBoxLayout->addWidget(m_taskStateButtonGrouop->button(TaskState_Executing), 0, 4);
	gCheckBoxLayout->addItem(new QSpacerItem(40, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 0, 5);
	gCheckBoxLayout->addWidget(m_taskStateButtonGrouop->button(TaskState_Abort), 1, 0);
	gCheckBoxLayout->addItem(new QSpacerItem(40, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 1, 1);
	gCheckBoxLayout->addWidget(m_taskStateButtonGrouop->button(TaskState_TaskOutOfDate), 1, 2);
	gCheckBoxLayout->addItem(new QSpacerItem(40, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 1, 3);
	gCheckBoxLayout->setVerticalSpacing(15);
	gCheckBoxLayout->setMargin(0);

	QHBoxLayout* hTaskStateLayout = new QHBoxLayout;
	hTaskStateLayout->addWidget(labelTaskState);
	hTaskStateLayout->addLayout(gCheckBoxLayout);
	hTaskStateLayout->setSpacing(15);

	QHBoxLayout* hTimeLayout = new QHBoxLayout;
	hTimeLayout->addWidget(labelStartTime);
	hTimeLayout->addWidget(m_startDateEdit);
	hTimeLayout->addWidget(labelEndTime);
	hTimeLayout->addWidget(m_endDateEdit);
	hTimeLayout->addStretch();
	hTimeLayout->setSpacing(15);
	hTimeLayout->setMargin(0);

	QHBoxLayout* hTaskNameLayout = new QHBoxLayout;
	hTaskNameLayout->addWidget(labelTaskName);
	hTaskNameLayout->addWidget(m_taskNameLineEdit);
	hTaskNameLayout->addStretch();
	hTaskNameLayout->setSpacing(15);
	hTaskNameLayout->setMargin(0);

	QVBoxLayout* vTaskSelectLayout = new QVBoxLayout(m_taskSelectBackWidget);
	vTaskSelectLayout->addLayout(hTaskStateLayout);
	vTaskSelectLayout->addLayout(hTimeLayout);
	vTaskSelectLayout->addLayout(hTaskNameLayout);
	vTaskSelectLayout->setSpacing(10);
	vTaskSelectLayout->setMargin(5);
}

void DLWheelTaskShow::initOperateButtonWidget()
{
	m_operateButtonBackWidget = new QWidget;
	m_operateButtonBackWidget->setFixedHeight(30);
	m_operateButtonBackWidget->setObjectName("CommonBackWidget");

	m_pButtonSearchRight = new QToolButton;
	m_pButtonSearchRight->setText("查询");
	m_pButtonSearchRight->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonSearchRight->setFixedSize(QSize(60, 20));
	m_pButtonSearchRight->setIcon(QIcon(":/Resources/Common/image/Search.png"));
	m_pButtonSearchRight->setIconSize(QSize(16, 16));
	connect(m_pButtonSearchRight, &QPushButton::clicked, this, [=] {
		m_currentPageIndex = 1;
		WheelTaskListSearchIndex searchIndex;
		for (int i = 0; i < m_taskStateButtonGrouop->buttons().count(); i++)
		{
			if (m_taskStateButtonGrouop->button(i)->isChecked())
			{
				searchIndex.m_task_status_id.append(QString::number(i + 1));
			}
		}
		searchIndex.m_start_time = m_startDateEdit->date().toString("yyyy-MM-dd");
		searchIndex.m_stop_time = m_endDateEdit->date().toString("yyyy-MM-dd");
		searchIndex.m_task_name = m_taskNameLineEdit->text();

        m_currentSeachCondition = searchIndex;
		initTableData(m_currentSeachCondition);
	});

	m_pButtonAdd = new QToolButton;
	m_pButtonAdd->setText("添加");
	m_pButtonAdd->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonAdd->setFixedSize(QSize(60, 20));
	m_pButtonAdd->setIcon(QIcon(":/Resources/Common/image/Button_Add.png"));
	m_pButtonAdd->setIconSize(QSize(16, 16));
	connect(m_pButtonAdd, &QPushButton::clicked, this, [=] {
		emit signalJumpToAllPatrolPage();
	});

	m_pButtonDelete = new QToolButton;
	m_pButtonDelete->setText("删除");
	m_pButtonDelete->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonDelete->setFixedSize(QSize(60, 20));
	m_pButtonDelete->setIcon(QIcon(":/Resources/Common/image/Button_Delete.png"));
	m_pButtonDelete->setIconSize(QSize(16, 16));
	connect(m_pButtonDelete, &QPushButton::clicked, this, &DLWheelTaskShow::onDeleteButtonClicked);

	QHBoxLayout* hButtonLayout = new QHBoxLayout(m_operateButtonBackWidget);
	hButtonLayout->addWidget(m_pButtonSearchRight);
	hButtonLayout->addWidget(new SpliterLabel);
	hButtonLayout->addWidget(m_pButtonAdd);
	hButtonLayout->addWidget(new SpliterLabel);
	hButtonLayout->addWidget(m_pButtonDelete);
	hButtonLayout->addStretch();
	hButtonLayout->setSpacing(10);
	hButtonLayout->setContentsMargins(10, 0, 0, 0);
}

void DLWheelTaskShow::initTaskShowTable()
{
	m_taskTableBackWidget = new QWidget;
	m_taskTableBackWidget->setObjectName("TaskTableBackWidget");

	m_taskShowTableWidget = new CustomTableWidget(5);
	m_taskShowTableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "任务名称" << "执行时间" << "任务状态" << "操作");
    connect(m_taskShowTableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableData(m_currentSeachCondition);
    });
    connect(m_taskShowTableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableData(m_currentSeachCondition);
    });

	QTableWidget* tableWidget = m_taskShowTableWidget->getTableWidget();
  	tableWidget->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);
 	tableWidget->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Fixed);

    connect(tableWidget, &QTableWidget::cellPressed, this, [=](int row) {
        if (QApplication::mouseButtons() == Qt::RightButton)
        {
			QTableWidgetItem *pItem = tableWidget->item(row, 4);
			if (nullptr != pItem)
			{
				if ("等待执行" == pItem->text())
				{
					if (!m_tableDataList[row].task_uuid.isEmpty())
					{//正常任务
						QMenu* taskStatusModifyMenu = new QMenu;
						QAction* taskAbortAction = taskStatusModifyMenu->addAction("任务中止");
						QAction* taskFinishAction = taskStatusModifyMenu->addAction("任务结束");
						connect(taskAbortAction, &QAction::triggered, this, [=] {
							if (row < m_tableDataList.count())
							{
								WHEEL_BACK_TO_CORE_SOCKET.robot_update_task_status_req(m_tableDataList[row].task_uuid, WHEEL_ROBOT_TASK_STATUS_ABORT);
							}
						});

						connect(taskFinishAction, &QAction::triggered, this, [=] {
							if (row < m_tableDataList.count())
							{
								WHEEL_BACK_TO_CORE_SOCKET.robot_update_task_status_req(m_tableDataList[row].task_uuid, WHEEL_ROBOT_TASK_STATUS_FINISH);
							}
						});
						taskStatusModifyMenu->exec(QCursor::pos());
						taskStatusModifyMenu->deleteLater();

						
					}
					else if (!m_tableDataList[row].task_template_uuid.isEmpty())
					{//周期任务
						QMenu* taskStatusModifyMenu = new QMenu;
						QAction *pTaskCancelAction = taskStatusModifyMenu->addAction("任务挂起");
						connect(pTaskCancelAction, &QAction::triggered, this, [=] {
							if (row < m_tableDataList.count())
							{
								WHEEL_BACK_TO_CORE_SOCKET.robot_update_task_status_req(m_tableDataList[row].task_template_uuid, WHEEL_ROBOT_TASK_STATUS_HANGUP);
							}
						});
						taskStatusModifyMenu->exec(QCursor::pos());
						taskStatusModifyMenu->deleteLater();
					}
				}
				else if ("任务挂起" == pItem->text() && !m_tableDataList[row].task_template_uuid.isEmpty())
				{
					QMenu* taskStatusModifyMenu = new QMenu;
					QAction *pTaskCancelAction = taskStatusModifyMenu->addAction("取消挂起");
					connect(pTaskCancelAction, &QAction::triggered, this, [=] {
						if (row < m_tableDataList.count())
						{
							WHEEL_BACK_TO_CORE_SOCKET.robot_update_task_status_req(m_tableDataList[row].task_template_uuid, WHEEL_ROBOT_TASK_STATUS_HANGDOWN);
						}
					});
					taskStatusModifyMenu->exec(QCursor::pos());
					taskStatusModifyMenu->deleteLater();
				}
				else if ("中止" == pItem->text() && !m_tableDataList[row].task_uuid.isEmpty())
				{
					QMenu* taskStatusModifyMenu = new QMenu;
					QAction* taskFinishAction = taskStatusModifyMenu->addAction("任务结束");

					connect(taskFinishAction, &QAction::triggered, this, [=] {
						if (row < m_tableDataList.count())
						{
							WHEEL_BACK_TO_CORE_SOCKET.robot_update_task_status_req(m_tableDataList[row].task_uuid, WHEEL_ROBOT_TASK_STATUS_FINISH);
						}
					});
					taskStatusModifyMenu->exec(QCursor::pos());
					taskStatusModifyMenu->deleteLater();
				}
			}
        }
    });

	QVBoxLayout* vTableLayout = new QVBoxLayout(m_taskTableBackWidget);
	vTableLayout->addWidget(m_taskShowTableWidget);
	vTableLayout->setSpacing(0);
	vTableLayout->setMargin(0);
}

void DLWheelTaskShow::initTableData(WheelTaskListSearchIndex seachIndex)
{
    // 先获取页数，把条件设置完之后再获取数据;
    int totalPage, totalCount;
    WHEEL_TASK_ADMINISTRATION.getWheelTaskListPage(totalPage, totalCount, TABLE_PER_PAGE_COUNT, seachIndex);
    m_taskShowTableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

	QTableWidget* tableWidget = m_taskShowTableWidget->getTableWidget();
    m_tableDataList = WHEEL_TASK_ADMINISTRATION.getTaskListShow(m_currentPageIndex, TABLE_PER_PAGE_COUNT);
	tableWidget->clearContents();
	tableWidget->setRowCount(0);


	for (int i = 0; i < m_tableDataList.count(); i++)
	{
		tableWidget->insertRow(i);
		QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
		checkBoxItem->setCheckState(Qt::Unchecked);
		checkBoxItem->setTextAlignment(Qt::AlignCenter);
		tableWidget->setItem(i, 0, checkBoxItem);

		QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
		idItem->setTextAlignment(Qt::AlignCenter);
		tableWidget->setItem(i, 1, idItem);

		QTableWidgetItem* taskNameItem = new QTableWidgetItem(m_tableDataList[i].task_name);
		tableWidget->setItem(i, 2, taskNameItem);
		taskNameItem->setTextAlignment(Qt::AlignCenter);

		QTableWidgetItem* taskDateItem = new QTableWidgetItem(m_tableDataList[i].task_start_time);
		taskDateItem->setTextAlignment(Qt::AlignCenter);
		tableWidget->setItem(i, 3, taskDateItem);

		QColor textColor = getTaskStatusColor(m_tableDataList[i].task_status_name);
		QTableWidgetItem* taskStateItem = new QTableWidgetItem(m_tableDataList[i].task_status_name);
		taskStateItem->setTextAlignment(Qt::AlignCenter);
		taskStateItem->setForeground(textColor);
		tableWidget->setItem(i, 4, taskStateItem);

		QTableWidgetItem* buttonItem = new QTableWidgetItem;
		tableWidget->setItem(i, 5, buttonItem);
		ModifyButtonWidget* modifyButtonWidget = new ModifyButtonWidget(m_tableDataList[i]);
        modifyButtonWidget->setCurrentRow(i);
		connect(modifyButtonWidget, &ModifyButtonWidget::signalModifyButtonClicked, this, &DLWheelTaskShow::onTableModifyButtonClicked);
		tableWidget->setCellWidget(i, 5, modifyButtonWidget);

		tableWidget->setRowHeight(i, 40);
	}
}

void DLWheelTaskShow::initRightWidget()
{
	initTaskSelectWidget();
	initOperateButtonWidget();
	initTaskShowTable();

	m_rightBackWidget = new QWidget;
	
	QVBoxLayout* vRightLayout = new QVBoxLayout(m_rightBackWidget);
	vRightLayout->addWidget(m_taskSelectBackWidget);
	vRightLayout->addWidget(m_operateButtonBackWidget);
	vRightLayout->addWidget(m_taskTableBackWidget);
	vRightLayout->setSpacing(0);
	vRightLayout->setMargin(0);
}

void DLWheelTaskShow::initWidget()
{
	if (m_isInitWidget)
	{
        onRefreshCalendar();
        initTableData(m_currentSeachCondition);
		return;
	}
	m_isInitWidget = true;
	initLeftWidget();
	initRightWidget();

    // 初始化给前后两个月时间;
    QDate currentData = QDate::currentDate();
    m_startDateEdit->setDate(currentData.addMonths(-1));
    m_endDateEdit->setDate(currentData.addMonths(1));
    
    m_currentSeachCondition.m_start_time = currentData.addMonths(-1).toString("yyyy-MM-dd");
    m_currentSeachCondition.m_stop_time = currentData.addMonths(1).toString("yyyy-MM-dd");

	initTableData(m_currentSeachCondition);
	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_leftBackWidget);
	hMainLayout->addWidget(m_rightBackWidget);
	hMainLayout->setSpacing(0);
	hMainLayout->setMargin(0);
}

void DLWheelTaskShow::onUpdateDeleteResult(QString strUuid, bool isSuccess, QString strMsg)
{
    m_currentPageIndex = 1;
    m_taskShowTableWidget->setHeaderCheckBoxState(false);
    initWidget();
	if (isSuccess)
	{
		initTableData(m_currentSeachCondition);
        onRefreshCalendar();
	}
	else
	{
		DLMessageBox* messageBox = new DLMessageBox(this);
		messageBox->setFixedWidth(250);
		messageBox->setMessageContent("删除失败");
		messageBox->setWindowModality(Qt::ApplicationModal);
		messageBox->show();
	}
}

QColor DLWheelTaskShow::getTaskStatusColor(QString strSaskStatus)
{
	QColor color;
	if (!strSaskStatus.compare("已执行"))
	{
		color = Qt::green;
	}
	else if (!strSaskStatus.compare("中止"))
	{
		color = QColor(140, 70, 20);
	}
	else if (!strSaskStatus.compare("正在执行"))
	{
		color = Qt::red;
	}
	else if (!strSaskStatus.compare("等待执行"))
	{
		color = Qt::blue;
	}
	else if (!strSaskStatus.compare("任务超期"))
	{
		color = Qt::yellow;
	}
	
	return color;
}

void DLWheelTaskShow::onTaskStateChooseChanged(int buttonId)
{
	CheckBoxTaskState taskState = CheckBoxTaskState(buttonId);

	switch (taskState)
	{
	case TaskState_WaitForExecute:
		break;
	case TaskState_ExecuteFinished:
		break;
	case TaskState_Executing:
		break;
	case TaskState_Abort:
		break;
	case TaskState_TaskOutOfDate:
		break;
	default:
		break;
	}
}

void DLWheelTaskShow::onTableModifyButtonClicked(WheelTaskListShowStruct itemData)
{
    ModifyButtonWidget* modifyButtonWidget = static_cast<ModifyButtonWidget*>(sender());

    if (modifyButtonWidget != NULL)
    {
        int currentRow = modifyButtonWidget->getCurrentRow();
        QTableWidget* tableWidget = m_taskShowTableWidget->getTableWidget();
        QString strTaskState = tableWidget->item(currentRow, 4)->text();
        if (strTaskState == "已执行" || strTaskState == "正在执行")
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "已执行和正在执行任务不可修改", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return;
        }
    }
	// 通知跳转到对应页面;
	emit signalModifyButtonClicked(itemData.task_edit_type_id, itemData.task_edit_uuid);
}

void DLWheelTaskShow::onRefreshCalendar()
{
	QDate currentDate = m_taskDateChooseBackWidget->getCurrentDate();
	QMap<int, QList<WheelCalendarData>> dataMap = WHEEL_TASK_ADMINISTRATION.getCalendarTaskMap(currentDate.toString("yyyy-MM"), currentDate.addMonths(1).toString("yyyy-MM"));
	m_taskDateChooseBackWidget->refreshData(dataMap);
}

void DLWheelTaskShow::onDeleteButtonClicked()
{
	QTableWidget* tableWidget = m_taskShowTableWidget->getTableWidget();
	QList<WheelDeleteTaskChoose> chooseItemList;

	for (int i = 0; i < tableWidget->rowCount(); i++)
	{
		QTableWidgetItem* item = tableWidget->item(i, 0);
		if (item != NULL)
		{
			if (item->checkState() == Qt::Checked)
			{
                QString strTaskState = tableWidget->item(i, 4)->text();
                if (strTaskState == "已执行" || strTaskState == "正在执行")
                {
                    DLMessageBox::showDLMessageBox(NULL, "提示", "已执行和正在执行任务不可刪除", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
                    return;
                }

				WheelDeleteTaskChoose chooseItem;
				ModifyButtonWidget* modifyButtonWidget = static_cast<ModifyButtonWidget*>(tableWidget->cellWidget(i, 5));
				WheelTaskListShowStruct itemData = modifyButtonWidget->getItemData();
				if (!itemData.task_template_uuid.isEmpty())
				{
					chooseItem.task_delete_uuid = itemData.task_template_uuid;
					chooseItem.taskChoose = Task_Template_Choo;
				}
				else
				{
					chooseItem.task_delete_uuid = itemData.task_uuid;
					chooseItem.taskChoose = Task_Choo;
				}
				chooseItemList.append(chooseItem);
			}
		}
	}

    if (!chooseItemList.isEmpty())
    {
        WHEEL_BACK_TO_CORE_SOCKET.robot_db_task_delete_req(chooseItemList);
    }
}
