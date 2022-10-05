#ifndef DL_WHEEL_TASK_SHOW_H
#define DL_WHEEL_TASK_SHOW_H

#include <QWidget>
#include <QLabel>
#include <QButtonGroup>
#include <QCheckBox>
#include <QDateEdit>
#include <QListWidget>
#include <QToolButton>
#include "LibDLWheelTaskDeviceDBSearch/DLWheelTaskAdministrationData.h"
#include <QPushButton>
#include <QHBoxLayout>

#pragma execution_character_set("utf-8")

enum CheckBoxTaskState
{
	TaskState_WaitForExecute,				// 等待执行;
	TaskState_ExecuteFinished,				// 已执行;
	TaskState_Executing,					// 正在执行;
	TaskState_Abort,						// 中止;
	TaskState_TaskOutOfDate					// 任务超期;
};

class ModifyButtonWidget : public QWidget
{
	Q_OBJECT
public:
	ModifyButtonWidget(WheelTaskListShowStruct data)
		: m_itemData(data)
	{
		m_pButtonModify = new QPushButton("修改");
		m_pButtonModify->setFixedSize(QSize(60, 30));
		connect(m_pButtonModify, &QPushButton::clicked, this, [=] {
			emit signalModifyButtonClicked(m_itemData);
		});

		QHBoxLayout* hLayout = new QHBoxLayout(this);
		hLayout->addStretch();
		hLayout->addWidget(m_pButtonModify);
		hLayout->addStretch();
		hLayout->setMargin(0);

		this->setAttribute(Qt::WA_TranslucentBackground);
		this->setStyleSheet("QPushButton{border:none;}QPushButton:hover{background:rgb(210, 255, 255);}QPushButton:pressed{padding-left:2px;padding-top:2px;}");
	}

	WheelTaskListShowStruct getItemData()
	{
		return m_itemData;
	}

    // 设置当前行数;
    void setCurrentRow(int currentRow)
    {
        m_currentRow = currentRow;
    }

    // 获取当前行数;
    int getCurrentRow()
    {
        return m_currentRow;
    }

signals:
	// 修改按钮点击;
	void signalModifyButtonClicked(WheelTaskListShowStruct itemData);

private:
	QPushButton* m_pButtonModify;

	WheelTaskListShowStruct m_itemData;

    // 保存当前行数;
    int m_currentRow;
};

class CustomTableWidget;
class CustomCalendarWidget;
class DLWheelTaskShow : public QWidget
{
	Q_OBJECT

public:
	DLWheelTaskShow(QWidget* parent = NULL);

	// 初始化窗口;
	void initWidget();

	// 更新删除结果;
	void onUpdateDeleteResult(QString strUuid, bool isSuccess, QString strMsg);

    // 只刷新当前月日历;
    void onRefreshCalendar();

private:
	void initTaskExecuteWidget();
	void initLeftWidget();

	void initTaskSelectWidget();
	void initOperateButtonWidget();
	void initTaskShowTable();
	void initTableData(WheelTaskListSearchIndex seachIndex);
	void initRightWidget();

	// 获取任务状态的颜色;
	QColor getTaskStatusColor(QString strSaskStatus);

signals:
	// 添加按钮点击跳转到任务管理-全面巡检页面;
	void signalJumpToAllPatrolPage();

	// TableWidget修改按钮点击;
	void signalModifyButtonClicked(WheelTaskAdminType task_edit_type_id, QString task_edit_uuid);

    // 任务状态修改回调;
    void signalTaskStatusModifyCallBack(bool, QString);

private slots:
	// checkBox状态改变;
	void onTaskStateChooseChanged(int buttonId);

	// table修改按钮点击;
	void onTableModifyButtonClicked(WheelTaskListShowStruct itemData);

	// 删除按钮点击;
	void onDeleteButtonClicked();

private:
	// 左半部分;
	QWidget* m_leftBackWidget;

	QWidget* m_taskExecuteTimeWidget;
	QDateEdit* m_executeTimeEdit;
	QToolButton* m_pButtonSearchLeft;

	CustomCalendarWidget* m_taskDateChooseBackWidget;

	QWidget* m_colorRemarkBackWidget;
	// 右半部分;
	QWidget* m_rightBackWidget;
	// 任务选择widget;
	QWidget* m_taskSelectBackWidget;
	QButtonGroup* m_taskStateButtonGrouop;
	QDateEdit* m_startDateEdit;
	QDateEdit* m_endDateEdit;
	QLineEdit* m_taskNameLineEdit;

	// 查询按钮widget;
	QWidget* m_operateButtonBackWidget;
	QToolButton* m_pButtonSearchRight;
	QToolButton* m_pButtonAdd;
	QToolButton* m_pButtonDelete;

	// 任务列表;
	QWidget* m_taskTableBackWidget;
	CustomTableWidget* m_taskShowTableWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 当前任务table的页数;
    int m_currentPageIndex;

    // 当前查询条件;
    WheelTaskListSearchIndex m_currentSeachCondition;

    QList<WheelTaskListShowStruct> m_tableDataList;
};


#endif // !DL_WHEEL_TASK_SHOW_H
