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
	TaskState_WaitForExecute,				// �ȴ�ִ��;
	TaskState_ExecuteFinished,				// ��ִ��;
	TaskState_Executing,					// ����ִ��;
	TaskState_Abort,						// ��ֹ;
	TaskState_TaskOutOfDate					// ������;
};

class ModifyButtonWidget : public QWidget
{
	Q_OBJECT
public:
	ModifyButtonWidget(WheelTaskListShowStruct data)
		: m_itemData(data)
	{
		m_pButtonModify = new QPushButton("�޸�");
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

    // ���õ�ǰ����;
    void setCurrentRow(int currentRow)
    {
        m_currentRow = currentRow;
    }

    // ��ȡ��ǰ����;
    int getCurrentRow()
    {
        return m_currentRow;
    }

signals:
	// �޸İ�ť���;
	void signalModifyButtonClicked(WheelTaskListShowStruct itemData);

private:
	QPushButton* m_pButtonModify;

	WheelTaskListShowStruct m_itemData;

    // ���浱ǰ����;
    int m_currentRow;
};

class CustomTableWidget;
class CustomCalendarWidget;
class DLWheelTaskShow : public QWidget
{
	Q_OBJECT

public:
	DLWheelTaskShow(QWidget* parent = NULL);

	// ��ʼ������;
	void initWidget();

	// ����ɾ�����;
	void onUpdateDeleteResult(QString strUuid, bool isSuccess, QString strMsg);

    // ֻˢ�µ�ǰ������;
    void onRefreshCalendar();

private:
	void initTaskExecuteWidget();
	void initLeftWidget();

	void initTaskSelectWidget();
	void initOperateButtonWidget();
	void initTaskShowTable();
	void initTableData(WheelTaskListSearchIndex seachIndex);
	void initRightWidget();

	// ��ȡ����״̬����ɫ;
	QColor getTaskStatusColor(QString strSaskStatus);

signals:
	// ��Ӱ�ť�����ת���������-ȫ��Ѳ��ҳ��;
	void signalJumpToAllPatrolPage();

	// TableWidget�޸İ�ť���;
	void signalModifyButtonClicked(WheelTaskAdminType task_edit_type_id, QString task_edit_uuid);

    // ����״̬�޸Ļص�;
    void signalTaskStatusModifyCallBack(bool, QString);

private slots:
	// checkBox״̬�ı�;
	void onTaskStateChooseChanged(int buttonId);

	// table�޸İ�ť���;
	void onTableModifyButtonClicked(WheelTaskListShowStruct itemData);

	// ɾ����ť���;
	void onDeleteButtonClicked();

private:
	// ��벿��;
	QWidget* m_leftBackWidget;

	QWidget* m_taskExecuteTimeWidget;
	QDateEdit* m_executeTimeEdit;
	QToolButton* m_pButtonSearchLeft;

	CustomCalendarWidget* m_taskDateChooseBackWidget;

	QWidget* m_colorRemarkBackWidget;
	// �Ұ벿��;
	QWidget* m_rightBackWidget;
	// ����ѡ��widget;
	QWidget* m_taskSelectBackWidget;
	QButtonGroup* m_taskStateButtonGrouop;
	QDateEdit* m_startDateEdit;
	QDateEdit* m_endDateEdit;
	QLineEdit* m_taskNameLineEdit;

	// ��ѯ��ťwidget;
	QWidget* m_operateButtonBackWidget;
	QToolButton* m_pButtonSearchRight;
	QToolButton* m_pButtonAdd;
	QToolButton* m_pButtonDelete;

	// �����б�;
	QWidget* m_taskTableBackWidget;
	CustomTableWidget* m_taskShowTableWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰ����table��ҳ��;
    int m_currentPageIndex;

    // ��ǰ��ѯ����;
    WheelTaskListSearchIndex m_currentSeachCondition;

    QList<WheelTaskListShowStruct> m_tableDataList;
};


#endif // !DL_WHEEL_TASK_SHOW_H
