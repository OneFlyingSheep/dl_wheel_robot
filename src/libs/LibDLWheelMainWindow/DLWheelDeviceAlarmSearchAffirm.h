#ifndef DL_WHEEL_DEVICE_ALARM_SEARCH_AFFIRM_H
#define DL_WHEEL_DEVICE_ALARM_SEARCH_AFFIRM_H

#include <QWidget>
#include <QDateEdit>
#include <QTimer>
#include "LibDLWheelCustomWidget/CheckBoxWidget.h"
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"

class DLWheelTaskCheck;

/********设备告警查询页面*********/

class DLWheelDeviceAlarmSearchAffirm : public QWidget
{
	Q_OBJECT

public:
	DLWheelDeviceAlarmSearchAffirm();

	// 设置是否是管理员登录(管理员登录可以多条审批);
	void isAdminLogin(bool isAdmin);

	// 初始化窗口;
	void initWidget();

private:
	// 初始化CheckBoxWidget;
    void initCheckBoxWidget();
	void initButtonListWidget();
	void initTopWidget();
	
	// 初始化设备树;
	void initDeviceaTreeWidget();

	// 初始化任务编制列表;
	void initTaskFormulateTableSingle();

    void initTaskFormulateTableMulti();

    // 初始化table数据;
    // 单条审批数据;
    void initTableDataSingle(WheelPatrolParameter wheelPatrolPara = WheelPatrolParameter());
    // 多条审批数据（管理员）;
    void initTableDataMulti();

	// 初始化下半部分;
	void initBottomWidget();

    // 初始化文字刷新时钟;
    void initTextColorRefreshTimer();

    // 查询按钮点击;
    void onSearchButtonClicked();

    // 重置按钮点击;
    void onResetButtonClicked();

    // 导出按钮点击;
    void onExportButtonClicked();

    // 根据当前记录告警状态获取颜色;
    QColor getItemColor(QString strSaskStatus);

private slots:
    // 操作按钮点击;
	void onButtonClicked(int buttonId);
    // table双击弹出任务审核窗口;
    void onShowTaskCheckWindow();

private:
	// 窗口上半部分widget;
	QWidget * m_topBackWidget;

    QWidget* m_checkBoxBackWidget;
    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	CustomButtonListWidget* m_customButtonListWidget;
	QDateEdit* m_startTimeEdit;
	QDateEdit* m_endTimeEdit;

	// 窗口下半部分widget;
	QWidget* m_bottomBackWidget;

	// 点位树;
	CustomTreeWidget* m_deviceTreeWidget;

	// 任务编制列表;
	CustomTableWidget* m_taskFormulateTable;
	
	// 任务审核;
	DLWheelTaskCheck* m_taskCheckWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 当前table的页数;
    int m_currentPageIndex;

    // 保存table表格中的数据;
    QList<DeviceAlarmSearchStruct> m_singleTableDataList;

    QList<WheelRobortTaskSearchStruct> m_multiCheckTableDataList;

    // 保存当前查询条件;
    WheelPatrolParameter m_currentSearchCondition;

    // 文字闪烁刷新时钟;
    QTimer m_textColorRefreshTimer;
    // 文字闪烁颜色切换标志位;
    bool m_isRefreshTextColor;

    // 当前是否是管理员登录;
    bool m_isAdminLogin;
};

#endif
