#ifndef DL_WHEEL_ALARM_MESSAGE_SUBSCRIBTION_H
#define DL_WHEEL_ALARM_MESSAGE_SUBSCRIBTION_H

#include <QWidget>
#include "common/DLWheelRobotGlobalDef.hpp"

class CustomButtonListWidget;
class CustomTableWidget;
class InputWidget;
class DLWheelMessageSubscriptionAdd;

/*******告警信息订阅页面**********/

class DLWheelAlarmMessageSubscription : public QWidget
{
	Q_OBJECT

public:
	DLWheelAlarmMessageSubscription();
	~DLWheelAlarmMessageSubscription();

	// 窗口控件初始化;
	void initWidget();

private:
    // 页面各个控件初始化;
	void initTopWidget();
	void initButtonListWidget();
	void initTableWidget();
    void initTableData();

private:
    // 操作按钮点击;
	void onButtonClicked(int buttonId);
    // 查询;
    void onSearchButtonClicked();
    // 删除;
    void onDeleteButtonClicked();
    // 重置;
    void onResetButtonClicked();

signals:
    // 插入数据回调;
    void signalInsertDataCallBack(bool, QString);
    // 删除数据回调;
    void signalDeleteDataCallBack(bool, QString);

private:
	QWidget* m_topBackWidget;
	InputWidget* m_unitWidget;
	InputWidget* m_transformerSubstationWidget;
	InputWidget* m_intervalVoltageWidget;
	InputWidget* m_intervalWidget;
    InputWidget* m_deviceTypeWidget;
	InputWidget* m_pointPosNameWidget;
	InputWidget* m_receiverAccountWidget;

	CustomButtonListWidget* m_buttonListWidget;

	CustomTableWidget* m_tableWidget;

	DLWheelMessageSubscriptionAdd* m_messageSubscriptionAdd;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 当前table的页数;
    int m_currentPageIndex;

    QList<WheelNoteMessage> m_tableDataList;

    QString m_strCompanyName;
    QString m_strStationName;

    // 查询条件;
    WheelAlarmMessageIf m_wheelAlarmMessageIf;

    bool m_isDeviceSearch;
};

#endif // DL_WHEEL_TASK_CHECK_H
