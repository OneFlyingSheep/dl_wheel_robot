#ifndef DL_WHEEL_MESSAGE_SUBSCRIPTION_ADD_H
#define DL_WHEEL_MESSAGE_SUBSCRIPTION_ADD_H

#include <QWidget>
#include <QToolButton>
#include <QLabel>
#include <QLineEdit>
#include "DLWheelTaskCheck.h"
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include <QTimer>

class CustomButtonListWidget;
class CustomTreeWidget;
class DLWheelMessageSubscriptionAdd : public QWidget
{
	Q_OBJECT

public:
	DLWheelMessageSubscriptionAdd();

    void closeWidget();

private:
	// 窗口控件初始化;
	void initTopWidget();
	void initButtonListWidget();
	void initSystemAlarmWidget();
	void initLeftWidget();
    void initMessageReceiverTreeWidget();
	void initRightWidget();
	void initWidget();

	// 窗口背景绘制事件;
	void paintEvent(QPaintEvent *event);

private slots:
    void onConfigButtonClicked();

private:
	CustomButtonListWidget * m_buttonListWidget;
	TitleWidget * m_messageSubscriptionAddWidget;

    // 告警等级;
    QList<QCheckBox*> m_alarmLevelChecoBoxList;
    // 发送时间;
    QList<QRadioButton*> m_sendTimeRadioButtonList;
    // 发送频率;
    QList<QRadioButton*> m_sendFreqRadioButtonList;

	QWidget* m_leftBackWidget;
	QTabWidget* m_tabWidget;
	CustomTreeWidget* m_deviceAlarmTreeWidget;
	
	QWidget* m_systemAlarmBackWidget;
	QTreeWidget* m_systemAlarmTreeWidget;

	TitleWidget* m_rightBackWidget;
	QTreeWidget* m_messageReceiverTreeWidget;

	QWidget* m_centerBackWidget;

	QToolButton* m_pButtonClose;

    QTimer m_timeoutTime;
};

#endif 
