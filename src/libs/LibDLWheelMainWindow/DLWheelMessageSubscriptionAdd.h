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
	// ���ڿؼ���ʼ��;
	void initTopWidget();
	void initButtonListWidget();
	void initSystemAlarmWidget();
	void initLeftWidget();
    void initMessageReceiverTreeWidget();
	void initRightWidget();
	void initWidget();

	// ���ڱ��������¼�;
	void paintEvent(QPaintEvent *event);

private slots:
    void onConfigButtonClicked();

private:
	CustomButtonListWidget * m_buttonListWidget;
	TitleWidget * m_messageSubscriptionAddWidget;

    // �澯�ȼ�;
    QList<QCheckBox*> m_alarmLevelChecoBoxList;
    // ����ʱ��;
    QList<QRadioButton*> m_sendTimeRadioButtonList;
    // ����Ƶ��;
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
