#ifndef DL_WHEEL_ALARM_MESSAGE_SUBSCRIBTION_H
#define DL_WHEEL_ALARM_MESSAGE_SUBSCRIBTION_H

#include <QWidget>
#include "common/DLWheelRobotGlobalDef.hpp"

class CustomButtonListWidget;
class CustomTableWidget;
class InputWidget;
class DLWheelMessageSubscriptionAdd;

/*******�澯��Ϣ����ҳ��**********/

class DLWheelAlarmMessageSubscription : public QWidget
{
	Q_OBJECT

public:
	DLWheelAlarmMessageSubscription();
	~DLWheelAlarmMessageSubscription();

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
    // ҳ������ؼ���ʼ��;
	void initTopWidget();
	void initButtonListWidget();
	void initTableWidget();
    void initTableData();

private:
    // ������ť���;
	void onButtonClicked(int buttonId);
    // ��ѯ;
    void onSearchButtonClicked();
    // ɾ��;
    void onDeleteButtonClicked();
    // ����;
    void onResetButtonClicked();

signals:
    // �������ݻص�;
    void signalInsertDataCallBack(bool, QString);
    // ɾ�����ݻص�;
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

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰtable��ҳ��;
    int m_currentPageIndex;

    QList<WheelNoteMessage> m_tableDataList;

    QString m_strCompanyName;
    QString m_strStationName;

    // ��ѯ����;
    WheelAlarmMessageIf m_wheelAlarmMessageIf;

    bool m_isDeviceSearch;
};

#endif // DL_WHEEL_TASK_CHECK_H
