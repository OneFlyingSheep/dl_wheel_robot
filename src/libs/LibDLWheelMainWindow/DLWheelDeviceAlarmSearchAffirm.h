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

/********�豸�澯��ѯҳ��*********/

class DLWheelDeviceAlarmSearchAffirm : public QWidget
{
	Q_OBJECT

public:
	DLWheelDeviceAlarmSearchAffirm();

	// �����Ƿ��ǹ���Ա��¼(����Ա��¼���Զ�������);
	void isAdminLogin(bool isAdmin);

	// ��ʼ������;
	void initWidget();

private:
	// ��ʼ��CheckBoxWidget;
    void initCheckBoxWidget();
	void initButtonListWidget();
	void initTopWidget();
	
	// ��ʼ���豸��;
	void initDeviceaTreeWidget();

	// ��ʼ����������б�;
	void initTaskFormulateTableSingle();

    void initTaskFormulateTableMulti();

    // ��ʼ��table����;
    // ������������;
    void initTableDataSingle(WheelPatrolParameter wheelPatrolPara = WheelPatrolParameter());
    // �����������ݣ�����Ա��;
    void initTableDataMulti();

	// ��ʼ���°벿��;
	void initBottomWidget();

    // ��ʼ������ˢ��ʱ��;
    void initTextColorRefreshTimer();

    // ��ѯ��ť���;
    void onSearchButtonClicked();

    // ���ð�ť���;
    void onResetButtonClicked();

    // ������ť���;
    void onExportButtonClicked();

    // ���ݵ�ǰ��¼�澯״̬��ȡ��ɫ;
    QColor getItemColor(QString strSaskStatus);

private slots:
    // ������ť���;
	void onButtonClicked(int buttonId);
    // table˫������������˴���;
    void onShowTaskCheckWindow();

private:
	// �����ϰ벿��widget;
	QWidget * m_topBackWidget;

    QWidget* m_checkBoxBackWidget;
    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	CustomButtonListWidget* m_customButtonListWidget;
	QDateEdit* m_startTimeEdit;
	QDateEdit* m_endTimeEdit;

	// �����°벿��widget;
	QWidget* m_bottomBackWidget;

	// ��λ��;
	CustomTreeWidget* m_deviceTreeWidget;

	// ��������б�;
	CustomTableWidget* m_taskFormulateTable;
	
	// �������;
	DLWheelTaskCheck* m_taskCheckWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰtable��ҳ��;
    int m_currentPageIndex;

    // ����table����е�����;
    QList<DeviceAlarmSearchStruct> m_singleTableDataList;

    QList<WheelRobortTaskSearchStruct> m_multiCheckTableDataList;

    // ���浱ǰ��ѯ����;
    WheelPatrolParameter m_currentSearchCondition;

    // ������˸ˢ��ʱ��;
    QTimer m_textColorRefreshTimer;
    // ������˸��ɫ�л���־λ;
    bool m_isRefreshTextColor;

    // ��ǰ�Ƿ��ǹ���Ա��¼;
    bool m_isAdminLogin;
};

#endif
