#ifndef DL_WHEEL_PATROL_POINT_POS_SET_H
#define DL_WHEEL_PATROL_POINT_POS_SET_H

#include <QWidget>
#include <QToolButton>
#include <QButtonGroup>
#include <QHBoxLayout>
#include "LibDLWheelCustomWidget/CheckBoxWidget.h"
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include <QLabel>
#include <QPainter>

class InputWidget;
class CheckBoxWidget;
class CustomTableWidget;
class CustomTreeWidget;
class CustomButtonListWidget;

/********Ѳ���λ����ҳ��**********/

class PointPosSetAddWindow : public QWidget
{
	Q_OBJECT

public:
	PointPosSetAddWindow(QWidget* parent = NULL);

    // ��������;
    void setData(WheelPatrolPointSet data);

private:
	// ��ʼ�����Ŀؼ�;
	void initCenterWidget();
    void initNewCenterWidget();

    // ��ʼ��comboBox;
    void initComboBoxData();

	// ��ʼ���ؼ�;
	void initWidget();
	// ���Ʊ߿�;
	void paintEvent(QPaintEvent *event);

signals:
	// ֪ͨ����ύ��ť;
	void signalCommitButtonClicked(WheelPatrolPointSet);

private:
    QLabel* m_titleLabel;

	QWidget* m_centerWidget;
	InputWidget* m_pointPosPath;
	InputWidget* m_deviceType;
	InputWidget* m_deviceArea;
	InputWidget* m_connectObject;
	InputWidget* m_protocolParam;
	InputWidget* m_pointPosChoose;
	QToolButton* m_pButtonSearch;
	InputWidget* m_pointPosCollect;
	InputWidget* m_pointPosSign;
	InputWidget* m_identifyType;
	InputWidget* m_meterType;
	InputWidget* m_feverType;
	InputWidget* m_deviceAppearenceCheckType;

    InputWidget* m_stationNameWidget;
    InputWidget* m_voltageLevelWidget;
    InputWidget* m_intervalNameWidget;
    InputWidget* m_deviceAreaWidget;
    InputWidget* m_deviceTypeWidget;
    InputWidget* m_deviceSubTypeWidget;
    InputWidget* m_pointNameWidget;

	QToolButton* m_pButtonCommit;
	
	QToolButton* m_pButtonClose;

	QCheckBox* m_checkBoxInfrared;
	QCheckBox* m_checkBoxPicture;
	QCheckBox* m_checkBoxAudio;

    // �豸 ID-Name Map;
    QMap<int, WheelStationConfigStruct> m_stationNameMap;
    QMap<QString, QString> m_voltageLevelMap;
    QMap<QString, QString> m_areaNameMap;
    QMap<QString, QString> m_intervalNameMap;
    QMap<QString, QString> m_deviceTypeMap;
    QMap<QString, QString> m_deviceChildTypeMap;
    QMap<QString, WheelDevicePointNameStruct> m_devicePointPosMap;
    // QString - ssid, QString - deviceName;
    QMap<QString, QString> m_deviceNameMapBySsid;
    // �������� ID-Name Map;
    QMap<int, QString> m_areasTypeMap;

    // �Ƿ����޸�����;
    bool m_isModifyData;
    WheelPatrolPointSet m_windowData;
};

class InputWidget;
class DLWheelPatrolPointPosSet : public QWidget
{
	Q_OBJECT

public:
	DLWheelPatrolPointPosSet(QWidget* parent = NULL);

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
	/********* ����ؼ���ʼ��;*********/
	void initTopBoxWidget();
    void initCheckBoxWidget();
	void initButtonWidget();
	void initDeviceTreeWidget();
	void initTableWidget();
    void initTableData();

    // ��ȡ��ǰѡ����豸;
    QStringList getChoosedDeviceIdList();

    // ɾ���豸;
    void deleteChoosedDevice();

    // �Ƿ������豸;
    void isStartUseDevice(WheelRootStartUsing start_using);

    // �޸ĵ�λ;
    void updatePointPos();

    // ����/�޸ĵ�λˢ��comboBox���ݿ�;
    void refreshComboBoxData();

    // ��ѯ;
    void searchButtonClicked();

    // ����;
    void exportTable();

    // ����;
    void resetButtonClicked();

signals:
    // ɾ����λ�ص�;
    void signalDeletePatrolPointCallBack(bool, QString);

    // �Ƿ����õ�λ�ص�;
    void signalIsStartUsingPointCallBack(bool, QString);

    // ��ӵ�λ�ص�;
    void signalAddPatrolPointCallBack(bool, QString);
    // �޸ĵ�λ�ص�;
    void signalUpdatePatrolPointCallBack(bool, QString);

private slots:
	// ��ť���;
	void onButtonClicked(int buttonId);

private:
	QWidget* m_topBoxBackWidget;
    // checkBoxwidget;
    QWidget* m_checkBoxBackWidget;

    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	// �豸����checkBox��Ϣ�б�;
	QList<CheckBoxInfo> m_deviceTypeCheckBoxInfoList;
	CheckBoxWidget* m_deviceTypeBackWidget;

	// ʶ������checkBox��Ϣ�б�;
	QList<CheckBoxInfo> m_identifyTypeCheckBoxInfoList;
	CheckBoxWidget* m_identifyTypeBackWidget;

	// �������checkBox��Ϣ�б�;
	QList<CheckBoxInfo> m_meterTypeCheckBoxInfoList;
	CheckBoxWidget* m_meterTypeBackWidget;

	InputWidget* m_enableStateComboBox;

	CustomButtonListWidget* m_buttonBackWidget;
	// �豸��;
	CustomTreeWidget* m_deviceTreeWidget;
	// tableWidget;
	CustomTableWidget* m_customTableWidget;

    // ��ǰtableҳ��index;
    int m_currentPageIndex;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // table����;
    QList<WheelPatrolPointSet> m_tableDataList;

    // table ��ѯ����;
    WheelPatrolParameter m_wheelPatrolPara;
};


#endif
