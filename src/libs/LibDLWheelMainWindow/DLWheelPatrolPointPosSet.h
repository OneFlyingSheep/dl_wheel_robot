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

/********巡检点位设置页面**********/

class PointPosSetAddWindow : public QWidget
{
	Q_OBJECT

public:
	PointPosSetAddWindow(QWidget* parent = NULL);

    // 设置数据;
    void setData(WheelPatrolPointSet data);

private:
	// 初始化中心控件;
	void initCenterWidget();
    void initNewCenterWidget();

    // 初始化comboBox;
    void initComboBoxData();

	// 初始化控件;
	void initWidget();
	// 绘制边框;
	void paintEvent(QPaintEvent *event);

signals:
	// 通知点击提交按钮;
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

    // 设备 ID-Name Map;
    QMap<int, WheelStationConfigStruct> m_stationNameMap;
    QMap<QString, QString> m_voltageLevelMap;
    QMap<QString, QString> m_areaNameMap;
    QMap<QString, QString> m_intervalNameMap;
    QMap<QString, QString> m_deviceTypeMap;
    QMap<QString, QString> m_deviceChildTypeMap;
    QMap<QString, WheelDevicePointNameStruct> m_devicePointPosMap;
    // QString - ssid, QString - deviceName;
    QMap<QString, QString> m_deviceNameMapBySsid;
    // 区域类型 ID-Name Map;
    QMap<int, QString> m_areasTypeMap;

    // 是否是修改数据;
    bool m_isModifyData;
    WheelPatrolPointSet m_windowData;
};

class InputWidget;
class DLWheelPatrolPointPosSet : public QWidget
{
	Q_OBJECT

public:
	DLWheelPatrolPointPosSet(QWidget* parent = NULL);

	// 窗口控件初始化;
	void initWidget();

private:
	/********* 界面控件初始化;*********/
	void initTopBoxWidget();
    void initCheckBoxWidget();
	void initButtonWidget();
	void initDeviceTreeWidget();
	void initTableWidget();
    void initTableData();

    // 获取当前选择的设备;
    QStringList getChoosedDeviceIdList();

    // 删除设备;
    void deleteChoosedDevice();

    // 是否启用设备;
    void isStartUseDevice(WheelRootStartUsing start_using);

    // 修改点位;
    void updatePointPos();

    // 新增/修改点位刷新comboBox数据库;
    void refreshComboBoxData();

    // 查询;
    void searchButtonClicked();

    // 导出;
    void exportTable();

    // 重置;
    void resetButtonClicked();

signals:
    // 删除点位回调;
    void signalDeletePatrolPointCallBack(bool, QString);

    // 是否启用点位回调;
    void signalIsStartUsingPointCallBack(bool, QString);

    // 添加点位回调;
    void signalAddPatrolPointCallBack(bool, QString);
    // 修改点位回调;
    void signalUpdatePatrolPointCallBack(bool, QString);

private slots:
	// 按钮点击;
	void onButtonClicked(int buttonId);

private:
	QWidget* m_topBoxBackWidget;
    // checkBoxwidget;
    QWidget* m_checkBoxBackWidget;

    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	// 设备类型checkBox信息列表;
	QList<CheckBoxInfo> m_deviceTypeCheckBoxInfoList;
	CheckBoxWidget* m_deviceTypeBackWidget;

	// 识别类型checkBox信息列表;
	QList<CheckBoxInfo> m_identifyTypeCheckBoxInfoList;
	CheckBoxWidget* m_identifyTypeBackWidget;

	// 表计类型checkBox信息列表;
	QList<CheckBoxInfo> m_meterTypeCheckBoxInfoList;
	CheckBoxWidget* m_meterTypeBackWidget;

	InputWidget* m_enableStateComboBox;

	CustomButtonListWidget* m_buttonBackWidget;
	// 设备树;
	CustomTreeWidget* m_deviceTreeWidget;
	// tableWidget;
	CustomTableWidget* m_customTableWidget;

    // 当前table页面index;
    int m_currentPageIndex;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // table数据;
    QList<WheelPatrolPointSet> m_tableDataList;

    // table 查询条件;
    WheelPatrolParameter m_wheelPatrolPara;
};


#endif
