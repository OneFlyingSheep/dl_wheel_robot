#ifndef __DL_WHEEL_POWER_STATION_EDIT_H__
#define __DL_WHEEL_POWER_STATION_EDIT_H__

#include <QWidget>
#include <QListWidget>
#include <QLineEdit>
#include <QPushButton>

class BaseWidget;

/*********电站编辑页面**********/

class DLWheelPowerStationEdit : public QWidget
{
	Q_OBJECT

public:
	DLWheelPowerStationEdit(QWidget *parent = NULL);
	~DLWheelPowerStationEdit();

public slots:
    // 更新列表;
    void onRefreshListWidget();

private:
	// 初始化控件;
	void initWidget();
	// 初始化数据;
	void initListWidgetData();

	// 创建ListWidget;
	QListWidget* getListWidget(BaseWidget* baseWidget, QPushButton** addButton, QPushButton** deleteButton, QLineEdit** lineEdit);

private slots:
    // 点击电压等级列表更新间隔名称;
	void onVoltageLevelItemClicked(QListWidgetItem* item);

private:
    /*******基本控件******/
	BaseWidget* m_voltageLevelWidget;
	BaseWidget* m_intervalNameWidget;
	BaseWidget* m_areaNameWidget;

	QListWidget* m_voltageLevelListWidget;
	QListWidget* m_intervalNameListWidget;
	QListWidget* m_areaNameListWidget;

	QMap<QString, QString> m_voltageLevelMap;
	QMap<QString, QString> m_areaNameMap;
	QMap<QString, QString> m_intervalNameMap;

	// 电压等级;
	QLineEdit* m_lineEditValtageLevel;
	QPushButton* m_pButtonAddValtageLevel;
	QPushButton* m_pButtonDeleteValtageLevel;

	// 间隔名称;
	QLineEdit* m_lineEditIntervalName;
	QPushButton* m_pButtonAddIntervalName;
	QPushButton* m_pButtonDeleteIntervalName;

	// 区域名称;
	QLineEdit* m_lineEditAreaName;
	QPushButton* m_pButtonAddAreaName;
	QPushButton* m_pButtonDeleteAreaName;
};

#endif ///