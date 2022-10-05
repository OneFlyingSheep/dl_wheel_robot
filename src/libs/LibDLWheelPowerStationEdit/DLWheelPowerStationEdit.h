#ifndef __DL_WHEEL_POWER_STATION_EDIT_H__
#define __DL_WHEEL_POWER_STATION_EDIT_H__

#include <QWidget>
#include <QListWidget>
#include <QLineEdit>
#include <QPushButton>

class BaseWidget;

/*********��վ�༭ҳ��**********/

class DLWheelPowerStationEdit : public QWidget
{
	Q_OBJECT

public:
	DLWheelPowerStationEdit(QWidget *parent = NULL);
	~DLWheelPowerStationEdit();

public slots:
    // �����б�;
    void onRefreshListWidget();

private:
	// ��ʼ���ؼ�;
	void initWidget();
	// ��ʼ������;
	void initListWidgetData();

	// ����ListWidget;
	QListWidget* getListWidget(BaseWidget* baseWidget, QPushButton** addButton, QPushButton** deleteButton, QLineEdit** lineEdit);

private slots:
    // �����ѹ�ȼ��б���¼������;
	void onVoltageLevelItemClicked(QListWidgetItem* item);

private:
    /*******�����ؼ�******/
	BaseWidget* m_voltageLevelWidget;
	BaseWidget* m_intervalNameWidget;
	BaseWidget* m_areaNameWidget;

	QListWidget* m_voltageLevelListWidget;
	QListWidget* m_intervalNameListWidget;
	QListWidget* m_areaNameListWidget;

	QMap<QString, QString> m_voltageLevelMap;
	QMap<QString, QString> m_areaNameMap;
	QMap<QString, QString> m_intervalNameMap;

	// ��ѹ�ȼ�;
	QLineEdit* m_lineEditValtageLevel;
	QPushButton* m_pButtonAddValtageLevel;
	QPushButton* m_pButtonDeleteValtageLevel;

	// �������;
	QLineEdit* m_lineEditIntervalName;
	QPushButton* m_pButtonAddIntervalName;
	QPushButton* m_pButtonDeleteIntervalName;

	// ��������;
	QLineEdit* m_lineEditAreaName;
	QPushButton* m_pButtonAddAreaName;
	QPushButton* m_pButtonDeleteAreaName;
};

#endif ///