#ifndef IDENTIFY_ABNORMAL_POINT_SEARCH_H
#define IDENTIFY_ABNORMAL_POINT_SEARCH_H

#include <QWidget>
#include <qlabel.h>
#include <QCheckBox>
#include <QButtonGroup>
#include <QToolButton>
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"

// ʶ������;
enum IdentifyStateType
{
	IdentifyNormal,				// ʶ������;
	IdentifyAbnormal,			// ʶ���쳣;
	ArtificialIdentify			// �˹�ʶ��;
};

class CustomTableWidget;

/************ʶ���쳣��λ��ѯ************/

class DLWheelIdentifyAbnormalPointSearch : public QWidget
{
	Q_OBJECT

public:
	DLWheelIdentifyAbnormalPointSearch(QWidget* parent = NULL);

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
    // ��ʼ�������ؼ�;
	void initCheckBoxWidget();
	void initButtonBackWidget();
	void initDeviceTreeWidget();
	void initTableWidget();
    void initTableData(QStringList conditionList);
	
    // ��ѯ;
    void searchTable();
    // ����;
    void resetCondition();
    // ����;
    void exportTable();

private slots:
    // ������ť���;
	void onButtonClicked(int buttonId);

private:
	QWidget* m_checkBoxBackWidget;
	QButtonGroup* m_checkBoxGroup;

	CustomButtonListWidget* m_customButtonListWidget;

	// �豸��;
	CustomTreeWidget* m_deviceTreeWidget;

	QWidget* m_tableBackWidget;
	CustomTableWidget* m_pointPosTableWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰtableҳ��;
    int m_currentPageIndex;

    // ��ǰѡ�������;
    QStringList m_conditionList;
};

#endif
