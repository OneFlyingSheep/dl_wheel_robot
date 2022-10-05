#ifndef IDENTIFY_ABNORMAL_POINT_SEARCH_H
#define IDENTIFY_ABNORMAL_POINT_SEARCH_H

#include <QWidget>
#include <qlabel.h>
#include <QCheckBox>
#include <QButtonGroup>
#include <QToolButton>
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"

// 识别类型;
enum IdentifyStateType
{
	IdentifyNormal,				// 识别正常;
	IdentifyAbnormal,			// 识别异常;
	ArtificialIdentify			// 人工识别;
};

class CustomTableWidget;

/************识别异常点位查询************/

class DLWheelIdentifyAbnormalPointSearch : public QWidget
{
	Q_OBJECT

public:
	DLWheelIdentifyAbnormalPointSearch(QWidget* parent = NULL);

	// 窗口控件初始化;
	void initWidget();

private:
    // 初始化各个控件;
	void initCheckBoxWidget();
	void initButtonBackWidget();
	void initDeviceTreeWidget();
	void initTableWidget();
    void initTableData(QStringList conditionList);
	
    // 查询;
    void searchTable();
    // 重置;
    void resetCondition();
    // 导出;
    void exportTable();

private slots:
    // 操作按钮点击;
	void onButtonClicked(int buttonId);

private:
	QWidget* m_checkBoxBackWidget;
	QButtonGroup* m_checkBoxGroup;

	CustomButtonListWidget* m_customButtonListWidget;

	// 设备树;
	CustomTreeWidget* m_deviceTreeWidget;

	QWidget* m_tableBackWidget;
	CustomTableWidget* m_pointPosTableWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 当前table页数;
    int m_currentPageIndex;

    // 当前选择的条件;
    QStringList m_conditionList;
};

#endif
