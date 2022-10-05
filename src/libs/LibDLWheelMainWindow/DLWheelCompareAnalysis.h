#ifndef DL_WHEEL_COMPARE_ANALYSIS_SHOW_H
#define DL_WHEEL_COMPARE_ANALYSIS_SHOW_H

#include <QWidget>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QDateTimeEdit>
#include "DLWheelPatrolResultBrowse.h"

class CustomTreeWidget;
class CustomButtonListWidget;
class CustomTableWidget;
class TurnPageWidget;
class CompareAnalyzeCurve;
class RCFImageDownload;

// 采集信息类型;
enum CollectInfoType
{
	CollectInfo_VisibleLight = 0,
	CollectInfo_Infrared,
	CollectInfo_AudioVideo
};

// 图片显示层数;
enum LayoutType
{
	Layout_22 = 0,
	Layout_23,
	Layout_33
};

/*********对比分析页面**********/

class DLWheelCompareAnalysis : public QWidget
{
	Q_OBJECT

public:
	DLWheelCompareAnalysis();
    ~DLWheelCompareAnalysis();

	// 窗口控件初始化;
	void initWidget();

private:
	// 界面控件初始化;
	void initDeviceTreeWidget();
	void initDateTimeSelectWidget();
	void initCollectInfoSelectWidget();
	void initButtonWidget();
	void initTableWidget();
	void initLineChartWidget();
	void initCenterWidget();
	void initRightWidget();

    void initTableData();
	// 绘制事件;
	void paintEvent(QPaintEvent *event);

	// 图片显示重新布局;
	void imageShowRelayout(int row, int column);

    // 刷新折线图;
    void updateLineChart();

private slots:
	// 采集信息选择;
	void onRadioButtonClicked(int buttonId);
	// 图片显示布局选择;
	void onImageLayoutSelect(int buttonId);
    // CustomButon点击;
    void onCustomButtonClicked(int buttonId);
    // 图片下载完成;
    void onRcfDownloadImageFinished(int result, int imageIndex, QString strImagePath);

private:
	// 设备树;
	CustomTreeWidget * m_deviceTreeWidget;

	// 日期选择;
	QWidget* m_dataTimeBackWidget;
	QDateTimeEdit* m_startDateEdit;
	QDateTimeEdit* m_endDateEdit;

	// 采集信息;
	QWidget* m_collectInfoBackWidget;

	// 查询等按钮;
	CustomButtonListWidget* m_customButtonListWidget;

	// table;
	CustomTableWidget* m_customTableWidget;

	// 折线图;
    CompareAnalyzeCurve* m_lineChartBackWidget;

	// 中心widget;
	QWidget* m_centerBackWidget;

	// 采集信息展示;
	QWidget* m_rightBackWidget;
	QWidget* m_imageShowBackWidget;
	TurnPageWidget* m_imageTurnPageWidget;

	QList<ImageWidget*> m_imageShowWidgetList;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 当前table页数;
    int m_currentPageIndex;

    // 当前table最多显示条数;
    int m_tableMaxShowCount;

    // 当前选中的设备id;
    QStringList m_currentChooseDeviceId;

    // 当前选中的采集信息的id;
    QStringList m_currentCollectInfoId;

    // 当前时间选择;
    QString m_startTime;
    QString m_endTime;

    RCFImageDownload* m_RCFImageDownload;
};

#endif
