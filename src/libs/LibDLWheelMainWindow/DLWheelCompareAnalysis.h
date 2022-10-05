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

// �ɼ���Ϣ����;
enum CollectInfoType
{
	CollectInfo_VisibleLight = 0,
	CollectInfo_Infrared,
	CollectInfo_AudioVideo
};

// ͼƬ��ʾ����;
enum LayoutType
{
	Layout_22 = 0,
	Layout_23,
	Layout_33
};

/*********�Աȷ���ҳ��**********/

class DLWheelCompareAnalysis : public QWidget
{
	Q_OBJECT

public:
	DLWheelCompareAnalysis();
    ~DLWheelCompareAnalysis();

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
	// ����ؼ���ʼ��;
	void initDeviceTreeWidget();
	void initDateTimeSelectWidget();
	void initCollectInfoSelectWidget();
	void initButtonWidget();
	void initTableWidget();
	void initLineChartWidget();
	void initCenterWidget();
	void initRightWidget();

    void initTableData();
	// �����¼�;
	void paintEvent(QPaintEvent *event);

	// ͼƬ��ʾ���²���;
	void imageShowRelayout(int row, int column);

    // ˢ������ͼ;
    void updateLineChart();

private slots:
	// �ɼ���Ϣѡ��;
	void onRadioButtonClicked(int buttonId);
	// ͼƬ��ʾ����ѡ��;
	void onImageLayoutSelect(int buttonId);
    // CustomButon���;
    void onCustomButtonClicked(int buttonId);
    // ͼƬ�������;
    void onRcfDownloadImageFinished(int result, int imageIndex, QString strImagePath);

private:
	// �豸��;
	CustomTreeWidget * m_deviceTreeWidget;

	// ����ѡ��;
	QWidget* m_dataTimeBackWidget;
	QDateTimeEdit* m_startDateEdit;
	QDateTimeEdit* m_endDateEdit;

	// �ɼ���Ϣ;
	QWidget* m_collectInfoBackWidget;

	// ��ѯ�Ȱ�ť;
	CustomButtonListWidget* m_customButtonListWidget;

	// table;
	CustomTableWidget* m_customTableWidget;

	// ����ͼ;
    CompareAnalyzeCurve* m_lineChartBackWidget;

	// ����widget;
	QWidget* m_centerBackWidget;

	// �ɼ���Ϣչʾ;
	QWidget* m_rightBackWidget;
	QWidget* m_imageShowBackWidget;
	TurnPageWidget* m_imageTurnPageWidget;

	QList<ImageWidget*> m_imageShowWidgetList;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰtableҳ��;
    int m_currentPageIndex;

    // ��ǰtable�����ʾ����;
    int m_tableMaxShowCount;

    // ��ǰѡ�е��豸id;
    QStringList m_currentChooseDeviceId;

    // ��ǰѡ�еĲɼ���Ϣ��id;
    QStringList m_currentCollectInfoId;

    // ��ǰʱ��ѡ��;
    QString m_startTime;
    QString m_endTime;

    RCFImageDownload* m_RCFImageDownload;
};

#endif
