#ifndef DL_WHEEL_PATROL_RESULT_BROWER_H
#define DL_WHEEL_PATROL_RESULT_BROWER_H

#include <QWidget>
#include <QDateEdit>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QListWidget>
#include <QStackedWidget>
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelCustomWidget/TurnPageWidget.h"
#include "LibProtoClient/ProtoClient.h"


class RCFImageDownload;
class DLWheelTaskCheck;
class DLMessageBox;

/*********显示设备图片控件**********/

class ImageWidget : public QWidget
{
public:
	ImageWidget()
	{
		m_imageLabel = new QLabel;
		m_imageLabel->setObjectName("ImageLabel");

		m_textLabel = new QLabel;
		m_textLabel->setFixedHeight(20);

		QVBoxLayout* vLayout = new QVBoxLayout(this);
		vLayout->addWidget(m_imageLabel);
		vLayout->addWidget(m_textLabel);
		vLayout->setSpacing(0);
		vLayout->setMargin(0);

		this->setStyleSheet("QLabel#ImageLabel{background:rgb(159,168,218);}");
	}

    // 设置文字;
	void setText(const QString& text)
	{
		m_textLabel->setText(text);
		m_textLabel->setScaledContents(true);
	}

    // 设置图片;
	void setImage(QString imagePath)
	{
		m_imageLabel->setPixmap(QPixmap(imagePath).scaled(m_imageLabel->size()));
	}

private:
	QLabel* m_imageLabel;
	QLabel* m_textLabel;
};

/********巡检结果浏览页面********/

class DLWheelPatrolResultBrowse : public QWidget
{
	Q_OBJECT

public:
	DLWheelPatrolResultBrowse();

    ~DLWheelPatrolResultBrowse();

	// 窗口控件初始化;
	void initWidget();

    // 巡检结果审核人提交回调;
    void onPatrolResultCheckPeopleCallBack(bool isSuccess, QString strTaskId, QString strMsg);

private:
	// 初始化设备树;
	void initDeviceTreeWidget();
    // 初始化任务列表;
    void initTaskList();
    // 初始化左部控件;
    void initLeftWidget();
	// 初始化按钮列表;
	void initButtonListWidget();
	// 初始化任务编制列表;
	void initTableWidget();
	// 初始化图片显示;
	void initImageShowWidget();

    // 更新table显示;
    void updateTabelData(QString strTaskId);

private slots:
	void onButtonClicked(int buttonId);
    // 对任务列表进行操作;
    void onTaskButtonClicked(int buttonId);
    // 任务列表点击;
    void onTaskListClciked(QListWidgetItem* item);
    // 图片下载完成;
    void onRcfDownloadImageFinished(int result, int imageIndex, QString strImagePath);

	//一键审核结束信号
	void AKeyAuditFinishSlot(bool bIsSuccess, QString strErrorMessage);


private:
    // 自定义树控件;
	CustomTreeWidget * m_deviceTreeWidget;

    // 任务列表;
    QListWidget* m_pTaskListWidget;
    QWidget* m_taskBackWidget;

    QStackedWidget* m_leftBackWidget;

	CustomButtonListWidget* m_pCustomButtonListWidget;
	QDateEdit* m_startTimeEdit;
	QDateEdit* m_endTimeEdit;

	CustomTableWidget* m_customTableWidget;

    QList<ImageWidget*> m_lstImageWgts;
	QWidget* m_pImageShowBackWidget;
	TurnPageWidget* m_imageTurnPageWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 保存table当前页;
    int m_currentPageIndex;
    // 当前点击的任务id;
    QString m_strCurrentTaskId;

    QLineEdit* m_checkPeopleLineEdit;

    QDateEdit* m_checkSuggestTextEdit;

    // 保存table数据;
    QList<DeviceAlarmSearchStruct> m_lstTableData;

    // 任务审核;
    DLWheelTaskCheck* m_taskCheckWidget;

    // 图片下载 线程;
    RCFImageDownload* m_pThreadRCFImageDownload;

	RCFImageDownload* m_pTaskThreadRCFImageDownload;			//整个任务下载线程
};

#endif
