#ifndef DL_WHEEL_PATROL_REPORT_GENERATE_H
#define DL_WHEEL_PATROL_REPORT_GENERATE_H

#include <QWidget>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QDateTimeEdit>
#include "LibDLWheelCustomWidget/CheckBoxWidget.h"
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibProtoClient/ProtoClient.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include <QDir>
#include <QThread>

class CustomButtonListWidget;
class CustomTableWidget;

/********excel下载线程*********/

class DownloadExcelFileThread : public QThread
{
    Q_OBJECT

public:
    DownloadExcelFileThread(QObject* parent = NULL)
        : QThread(parent)
    {
        m_filePath = "report";
        m_downloadFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/PatrolReport/";
    }

    // 设置报告名称;
    void setFileInfo(QString reportName)
    {
		m_reportName = reportName + ".xlsx";
	//	m_reportName = reportName + ".xls";
    }

private:
    void run()
    {
        QDir downloadDir;
        if (!downloadDir.exists(m_downloadFilePath))
        {
            bool isSuccess = downloadDir.mkpath(m_downloadFilePath);
            if (!isSuccess)
            {
                DLMessageBox::showDLMessageBox(NULL, "错误", "文件目录创建失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
                return;
            }
        }
        
        int downloadResult = GFILE_TRANSFER_CLIENT.downloadFile(m_filePath.toLocal8Bit().data(), m_reportName.toLocal8Bit().data(), m_downloadFilePath.toLocal8Bit().data());
        emit signalDowmloadFinished(downloadResult);
    }

signals:
    // 下载完成;
    void signalDowmloadFinished(int);

private:
    QString m_filePath;
    QString m_reportName;
    QString m_downloadFilePath;
};

/********巡检报告生成页面**********/

class DLWheelPatrolReportGenerate : public QWidget
{
	Q_OBJECT

public:
    DLWheelPatrolReportGenerate();

	// 窗口控件初始化;
	void initWidget();

private:
	// 界面控件初始化;
	// 初始化CheckBoxWidget;
    void initCheckBoxList();
	void initButtonWidget();
	void initTopWidget();
	void initTableWidget();
    // 初始化table数据;
    void initTableData(WheelPatrolParameter wheelPatrolPara = WheelPatrolParameter());

    // 初始化下载线程;
    void initDownloadThread();
	// 绘制事件;
	void paintEvent(QPaintEvent *event);

    // 查询按钮点击;
    void onSearchButtonClicked();

    // 重置按钮点击;
    void onResetButtonClicked();

    // 导出按钮点击;
    void onExportButtonClicked();

signals:
    // 返回当前导出的报告是否已经生成;
    void signalExamineReportIsExistStatusCallBack(bool, QString, WheelTaskShow);
    // 反hi当前Core是否成功生成报告;
    void signalCreateReportStatus(bool, QString);

private slots:
    // 操作按钮点击;
    void onButtonClicked(int buttonId);
private:
	// TopWidget;
	QWidget* m_topBackWidget;

    QWidget* m_checkBoxBackWidget;
    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	// 日期选择;
	QDateTimeEdit* m_startDateEdit;
	QDateTimeEdit* m_endDateEdit;

	// 查询等按钮;
	CustomButtonListWidget* m_customButtonListWidget;

	// table;
	CustomTableWidget* m_customTableWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 当前table的页数;
    int m_currentPageIndex;

    // 保存当前查询条件;
    WheelPatrolParameter m_currentSearchCondition;

    // 保存table表格中的数据;
    
    QList<WheelCreateReport> m_tableDataList;

    DownloadExcelFileThread* m_downloadExcelFileThread;
};

#endif
