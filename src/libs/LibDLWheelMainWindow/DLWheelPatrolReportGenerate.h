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

/********excel�����߳�*********/

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

    // ���ñ�������;
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
                DLMessageBox::showDLMessageBox(NULL, "����", "�ļ�Ŀ¼����ʧ��", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
                return;
            }
        }
        
        int downloadResult = GFILE_TRANSFER_CLIENT.downloadFile(m_filePath.toLocal8Bit().data(), m_reportName.toLocal8Bit().data(), m_downloadFilePath.toLocal8Bit().data());
        emit signalDowmloadFinished(downloadResult);
    }

signals:
    // �������;
    void signalDowmloadFinished(int);

private:
    QString m_filePath;
    QString m_reportName;
    QString m_downloadFilePath;
};

/********Ѳ�챨������ҳ��**********/

class DLWheelPatrolReportGenerate : public QWidget
{
	Q_OBJECT

public:
    DLWheelPatrolReportGenerate();

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
	// ����ؼ���ʼ��;
	// ��ʼ��CheckBoxWidget;
    void initCheckBoxList();
	void initButtonWidget();
	void initTopWidget();
	void initTableWidget();
    // ��ʼ��table����;
    void initTableData(WheelPatrolParameter wheelPatrolPara = WheelPatrolParameter());

    // ��ʼ�������߳�;
    void initDownloadThread();
	// �����¼�;
	void paintEvent(QPaintEvent *event);

    // ��ѯ��ť���;
    void onSearchButtonClicked();

    // ���ð�ť���;
    void onResetButtonClicked();

    // ������ť���;
    void onExportButtonClicked();

signals:
    // ���ص�ǰ�����ı����Ƿ��Ѿ�����;
    void signalExamineReportIsExistStatusCallBack(bool, QString, WheelTaskShow);
    // ��hi��ǰCore�Ƿ�ɹ����ɱ���;
    void signalCreateReportStatus(bool, QString);

private slots:
    // ������ť���;
    void onButtonClicked(int buttonId);
private:
	// TopWidget;
	QWidget* m_topBackWidget;

    QWidget* m_checkBoxBackWidget;
    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	// ����ѡ��;
	QDateTimeEdit* m_startDateEdit;
	QDateTimeEdit* m_endDateEdit;

	// ��ѯ�Ȱ�ť;
	CustomButtonListWidget* m_customButtonListWidget;

	// table;
	CustomTableWidget* m_customTableWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰtable��ҳ��;
    int m_currentPageIndex;

    // ���浱ǰ��ѯ����;
    WheelPatrolParameter m_currentSearchCondition;

    // ����table����е�����;
    
    QList<WheelCreateReport> m_tableDataList;

    DownloadExcelFileThread* m_downloadExcelFileThread;
};

#endif
