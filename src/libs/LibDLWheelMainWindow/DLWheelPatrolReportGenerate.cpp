#include "DLWheelPatrolReportGenerate.h"
#include <QDateTime>
#include <QCalendarWidget>
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelTaskAdministrationData.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QFileDialog>
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include <QDesktopServices>

#pragma execution_character_set("utf-8")

#define TABLE_PER_PAGE_COUNT 16

DLWheelPatrolReportGenerate::DLWheelPatrolReportGenerate()
	: m_isInitWidget(false)
{
    qRegisterMetaType<WheelTaskShow>("WheelTaskShow");

	this->setStyleSheet("QWidget#TopBackWidget{background:rgb(240, 240, 240);}");

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotExamineReportIsExistStatus.connect(boost::bind(&DLWheelPatrolReportGenerate::signalExamineReportIsExistStatusCallBack, this, _1, _2, _3));
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotCreateReportStatus.connect(boost::bind(&DLWheelPatrolReportGenerate::signalCreateReportStatus, this, _1, _2));
    
    connect(this, &DLWheelPatrolReportGenerate::signalExamineReportIsExistStatusCallBack, this, [=](bool isExist, QString reportName, WheelTaskShow task) {
        if (isExist)
        {
            // 从Core上进行下载;
            m_downloadExcelFileThread->setFileInfo(reportName);
            m_downloadExcelFileThread->start();
        }
        else
        {
            WHEEL_BACK_TO_CORE_SOCKET.robot_create_report_req(task);
        }
    });

    connect(this, &DLWheelPatrolReportGenerate::signalCreateReportStatus, this, [=](bool isSuccess, QString reportName) {
        if (isSuccess)
        {
            // 生成成功,从Core上进行下载;
            m_downloadExcelFileThread->setFileInfo(reportName);
            m_downloadExcelFileThread->start();
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "错误", "报告生成失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });

    initDownloadThread();
}

void DLWheelPatrolReportGenerate::initCheckBoxList()
{
    m_checkBoxBackWidget = new QWidget;

    QList<WheelTaskAdminCheckBoxStruct> checkBoxDataList = WHEEL_TASK_ADMINISTRATION.getTaskAdminCheckBoxData(WHEEL_CREATE_EXCEL_REPORT);
    QVBoxLayout* vCheckBoxLayout = new QVBoxLayout(m_checkBoxBackWidget);
    for (int i = 0; i < 3; i++)
    {
        CheckBoxWidget* checkBoxWidget = new CheckBoxWidget;
        checkBoxWidget->setCheckBoxType(checkBoxDataList[i].CheckBoxTypeEnum);
        checkBoxWidget->addCheckBoxWidget(checkBoxDataList[i].typeName, checkBoxDataList[i].PackageCheckBoxStru);

        m_checkBoxWidgetList.append(checkBoxWidget);
        vCheckBoxLayout->addWidget(checkBoxWidget);
    }
    vCheckBoxLayout->setSpacing(20);
    vCheckBoxLayout->setMargin(0);
}

void DLWheelPatrolReportGenerate::initButtonWidget()
{
	QLabel* labelStartTime = new QLabel("开始时间:");

	QLabel* labelEndTime = new QLabel("结束时间:");

	QCalendarWidget* startDateWidget = new QCalendarWidget;
	m_startDateEdit = new QDateTimeEdit;
	m_startDateEdit->setFixedSize(QSize(140, 22));
	m_startDateEdit->setCalendarPopup(true);
	m_startDateEdit->setCalendarWidget(startDateWidget);
	m_startDateEdit->setDateTime(QDateTime::currentDateTime());
	m_startDateEdit->setDisplayFormat("yyyy-MM-dd hh:MM:ss");

	QCalendarWidget* endDateWidget = new QCalendarWidget;
	m_endDateEdit = new QDateTimeEdit;
	m_endDateEdit->setFixedSize(QSize(140, 22));
	m_endDateEdit->setCalendarPopup(true);
	m_endDateEdit->setCalendarWidget(endDateWidget);
	m_endDateEdit->setDateTime(QDateTime::currentDateTime());
	m_endDateEdit->setDisplayFormat("yyyy-MM-dd hh:MM:ss");

	QWidget* dataTimeBackWidget = new QWidget;
	dataTimeBackWidget->setFixedHeight(30);
	QHBoxLayout* hTimeLayout = new QHBoxLayout(dataTimeBackWidget);
	hTimeLayout->addWidget(labelStartTime);
	hTimeLayout->addWidget(m_startDateEdit);
	hTimeLayout->addWidget(labelEndTime);
	hTimeLayout->addWidget(m_endDateEdit);
	hTimeLayout->setSpacing(5);
	hTimeLayout->setContentsMargins(5, 0, 5, 0);

	m_customButtonListWidget = new CustomButtonListWidget;
	m_customButtonListWidget->addWidget(dataTimeBackWidget);
	m_customButtonListWidget->addToolButton(0, "查询", ":/Resources/Common/image/Search.png");
	m_customButtonListWidget->addToolButton(1, "重置", ":/Resources/Common/image/Reset.png");
	m_customButtonListWidget->addToolButton(2, "导出", ":/Resources/Common/image/ExportButton.png");
    m_customButtonListWidget->addToolButton(3, "查看报告", ":/Resources/Common/image/Search.png", QSize(75, 20));
	m_customButtonListWidget->addWidgetFinished();

    connect(m_customButtonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelPatrolReportGenerate::onButtonClicked);
}

void DLWheelPatrolReportGenerate::initTopWidget()
{
    initCheckBoxList();
	initButtonWidget();

	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("TopBackWidget");

	QVBoxLayout* vTopLayout = new QVBoxLayout(m_topBackWidget);
	vTopLayout->addWidget(m_checkBoxBackWidget);
	vTopLayout->addWidget(m_customButtonListWidget);
	vTopLayout->setSpacing(20);
	vTopLayout->setMargin(6);
}

void DLWheelPatrolReportGenerate::initTableWidget()
{
    m_customTableWidget = new CustomTableWidget(6, false);
    m_customTableWidget->setIsShowTurnPageWidget(false);
    m_customTableWidget->getTableWidget()->setHorizontalHeaderLabels(QStringList() << "序号" << "任务名称" << "开始时间" << "巡检总点数" << "任务状态" << "巡检类型");
    connect(m_customTableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        initTableData(m_currentSearchCondition);
    });
    connect(m_customTableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableData(m_currentSearchCondition);
    });
}

void DLWheelPatrolReportGenerate::initTableData(WheelPatrolParameter wheelPatrolPara /*= WheelPatrolParameter()*/)
{
    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();

    m_tableDataList = WHEEL_TASK_ADMINISTRATION.getCreateReportStru(wheelPatrolPara.m_start_time, wheelPatrolPara.m_stop_time);
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    for (int i = 0; i < m_tableDataList.count(); i++)
    {
        tableWidget->insertRow(i);
        
        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, idItem);
        idItem->setBackground(QBrush(QColor(255, 0, 0)));

        QTableWidgetItem* recognitionItem = new QTableWidgetItem(m_tableDataList[i].task_name);
        tableWidget->setItem(i, 1, recognitionItem);
        recognitionItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* pointNameItem = new QTableWidgetItem(m_tableDataList[i].task_time);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 2, pointNameItem);

        QTableWidgetItem* recognitionResultItem = new QTableWidgetItem(QString::number(m_tableDataList[i].all_point));
        recognitionResultItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, recognitionResultItem);

        QTableWidgetItem* alarmLevelItem = new QTableWidgetItem(m_tableDataList[i].task_status);
        alarmLevelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, alarmLevelItem);

        QTableWidgetItem* recognitionTimeItem = new QTableWidgetItem(m_tableDataList[i].patrol_type);
        recognitionTimeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, recognitionTimeItem);

        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelPatrolReportGenerate::initDownloadThread()
{
    m_downloadExcelFileThread = new DownloadExcelFileThread(this);

    connect(m_downloadExcelFileThread, &DownloadExcelFileThread::signalDowmloadFinished, this, [=](int downloadResult) {
        if (downloadResult >= 0)
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "巡检报告下载完成", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "错误", "巡检报告下载失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });
}

void DLWheelPatrolReportGenerate::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initTopWidget();
	initTableWidget();
    initTableData(m_currentSearchCondition);

	QHBoxLayout* hBottomLayout = new QHBoxLayout;
	hBottomLayout->addWidget(m_customTableWidget);
	hBottomLayout->setSpacing(0);
	hBottomLayout->setMargin(0);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_topBackWidget);
	vMainLayout->addLayout(hBottomLayout);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(0);
}

void DLWheelPatrolReportGenerate::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), QColor(240, 240, 240));
}

void DLWheelPatrolReportGenerate::onButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
    {
        onSearchButtonClicked();
    }
    break;
    case 1:
    {
        onResetButtonClicked();
    }
    break;
    case 2:
    {
        int currentRow = m_customTableWidget->getTableWidget()->currentRow();
        if (currentRow >= 0)
        {
            bool isChecked = WHEEL_PATROL_RESULT.isAuditFinish(m_tableDataList[currentRow].task_uuid);
            if (!isChecked)
            {
                DLMessageBox::showDLMessageBox(NULL, "提示", "当前任务未审核完成", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
                return;
            }

            QString strInfraredImageFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/PatrolReport";
            QString strTaskTime = m_tableDataList[currentRow].task_time;
            QString reportName = m_tableDataList[currentRow].task_name + strTaskTime.remove("-").remove("T").remove(":").remove(" ");
			QString fileName = QString("%1/%2.xlsx").arg(strInfraredImageFilePath).arg(reportName);
		//	QString fileName = QString("%1/%2.xls").arg(strInfraredImageFilePath).arg(reportName);
            bool bExist = QFile::exists(fileName);
            if (bExist)
            {
                DLMessageBox::showDLMessageBox(NULL, "提示", "已导出，请在本地查看", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
            else
            {
                // 向Core发送命令是否已经存在生成报告;
                WheelTaskShow task;
                task.task_uuid = m_tableDataList[currentRow].task_uuid;
                task.task_name = m_tableDataList[currentRow].task_name;
                task.task_time = m_tableDataList[currentRow].task_time;

                WHEEL_BACK_TO_CORE_SOCKET.robot_examine_report_isexist_req(reportName, task);
            }
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "请选择任务", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        
    }
    break;
    case 3:
    {
        int currentRow = m_customTableWidget->getTableWidget()->currentRow();
        if (currentRow >= 0)
        {
            QString strInfraredImageFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/PatrolReport";
            QString strTaskTime = m_tableDataList[currentRow].task_time;
            QString reportName = m_tableDataList[currentRow].task_name + strTaskTime.remove("-").remove("T").remove(":").remove(" ");
			QString fileName = QString("%1/%2.xlsx").arg(strInfraredImageFilePath).arg(reportName);
		//	QString fileName = QString("%1/%2.xls").arg(strInfraredImageFilePath).arg(reportName);
            bool bExist = QFile::exists(fileName); 
            if (bExist)
            {
                QDesktopServices::openUrl(QUrl::fromLocalFile(fileName));
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "提示", "本地不存在，请先导出", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "请选择任务", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }

    }
        break;
    default:
        break;
    }
}

void DLWheelPatrolReportGenerate::onSearchButtonClicked()
{
    for (int i = 0; i < m_checkBoxWidgetList.count(); i++)
    {
        QStringList strCheckedIdList = m_checkBoxWidgetList[i]->getCheckedIdList();
        WheelCheckBoxTypeEnum currentType = m_checkBoxWidgetList[i]->getCheckBoxType();
        switch (currentType)
        {
        case WHEEL_DEVICE_AREA_NAME:
            m_currentSearchCondition.m_device_area_uuid = strCheckedIdList;
            break;
        case WHEEL_DEVICE_TYPE:
            m_currentSearchCondition.m_device_type_uuid = strCheckedIdList;
            break;
        case WHEEL_RECOGNITION_TYPE:
            m_currentSearchCondition.m_recognition_type_uuid = strCheckedIdList;
            break;
        case WHEEL_ALARM_LEVEL:
            m_currentSearchCondition.m_alarm_level_id = strCheckedIdList;
            break;
        default:
            break;
        }
    }

    m_currentSearchCondition.m_start_time = m_startDateEdit->date().toString("yyyy-MM-dd");
    m_currentSearchCondition.m_stop_time = m_endDateEdit->date().toString("yyyy-MM-dd");

    initTableData(m_currentSearchCondition);
}

void DLWheelPatrolReportGenerate::onResetButtonClicked()
{
    for (int i = 0; i < m_checkBoxWidgetList.count(); i++)
    {
        m_checkBoxWidgetList[i]->resetCheckBox();
    }
}

void DLWheelPatrolReportGenerate::onExportButtonClicked()
{
    
}