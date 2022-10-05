#include "DLWheelPatrolResultBrowse.h"
#include <QDateTime>
#include <QLabel>
#include <QCalendarWidget>
#include <QLineEdit>
#include <QDateTime>
#include <QFileDialog>
#include <QApplication>
#include <QMessageBox>
#include "LibDLWheelTaskDeviceDBSearch/DLWheelTaskAdministrationData.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "DLWheelTaskCheck.h"
#include "RCFImageDownload.h"
#include "LibDLWheelCustomWidget/DefData.h"

#pragma execution_character_set("utf-8")
#define TABLE_PER_PAGE_COUNT 6

enum CustomButtonList
{
	BTN_NONE,
	BTN_EXPORT,							//导出
	BTN_EXPORT_IMAGE,					//导出任务图片
	BTN_KEY_AUDIT,						//一键审核
	BTN_NUM
};


DLWheelPatrolResultBrowse::DLWheelPatrolResultBrowse()
	: m_isInitWidget(false)
    , m_currentPageIndex(1)
    , m_strCurrentTaskId("")
    , m_pThreadRCFImageDownload(NULL)
	,m_pTaskThreadRCFImageDownload(NULL)
{
	this->setStyleSheet("QWidget#ImageShowBackWidget{background:white;}\
						    QWidget#CommitButtonBackWidget{background:rgb(159,168,218);}\
                            QListWidget::item{height:30px;}");
   
}

DLWheelPatrolResultBrowse::~DLWheelPatrolResultBrowse()
{
    if (m_pThreadRCFImageDownload != NULL)
    {
        m_pThreadRCFImageDownload->stopDownloadImage();
    }
}

void DLWheelPatrolResultBrowse::initDeviceTreeWidget()
{
	m_deviceTreeWidget = new CustomTreeWidget;
    m_deviceTreeWidget->setFixedWidth(320);
    m_deviceTreeWidget->setIsShrink(false);
	m_deviceTreeWidget->setTreeWidgetType(TreeItemWidgetType::ColorRect_CheckBox_Without);
	m_deviceTreeWidget->refreshTreeItemList();

    connect(m_deviceTreeWidget, &CustomTreeWidget::signalDeviceNodeClicked, this, [=](QString strDeviceId) {
        QList<WheelTaskShow> taskList = WHEEL_PATROL_RESULT.getWheelTaskDataForDeviceUUid(strDeviceId);
        m_pTaskListWidget->clear();
        for (int i = 0; i < taskList.count(); i++)
        {
            QString strItemName = QString("%1  |  %2").arg(taskList[i].task_time).arg(taskList[i].task_name);
            QListWidgetItem* item = new QListWidgetItem(strItemName);
            item->setData(Qt::UserRole, taskList[i].task_uuid);
            m_pTaskListWidget->addItem(item);
        }
        m_leftBackWidget->setCurrentIndex(1);
    });
}

void DLWheelPatrolResultBrowse::initTaskList()
{

    CustomButtonListWidget* taskButtonListWidget = new CustomButtonListWidget(this, RIGHT_START_DISPLAY);
   // taskButtonListWidget->addToolButton(0, "返回", ":/Resources/Common/image/doubleLeftArrow.png");
	taskButtonListWidget->addToolButton(0, "刷新", ":/Resources/Common/image/Reset.png");
    taskButtonListWidget->addWidgetFinished();

    connect(taskButtonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelPatrolResultBrowse::onTaskButtonClicked);

    m_pTaskListWidget = new QListWidget;
    connect(m_pTaskListWidget, &QListWidget::itemClicked, this, &DLWheelPatrolResultBrowse::onTaskListClciked);

    m_taskBackWidget = new QWidget;
    m_taskBackWidget->setFixedWidth(320);
    QVBoxLayout* vLayout = new QVBoxLayout(m_taskBackWidget);
    vLayout->addWidget(taskButtonListWidget);
    vLayout->addWidget(m_pTaskListWidget);
    vLayout->setSpacing(0);
    vLayout->setMargin(0);


	//获取所有数据，加载列表
	QList<WheelTaskShow> taskList = WHEEL_PATROL_RESULT.getWheelTaskDataForDeviceUUid("");
	m_pTaskListWidget->clear();
	for (int i = 0; i < taskList.count(); i++)
	{
		QString strItemName = QString("%1  |  %2").arg(taskList[i].task_time).arg(taskList[i].task_name);
		QListWidgetItem* item = new QListWidgetItem(strItemName);
		item->setData(Qt::UserRole, taskList[i].task_uuid);
		m_pTaskListWidget->addItem(item);
	}

}

void DLWheelPatrolResultBrowse::initLeftWidget()
{
    initDeviceTreeWidget();
    initTaskList();

    m_leftBackWidget = new QStackedWidget;
    m_leftBackWidget->setFixedWidth(320);
    m_leftBackWidget->insertWidget(0, m_deviceTreeWidget);
 //   m_leftBackWidget->insertWidget(1, m_taskBackWidget);

}

void DLWheelPatrolResultBrowse::initButtonListWidget()
{
	m_pCustomButtonListWidget = new CustomButtonListWidget;
	m_pCustomButtonListWidget->addToolButton(BTN_EXPORT, "导出", ":/Resources/Common/image/ExportButton.png");
	m_pCustomButtonListWidget->addToolButton(BTN_EXPORT_IMAGE, "导出任务图片", ":/Resources/Common/image/ExportImage.png", QSize(100, 20));
	m_pCustomButtonListWidget->addToolButton(BTN_KEY_AUDIT, "一键审核", ":/Resources/Common/image/KeyAudit.png", QSize(80, 20));
	m_pCustomButtonListWidget->addWidgetFinished();

	connect(m_pCustomButtonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelPatrolResultBrowse::onButtonClicked);
}

void DLWheelPatrolResultBrowse::initTableWidget()
{
	m_customTableWidget = new CustomTableWidget(6);
    m_customTableWidget->setItemBackWhite();
	m_customTableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "任务名称" << "点位名称" << "识别时间" << "识别结果" << "采集信息");
	QTableWidget* tableWidget = m_customTableWidget->getTableWidget();

	tableWidget->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
	tableWidget->setColumnWidth(1, 50);

    connect(m_customTableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        updateTabelData(m_strCurrentTaskId);
    });
    connect(m_customTableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        updateTabelData(m_strCurrentTaskId);
    });

    connect(m_imageTurnPageWidget, &TurnPageWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        updateTabelData(m_strCurrentTaskId);
    });
    connect(m_imageTurnPageWidget, &TurnPageWidget::signalRefreshCurrentPage, this, [=] {
        updateTabelData(m_strCurrentTaskId);
    });
    connect(m_customTableWidget, &CustomTableWidget::signalTableDoubleClicked, this, [=](int row) {
        if (row < m_lstTableData.count())
        {
            DeviceAlarmSearchStruct data = m_lstTableData[row];
            m_taskCheckWidget->setData(data);
            m_taskCheckWidget->setIsSingleTask(true);
            m_taskCheckWidget->show();
        }
    });
    
}

void DLWheelPatrolResultBrowse::initImageShowWidget()
{
	m_pImageShowBackWidget = new QWidget;
	m_pImageShowBackWidget->setObjectName("ImageShowBackWidget");
	m_pImageShowBackWidget->setFixedHeight(455);

	for (int i = 0; i < 6; i++)
	{
		ImageWidget* imageWidget = new ImageWidget;
        m_lstImageWgts.append(imageWidget);
	}

	QGridLayout* gImageLayout = new QGridLayout;
	gImageLayout->addWidget(m_lstImageWgts[0], 0, 0);
	gImageLayout->addWidget(m_lstImageWgts[1], 0, 1);
	gImageLayout->addWidget(m_lstImageWgts[2], 0, 2);
	gImageLayout->addWidget(m_lstImageWgts[3], 1, 0);
	gImageLayout->addWidget(m_lstImageWgts[4], 1, 1);
	gImageLayout->addWidget(m_lstImageWgts[5], 1, 2);
	gImageLayout->setHorizontalSpacing(50);
	gImageLayout->setVerticalSpacing(10);
	gImageLayout->setContentsMargins(0, 0, 10, 0);

    m_checkSuggestTextEdit = new QDateEdit;
    m_checkSuggestTextEdit->setDate(QDate::currentDate());
    m_checkSuggestTextEdit->setFixedWidth(180);

	QLabel* checkPeopleLabel = new QLabel("审查人员:");
	checkPeopleLabel->setAlignment(Qt::AlignLeft);
	
    m_checkPeopleLineEdit = new QLineEdit;
    m_checkPeopleLineEdit->setFixedSize(QSize(180, 25));

	QWidget* commitButtonBackWidget = new QWidget;
	commitButtonBackWidget->setFixedSize(QSize(200, 30));
	commitButtonBackWidget->setObjectName("CommitButtonBackWidget");
	QToolButton* pButtonCommit = new QToolButton;
	pButtonCommit->setText("提交");
	pButtonCommit->setFixedSize(QSize(70, 25));
    connect(pButtonCommit, &QToolButton::clicked, this, [=] {
        if (m_strCurrentTaskId.isEmpty())
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "未选择任务", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return;
        }
        else
        {
            bool isFinish = WHEEL_PATROL_RESULT.isAuditFinish(m_strCurrentTaskId);
            if (isFinish)
            {
                WheelPartrolResultAudit resultAudit;
                resultAudit.task_uuid = m_strCurrentTaskId;
                resultAudit.dealed_user = m_checkPeopleLineEdit->text();
                resultAudit.dealed_time = m_checkSuggestTextEdit->date().toString("yyyy-MM-dd");
                WHEEL_BACK_TO_CORE_SOCKET.robot_patrol_result_audit_req(resultAudit);
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "提示", "存在未审核的任务", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        }
    });
	
	QHBoxLayout* hButtonLayout = new QHBoxLayout(commitButtonBackWidget);
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(pButtonCommit);
	hButtonLayout->setContentsMargins(0, 0, 20, 0);

	QVBoxLayout* vCheckPeopleLayout = new QVBoxLayout;
    vCheckPeopleLayout->addStretch();
    vCheckPeopleLayout->addWidget(m_checkSuggestTextEdit);
	vCheckPeopleLayout->addWidget(checkPeopleLabel);
	vCheckPeopleLayout->addWidget(m_checkPeopleLineEdit);
	vCheckPeopleLayout->addSpacing(30);
	vCheckPeopleLayout->addWidget(commitButtonBackWidget);
	vCheckPeopleLayout->setSpacing(5);
	vCheckPeopleLayout->setMargin(0);

	m_imageTurnPageWidget = new TurnPageWidget;

	QVBoxLayout* vImageShowWidgetLayout = new QVBoxLayout;
	vImageShowWidgetLayout->addLayout(gImageLayout);
	vImageShowWidgetLayout->addWidget(m_imageTurnPageWidget);
	vImageShowWidgetLayout->setSpacing(10);
	vImageShowWidgetLayout->setContentsMargins(10, 10, 0, 0);

	QHBoxLayout* hImageShowWidgetLayout = new QHBoxLayout(m_pImageShowBackWidget);
	hImageShowWidgetLayout->addLayout(vImageShowWidgetLayout);
	hImageShowWidgetLayout->addLayout(vCheckPeopleLayout);
	hImageShowWidgetLayout->setSpacing(50);
	hImageShowWidgetLayout->setMargin(0);
}

void DLWheelPatrolResultBrowse::updateTabelData(QString strTaskId)
{
    int totalPage, totalCount;
    WHEEL_PATROL_RESULT.getWheelDeviceAlarmForTaskUUidPage(totalPage, totalCount, TABLE_PER_PAGE_COUNT, strTaskId, "", "");
    m_customTableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);
    m_imageTurnPageWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

    m_lstTableData = WHEEL_PATROL_RESULT.getWheelDeviceAlarmForTaskUUidList(m_currentPageIndex, TABLE_PER_PAGE_COUNT, strTaskId, "", "");

    for (int i = 0; i < m_lstImageWgts.count(); i++)
    {
        m_lstImageWgts[i]->setImage("");
        m_lstImageWgts[i]->setText("");
    }

    if (!m_lstTableData.isEmpty())
    {
        QStringList strTaskIdList, strDeviceIdList;
        for (int i = 0; i < m_lstTableData.count(); i++)
        {
            strTaskIdList.append(m_lstTableData[i].task_uuid);
            strDeviceIdList.append(m_lstTableData[i].device_uuid);

            QString strImageText = m_lstTableData[i].device_point_type_name + "  " + m_lstTableData[i].inspect_time;
            m_lstImageWgts[i]->setText(strImageText);
        }

        // 如果table进行了刷新，释放之前下载图片的线程，重新开辟线程进行新的下载;
        if (m_pThreadRCFImageDownload != NULL)
        {
            m_pThreadRCFImageDownload->stopDownloadImage();
            disconnect(m_pThreadRCFImageDownload, &RCFImageDownload::signalDowmloadFinished, this, &DLWheelPatrolResultBrowse::onRcfDownloadImageFinished);
            m_pThreadRCFImageDownload->deleteLater();
        }
        // 这里加上一个图片加载窗口，防止用户多次切换任务，下载和显示时界面操作卡顿;
		DLMessageBox *pMessageBox = new DLMessageBox(this);
		pMessageBox->setFixedWidth(250);
		pMessageBox->setMessageContent("图片加载中");
		pMessageBox->setButtonOKVisible(false);
		pMessageBox->setWindowModality(Qt::ApplicationModal);

        m_pThreadRCFImageDownload = new RCFImageDownload(this);
        connect(m_pThreadRCFImageDownload, &RCFImageDownload::signalDowmloadFinished, this, &DLWheelPatrolResultBrowse::onRcfDownloadImageFinished);
        // 所有图片下载完成时，关闭图片加载窗口;
        connect(m_pThreadRCFImageDownload, &RCFImageDownload::signalAllDownloaded, this, [=] {
			pMessageBox->close();
        });
        m_pThreadRCFImageDownload->setImageInfoList(strTaskIdList, strDeviceIdList);
        m_pThreadRCFImageDownload->start();

		pMessageBox->show();
    }

    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
    tableWidget->clearContents();
    tableWidget->setRowCount(0);
    for (int i = 0; i < m_lstTableData.count(); i++)
    {
        tableWidget->insertRow(i);
        QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
        checkBoxItem->setCheckState(Qt::Unchecked);
        checkBoxItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, checkBoxItem);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 1, idItem);

        QTableWidgetItem* taskNameItem = new QTableWidgetItem(m_lstTableData[i].save_type_name);
        tableWidget->setItem(i, 2, taskNameItem);
        taskNameItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* PointNameItem = new QTableWidgetItem(m_lstTableData[i].device_point_type_name);
        PointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, PointNameItem);

        QTableWidgetItem* recognitionTimeItem = new QTableWidgetItem(m_lstTableData[i].inspect_time);
        recognitionTimeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, recognitionTimeItem);

        QTableWidgetItem* recognitionResultItem = new QTableWidgetItem(m_lstTableData[i].inspect_result);
        recognitionResultItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, recognitionResultItem);

        QTableWidgetItem* collectInfoItem = new QTableWidgetItem(m_lstTableData[i].recognition_type_name);
        collectInfoItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 6, collectInfoItem);

        if (m_lstTableData[i].is_dealed)
        {
            checkBoxItem->setBackgroundColor(QColor(175, 191, 255));
            idItem->setBackgroundColor(QColor(175, 191, 255));
            taskNameItem->setBackgroundColor(QColor(175, 191, 255));
            PointNameItem->setBackgroundColor(QColor(175, 191, 255));
            recognitionTimeItem->setBackgroundColor(QColor(175, 191, 255));
            recognitionResultItem->setBackgroundColor(QColor(175, 191, 255));
            collectInfoItem->setBackgroundColor(QColor(175, 191, 255));
        }

        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelPatrolResultBrowse::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

    initLeftWidget();
	initButtonListWidget();
    initImageShowWidget();
	initTableWidget();

	QVBoxLayout* vRightLayout = new QVBoxLayout;
	vRightLayout->addWidget(m_pCustomButtonListWidget);
	vRightLayout->addWidget(m_customTableWidget);
	vRightLayout->addWidget(m_pImageShowBackWidget);
	vRightLayout->setSpacing(0);
	vRightLayout->setMargin(0);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	//hMainLayout->addWidget(m_leftBackWidget);
	hMainLayout->addWidget(m_taskBackWidget);
	hMainLayout->addLayout(vRightLayout);
	hMainLayout->setSpacing(0);
	hMainLayout->setMargin(0);

    m_taskCheckWidget = new DLWheelTaskCheck(this);
    m_taskCheckWidget->setTaskCheckType(PatrolResultBrowse);
    connect(m_taskCheckWidget, &DLWheelTaskCheck::signalRefreshTable, this, [=] {
        updateTabelData(m_strCurrentTaskId);
    });
}

void DLWheelPatrolResultBrowse::onPatrolResultCheckPeopleCallBack(bool isSuccess, QString strTaskId, QString strMsg)
{
    if (isSuccess)
    {
        m_checkSuggestTextEdit->clear();
        m_checkPeopleLineEdit->clear();
        DLMessageBox::showDLMessageBox(NULL, "提示", "提交成功", MessageButtonType::BUTTON_OK, true, QSize(250, 180));

        QString strInfraredImageFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/PatrolReport";

        WheelTaskShow task = WHEEL_TASK_ADMINISTRATION.getWheelTaskStru(strTaskId);
        QString reportName = task.task_name + task.task_time.remove("-").remove("T").remove(":");
        QString fileName = QString("%1/%2.xlsx").arg(strInfraredImageFilePath).arg(reportName);
        bool bExist = QFile::exists(fileName);
        if (!bExist)
        {
            // 向Core发送命令是否已经存在生成报告;
            WHEEL_BACK_TO_CORE_SOCKET.robot_examine_report_isexist_req(reportName, task);
        }
    }
    else
    {
        DLMessageBox::showDLMessageBox(NULL, "错误", "提交失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
    }
}

void DLWheelPatrolResultBrowse::onButtonClicked(int buttonId)
{
    
    if (buttonId == BTN_EXPORT)
    {// 导出;
		//qDebug() << "导出" << __LINE__;
        QList<QStringList> excelData;
        excelData.append(QStringList() << "任务名称" << "点位名称" << "识别时间" << "识别结果" << "采集信息" << "采集信息");

        QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
        for (int i = 0; i < tableWidget->rowCount(); i++)
        {
            QStringList strDataList;
            if (tableWidget->item(i, 0)->checkState() == Qt::Checked)
            {
                for (int j = 2; j < tableWidget->columnCount() - 1; j++)
                {
                    strDataList.append(tableWidget->item(i, j)->text());
                }

                strDataList << "" << "";
                excelData.append(strDataList);
            }
        }

        if (excelData.count() == 1)
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "未选择导出项", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return;
        }

        QString dir = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("excel(*.xlsx)"));
        if (dir.isEmpty())
        {
            return;
        }

        SearchRecordCreateExcel excel;
        excel.CreateNewExcelForList(excelData, dir, EXCEL_PATROL_RESULT_BROWSE);
    }
	else if (buttonId == BTN_EXPORT_IMAGE)
	{
        //导出图片
		//qDebug() << "导出图片" << __LINE__;
        if (m_customTableWidget->get_all_check_status())
        {
	        QListWidgetItem *pItem = m_pTaskListWidget->currentItem();
            if (pItem)
            {
                QString strTaskName = "";
                QStringList lstTaskNames = pItem->text().split("|");
                if (lstTaskNames.size() >= 2)
                {
                    strTaskName = lstTaskNames.at(1);
                }
                strTaskName = strTaskName.replace(" ", "");
                QString strTaskId = pItem->data(Qt::UserRole).toString();			//获取当前任务的id
                if (!strTaskId.isEmpty())
                {
                    QString strSelectDir = QFileDialog::getExistingDirectory(this, tr("请选择文件夹"), QString(qApp->applicationDirPath()), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
                    if (strSelectDir.isEmpty())
                    {
                        return;
                    }
                    if (NULL == m_pTaskThreadRCFImageDownload)
                    {
                        m_pTaskThreadRCFImageDownload = new RCFImageDownload(this);
                    }

                    // 这里加上一个图片加载窗口，防止用户多次切换任务，下载和显示时界面操作卡顿;
                    DLMessageBox *pMessageBox = new DLMessageBox(this);
                    pMessageBox->setFixedWidth(250);
                    pMessageBox->setMessageContent("图片下载中");
                    pMessageBox->setButtonOKVisible(false);
                    pMessageBox->setWindowModality(Qt::ApplicationModal);

                    m_pTaskThreadRCFImageDownload->SetTaskExportImage(strTaskId, strTaskName, strSelectDir);
                    // 所有图片下载完成时，关闭图片加载窗口;
                    connect(m_pTaskThreadRCFImageDownload, &RCFImageDownload::signalAllDownloaded, this, [=] {
                        m_pTaskThreadRCFImageDownload->disconnect();
                        pMessageBox->close();
                    });
                    m_pTaskThreadRCFImageDownload->start();
                    pMessageBox->show();

                }
            }
        }
        else
        {
            QString strSelectDir = QFileDialog::getExistingDirectory(this, tr("请选择文件夹"), QString(qApp->applicationDirPath()), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
            QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
            for (int i = 0; i < tableWidget->rowCount(); i++)
            {
                if (tableWidget->item(i, 0)->checkState() == Qt::Checked)
                {
                    DeviceAlarmSearchStruct vv = m_lstTableData[i];
                    QDir sourceDir(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/DeviceImage/" + m_lstTableData[i].task_uuid + "/" + m_lstTableData[i].device_uuid);
                    QDir targetDir(strSelectDir + "/" + m_lstTableData[i].device_uuid);
                    if (!targetDir.exists())
                    {
                        if (!targetDir.mkdir(targetDir.absolutePath()))
                            return;
                    }

                    QFileInfoList fileInfoList = sourceDir.entryInfoList();
                    foreach(QFileInfo fileInfo, fileInfoList)
                    {
                        if (fileInfo.fileName() == "." || fileInfo.fileName() == "..")
                            continue;

                        //                     if (fileInfo.isDir()) {    /**< 当为目录时，递归的进行copy */
                        //                         if (!copyDirectoryFiles(fileInfo.filePath(),
                        //                             targetDir.filePath(fileInfo.fileName()),
                        //                             coverFileIfExist))
                        //                             return false;
                        //                     }
                                            //< 当允许覆盖操作时，将旧文件进行删除操作 
                        if (targetDir.exists(fileInfo.fileName()))
                        {
                            targetDir.remove(fileInfo.fileName());
                        }
                        // 进行文件copy
                        QFile::copy(fileInfo.filePath(), targetDir.filePath(fileInfo.fileName()));
                    }
                }
            }
        }
	}
	else if (buttonId == BTN_KEY_AUDIT)
	{//一键审核
		//qDebug() << "一键审核" << __LINE__;
		QListWidgetItem *pItem = m_pTaskListWidget->currentItem();
		if (pItem)
		{
			QString strTaskId = pItem->data(Qt::UserRole).toString();			//获取当前任务的id
			if (!strTaskId.isEmpty())
			{
				//class LibDLHangRailRobotBackground HangRail_BACK_TO_CORE_SOCKET.
					//一键审核
				//	void robot_fast_audit_task_req(QString taskUUid);
				//一键审核信号 
				//boost::signals2::signal<void(bool, QString)> singnal_database_fast_audit_task;
				WHEEL_BACK_TO_CORE_SOCKET.robot_fast_audit_task_req(strTaskId);
			}
		}
	}
}

void DLWheelPatrolResultBrowse::onTaskButtonClicked(int buttonId)
{
    if (buttonId == 0)
    {
       // m_leftBackWidget->setCurrentIndex(0);
		//获取所有数据，加载列表
		QList<WheelTaskShow> taskList = WHEEL_PATROL_RESULT.getWheelTaskDataForDeviceUUid("");
		m_pTaskListWidget->clear();
		for (int i = 0; i < taskList.count(); i++)
		{
			QString strItemName = QString("%1  |  %2").arg(taskList[i].task_time).arg(taskList[i].task_name);
			QListWidgetItem* item = new QListWidgetItem(strItemName);
			item->setData(Qt::UserRole, taskList[i].task_uuid);
			m_pTaskListWidget->addItem(item);
		}
    }
}

void DLWheelPatrolResultBrowse::onTaskListClciked(QListWidgetItem* item)
{
    if (item != NULL)
    {
		m_currentPageIndex = 1;
        QString taskId = item->data(Qt::UserRole).toString();
        m_strCurrentTaskId = taskId;
        updateTabelData(taskId);
    }    
}

void DLWheelPatrolResultBrowse::onRcfDownloadImageFinished(int result, int imageIndex, QString strImagePath)
 {
    if (imageIndex >= m_lstImageWgts.count())
    {
        return;
    }

    if (result >= 0)
    {
        m_lstImageWgts[imageIndex]->setImage(strImagePath);
    }
    else
    {
        m_lstImageWgts[imageIndex]->setImage(":/Resources/Common/image/ImageLoadFailed.png");
    }        
}

void DLWheelPatrolResultBrowse::AKeyAuditFinishSlot(bool bIsSuccess, QString strErrorMessage)
{
	//qDebug() << "审核是否成功" << bIsSuccess << __LINE__;
	if (bIsSuccess)
	{//成功
		QMessageBox msgBox;
		msgBox.setText("审核成功！");
		msgBox.exec();
	}
	else
	{//失败
		QMessageBox msgBox;
		msgBox.setText("审核失败");
		msgBox.setInformativeText(strErrorMessage);
		msgBox.exec();
	}
}

