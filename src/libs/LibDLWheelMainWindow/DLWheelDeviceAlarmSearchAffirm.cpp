#include "DLWheelDeviceAlarmSearchAffirm.h"
#include <QDateTime>
#include <QCalendarWidget>
#include "DLWheelTaskCheck.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelTaskAdministrationData.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"
#include <QFileDialog>

#include "LibDLWheelCustomWidget/DefData.h"

#define TABLE_PER_PAGE_COUNT 16

DLWheelDeviceAlarmSearchAffirm::DLWheelDeviceAlarmSearchAffirm()
	: m_isInitWidget(false)
    , m_currentPageIndex(1)
    , m_isAdminLogin(false)
{
	this->setStyleSheet("QWidget#TopBackWidget{background:rgb(240, 240, 240);}\
							QToolButton{border:none;}\
							QTableWidget{border:none;}");
}

void DLWheelDeviceAlarmSearchAffirm::initCheckBoxWidget()
{
    m_checkBoxBackWidget = new QWidget;

    QList<WheelTaskAdminCheckBoxStruct> checkBoxDataList = WHEEL_TASK_ADMINISTRATION.getTaskAdminCheckBoxData(WHEEL_DEVICE_ALARM_MES);
    QVBoxLayout* vCheckBoxLayout = new QVBoxLayout(m_checkBoxBackWidget);
    for (int i = 0; i < checkBoxDataList.count(); i++)
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

void DLWheelDeviceAlarmSearchAffirm::initButtonListWidget()
{
	QLabel* startTimeLabel = new QLabel("开始时间:");
	m_startTimeEdit = new QDateEdit;
	m_startTimeEdit->setDate(QDate::currentDate());
	m_startTimeEdit->setCalendarPopup(true);
	m_startTimeEdit->setCalendarWidget(new QCalendarWidget);

	QLabel* endTimeLabel = new QLabel("结束时间:");
	m_endTimeEdit = new QDateEdit;
	m_endTimeEdit->setDate(QDate::currentDate().addMonths(1));
	m_endTimeEdit->setCalendarPopup(true);
	m_endTimeEdit->setCalendarWidget(new QCalendarWidget);

	QWidget* timeChooseBackWidget = new QWidget;
	QHBoxLayout* hTimeChooseLayout = new QHBoxLayout(timeChooseBackWidget);
	hTimeChooseLayout->addWidget(startTimeLabel);
	hTimeChooseLayout->addWidget(m_startTimeEdit);
	hTimeChooseLayout->addWidget(endTimeLabel);
	hTimeChooseLayout->addWidget(m_endTimeEdit);
	hTimeChooseLayout->setSpacing(3);
	hTimeChooseLayout->setContentsMargins(0, 0, 3, 0);

	m_customButtonListWidget = new CustomButtonListWidget;
	m_customButtonListWidget->addWidget(timeChooseBackWidget);
	m_customButtonListWidget->addToolButton(0, "查询", ":/Resources/Common/image/Search.png");
	m_customButtonListWidget->addToolButton(1, "重置", ":/Resources/Common/image/Reset.png");
	m_customButtonListWidget->addToolButton(2, "导出", ":/Resources/Common/image/ExportButton.png");
	m_customButtonListWidget->addWidgetFinished();

	connect(m_customButtonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelDeviceAlarmSearchAffirm::onButtonClicked);
}

void DLWheelDeviceAlarmSearchAffirm::initTopWidget()
{
    initCheckBoxWidget();
	initButtonListWidget();

	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("TopBackWidget");

	QVBoxLayout* vTopLayout = new QVBoxLayout(m_topBackWidget);
    vTopLayout->addWidget(m_checkBoxBackWidget);
	vTopLayout->addWidget(m_customButtonListWidget);
	vTopLayout->setSpacing(20);
	vTopLayout->setMargin(6);
}

void DLWheelDeviceAlarmSearchAffirm::initDeviceaTreeWidget()
{
	m_deviceTreeWidget = new CustomTreeWidget;
	m_deviceTreeWidget->setTreeWidgetType(TreeItemWidgetType::ColorRect_CheckBox_Without);
	m_deviceTreeWidget->refreshTreeItemList();
}

void DLWheelDeviceAlarmSearchAffirm::initTaskFormulateTableSingle()
{
	m_taskFormulateTable = new CustomTableWidget(7);
    m_taskFormulateTable->setItemBackWhite();
	m_taskFormulateTable->setHorizontalHeaderLabels(QStringList() << "序号" << "识别类型" << "点位名称" << "识别结果" << "告警等级" << "识别时间" << "采集信息");
    connect(m_taskFormulateTable, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableDataSingle(m_currentSearchCondition);
    });
    connect(m_taskFormulateTable, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableDataSingle(m_currentSearchCondition);
    });
    QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();
    connect(m_taskFormulateTable, &CustomTableWidget::signalTableDoubleClicked, this, [=](int row) {
        if (row < m_singleTableDataList.count())
        {
            DeviceAlarmSearchStruct data = m_singleTableDataList[row];
            m_taskCheckWidget->setData(data);
            m_taskCheckWidget->setIsSingleTask(true);
            m_taskCheckWidget->show();
        }
    });
}

void DLWheelDeviceAlarmSearchAffirm::initTaskFormulateTableMulti()
{
    m_taskFormulateTable = new CustomTableWidget(11);
    m_taskFormulateTable->setHorizontalHeaderLabels(QStringList() << "序号" << "审核状态" << "任务名称" << "任务状态" << "开始时间" \
                                                                    << "结束时间" << "总点数" << "告警点数" << "识别异常总数" << "不识别点数" << "识别异常点数");

    connect(m_taskFormulateTable, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableDataMulti();
    });
    connect(m_taskFormulateTable, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableDataMulti();
    });
    QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();
    connect(m_taskFormulateTable, &CustomTableWidget::signalTableDoubleClicked, this, [=](int row) {
        if (row < m_multiCheckTableDataList.count())
        {
            QString strTaskId = m_multiCheckTableDataList[row].task_uuid;
            int totalCount, pageCount;
            WHEEL_PATROL_RESULT.getWheelDeviceAlarmForTaskUUidPage(pageCount, totalCount, 20, strTaskId, "", "");
            QList<DeviceAlarmSearchStruct> dataList = WHEEL_PATROL_RESULT.getWheelDeviceAlarmForTaskUUidList(1, totalCount, strTaskId, "", "");
            if (dataList.isEmpty())
            {
                DLMessageBox::showDLMessageBox(NULL, "错误", "未查找到数据", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
                return;
            }

            m_taskCheckWidget->setDataList(dataList);
            m_taskCheckWidget->show();
        }
    });
}

void DLWheelDeviceAlarmSearchAffirm::initTableDataSingle(WheelPatrolParameter wheelPatrolPara)
{
    QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();
    
    int totalPage, totalCount;
    WHEEL_PATROL_RESULT.getWheelDeviceAlarmSearchPage(totalPage, totalCount, TABLE_PER_PAGE_COUNT, wheelPatrolPara);
    if (m_currentPageIndex > totalPage)
    {
        m_currentPageIndex = totalPage;
    }

    m_taskFormulateTable->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

    m_singleTableDataList = WHEEL_PATROL_RESULT.getWheelDeviceAlarmSearchList(m_currentPageIndex, TABLE_PER_PAGE_COUNT, wheelPatrolPara);
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    for (int i = 0; i < m_singleTableDataList.count(); i++)
    {
        tableWidget->insertRow(i);
        QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
        checkBoxItem->setCheckState(Qt::Unchecked);
        checkBoxItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, checkBoxItem);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 1, idItem);

        QTableWidgetItem* recognitionItem = new QTableWidgetItem(m_singleTableDataList[i].recognition_type_name);
        tableWidget->setItem(i, 2, recognitionItem);
        recognitionItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* pointNameItem = new QTableWidgetItem(m_singleTableDataList[i].device_point_type_name);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, pointNameItem);

        QTableWidgetItem* recognitionResultItem = new QTableWidgetItem(m_singleTableDataList[i].inspect_result);
        recognitionResultItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, recognitionResultItem);
       
        QTableWidgetItem* alarmLevelItem = new QTableWidgetItem(m_singleTableDataList[i].alarm_level_name);
        alarmLevelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, alarmLevelItem);

        QTableWidgetItem* recognitionTimeItem = new QTableWidgetItem(m_singleTableDataList[i].inspect_time);
        recognitionTimeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 6, recognitionTimeItem);

        QTableWidgetItem* collectInfoItem = new QTableWidgetItem(m_singleTableDataList[i].save_type_name);
        collectInfoItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 7, collectInfoItem);

        if (m_singleTableDataList[i].is_dealed)
        {
            checkBoxItem->setBackgroundColor(QColor(175, 191, 255));
            idItem->setBackgroundColor(QColor(175, 191, 255));
            recognitionItem->setBackgroundColor(QColor(175, 191, 255));
            pointNameItem->setBackgroundColor(QColor(175, 191, 255));
            recognitionResultItem->setBackgroundColor(QColor(175, 191, 255));
            alarmLevelItem->setBackgroundColor(QColor(175, 191, 255));
            recognitionTimeItem->setBackgroundColor(QColor(175, 191, 255));
            collectInfoItem->setBackgroundColor(QColor(175, 191, 255));
        }

        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelDeviceAlarmSearchAffirm::initTableDataMulti()
{
    QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();

    int totalPage, totalCount;
    WHEEL_PATROL_RESULT.getDeviceAlarmSearchVerifyPage(totalPage, totalCount, TABLE_PER_PAGE_COUNT);
    if (m_currentPageIndex > totalPage)
    {
        m_currentPageIndex = totalPage;
    }

    m_taskFormulateTable->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

    m_multiCheckTableDataList = WHEEL_PATROL_RESULT.getDeviceAlarmSearchVerify(m_currentPageIndex, TABLE_PER_PAGE_COUNT);
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    for (int i = 0; i < m_multiCheckTableDataList.count(); i++)
    {
        tableWidget->insertRow(i);
        QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
        checkBoxItem->setCheckState(Qt::Unchecked);
        checkBoxItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, checkBoxItem);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 1, idItem);
        idItem->setBackground(QBrush(QColor(255, 0, 0)));

        QTableWidgetItem* recognitionItem = new QTableWidgetItem(m_multiCheckTableDataList[i].task_audi_status);
        tableWidget->setItem(i, 2, recognitionItem);
        recognitionItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* pointNameItem = new QTableWidgetItem(m_multiCheckTableDataList[i].task_name);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, pointNameItem);

        QTableWidgetItem* recognitionResultItem = new QTableWidgetItem(m_multiCheckTableDataList[i].task_status_name);
        recognitionResultItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, recognitionResultItem);

        QTableWidgetItem* alarmLevelItem = new QTableWidgetItem(m_multiCheckTableDataList[i].task_start_time);
        alarmLevelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, alarmLevelItem);

        QTableWidgetItem* recognitionTimeItem = new QTableWidgetItem(m_multiCheckTableDataList[i].task_end_time);
        recognitionTimeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 6, recognitionTimeItem);

        QTableWidgetItem* collectInfoItem = new QTableWidgetItem(QString::number(m_multiCheckTableDataList[i].task_total_devices));
        collectInfoItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 7, collectInfoItem);

        QTableWidgetItem* collectInfo1Item = new QTableWidgetItem(QString::number(m_multiCheckTableDataList[i].task_total_bugs));
        collectInfo1Item->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 8, collectInfo1Item);

        QTableWidgetItem* collectInfo2Item = new QTableWidgetItem(QString::number(m_multiCheckTableDataList[i].task_total_mistake));
        collectInfo2Item->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 9, collectInfo2Item);

        QTableWidgetItem* collectInfo3Item = new QTableWidgetItem(QString::number(m_multiCheckTableDataList[i].task_total_nonrecognition));
        collectInfo3Item->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 10, collectInfo3Item);

        QTableWidgetItem* collectInfo4Item = new QTableWidgetItem(QString::number(m_multiCheckTableDataList[i].task_total_unusual));
        collectInfo4Item->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 11, collectInfo4Item);

        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelDeviceAlarmSearchAffirm::initBottomWidget()
{
	initDeviceaTreeWidget();

	m_bottomBackWidget = new QWidget;

	QHBoxLayout* hBottomLayout = new QHBoxLayout(m_bottomBackWidget);
	hBottomLayout->addWidget(m_deviceTreeWidget);
	hBottomLayout->addWidget(m_taskFormulateTable);
	hBottomLayout->setMargin(0);
	hBottomLayout->setSpacing(5);
}

void DLWheelDeviceAlarmSearchAffirm::initTextColorRefreshTimer()
{
    m_isRefreshTextColor = true;
    m_textColorRefreshTimer.setInterval(1000);
    connect(&m_textColorRefreshTimer, &QTimer::timeout, this, [=] {
        m_isRefreshTextColor = !m_isRefreshTextColor;
        QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();
        for (int i = 0; i < tableWidget->rowCount(); i++)
        {
            QColor textColor;
            if (m_isRefreshTextColor)
            {
                QString strAlarmText = tableWidget->item(i, 5)->text();
                textColor = getItemColor(strAlarmText);
            }
            else
            {
                textColor = Qt::black;
            }
           
            for (int j = 0; j < tableWidget->columnCount(); j++)
            {
                tableWidget->item(i, j)->setForeground(textColor);
            }
        }
                
    });

    m_textColorRefreshTimer.start();
}

void DLWheelDeviceAlarmSearchAffirm::onSearchButtonClicked()
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

    m_currentSearchCondition.m_start_time = m_startTimeEdit->date().toString("yyyy-MM-dd");
    m_currentSearchCondition.m_stop_time = m_endTimeEdit->date().toString("yyyy-MM-dd");

    m_currentPageIndex = 1;
    initTableDataSingle(m_currentSearchCondition);
}

void DLWheelDeviceAlarmSearchAffirm::onResetButtonClicked()
{
    for (int i = 0; i < m_checkBoxWidgetList.count(); i++)
    {
        m_checkBoxWidgetList[i]->resetCheckBox();
    }
}

void DLWheelDeviceAlarmSearchAffirm::onExportButtonClicked()
{
    QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();

    QList<QStringList> excelData;
    excelData.append(QStringList() << "识别类型" << "点位名称" << "识别结果" << "告警等级" << "识别时间" << "采集信息" << "采集信息");

    for (int i = 0; i < tableWidget->rowCount(); i++)
    {
        QStringList strDataList;
        if (tableWidget->item(i, 0)->checkState() == Qt::Checked)
        {
            if (m_singleTableDataList[i].is_dealed)
            {
                for (int j = 2; j < tableWidget->columnCount() - 1; j++)
                {
                    strDataList.append(tableWidget->item(i, j)->text());
                }
                strDataList << "" << "";
                excelData.append(strDataList);
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "提示", "当前有未确认的任务", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
                return;
            }
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
    excel.CreateNewExcelForList(excelData, dir, EXCEL_DEVICE_ALARM_SEARCH_AUDIT);
}

void DLWheelDeviceAlarmSearchAffirm::initWidget()
{
	if (m_isInitWidget)
	{
        if (!m_isAdminLogin)
        {
            initTableDataSingle(m_currentSearchCondition);
        }
		return;
	}
	m_isInitWidget = true;

	initTopWidget();

    if (m_isAdminLogin)
    {
        m_topBackWidget->setVisible(false);
        initTaskFormulateTableMulti();
        initTableDataMulti();
    }
    else
    {
        initTaskFormulateTableSingle();
        initTableDataSingle(m_currentSearchCondition);
        initTextColorRefreshTimer();
    }
    
	initBottomWidget();

	m_taskCheckWidget = new DLWheelTaskCheck(this);
    m_taskCheckWidget->setTaskCheckType(DeviceAlarmSearchAffirm);
    connect(m_taskCheckWidget, &DLWheelTaskCheck::signalRefreshTable, this, [=] {
        if (m_isAdminLogin)
        {
            initTableDataMulti();
        }
        else
        {
            initTableDataSingle(m_currentSearchCondition);
        }
    });

	QVBoxLayout* vTopLayout = new QVBoxLayout(this);
	vTopLayout->addWidget(m_topBackWidget);
	vTopLayout->addWidget(m_bottomBackWidget);
	vTopLayout->setSpacing(0);
	vTopLayout->setMargin(0);

   
}

void DLWheelDeviceAlarmSearchAffirm::onButtonClicked(int buttonId)
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
        onExportButtonClicked();
    }
        break;
    default:
        break;
    }
}

void DLWheelDeviceAlarmSearchAffirm::onShowTaskCheckWindow()
{

}

void DLWheelDeviceAlarmSearchAffirm::isAdminLogin(bool isAdmin)
{
	// 如果不是管理员需要隐藏顶部查询项;
    m_isAdminLogin = isAdmin;
}

QColor DLWheelDeviceAlarmSearchAffirm::getItemColor(QString strSaskStatus)
{
    QColor color;
    if (!strSaskStatus.compare("正常"))
    {
        color = QColor(0, 128, 0);
    }
    else if (!strSaskStatus.compare("预警"))
    {
        color = QColor(0, 0, 255);
    }
    else if (!strSaskStatus.compare("一般告警"))
    {
        color = QColor(255, 255, 0);
    }
    else if (!strSaskStatus.compare("严重告警"))
    {
        color = QColor(255, 128, 10);
    }
    else if (!strSaskStatus.compare("危急告警"))
    {
        color = QColor(255, 0, 0);
    }
    else if (!strSaskStatus.compare("未识别异常"))
    {
        color = Qt::gray;
    }

    return color;
}