#include "DLWheelGenerateExcel.h"
#include <QDateTime>
#include <QCalendarWidget>
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelTaskAdministrationData.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QFileDialog>
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"

#include "LibDLWheelCustomWidget/DefData.h"
#include "LibDLCreateExcel/CreateExcelForQt5.h"


#pragma execution_character_set("utf-8")

#define TABLE_PER_PAGE_COUNT 16

DLWheelGenerateExcel::DLWheelGenerateExcel()
	: m_isInitWidget(false)
    , m_currentPageIndex(1)
{
	this->setStyleSheet("QWidget#TopBackWidget{background:rgb(240, 240, 240);}");
}

void DLWheelGenerateExcel::initCheckBoxList()
{
    m_checkBoxBackWidget = new QWidget;

    QList<WheelTaskAdminCheckBoxStruct> checkBoxDataList = WHEEL_TASK_ADMINISTRATION.getTaskAdminCheckBoxData(WHEEL_CREATE_EXCEL_REPORT);
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

void DLWheelGenerateExcel::initButtonWidget()
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
	m_customButtonListWidget->addWidgetFinished();

    connect(m_customButtonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelGenerateExcel::onButtonClicked);
}

void DLWheelGenerateExcel::initTopWidget()
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

void DLWheelGenerateExcel::initDeviceTreeWidget()
{
	m_deviceTreeWidget = new CustomTreeWidget;
	m_deviceTreeWidget->setTreeWidgetType(TreeItemWidgetType::ColorRect_CheckBox_Without);
	m_deviceTreeWidget->setSearchLineEditVisible(false);
	m_deviceTreeWidget->refreshTreeItemList();
}

void DLWheelGenerateExcel::initTableWidget()
{
    m_customTableWidget = new CustomTableWidget(7);
    m_customTableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "识别类型" << "点位名称" << "识别结果" << "告警等级" << "识别时间" << "采集信息");
    connect(m_customTableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableData(m_currentSearchCondition);
    });
    connect(m_customTableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableData(m_currentSearchCondition);
    });
}

void DLWheelGenerateExcel::initTableData(WheelPatrolParameter wheelPatrolPara /*= WheelPatrolParameter()*/)
{
    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();

    int totalPage, totalCount;
    WHEEL_PATROL_RESULT.getCreateExcelAlarmSearchCount(totalPage, totalCount, TABLE_PER_PAGE_COUNT, wheelPatrolPara);
    if (m_currentPageIndex > totalPage)
    {
        m_currentPageIndex = totalPage;
    }

    m_customTableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

    m_tableDataList = WHEEL_PATROL_RESULT.getCreateExcelAlarmSearch(m_currentPageIndex, TABLE_PER_PAGE_COUNT, wheelPatrolPara);
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    for (int i = 0; i < m_tableDataList.count(); i++)
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

        QTableWidgetItem* recognitionItem = new QTableWidgetItem(m_tableDataList[i].at(FIELDS_RECOGNITION_TYPE_NAME + 2));
        tableWidget->setItem(i, 2, recognitionItem);
        recognitionItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* pointNameItem = new QTableWidgetItem(m_tableDataList[i].at(FIELDS_DEVICE_POINT_TYPE_NAME + 2));
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, pointNameItem);

        QTableWidgetItem* recognitionResultItem = new QTableWidgetItem(m_tableDataList[i].at(FIELDS_INSPECT_RESULT + 2));
        recognitionResultItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, recognitionResultItem);

        QTableWidgetItem* alarmLevelItem = new QTableWidgetItem(m_tableDataList[i].at(FIELDS_ALARM_LEVEL_NAME + 2));
        alarmLevelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, alarmLevelItem);

        QTableWidgetItem* recognitionTimeItem = new QTableWidgetItem(m_tableDataList[i].at(FIELDS_INSPECT_TIME + 2));
        recognitionTimeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 6, recognitionTimeItem);

        QTableWidgetItem* collectInfoItem = new QTableWidgetItem(m_tableDataList[i].at(FIELDS_COLLECT_MESSAGE + 2));
        collectInfoItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 7, collectInfoItem);

        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelGenerateExcel::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initTopWidget();
	initDeviceTreeWidget();
	initTableWidget();
    initTableData(m_currentSearchCondition);

    m_fieldChooseWindow = new FieldChooseWindow(this);
    connect(m_fieldChooseWindow, &FieldChooseWindow::signalFieldChoosed, this, &DLWheelGenerateExcel::onExportButtonClicked);

	QHBoxLayout* hBottomLayout = new QHBoxLayout;
	hBottomLayout->addWidget(m_deviceTreeWidget);
	hBottomLayout->addWidget(m_customTableWidget);
	hBottomLayout->setSpacing(0);
	hBottomLayout->setMargin(0);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_topBackWidget);
	vMainLayout->addLayout(hBottomLayout);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(0);
}

void DLWheelGenerateExcel::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), QColor(240, 240, 240));
}

void DLWheelGenerateExcel::onButtonClicked(int buttonId)
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
        m_fieldChooseWindow->onResetCheckBox();
        m_fieldChooseWindow->show();
    }
    break;
    default:
        break;
    }
}

void DLWheelGenerateExcel::onSearchButtonClicked()
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

    m_currentPageIndex = 1;
    initTableData(m_currentSearchCondition);
}

void DLWheelGenerateExcel::onResetButtonClicked()
{
    for (int i = 0; i < m_checkBoxWidgetList.count(); i++)
    {
        m_checkBoxWidgetList[i]->resetCheckBox();
    }
}

void DLWheelGenerateExcel::onExportButtonClicked()
{
    QList<int> checkBoxIdList = m_fieldChooseWindow->getChooseIdList();
    QStringList strHeadList = m_fieldChooseWindow->getChoosedNameList();

    QList<QStringList> excelData;
    excelData.append(strHeadList);

    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
    for (int i = 0; i < m_tableDataList.count(); i++)
    {
        QStringList strDataList;
        if (tableWidget->item(i, 0)->checkState() == Qt::Checked)
        {
            for (int j = 0; j < checkBoxIdList.count(); j++)
            {
                strDataList.append(m_tableDataList[i].at(checkBoxIdList[j] + 2));
            }

            excelData.append(strDataList);
        }
    }

    if (excelData.count() == 1)
    {
        DLMessageBox::showDLMessageBox(NULL, "提示", "未选择导出项", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        return;
    }

    QString dir = QFileDialog::getSaveFileName(NULL, ("Save File"), "", ("excel(*.xlsx)"));
    if (dir.isEmpty())
    {
        return;
    }

    CreateExcelForQt5 excel;
    excel.import_excel_report(excelData, dir);
}