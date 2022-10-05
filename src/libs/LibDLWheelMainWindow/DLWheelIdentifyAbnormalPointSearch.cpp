#include "DLWheelIdentifyAbnormalPointSearch.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include <QHeaderView>
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"
#include <QFileDialog>
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelCustomWidget/DefData.h"

#pragma execution_character_set("utf-8")
#define TABLE_PER_PAGE_COUNT 20

DLWheelIdentifyAbnormalPointSearch::DLWheelIdentifyAbnormalPointSearch(QWidget* parent /* = NULL */)
	: QWidget(parent)
	, m_isInitWidget(false)
    , m_currentPageIndex(1)
{
	this->setStyleSheet("QWidget#CheckBoxBackWidget{background:rgb(240, 240, 240);}\
							QToolButton{border:none;}");
}

void DLWheelIdentifyAbnormalPointSearch::initCheckBoxWidget()
{
	m_checkBoxBackWidget = new QWidget;
	m_checkBoxBackWidget->setFixedHeight(30);
	m_checkBoxBackWidget->setObjectName("CheckBoxBackWidget");

	m_checkBoxGroup = new QButtonGroup(this);
    m_checkBoxGroup->setExclusive(false);
	QLabel* titleLabel = new QLabel("识别状态:");

	QCheckBox* checkBoxIdentifyNormal = new QCheckBox("识别正常");
	m_checkBoxGroup->addButton(checkBoxIdentifyNormal, IdentifyNormal);

	QCheckBox* checkBoxIdentifyAbnormal = new QCheckBox("识别异常");
	m_checkBoxGroup->addButton(checkBoxIdentifyAbnormal, IdentifyAbnormal);

	QCheckBox* checkBoxArtificialIdentify = new QCheckBox("人工识别");
	m_checkBoxGroup->addButton(checkBoxArtificialIdentify, ArtificialIdentify);

	QHBoxLayout* hCheckBoxLayout = new QHBoxLayout(m_checkBoxBackWidget);
	hCheckBoxLayout->addWidget(titleLabel);
	hCheckBoxLayout->addWidget(checkBoxIdentifyNormal);
	hCheckBoxLayout->addWidget(checkBoxIdentifyAbnormal);
	hCheckBoxLayout->addWidget(checkBoxArtificialIdentify);
	hCheckBoxLayout->addStretch();
	hCheckBoxLayout->setSpacing(50);
	hCheckBoxLayout->setContentsMargins(5, 0, 0, 0);
}

void DLWheelIdentifyAbnormalPointSearch::initButtonBackWidget()
{
	m_customButtonListWidget = new CustomButtonListWidget;
	m_customButtonListWidget->addToolButton(0, "查询", ":/Resources/Common/image/Search.png");
	m_customButtonListWidget->addToolButton(1, "重置", ":/Resources/Common/image/Reset.png");
	m_customButtonListWidget->addToolButton(2, "导出", ":/Resources/Common/image/ExportButton.png");
	m_customButtonListWidget->addWidgetFinished();

	connect(m_customButtonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelIdentifyAbnormalPointSearch::onButtonClicked);
}

void DLWheelIdentifyAbnormalPointSearch::initDeviceTreeWidget()
{
	m_deviceTreeWidget = new CustomTreeWidget;
	m_deviceTreeWidget->setTreeWidgetType(TreeItemWidgetType::ColorRect_CheckBox_Without);
	m_deviceTreeWidget->refreshTreeItemList();
}

void DLWheelIdentifyAbnormalPointSearch::initTableWidget()
{
	m_tableBackWidget = new QWidget;

    m_pointPosTableWidget = new CustomTableWidget(6);
    m_pointPosTableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "点位名称" << "识别类型" << "原始结果" << "审核结果" << "采集信息");
    connect(m_pointPosTableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableData(m_conditionList);
    });
    connect(m_pointPosTableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableData(m_conditionList);
    });

	QVBoxLayout* vTableLayout = new QVBoxLayout(m_tableBackWidget);
	vTableLayout->addWidget(m_pointPosTableWidget);
	vTableLayout->setSpacing(0);
	vTableLayout->setMargin(0);
}

void DLWheelIdentifyAbnormalPointSearch::initTableData(QStringList conditionList)
{
    int totalPage, totalCount;
    WHEEL_PATROL_RESULT.getWheelUnusualPointSearchCount(totalPage, totalCount, TABLE_PER_PAGE_COUNT, conditionList);
    m_pointPosTableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

    QTableWidget* tableWidget = m_pointPosTableWidget->getTableWidget();
    QList<DeviceAlarmSearchStruct> tableDataList = WHEEL_PATROL_RESULT.getWheelUnusualPointSearchList(m_currentPageIndex, TABLE_PER_PAGE_COUNT, conditionList);
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    for (int i = 0; i < tableDataList.count(); i++)
    {
        tableWidget->insertRow(i);
        QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
        checkBoxItem->setCheckState(Qt::Unchecked);
        checkBoxItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, checkBoxItem);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 1, idItem);

        QTableWidgetItem* taskNameItem = new QTableWidgetItem(tableDataList[i].device_point_type_name);
        tableWidget->setItem(i, 2, taskNameItem);
        taskNameItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* taskDateItem = new QTableWidgetItem(tableDataList[i].recognition_type_name);
        taskDateItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, taskDateItem);

        QTableWidgetItem* inspectItem = new QTableWidgetItem(tableDataList[i].inspect_result);
        inspectItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, inspectItem);
       
        QTableWidgetItem* checkItem = new QTableWidgetItem(tableDataList[i].deal_result);
        checkItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, checkItem);

        QTableWidgetItem* save_type_name = new QTableWidgetItem(tableDataList[i].save_type_name);
        save_type_name->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 6, save_type_name);

        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelIdentifyAbnormalPointSearch::searchTable()
{
    m_conditionList.clear();
    for (int i = 0; i < m_checkBoxGroup->buttons().count(); i++)
    {
        if (m_checkBoxGroup->button(i)->isChecked())
        {
            m_conditionList.append(QString::number(i + 1));
        }
    }
    initTableData(m_conditionList);
}

void DLWheelIdentifyAbnormalPointSearch::resetCondition()
{
    for (int i = 0; i < m_checkBoxGroup->buttons().count(); i++)
    {
        m_checkBoxGroup->button(i)->setChecked(false);
    }
}

void DLWheelIdentifyAbnormalPointSearch::exportTable()
{
    QList<QStringList> excelData;
    excelData.append(QStringList() << "点位名称" << "识别类型" << "原始结果" << "审核结果" << "采集信息" << "采集信息");

    QTableWidget* tableWidget = m_pointPosTableWidget->getTableWidget();
    for (int i = 0 ; i < tableWidget->rowCount(); i++)
    {
        QStringList strDataList;
        if (tableWidget->item(i, 0)->checkState() == Qt::Checked)
        {
            for (int j = 2; j < tableWidget->columnCount() - 1; j++)
            {
                strDataList.append(tableWidget->item(i, j)->text());
            }
            
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
    excel.CreateNewExcelForList(excelData, dir, EXCEL_RECOGNITION_UNUSUAL_POINT_SEARCH);
}

void DLWheelIdentifyAbnormalPointSearch::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initCheckBoxWidget();
	initButtonBackWidget();
	initDeviceTreeWidget();
	initTableWidget();
    initTableData(m_conditionList);

	QHBoxLayout* hCenterLayout = new QHBoxLayout;
	hCenterLayout->addWidget(m_deviceTreeWidget);
	hCenterLayout->addWidget(m_tableBackWidget);
	hCenterLayout->setSpacing(5);
	hCenterLayout->setMargin(0);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_checkBoxBackWidget);
	vMainLayout->addWidget(m_customButtonListWidget);
	vMainLayout->addLayout(hCenterLayout);
	vMainLayout->setSpacing(5);
	vMainLayout->setMargin(3);
}

void DLWheelIdentifyAbnormalPointSearch::onButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
        searchTable();
        break;
    case 1:
        resetCondition();
        break;
    case 2:
        exportTable();
        break;
    default:
        break;
    }
}