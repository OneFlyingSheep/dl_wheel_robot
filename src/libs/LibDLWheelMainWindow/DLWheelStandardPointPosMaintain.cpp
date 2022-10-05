#include "DLWheelStandardPointPosMaintain.h"
#include <QHBoxLayout>
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelTaskAdministrationData.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include <QScrollBar>
#include <QFileDialog>
#include <fstream>

DLWheelStandardPointPosMaintain::DLWheelStandardPointPosMaintain()
	: m_isInitWidget(false)
    , m_currentTableRow(0)
    , m_sendPointPosLibDataIndex(0)
    , m_isStopImportExcelData(false)
{
	this->setStyleSheet("QWidget#TopBackWidget{background:rgb(240, 240, 240);}");

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotDeleteStandardStatus.connect(boost::bind(&DLWheelStandardPointPosMaintain::signalDeleteCallBack, this, _1, _2, _3));

    // 删除数据回调;
    connect(this, &DLWheelStandardPointPosMaintain::signalDeleteCallBack, this, [=](QString strUuid, bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            initTableData();
        }
        else
        {
            // 如果删除失败先恢复删除按钮;
            resetDeleteButton(strUuid);
            DLMessageBox::showDLMessageBox(NULL, "错误", "删除失败:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        WHEEL_DEVICE_CONFIG.refreshData();
    });

    // excel数据导入回调;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotExcelImportStatus.connect(boost::bind(&DLWheelStandardPointPosMaintain::signalExcelImportCallBack, this, _1, _2));

    connect(this, &DLWheelStandardPointPosMaintain::signalExcelImportCallBack, this, [=](bool isSuccess, QString strMsg) {
        m_sendPointPosLibDataIndex++;
        m_progressBarWidget->setCurrentProgress(m_sendPointPosLibDataIndex);
        if (!isSuccess)
        {
            m_progressBarWidget->addErrorMsg(strMsg);
        }
        m_standardPointPosLibDataList.pop_front();
        if (m_standardPointPosLibDataList.isEmpty())
        {
            m_progressBarWidget->hide();
            DLMessageBox::showDLMessageBox(NULL, "提示", "导入完成:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            initTableData();
            return;
        }

        if (!m_isStopImportExcelData)
        {
            onStandardPointPosLibImport(m_standardPointPosLibDataList.first());
        }
    });
}

void DLWheelStandardPointPosMaintain::initCheckBoxWidget()
{
    m_checkBoxBackWidget = new QWidget;

    QList<WheelTaskAdminCheckBoxStruct> checkBoxDataList = WHEEL_TASK_ADMINISTRATION.getTaskAdminCheckBoxData(WHEEL_STANDARD_PATROL_VINDICATE);
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

void DLWheelStandardPointPosMaintain::initButtonListWidget()
{
	m_buttonListWidget = new CustomButtonListWidget;
	m_buttonListWidget->addToolButton(0, "查询", ":/Resources/Common/image/Search.png");
	m_buttonListWidget->addToolButton(1, "添加", ":/Resources/Common/image/Button_Add.png");
	m_buttonListWidget->addToolButton(2, "重置", ":/Resources/Common/image/Reset.png");
	m_buttonListWidget->addToolButton(3, "导出", ":/Resources/Common/image/ExportButton.png");
	m_buttonListWidget->addToolButton(4, "库导入", ":/Resources/Common/image/ExportButton.png", QSize(120, 20));
	m_buttonListWidget->addToolButton(5, "文件导入", ":/Resources/Common/image/file_import.png");
	m_buttonListWidget->addWidgetFinished();

	connect(m_buttonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelStandardPointPosMaintain::onButtonClicked);
}

void DLWheelStandardPointPosMaintain::initTopWidget()
{
	initCheckBoxWidget();
	
	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("TopBackWidget");
	QVBoxLayout* vTopLayout = new QVBoxLayout(m_topBackWidget);
	vTopLayout->addWidget(m_checkBoxBackWidget);
	vTopLayout->setContentsMargins(15, 10, 0, 20);
}

void DLWheelStandardPointPosMaintain::initTableWidget()
{
    m_customTableWidget = new CustomTableWidget(9);
    m_customTableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "设备类型" << "小类设备" << "点位名称" << "识别类型" << "表计类型" << "发热类型" << "保存类型" << "操作");
    m_customTableWidget->setIsShowTurnPageWidget(false);
}

void DLWheelStandardPointPosMaintain::initTableData()
{
    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();

    m_tableDataList = WHEEL_DEVICE_CONFIG.getWheelStandardPatrolVindicate(m_device_type_uuid);
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

        QTableWidgetItem* deviceTypeItem = new QTableWidgetItem(m_tableDataList[i].m_device_type_name);
        tableWidget->setItem(i, 2, deviceTypeItem);
        deviceTypeItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* subDeviceItem = new QTableWidgetItem(m_tableDataList[i].m_sub_device_name);
        subDeviceItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, subDeviceItem);

        QTableWidgetItem* pointNameItem = new QTableWidgetItem(m_tableDataList[i].m_device_point_name);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, pointNameItem);

        QTableWidgetItem* recognitionTypeItem = new QTableWidgetItem(m_tableDataList[i].m_recognition_type_name);
        recognitionTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, recognitionTypeItem);

        QTableWidgetItem* meterTypeItem = new QTableWidgetItem(m_tableDataList[i].m_meter_type_name);
        meterTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 6, meterTypeItem);

        QTableWidgetItem* feverTypeItem = new QTableWidgetItem(m_tableDataList[i].m_fever_type_name);
        feverTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 7, feverTypeItem);

        QTableWidgetItem* saveTypeItem = new QTableWidgetItem(m_tableDataList[i].m_save_type_name);
        saveTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 8, saveTypeItem);

        OperateButtonWidget* buttonWidget = new OperateButtonWidget(tableWidget);
        buttonWidget->setButtonRow(i);
        connect(buttonWidget, &OperateButtonWidget::signalModifyButtonClicked, this, &DLWheelStandardPointPosMaintain::onModifyButtonClicked);
        connect(buttonWidget, &OperateButtonWidget::signalDeleteButtonClicked, this, &DLWheelStandardPointPosMaintain::onDeleteButtonClicked);

        QTableWidgetItem* operateItem = new QTableWidgetItem();
        operateItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 9, operateItem);
        tableWidget->setCellWidget(i, 9, buttonWidget);

        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelStandardPointPosMaintain::resetDeleteButton(QString strUuid)
{
    for (int i = 0; i < m_tableDataList.count(); i++)
    {
        if (m_tableDataList[i].m_device_point_uuid == strUuid)
        {
            QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
            OperateButtonWidget* buttonWidget = static_cast<OperateButtonWidget*>(tableWidget->cellWidget(i, 9));
            if (buttonWidget != NULL)
            {
                buttonWidget->resetDeleteButton();
            }
        }
    }
}

void DLWheelStandardPointPosMaintain::exportTable()
{
    QList<QStringList> excelData;
    excelData.append(QStringList() << "设备类型" << "小类设备" << "点位名称" << "识别类型" << "表计类型" << "发热类型" << "保存类型");

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
    excel.CreateNewExcelForList(excelData, dir, EXCEL_STANDARD_POINT);
}

void DLWheelStandardPointPosMaintain::initWidget()
{
	if (m_isInitWidget)
	{
        initTableData();
		return;
	}
	m_isInitWidget = true;

	initTopWidget();
	initButtonListWidget();
	initTableWidget();
    initTableData();

    m_addPointPosWidget = new AddPointPosWidget(this);
    connect(m_addPointPosWidget, &AddPointPosWidget::signalSaveDataSuccess, this, &DLWheelStandardPointPosMaintain::onRefreshTable);

    m_sheetChooseWidget = new SheetChooseWidget(this);
    connect(m_sheetChooseWidget, &SheetChooseWidget::signalChooseSheetName, this, [=](QString sheetName) {
        m_sendPointPosLibDataIndex = 0;
        m_standardPointPosLibDataList = m_searchRecordCreateExcel.chooseSheetName(sheetName);
        if (m_standardPointPosLibDataList.isEmpty())
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "当前Sheet数据为空", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        else
        {
            m_sendPointPosLibDataIndex = 0;
            m_progressBarWidget->resetProgress();
            m_progressBarWidget->setCurrentProgress(1);
            m_progressBarWidget->setTotelSize(m_standardPointPosLibDataList.count());
            // 开始发送数据;
            onStandardPointPosLibImport(m_standardPointPosLibDataList.first());

            m_progressBarWidget->show();
        }
    });

    m_progressBarWidget = new ProgressBarWidget(this);
    connect(m_progressBarWidget, &ProgressBarWidget::signalStopImportExcelData, this, [=] {
        m_isStopImportExcelData = true;
    });

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_topBackWidget);
	vMainLayout->addWidget(m_buttonListWidget);
	vMainLayout->addWidget(m_customTableWidget);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(0);
}

void DLWheelStandardPointPosMaintain::standardPointPosLibImport()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Spreadsheet"), ".",
        tr("Spreadsheet files (*.xlsx *xls)"));

    if (!fileName.isNull())
    {
        QStringList sheetName = m_searchRecordCreateExcel.LoadExcelAndGetData(fileName);
        if (sheetName.size() == 0)
            return;
        m_sheetChooseWidget->setSheetListData(sheetName);
        m_sheetChooseWidget->show();
    }
}

void DLWheelStandardPointPosMaintain::onButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
    {
        // 查询;
        if (!m_checkBoxWidgetList.isEmpty())
        {
            m_device_type_uuid = m_checkBoxWidgetList.first()->getCheckedIdList();
            initTableData();
        }
    }
        break;
    case 1:
    {
        // 添加;
        m_addPointPosWidget->initData();
        m_addPointPosWidget->show();
    }
        break;
    case 2:
    {
        // 重置;
        if (!m_checkBoxWidgetList.isEmpty())
        {
            m_checkBoxWidgetList.first()->resetCheckBox();
        }
    }
        break;
    case 3:
    {
        // 导出;
        exportTable();
    }
        break;
    case 4:
    {
        // 标准点位库导入;
        standardPointPosLibImport();
    }
        break;

	case 5:
	{
		// 文件导入;
		file_import_patrol_list();
		break;
	}

    default:
        break;
    }
}

void DLWheelStandardPointPosMaintain::onModifyButtonClicked(int buttonRow)
{
    m_currentTableRow = buttonRow;
    m_addPointPosWidget->setCurrentData(m_tableDataList[buttonRow]);
    m_addPointPosWidget->show();
}

void DLWheelStandardPointPosMaintain::onDeleteButtonClicked(int buttonRow)
{
    WHEEL_BACK_TO_CORE_SOCKET.robot_delete_standard_patrol_vindicate_req(m_tableDataList[buttonRow].m_device_point_uuid);
}

void DLWheelStandardPointPosMaintain::onRefreshTable(bool isModify, WheelStandardPatrolVindicateStruct data)
{
    if (isModify)
    {
        m_tableDataList[m_currentTableRow] = data;
        QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
        tableWidget->item(m_currentTableRow, 2)->setText(m_tableDataList[m_currentTableRow].m_device_type_name);
        tableWidget->item(m_currentTableRow, 3)->setText(m_tableDataList[m_currentTableRow].m_sub_device_name);
        tableWidget->item(m_currentTableRow, 4)->setText(m_tableDataList[m_currentTableRow].m_device_point_name);
        tableWidget->item(m_currentTableRow, 5)->setText(m_tableDataList[m_currentTableRow].m_recognition_type_name);
        tableWidget->item(m_currentTableRow, 6)->setText(m_tableDataList[m_currentTableRow].m_meter_type_name);
        tableWidget->item(m_currentTableRow, 7)->setText(m_tableDataList[m_currentTableRow].m_fever_type_name);
        tableWidget->item(m_currentTableRow, 8)->setText(m_tableDataList[m_currentTableRow].m_save_type_name);

    }
    else
    {
        m_tableDataList.append(data);
        QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
        int rowCount = tableWidget->rowCount();
        tableWidget->insertRow(rowCount);
        QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
        checkBoxItem->setCheckState(Qt::Unchecked);
        checkBoxItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 0, checkBoxItem);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(rowCount + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 1, idItem);
        idItem->setBackground(QBrush(QColor(255, 0, 0)));

        QTableWidgetItem* deviceTypeItem = new QTableWidgetItem(m_tableDataList[rowCount].m_device_type_name);
        tableWidget->setItem(rowCount, 2, deviceTypeItem);
        deviceTypeItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* subDeviceItem = new QTableWidgetItem(m_tableDataList[rowCount].m_sub_device_name);
        subDeviceItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 3, subDeviceItem);

        QTableWidgetItem* pointNameItem = new QTableWidgetItem(m_tableDataList[rowCount].m_device_point_name);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 4, pointNameItem);

        QTableWidgetItem* recognitionTypeItem = new QTableWidgetItem(m_tableDataList[rowCount].m_recognition_type_name);
        recognitionTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 5, recognitionTypeItem);

        QTableWidgetItem* meterTypeItem = new QTableWidgetItem(m_tableDataList[rowCount].m_meter_type_name);
        meterTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 6, meterTypeItem);

        QTableWidgetItem* feverTypeItem = new QTableWidgetItem(m_tableDataList[rowCount].m_fever_type_name);
        feverTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 7, feverTypeItem);

        QTableWidgetItem* saveTypeItem = new QTableWidgetItem(m_tableDataList[rowCount].m_save_type_name);
        saveTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 8, saveTypeItem);

        OperateButtonWidget* buttonWidget = new OperateButtonWidget(tableWidget);
        buttonWidget->setButtonRow(rowCount);
        connect(buttonWidget, &OperateButtonWidget::signalModifyButtonClicked, this, &DLWheelStandardPointPosMaintain::onModifyButtonClicked);
        connect(buttonWidget, &OperateButtonWidget::signalDeleteButtonClicked, this, &DLWheelStandardPointPosMaintain::onDeleteButtonClicked);

        QTableWidgetItem* operateItem = new QTableWidgetItem();
        operateItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowCount, 9, operateItem);
        tableWidget->setCellWidget(rowCount, 9, buttonWidget);

        tableWidget->setRowHeight(rowCount, 40);

        tableWidget->verticalScrollBar()->setMaximum(tableWidget->verticalScrollBar()->maximum() + 1);
        QScrollBar* ScrollBarLocation = tableWidget->verticalScrollBar();
        ScrollBarLocation->setValue(ScrollBarLocation->maximum());
    }
}

void DLWheelStandardPointPosMaintain::onStandardPointPosLibImport(QStringList excelData)
{
    WHEEL_BACK_TO_CORE_SOCKET.robot_excel_import_req(excelData);
}

void DLWheelStandardPointPosMaintain::file_import_patrol_list( )
{
	// 1.获取批量导入文件
	QString fileName = QFileDialog::getOpenFileName(this, tr("Choose File"), "", tr("txt(*.txt)"));
	if (fileName.isNull())
	{
		return;
	}

	// 2.解析文件 & 插入数据
	std::ifstream file;

	file.open(fileName.toStdString().c_str(), std::ios_base::in);
	if (!file.is_open())
	{
		std::cout << "打开文件失败";
		return;
	}

	int device_type_id = 0, sub_device_type_id = 0;
	QString device_point_name = " ";
	int recognition_type = 0, meter_type = 0;
	QString device_sn = " ";
	int save_type = 0;

	std::string file_string, temp_string;
	int blank_index = 0;

	while (getline(file, file_string))
	{
		std::cout << file_string << endl;

		temp_string.clear();
		temp_string = file_string;

		//device_type_id
		blank_index = temp_string.find_first_of(',');
		device_type_id = atoi(temp_string.substr(0, blank_index).c_str());
		temp_string = temp_string.substr(blank_index + 1);

		//sub_device_type_id
		blank_index = temp_string.find_first_of(',');
		sub_device_type_id = atoi(temp_string.substr(0, blank_index).c_str());
		temp_string = temp_string.substr(blank_index + 1);

		//device_point_name
		blank_index = temp_string.find_first_of(',');
		device_point_name = QString().fromStdString(temp_string.substr(0, blank_index));
		temp_string = temp_string.substr(blank_index + 1);

		//recognition_type
		blank_index = temp_string.find_first_of(',');
		recognition_type = atoi(temp_string.substr(0, blank_index).c_str());
		temp_string = temp_string.substr(blank_index + 1);

		//meter_type
		blank_index = temp_string.find_first_of(',');
		meter_type = atoi(temp_string.substr(0, blank_index).c_str());
		temp_string = temp_string.substr(blank_index + 1);

		//device_sn 不需要处理

		//save_type
		blank_index = temp_string.find_first_of(',');
		save_type = atoi(temp_string.substr(0, blank_index).c_str());
		temp_string = temp_string.substr(blank_index + 1);

		m_addPointPosWidget->insertData(device_type_id, sub_device_type_id, device_point_name, recognition_type, meter_type, device_sn, save_type);
	}

	// 3.返回保存结果
	DLMessageBox::showDLMessageBox(NULL, "提示", "导入成功", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
	file.close();
}

