#include "DLWheelAlarmMessageSubscription.h"
#include <QHBoxLayout>
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "DLWheelMessageSubscriptionAdd.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPointTreeData.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"

#pragma execution_character_set("utf-8")

#define TITLE_WIDGET_HEIGHT 150
#define TABLE_PER_PAGE_COUNT 20

DLWheelAlarmMessageSubscription::DLWheelAlarmMessageSubscription()
	: m_isInitWidget(false)
	, m_messageSubscriptionAdd(NULL)
    , m_currentPageIndex(1)
    , m_isDeviceSearch(true)
{
	this->setStyleSheet("QWidget#TopBackWidget{background:rgb(240, 240, 240);}");

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotInsertNoteMessageStatus.connect(boost::bind(&DLWheelAlarmMessageSubscription::signalInsertDataCallBack, this, _1, _2));

    // 插入数据回调;
    connect(this, &DLWheelAlarmMessageSubscription::signalInsertDataCallBack, this, [=](bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "添加成功", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            m_messageSubscriptionAdd->closeWidget();
            m_wheelAlarmMessageIf = WheelAlarmMessageIf();
            initTableData();
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "错误", "添加失败:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });
}

DLWheelAlarmMessageSubscription::~DLWheelAlarmMessageSubscription()
{

}

void DLWheelAlarmMessageSubscription::initTopWidget()
{
    int level;
    DLWheelPointTreeData wheelPointTreeData;
    wheelPointTreeData.getPointTreeFirstRootNode(m_strCompanyName, level);
    wheelPointTreeData.getPointTreeSecondRootNode(m_strStationName, level);

	m_unitWidget = new InputWidget(InputWidgetType::WheelComboBox);
	m_unitWidget->setTipText("单位");
	m_unitWidget->setTipLabelWidth(85);
	m_unitWidget->setFixedWidth(210);
    m_unitWidget->setComboBoxContent(QStringList() << m_strCompanyName);

	m_transformerSubstationWidget = new InputWidget(InputWidgetType::WheelComboBox);
	m_transformerSubstationWidget->setTipText("变电站");
	m_transformerSubstationWidget->setTipLabelWidth(85);
	m_transformerSubstationWidget->setFixedWidth(210);
    m_transformerSubstationWidget->setComboBoxContent(QStringList() << m_strStationName);

	m_intervalVoltageWidget = new InputWidget(InputWidgetType::WheelComboBox);
	m_intervalVoltageWidget->setTipText("间隔电压");
	m_intervalVoltageWidget->setTipLabelWidth(85);
	m_intervalVoltageWidget->setFixedWidth(210);
    m_intervalVoltageWidget->setComboBoxContent(WHEEL_DEVICE_CONFIG.getWheelVoltageLevelNameQList());
    
    std::map<QString, WheelRobortEquipmentIntervalStruct> intervalNameMap = WHEEL_DEVICE_CONFIG.getWheelRobortEquipmentIntervalDataMap();
    QStringList intervalNameList;
    std::map<QString, WheelRobortEquipmentIntervalStruct>::iterator intervalNameIter;
    for (intervalNameIter = intervalNameMap.begin(); intervalNameIter != intervalNameMap.end(); intervalNameIter++)
    {
        intervalNameList.append(intervalNameIter->second.equipment_interval_name);
    }
	m_intervalWidget = new InputWidget(InputWidgetType::WheelComboBox);
	m_intervalWidget->setTipText("间隔");
	m_intervalWidget->setTipLabelWidth(85);
	m_intervalWidget->setFixedWidth(210);
    m_intervalWidget->setComboBoxContent(intervalNameList);

    m_deviceTypeWidget = new InputWidget(InputWidgetType::WheelComboBox);
    m_deviceTypeWidget->setTipText("设备类型");
    m_deviceTypeWidget->setTipLabelWidth(85);
    m_deviceTypeWidget->setFixedWidth(210);
    m_deviceTypeWidget->setComboBoxContent(QStringList() << "设备" << "告警");

	m_pointPosNameWidget = new InputWidget(InputWidgetType::WheelLineEdit);
	m_pointPosNameWidget->setTipText("点位名称");

	m_receiverAccountWidget = new InputWidget(InputWidgetType::WheelLineEdit);
	m_receiverAccountWidget->setTipText("接收人账户");

	QHBoxLayout* hComboBoxLayout = new QHBoxLayout;
	hComboBoxLayout->addWidget(m_unitWidget);
	hComboBoxLayout->addWidget(m_transformerSubstationWidget);
	hComboBoxLayout->addWidget(m_intervalVoltageWidget);
	hComboBoxLayout->addWidget(m_intervalWidget);
    hComboBoxLayout->addWidget(m_deviceTypeWidget);
	hComboBoxLayout->addStretch();
	hComboBoxLayout->setSpacing(100);
	hComboBoxLayout->setMargin(0);

	QHBoxLayout* hLineEditLayout = new QHBoxLayout;
	hLineEditLayout->addWidget(m_pointPosNameWidget);
	hLineEditLayout->addWidget(m_receiverAccountWidget);
	hLineEditLayout->addStretch();
	hLineEditLayout->setSpacing(100);
	hLineEditLayout->setMargin(0);

	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("TopBackWidget");
	m_topBackWidget->setFixedHeight(100);
	QVBoxLayout* vTopLayout = new QVBoxLayout(m_topBackWidget);
	vTopLayout->addLayout(hComboBoxLayout);
	vTopLayout->addLayout(hLineEditLayout);
	vTopLayout->setSpacing(20);
	vTopLayout->setContentsMargins(15, 10, 0, 20);
}

void DLWheelAlarmMessageSubscription::initButtonListWidget()
{
	m_buttonListWidget = new CustomButtonListWidget;
	m_buttonListWidget->addToolButton(0, "查询", ":/Resources/Common/image/Search.png");
	m_buttonListWidget->addToolButton(1, "添加", ":/Resources/Common/image/Button_Add.png");
	m_buttonListWidget->addToolButton(2, "删除", ":/Resources/Common/image/Button_Delete.png");
	m_buttonListWidget->addToolButton(3, "重置", ":/Resources/Common/image/Reset.png");
	m_buttonListWidget->addWidgetFinished();

	connect(m_buttonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelAlarmMessageSubscription::onButtonClicked);
}

void DLWheelAlarmMessageSubscription::initTableWidget()
{
    m_tableWidget = new CustomTableWidget(9);
    m_tableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "单位" << "变电站" << "间隔电压" << "间隔名称" << "点位名称" << "短信接收人账号" << "发送频率(天)" << "告警通知");
    connect(m_tableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableData();
    });
    connect(m_tableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableData();
    });
}

void DLWheelAlarmMessageSubscription::initTableData()
{
    int totalPage, totalCount;

    if (m_isDeviceSearch)
    {
        WHEEL_DEVICE_CONFIG.getAlarmMessageSubscribeCount(totalPage, totalCount, TABLE_PER_PAGE_COUNT, m_wheelAlarmMessageIf);
        m_tableDataList = WHEEL_DEVICE_CONFIG.getAlarmMessageSubscribe(m_currentPageIndex, TABLE_PER_PAGE_COUNT, m_wheelAlarmMessageIf);
        m_tableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);
    }
    else
    {
        WHEEL_DEVICE_CONFIG.getSystemMessageSubscribeCount(totalPage, totalCount, TABLE_PER_PAGE_COUNT, m_wheelAlarmMessageIf);
        m_tableDataList = WHEEL_DEVICE_CONFIG.getSystemMessageSubscribe(m_currentPageIndex, TABLE_PER_PAGE_COUNT, m_wheelAlarmMessageIf);
        m_tableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);
    }

    QTableWidget* tableWidget = m_tableWidget->getTableWidget();
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

        QTableWidgetItem* companyNameItem = new QTableWidgetItem(m_strCompanyName);
        tableWidget->setItem(i, 2, companyNameItem);
        companyNameItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* stationItem = new QTableWidgetItem(m_strStationName);
        stationItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, stationItem);

        QTableWidgetItem* voltageLevelItem = new QTableWidgetItem(m_tableDataList[i].voltage_level_name);
        voltageLevelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, voltageLevelItem);
        
        QTableWidgetItem* intervalItem = new QTableWidgetItem(m_tableDataList[i].equipment_interval_name);
        intervalItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, intervalItem);

        if (!m_isDeviceSearch)
        {
            voltageLevelItem->setText("-");
            intervalItem->setText("-");
        }

        QTableWidgetItem* pointNameItem = new QTableWidgetItem(m_tableDataList[i].fault_name);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 6, pointNameItem);

        QTableWidgetItem* receiverNameItem = new QTableWidgetItem(m_tableDataList[i].revice_user_name);
        receiverNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 7, receiverNameItem);

        QTableWidgetItem* sendFrqItem = new QTableWidgetItem(m_tableDataList[i].send_freq);
        sendFrqItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 8, sendFrqItem);

        QTableWidgetItem* alarmNotifyItem = new QTableWidgetItem(m_tableDataList[i].alarm_type);
        alarmNotifyItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 9, alarmNotifyItem);

        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelAlarmMessageSubscription::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initTopWidget();
	initButtonListWidget();
	initTableWidget();
    initTableData();

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_topBackWidget);
	vMainLayout->addWidget(m_buttonListWidget);
	vMainLayout->addWidget(m_tableWidget);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(0);
}

void DLWheelAlarmMessageSubscription::onButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
        onSearchButtonClicked();
        break;
    case 1:
        m_messageSubscriptionAdd = new DLWheelMessageSubscriptionAdd;
        m_messageSubscriptionAdd->show();
        break;
    case 2:
        onDeleteButtonClicked();
        break;
    case 3:
        onResetButtonClicked();
        break;
    default:
        break;
    }
}

void DLWheelAlarmMessageSubscription::onSearchButtonClicked()
{
    if (m_deviceTypeWidget->getComboBoxCurrentIndex() == 0)
    {
        m_isDeviceSearch = true;
    }
    else
    {
        m_isDeviceSearch = false;
    }

    m_wheelAlarmMessageIf.equipment_interval_uuid = WHEEL_DEVICE_CONFIG.getWheelEquipmentIntervalQString(m_intervalWidget->getComboBoxCurrentContent());
    m_wheelAlarmMessageIf.voltage_level_id = WHEEL_DEVICE_CONFIG.getWheelVoltageLevelIdInt(m_intervalVoltageWidget->getComboBoxCurrentContent());
    m_wheelAlarmMessageIf.revice_user_name = m_receiverAccountWidget->getLineEditText();
    m_wheelAlarmMessageIf.device_point_name = m_pointPosNameWidget->getLineEditText();
    initTableData();
}

void DLWheelAlarmMessageSubscription::onDeleteButtonClicked()
{
    QStringList strNodeIdList, strFaultNameList;
    QTableWidget* tableWidget = m_tableWidget->getTableWidget();
    for (int i = 0; i < tableWidget->rowCount(); i++)
    {
        QTableWidgetItem* checkItem = tableWidget->item(i, 0);
        if (checkItem->checkState() == Qt::Checked)
        {
            strNodeIdList.append(m_tableDataList[i].note_uuid);
            strFaultNameList.append(m_tableDataList[i].fault_name);
        }
    }

    // 将选择的item进行删除;
}

void DLWheelAlarmMessageSubscription::onResetButtonClicked()
{
    m_intervalVoltageWidget->setComboBoxCurrentIndex(0);
    m_intervalWidget->setComboBoxCurrentIndex(0);
    m_receiverAccountWidget->setLineEditText("");
    m_pointPosNameWidget->setLineEditText("");
}
