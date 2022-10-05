#include "DLWheelPowerStationEdit.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QHBoxLayout>
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"

#pragma execution_character_set("utf-8")

DLWheelPowerStationEdit::DLWheelPowerStationEdit(QWidget *parent /*= NULL*/)
{
	initWidget();

	this->setStyleSheet("QPushButton#CommonButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton#CommonButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton#CommonButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}\
						QPushButton#CommonButton:disabled{background:gray;}");		
}

DLWheelPowerStationEdit::~DLWheelPowerStationEdit()
{

}

void DLWheelPowerStationEdit::onRefreshListWidget()
{
    initListWidgetData();
}

void DLWheelPowerStationEdit::initWidget()
{
	m_voltageLevelWidget = new BaseWidget;
	m_voltageLevelWidget->setTitleContent("电压等级");
	m_voltageLevelWidget->setFixedWidth(250);

	m_voltageLevelListWidget = getListWidget(m_voltageLevelWidget, &m_pButtonAddValtageLevel, &m_pButtonDeleteValtageLevel, &m_lineEditValtageLevel);
	connect(m_voltageLevelListWidget, &QListWidget::itemClicked, this, &DLWheelPowerStationEdit::onVoltageLevelItemClicked);
    connect(m_pButtonAddValtageLevel, &QPushButton::clicked, this, [=] {
        if (m_lineEditValtageLevel->text().isEmpty())
        {
            return;
        }
        WHEEL_BACK_TO_CORE_SOCKET.robot_config_insert_voltage_level(m_lineEditValtageLevel->text());
        m_lineEditValtageLevel->clear();
    });

    connect(m_pButtonDeleteValtageLevel, &QPushButton::clicked, this, [=] {
        QListWidgetItem* currentItem = m_voltageLevelListWidget->currentItem();
        if (currentItem != NULL)
        {
            QString strText = currentItem->text();
            QString valtageId = m_voltageLevelMap.key(strText);
            WHEEL_BACK_TO_CORE_SOCKET.robot_config_delete_voltage_level(valtageId);
        }
    });


	m_intervalNameWidget = new BaseWidget;
	m_intervalNameWidget->setTitleContent("间隔名称");
	m_intervalNameWidget->setFixedWidth(250);

	m_intervalNameListWidget = getListWidget(m_intervalNameWidget, &m_pButtonAddIntervalName, &m_pButtonDeleteIntervalName, &m_lineEditIntervalName);

    connect(m_pButtonAddIntervalName, &QPushButton::clicked, this, [=] {
        if (m_lineEditIntervalName->text().isEmpty())
        {
            return;
        }
        QListWidgetItem* currentItem = m_voltageLevelListWidget->currentItem();
        if (currentItem != NULL)
        {
            QString strText = currentItem->text();
            QString valtageId = m_voltageLevelMap.key(strText);
            WHEEL_BACK_TO_CORE_SOCKET.robot_config_insert_interval(valtageId, m_lineEditIntervalName->text());
            m_lineEditIntervalName->clear();
        }
        else
        {
            DLMessageBox::showDLMessageBox(this, "提示", "请选择电压等级", MessageButtonType::BUTTON_OK, true);
        }
    });

    connect(m_pButtonDeleteIntervalName, &QPushButton::clicked, this, [=] {
        QListWidgetItem* currentItem = m_intervalNameListWidget->currentItem();
        if (currentItem != NULL)
        {
            QString strText = currentItem->text();
            QString valtageId = m_intervalNameMap.key(strText);
            WHEEL_BACK_TO_CORE_SOCKET.robot_config_delete_interval(valtageId);
        }
    });

	m_areaNameWidget = new BaseWidget;
	m_areaNameWidget->setTitleContent("区域名称");
	m_areaNameWidget->setFixedWidth(250);

	m_areaNameListWidget = getListWidget(m_areaNameWidget, &m_pButtonAddAreaName, &m_pButtonDeleteAreaName, &m_lineEditAreaName);

    connect(m_pButtonAddAreaName, &QPushButton::clicked, this, [=] {
        if (m_lineEditAreaName->text().isEmpty())
        {
            return;
        }
        WHEEL_BACK_TO_CORE_SOCKET.robot_config_insert_area(m_lineEditAreaName->text());
        m_lineEditAreaName->clear();
    });

    connect(m_pButtonDeleteAreaName, &QPushButton::clicked, this, [=] {
        QListWidgetItem* currentItem = m_areaNameListWidget->currentItem();
        if (currentItem != NULL)
        {
            QString strText = currentItem->text();
            QString valtageId = m_areaNameMap.key(strText);
            WHEEL_BACK_TO_CORE_SOCKET.robot_config_delete_area(valtageId);
        }
    });

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_voltageLevelWidget);
	hMainLayout->addWidget(m_intervalNameWidget);
	//hMainLayout->addWidget(m_areaNameWidget);
	hMainLayout->addStretch();
	hMainLayout->setSpacing(10);
	hMainLayout->setMargin(10);

	initListWidgetData();
}

void DLWheelPowerStationEdit::initListWidgetData()
{
	// 电压等级;
	m_voltageLevelMap.clear();
    WHEEL_DEVICE_CONFIG.loadWheelVoltageLevelData();
	std::map<QString, QString> voltageLevelMap = WHEEL_DEVICE_CONFIG.getWheelVoltageLevelDataMap();
	QStringList voltageLevelList;
	std::map<QString, QString>::iterator voltageLevelIter;
	for (voltageLevelIter = voltageLevelMap.begin(); voltageLevelIter != voltageLevelMap.end(); voltageLevelIter++)
	{
		m_voltageLevelMap.insert(voltageLevelIter->first, voltageLevelIter->second);
		voltageLevelList.append(voltageLevelIter->second);
	}
//	m_voltageLevel->setComboBoxContent(voltageLevelList);

    m_voltageLevelListWidget->clear();
	m_voltageLevelListWidget->addItems(voltageLevelList);
    m_voltageLevelListWidget->sortItems(Qt::DescendingOrder);
// 	if (voltageLevelList.count() > m_voltageLevelIndex)
// 	{
// 		m_voltageLevel->setComboBoxCurrentIndex(m_voltageLevelIndex);
// 	}
// 	else
// 	{
// 		m_voltageLevelIndex = 0;
// 	}

	// 间隔名称;
	m_intervalNameMap.clear();
	if (voltageLevelList.size() != 0)
	{
        WHEEL_DEVICE_CONFIG.loadWheelEquipmentIntervalData();
		std::map<QString, QString> intervalNameMap = WHEEL_DEVICE_CONFIG.getWheelRobortEquipmentIntervalFromVoltageLevelId(m_voltageLevelMap.key(m_voltageLevelListWidget->item(0)->text()));
		QStringList intervalNameList;
		std::map<QString, QString>::iterator intervalNameIter;
		for (intervalNameIter = intervalNameMap.begin(); intervalNameIter != intervalNameMap.end(); intervalNameIter++)
		{
			m_intervalNameMap.insert(intervalNameIter->first, intervalNameIter->second);
			intervalNameList.append(intervalNameIter->second);
		}
        m_intervalNameListWidget->clear();
		m_intervalNameListWidget->addItems(intervalNameList);
        m_intervalNameListWidget->sortItems(Qt::DescendingOrder);
	}
    else
    {
        m_intervalNameListWidget->clear();
    }
	
	//	m_intervalName->setComboBoxContent(intervalNameList);
	// 	if (intervalNameList.count() > m_intervalNameIndex)
	// 	{
	// 		m_intervalName->setComboBoxCurrentIndex(m_intervalNameIndex);
	// 	}
	// 	else
	// 	{
	// 		m_intervalNameIndex = 0;
	// 	}


	// 区域名称;
	m_areaNameMap.clear();
    WHEEL_DEVICE_CONFIG.loadWheelDeviceAreaData();
	std::map<QString, QString> areaNameMap = WHEEL_DEVICE_CONFIG.getWheelDeviceAreaDataMap();
	QStringList areaNameList;
	std::map<QString, QString>::iterator areaNameIter;
	for (areaNameIter = areaNameMap.begin(); areaNameIter != areaNameMap.end(); areaNameIter++)
	{
		m_areaNameMap.insert(areaNameIter->first, areaNameIter->second);
		areaNameList.append(areaNameIter->second);
	}

    m_areaNameListWidget->clear();
	m_areaNameListWidget->addItems(areaNameList);
    m_areaNameListWidget->sortItems(Qt::DescendingOrder);

//	m_areaName->setComboBoxContent(areaNameList);

// 	if (areaNameList.count() > m_areaNameIndex)
// 	{
// 		m_areaName->setComboBoxCurrentIndex(m_areaNameIndex);
// 	}
// 	else
// 	{
// 		m_areaNameIndex = 0;
// 	}

	
}

QListWidget* DLWheelPowerStationEdit::getListWidget(BaseWidget* baseWidget, QPushButton** addButton, QPushButton** deleteButton, QLineEdit** lineEdit)
{
	*addButton = new QPushButton("添加");
    (*addButton)->setObjectName("CommonButton");
    (*addButton)->setFixedSize(QSize(60, 25));

	*deleteButton = new QPushButton("删除");
	(*deleteButton)->setObjectName("CommonButton");
	(*deleteButton)->setFixedSize(QSize(60, 25));

	*lineEdit = new QLineEdit;

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addWidget(*lineEdit);
	hButtonLayout->addWidget(*addButton);
	hButtonLayout->addWidget(*deleteButton);
	hButtonLayout->setSpacing(6);
	hButtonLayout->setMargin(0);

	QListWidget* listWidget = new QListWidget;
	QVBoxLayout* vLayout = new QVBoxLayout(baseWidget->getCenterWidget());
	vLayout->addWidget(listWidget);
	vLayout->addLayout(hButtonLayout);
	vLayout->setMargin(10);

	return listWidget;
}

void DLWheelPowerStationEdit::onVoltageLevelItemClicked(QListWidgetItem* item)
{
	QString voltageLevelId = m_voltageLevelMap.key(item->text());
	m_intervalNameMap.clear();
	std::map<QString, QString> intervalNameMap = WHEEL_DEVICE_CONFIG.getWheelRobortEquipmentIntervalFromVoltageLevelId(voltageLevelId);
	QStringList intervalNameList;
	std::map<QString, QString>::iterator intervalNameIter;
	for (intervalNameIter = intervalNameMap.begin(); intervalNameIter != intervalNameMap.end(); intervalNameIter++)
	{
		m_intervalNameMap.insert(intervalNameIter->first, intervalNameIter->second);
		intervalNameList.append(intervalNameIter->second);
	}
	m_intervalNameListWidget->clear();
	m_intervalNameListWidget->addItems(intervalNameList);
    m_intervalNameListWidget->sortItems(Qt::DescendingOrder);
}
