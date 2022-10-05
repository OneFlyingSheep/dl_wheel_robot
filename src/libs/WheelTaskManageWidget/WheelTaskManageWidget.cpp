#include "WheelTaskManageWidget.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include <QRadioButton>
#include <QGridLayout>
#include <QPainter>
#include <QDesktopWidget>
#include <QApplication>
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPointTreeData.h"
#include <QDateTime>
#include <QUuid>
#include "DLWheelTaskExcuteWindow.h"

#define RADIOBUTTON_WIDTH 130					// radioButton宽度;
#define TIP_LABEL_WIDTH 80						// 每行标题文字Label宽度;
#define TABLE_PER_PAGE_COUNT 15					// table每页总条数;

DLWheelTaskManage::DLWheelTaskManage(SystemGuideMenuItemType itemType)
	: m_taskPageType(WheelTaskAdminType::WHEEL_TOTAL_PATROL)
	, m_itemType(itemType)
	, m_isInitWidget(false)
	, m_currentPageIndex(1)
	, m_isModifyStatus(false)
	, m_historyTaskImportWindow(NULL)
	, m_specialModeButtonGroup(NULL)
{
	QDesktopWidget* desktopWidget = QApplication::desktop();
	QRect screenRect = desktopWidget->screenGeometry();

	m_radioButtonInterval = (screenRect.width() - TIP_LABEL_WIDTH - RADIOBUTTON_WIDTH * 8 - 20) / 7;

	this->setStyleSheet("QWidget#TopBackWidget{background:rgb(240, 240, 240);}\
							QWidget#SearchBackWidget{ background:rgb(175, 191, 255); border:1px solid rgb(166, 233, 210); }\
							QToolButton{border:none;}\
							QTableWidget{border:none;}\
                            QPushButton:pressed{padding-left:2px;padding-top:2px;}");
}

DLWheelTaskManage::~DLWheelTaskManage()
{
}

int DLWheelTaskManage::GetMenuItemType()
{
	return m_itemType;
}

void DLWheelTaskManage::initLineEditWidget()
{
	m_lineEditBackWidget = new QWidget;

	m_taskNameInputWidget = new InputWidget(WheelLineEdit);
	m_taskNameInputWidget->setTipText("任务名称");

	m_taskDescribeInputWidget = new InputWidget(WheelLineEdit);
	m_taskDescribeInputWidget->setTipText("任务描述");

	QGridLayout* gLineEditLayout = new QGridLayout(m_lineEditBackWidget);
	gLineEditLayout->addWidget(m_taskNameInputWidget, 0, 0);
	gLineEditLayout->addItem(new QSpacerItem(40, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 0, 1);
	gLineEditLayout->addWidget(m_taskDescribeInputWidget, 1, 0);
	gLineEditLayout->addItem(new QSpacerItem(40, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 1, 1);
	gLineEditLayout->setSpacing(10);
	gLineEditLayout->setMargin(0);
}

void DLWheelTaskManage::initPatrolTypeWidget()
{
	m_patrolTypeBackWidget = new QWidget;

	QLabel* patrolTypeLabel = new QLabel;
	patrolTypeLabel->setText("巡检类型:");
	patrolTypeLabel->setFixedWidth(TIP_LABEL_WIDTH);

	QHBoxLayout* hPatrolTypeLayout = new QHBoxLayout(m_patrolTypeBackWidget);
	hPatrolTypeLayout->addWidget(patrolTypeLabel);
	m_patrolTypeButtonGroup = new QButtonGroup(this);
	for (int i = 0; i < 4; i++)
	{
		QRadioButton* pRadioButton = new QRadioButton;
		pRadioButton->setFixedWidth(RADIOBUTTON_WIDTH);
		pRadioButton->setDisabled(true);
		hPatrolTypeLayout->addWidget(pRadioButton);
		hPatrolTypeLayout->addSpacing(m_radioButtonInterval - 10);
		m_patrolTypeButtonGroup->addButton(pRadioButton, i);
	}
	hPatrolTypeLayout->addStretch();
	hPatrolTypeLayout->setSpacing(10);
	hPatrolTypeLayout->setMargin(0);

	m_patrolTypeButtonGroup->button(Routine_Patrol)->setText("例行巡视");
	m_patrolTypeButtonGroup->button(All_Patrol)->setText("全面巡视");
	m_patrolTypeButtonGroup->button(Single_Patrol)->setText("专项巡视");
	m_patrolTypeButtonGroup->button(Special_Patrol)->setText("特殊巡视");
}

void DLWheelTaskManage::initSpecialModeWidget()
{
	m_specialModeBackWidget = new QWidget;

	QLabel* specialModeLabel = new QLabel;
	specialModeLabel->setText("特巡模式:");
	specialModeLabel->setFixedWidth(TIP_LABEL_WIDTH);

	QHBoxLayout* hSpecialModeLayout = new QHBoxLayout(m_specialModeBackWidget);
	hSpecialModeLayout->addWidget(specialModeLabel);
	m_specialModeButtonGroup = new QButtonGroup(this);
	for (int i = 0; i < 6; i++)
	{
		QRadioButton* pRadioButton = new QRadioButton;
		if (i == 0)
		{
			pRadioButton->setChecked(true);
		}
		pRadioButton->setFixedWidth(RADIOBUTTON_WIDTH);
		hSpecialModeLayout->addWidget(pRadioButton);
		hSpecialModeLayout->addSpacing(m_radioButtonInterval - 10);
		m_specialModeButtonGroup->addButton(pRadioButton, i);
	}
	hSpecialModeLayout->addStretch();
	hSpecialModeLayout->setSpacing(10);
	hSpecialModeLayout->setMargin(0);

	m_specialModeButtonGroup->button(ClimbSummer_Mode)->setText("迎峰度夏特巡");
	m_specialModeButtonGroup->button(ThunderStorm_Mode)->setText("雷暴天气特巡");
	m_specialModeButtonGroup->button(FloodPrevention_Mode)->setText("防汛坑台特巡");
	m_specialModeButtonGroup->button(SleetFrozen_Mode)->setText("雨雪冰冻特巡");
	m_specialModeButtonGroup->button(HazeWeahter_Mode)->setText("雾霾天气特巡");
	m_specialModeButtonGroup->button(GaleWeather_Mode)->setText("大风天气特巡");
}

void DLWheelTaskManage::refreshCheckBox(WheelTaskAdminType taskPageType)
{
	m_taskPageType = taskPageType;

	QList<WheelTaskAdminCheckBoxStruct> checkBoxDataList = WHEEL_TASK_ADMINISTRATION.getTaskAdminCheckBoxData(m_taskPageType);
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

void DLWheelTaskManage::initSearchWidget()
{
	m_searchBackWidget = new QWidget;
	m_searchBackWidget->setObjectName("SearchBackWidget");
	m_searchBackWidget->setFixedHeight(30);

	m_deviceSearchLineEdit = new QLineEdit;
	m_deviceSearchLineEdit->setFixedSize(QSize(200, 20));
	m_deviceSearchLineEdit->setStyleSheet("QLineEdit{;padding-left:5px;border:1px solid gray;}");

	m_pButtonSearch = new QToolButton;
	m_pButtonSearch->setText("查询");
	m_pButtonSearch->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonSearch->setFixedSize(QSize(60, 20));
	m_pButtonSearch->setIcon(QIcon(":/Resources/Common/image/Search.png"));
	m_pButtonSearch->setIconSize(QSize(16, 16));
	connect(m_pButtonSearch, &QPushButton::clicked, this, &DLWheelTaskManage::onSearchButtonClicked);

	m_pButtonSave = new QToolButton;
	m_pButtonSave->setText("保存");
	m_pButtonSave->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonSave->setFixedSize(QSize(60, 20));
	m_pButtonSave->setIcon(QIcon(":/Resources/Common/image/Save.png"));
	m_pButtonSave->setIconSize(QSize(16, 16));
	connect(m_pButtonSave, &QPushButton::clicked, this, &DLWheelTaskManage::onSaveButtonClicked);

	m_pButtonReset = new QToolButton;
	m_pButtonReset->setText("重置");
	m_pButtonReset->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonReset->setFixedSize(QSize(60, 20));
	m_pButtonReset->setIcon(QIcon(":/Resources/Common/image/Reset.png"));
	m_pButtonReset->setIconSize(QSize(16, 16));
	connect(m_pButtonReset, &QPushButton::clicked, this, &DLWheelTaskManage::onResetButtonClicked);

	m_pButtonImportTask = new QPushButton;
	m_pButtonImportTask->setText("任务导入");
	m_pButtonImportTask->setFixedSize(QSize(60, 16));
	m_pButtonImportTask->setStyleSheet("border:none;border-left:1px solid white;");
	connect(m_pButtonImportTask, &QPushButton::clicked, this, [=] {
		m_historyTaskImportWindow->resetWidget();
		m_historyTaskImportWindow->show();
	});

	m_pButtonGoBack = new QPushButton("返回");
	m_pButtonGoBack->setFixedSize(QSize(60, 16));
	m_pButtonGoBack->setStyleSheet("border:none;border-left:1px solid white;");
	m_pButtonGoBack->setVisible(false);

	connect(m_pButtonGoBack, &QPushButton::clicked, this, &DLWheelTaskManage::signalGoBackToTaskShow);

	QLabel* separatorLabel[2];
	for (int i = 0; i < 2; i++)
	{
		separatorLabel[i] = new QLabel;
		separatorLabel[i]->setFrameShape(QFrame::VLine);
		separatorLabel[i]->setFixedSize(QSize(1, 16));
		separatorLabel[i]->setStyleSheet("border:1px solid white;");
	}

	QHBoxLayout* hSearchButton = new QHBoxLayout(m_searchBackWidget);
	hSearchButton->addWidget(m_deviceSearchLineEdit);
	hSearchButton->addWidget(m_pButtonSearch);
	hSearchButton->addWidget(separatorLabel[0]);
	hSearchButton->addWidget(m_pButtonSave);
	hSearchButton->addWidget(separatorLabel[1]);
	hSearchButton->addWidget(m_pButtonReset);
	hSearchButton->addWidget(m_pButtonImportTask);
	hSearchButton->addWidget(m_pButtonGoBack);
	hSearchButton->addStretch();
	hSearchButton->setContentsMargins(3, 0, 0, 0);
	hSearchButton->setSpacing(2);
}

void DLWheelTaskManage::initTopWidget()
{
	initLineEditWidget();
	initPatrolTypeWidget();
	initSpecialModeWidget();
	initSearchWidget();

	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("TopBackWidget");
	m_checkBoxBackWidget = new QWidget;

	m_checkBoxBackWidget->setObjectName("checkboxwgt");

	QVBoxLayout* vTopLayout = new QVBoxLayout(m_topBackWidget);
	vTopLayout->addWidget(m_lineEditBackWidget);
	vTopLayout->addWidget(m_patrolTypeBackWidget);
	vTopLayout->addWidget(m_specialModeBackWidget);
	vTopLayout->addSpacerItem(new QSpacerItem(0, 10));
	vTopLayout->addWidget(m_checkBoxBackWidget);
	vTopLayout->addWidget(m_searchBackWidget);
	vTopLayout->setSpacing(20);
	vTopLayout->setMargin(6);
}

void DLWheelTaskManage::initPointPosTreeWidget()
{
	m_pointPosTreeWidget = new CustomTreeWidget;
	m_pointPosTreeWidget->setObjectName("treewidget");
	m_pointPosTreeWidget->setSearchLineEditVisible(false);
	connect(this, SIGNAL(signalUpdateTreeLevel(QString, DeviceAlarmLevel)), m_pointPosTreeWidget, SLOT(UpdateTreeItemLevelSlot(QString, DeviceAlarmLevel)));
	// 这里应该根据当前巡检的CheckBox条件进行筛选;
	//m_pointPosTreeWidget->refreshTreeItemList();
}

void DLWheelTaskManage::initTaskFormulateTable()
{
	m_wheelPointTreeData = new DLWheelPointTreeData;

	m_taskFormulateTableBackWidget = new QWidget;

	QWidget* titleBackWidget = new QWidget;
	titleBackWidget->setObjectName("SearchBackWidget");
	titleBackWidget->setFixedHeight(30);

	QLabel* titleLabel = new QLabel;
	titleLabel->setText("任务编制列表");
	titleLabel->setStyleSheet("font-weight:bold;");

	QHBoxLayout* hTitleLayout = new QHBoxLayout(titleBackWidget);
	hTitleLayout->addWidget(titleLabel);
	hTitleLayout->addStretch();
	hTitleLayout->setMargin(5);
	hTitleLayout->setSpacing(0);

	m_taskFormulateTable = new CustomTableWidget(4);
	m_taskFormulateTable->setHorizontalHeaderLabels(QStringList() << "序号" << "任务名称" << "编辑时间" << "操作");
	connect(m_taskFormulateTable, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
		m_currentPageIndex = currentPageIndex;
		initTableData();
	});
	connect(m_taskFormulateTable, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
		initTableData();
	});

	connect(m_taskFormulateTable, &CustomTableWidget::signalTableDoubleClicked, this, &DLWheelTaskManage::onTaskTableDoubleClicked);

	QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();
	connect(tableWidget, &QTableWidget::cellPressed, this, [=](int row) {
		if (QApplication::mouseButtons() == Qt::RightButton)
		{
			if (!m_isModifyStatus)
			{
				QMenu* deleteMenu = new QMenu;
				QAction* deleteAction = deleteMenu->addAction("删除");
				connect(deleteAction, &QAction::triggered, this, [=] {
					if (row < tableWidget->rowCount())
					{
						QString strId = m_tableDataList[row].task_edit_uuid;
						WHEEL_BACK_TO_CORE_SOCKET.robot_task_edit_delete_req(strId.toStdString(), m_taskPageType);
					}
				});
				deleteMenu->exec(QCursor::pos());
				deleteMenu->deleteLater();
			}
		}
	});

	QVBoxLayout* vTaskFormulateTableLayout = new QVBoxLayout(m_taskFormulateTableBackWidget);
	vTaskFormulateTableLayout->addWidget(titleBackWidget);
	vTaskFormulateTableLayout->addWidget(m_taskFormulateTable);
	vTaskFormulateTableLayout->setSpacing(0);
	vTaskFormulateTableLayout->setMargin(0);

}

void DLWheelTaskManage::initTableData()
{
	if (NULL == m_taskFormulateTable) return;
	QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();
	m_tableDataList = WHEEL_TASK_ADMINISTRATION.getWheelTaskEditList(m_currentPageIndex, TABLE_PER_PAGE_COUNT, m_taskPageType);
	tableWidget->clearContents();
	tableWidget->setRowCount(0);

	int totalPage, totalCount;
	WHEEL_TASK_ADMINISTRATION.getWheelTaskEditPage(totalPage, totalCount, TABLE_PER_PAGE_COUNT, m_taskPageType);
	m_taskFormulateTable->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

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

		QTableWidgetItem* taskNameItem = new QTableWidgetItem(m_tableDataList[i].task_edit_name);
		tableWidget->setItem(i, 2, taskNameItem);
		taskNameItem->setTextAlignment(Qt::AlignCenter);

		QTableWidgetItem* taskDateItem = new QTableWidgetItem(m_tableDataList[i].task_edit_date);
		taskDateItem->setTextAlignment(Qt::AlignCenter);
		tableWidget->setItem(i, 3, taskDateItem);

		QTableWidgetItem* buttonItem = new QTableWidgetItem;
		buttonItem->setTextAlignment(Qt::AlignCenter);
		tableWidget->setItem(i, 4, buttonItem);
		ButtonWidget* buttonWidget = new ButtonWidget(m_tableDataList[i]);
		buttonWidget->setRowIndex(i);
		connect(buttonWidget, &ButtonWidget::signalButtonClicked, this, &DLWheelTaskManage::onTaskOperateTypeButtonClicked);
		tableWidget->setCellWidget(i, 4, buttonWidget);

		tableWidget->setRowHeight(i, 40);
	}
}

void DLWheelTaskManage::initBottomWidget()
{
	initPointPosTreeWidget();
	initTaskFormulateTable();

	m_bottomBackWidget = new QWidget;

	QHBoxLayout* hBottomLayout = new QHBoxLayout(m_bottomBackWidget);
	hBottomLayout->addWidget(m_pointPosTreeWidget);
	hBottomLayout->addWidget(m_taskFormulateTableBackWidget);
	hBottomLayout->setMargin(0);
	hBottomLayout->setSpacing(5);
}

void DLWheelTaskManage::initWidget()
{
	if (!m_isInitWidget)
	{
		m_isInitWidget = true;
		initTopWidget();
		initBottomWidget();

		QVBoxLayout* vTopLayout = new QVBoxLayout(this);
		vTopLayout->addWidget(m_topBackWidget);
		vTopLayout->addWidget(m_bottomBackWidget);
		vTopLayout->setSpacing(0);
		vTopLayout->setMargin(0);

		//设置当前任务页的类型，加载不同任务页的checkbox
		setTaskShowType(m_itemType);
		initTableData();

		m_historyTaskImportWindow = new HistoryTaskImportWindow;
		connect(m_historyTaskImportWindow, &HistoryTaskImportWindow::signalWindowClose, this, [=] {
			m_historyTaskImportWindow->hide();
		});

		connect(m_historyTaskImportWindow, &HistoryTaskImportWindow::signalImportSuccess, this, [=] {
			m_historyTaskImportWindow->hide();
			DLMessageBox::showDLMessageBox(NULL, "提示", "导入成功", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
			initTableData();
		});
	}
	else
	{
		initTableData();
		// 说明已经重新再进入到此页面，所以不再维护任务展示点击修改时维护的状态;
		m_pButtonGoBack->setVisible(false);
		m_isModifyStatus = false;
	}
}

void DLWheelTaskManage::onModifyTaskTable(QString task_edit_uuid)
{
	m_isModifyStatus = true;
	m_pButtonGoBack->setVisible(true);

	QMap<QString, QString> deviceMap;
	WheelTaskAdminType taskAdminType;

	WHEEL_TASK_ADMINISTRATION.getDeviceUUidMapAndEditType(deviceMap, taskAdminType, m_modifyTaskEditStruct, task_edit_uuid);

	// 这里应该根据当前巡检的CheckBox条件进行筛选;
	//m_pointPosTreeWidget->refreshTreeItemList();

	QStringList strDeviceList = WHEEL_TASK_ADMINISTRATION.getDeviceUUidFromTaskDevicesList(task_edit_uuid);

	m_pointPosTreeWidget->ClearAllItemState();
	for (int i = 0; i < strDeviceList.count(); i++)
	{
		QStringList treeItemPath = m_wheelPointTreeData->getDeviceNodePathFromDevices(strDeviceList[i]);
		m_pointPosTreeWidget->expandTreeNodeByDevice(treeItemPath, strDeviceList[i]);
	}

	QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();
	QList<WheelTaskEditStruct> tableDataList = WHEEL_TASK_ADMINISTRATION.getWheelTaskEditList(m_currentPageIndex, TABLE_PER_PAGE_COUNT, m_taskPageType);
	tableWidget->clearContents();
	tableWidget->setRowCount(0);

	m_currentPageIndex = 1;
	m_taskFormulateTable->setTurnPageInfo(m_currentPageIndex, 1, 1, TABLE_PER_PAGE_COUNT);

	tableWidget->insertRow(0);
	QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
	checkBoxItem->setCheckState(Qt::Unchecked);
	checkBoxItem->setTextAlignment(Qt::AlignCenter);
	tableWidget->setItem(0, 0, checkBoxItem);

	QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(0 + 1));
	idItem->setTextAlignment(Qt::AlignCenter);
	tableWidget->setItem(0, 1, idItem);

	QTableWidgetItem* taskNameItem = new QTableWidgetItem(m_modifyTaskEditStruct.task_edit_name);
	tableWidget->setItem(0, 2, taskNameItem);
	taskNameItem->setTextAlignment(Qt::AlignCenter);

	QTableWidgetItem* taskDateItem = new QTableWidgetItem(m_modifyTaskEditStruct.task_edit_date);
	taskDateItem->setTextAlignment(Qt::AlignCenter);
	tableWidget->setItem(0, 3, taskDateItem);

	QTableWidgetItem* buttonItem = new QTableWidgetItem;
	buttonItem->setTextAlignment(Qt::AlignCenter);
	tableWidget->setItem(0, 4, buttonItem);
	ButtonWidget* buttonWidget = new ButtonWidget(m_modifyTaskEditStruct);
	buttonWidget->setRowIndex(0);
	connect(buttonWidget, &ButtonWidget::signalButtonClicked, this, &DLWheelTaskManage::onTaskOperateTypeButtonClicked);
	tableWidget->setCellWidget(0, 4, buttonWidget);

	tableWidget->setRowHeight(0, 40);
}

void DLWheelTaskManage::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), Qt::white);
}

QString DLWheelTaskManage::getCurrentTaskTypeName()
{
	QString strTaskTypeName = "";
	switch (m_taskPageType)
	{
	case WHEEL_TOTAL_PATROL:
		strTaskTypeName = "全面巡检";
		break;
	case WHEEL_ROUTINE_PATROL:
		strTaskTypeName = "例行巡检";
		break;
	case WHEEL_INFRARED_THERMOMETRY:
	case WHEEL_OIL_LEVEL_THERMOMETRY:
	case WHEEL_ARRESTER_METER_COPY:
	case WHEEL_SF6_PRESSURE_GAGE:
	case WHEEL_HYDRAULIC_PRESSURE_GAUGE:
	case WHEEL_LOCATION_STATE_DISCERN:
		strTaskTypeName = "专项巡检";
		break;
	case WHEEL_SCURVINESS_WEATHER_PAROL:
	case WHEEL_FLAW_TAIL_AFTER:
	case WHEEL_DISTANT_ABNORMAL_ALARM:
	case WHEEL_DISTANT_STATE_AFFIRM:
	case WHEEL_SECURITY_LINKAGE:
	case WHEEL_ASSIST_ACCIDENT_DISPOSE:
	{
		strTaskTypeName = "特殊巡检";
		if (NULL != m_specialModeButtonGroup)
		{
			QAbstractButton *pBtn = m_specialModeButtonGroup->checkedButton();
			QString str = "";
			if (NULL != pBtn)
			{
				str = pBtn->text();
			}
			if (!str.isEmpty())
			{
				strTaskTypeName = QString("%1-%2").arg(strTaskTypeName).arg(str);
			}
		}

	}
	break;
	case WHEEL_USER_DEFINED_TASK:
	{
		strTaskTypeName = m_taskNameInputWidget->getLineEditText();
	}
	break;
	case WHEEL_DEVICE_ALARM_MES:
		break;
	default:
		break;
	}

	int level;
	QString strStationName;
	DLWheelPointTreeData data;
	data.getPointTreeSecondRootNode(strStationName, level);
	QString strDate = QDate::currentDate().toString("yyyyMMdd");

	strTaskTypeName = strStationName + strTaskTypeName + strDate;

	return strTaskTypeName;
}

void DLWheelTaskManage::onSearchButtonClicked()
{
	WheelPatrolParameter wheelPatrolPara;

	for (int i = 0; i < m_checkBoxWidgetList.count(); i++)
	{
		QStringList strCheckedIdList = m_checkBoxWidgetList[i]->getCheckedIdList();
		WheelCheckBoxTypeEnum currentType = m_checkBoxWidgetList[i]->getCheckBoxType();

		switch (currentType)
		{
		case WHEEL_DEVICE_AREA_NAME:
			wheelPatrolPara.m_device_area_uuid = strCheckedIdList;
			break;
		case WHEEL_DEVICE_TYPE:
			wheelPatrolPara.m_device_type_uuid = strCheckedIdList;
			break;
		case WHEEL_RECOGNITION_TYPE:
			wheelPatrolPara.m_recognition_type_uuid = strCheckedIdList;
			break;
		case WHEEL_METER_TYPE:
			wheelPatrolPara.m_meter_type_id = strCheckedIdList;
			break;
			// case WHEEL_DEVICE_APPEARANCE_TYPE:
			//	wheelPatrolPara.m_device_area_uuid = strCheckedIdList;
			//	break;
		default:
			break;
		}
	}
	wheelPatrolPara.device_point_name = m_deviceSearchLineEdit->text();
	m_pointPosTreeWidget->updataTreeData(wheelPatrolPara);
}

void DLWheelTaskManage::onSaveButtonClicked()
{
	QStringList deviceIdLsit = m_pointPosTreeWidget->getChoosedDeviceIdList();
	if (deviceIdLsit.isEmpty())
	{
		DLMessageBox::showDLMessageBox(NULL, "提示", "未选择设备", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
		return;
	}

	if (m_taskPageType == WHEEL_USER_DEFINED_TASK)
	{
		if (m_taskNameInputWidget->getLineEditText().isEmpty())
		{
			DLMessageBox::showDLMessageBox(NULL, "提示", "请填写任务名称", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
			return;
		}
	}

	// 当前是否是修改;
	if (m_isModifyStatus)
	{
		// 发送给Core;
		m_modifyTaskEditStruct.task_edit_date = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
		WHEEL_BACK_TO_CORE_SOCKET.robot_task_edit_updata_req(m_modifyTaskEditStruct, deviceIdLsit);
	}
	else
	{
		// 发送给Core;
		WheelTaskEditStruct wheelTaskEditStru;
		wheelTaskEditStru.task_edit_name = getCurrentTaskTypeName();
		wheelTaskEditStru.task_edit_date = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
		wheelTaskEditStru.task_edit_type_id = m_taskPageType;
		wheelTaskEditStru.task_edit_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
		WHEEL_BACK_TO_CORE_SOCKET.robot_task_edit_insert_req(wheelTaskEditStru, deviceIdLsit);
	}
}

void DLWheelTaskManage::onResetButtonClicked()
{
	for (int i = 0; i < m_checkBoxWidgetList.count(); i++)
	{
		m_checkBoxWidgetList[i]->resetCheckBox();
	}
}

void DLWheelTaskManage::onTaskTableDoubleClicked(int row)
{
	if (row >= m_tableDataList.count())
	{
		return;
	}

	//m_pointPosTreeWidget->refreshTreeItemList();

	QString strTaskEditId = m_tableDataList[row].task_edit_uuid;
	QStringList strDeviceList = WHEEL_TASK_ADMINISTRATION.getDeviceUUidFromTaskDevicesList(strTaskEditId);

	for (int i = 0; i < strDeviceList.count(); i++)
	{
		QStringList treeItemPath = m_wheelPointTreeData->getDeviceNodePathFromDevices(strDeviceList[i]);
		m_pointPosTreeWidget->expandTreeNodeByDevice(treeItemPath, strDeviceList[i]);
	}
}

void DLWheelTaskManage::onUpdateDeleteResult(int strUuid, bool isSuccess, QString strMsg)
{
	m_currentPageIndex = 1;
	if (isSuccess)
	{
		initTableData();
	}
	else
	{
		DLMessageBox* messageBox = new DLMessageBox(this);
		messageBox->setFixedWidth(250);
		messageBox->setMessageContent("删除失败");
		messageBox->setWindowModality(Qt::ApplicationModal);
		messageBox->show();
	}
}

void DLWheelTaskManage::onUpdateSaveTaskResult(int strUuid, bool isSuccess, QString strMsg)
{
	m_currentPageIndex = 1;
	if (isSuccess)
	{
		initTableData();
	}
	else
	{
		DLMessageBox::showDLMessageBox(this, "错误", "保存任务编制失败\n错误码:" + strMsg, MessageButtonType::BUTTON_OK, true);
	}
}

void DLWheelTaskManage::setTaskShowType(SystemGuideMenuItemType itemType)
{
	// 先将控件都显示出来，再隐藏需要的;
	m_lineEditBackWidget->setVisible(true);
	m_taskNameInputWidget->setVisible(true);
	m_taskDescribeInputWidget->setVisible(true);
	m_specialModeBackWidget->setVisible(true);
	m_pButtonImportTask->setVisible(false);

	switch (itemType)
	{
	case Menu_All_Patrol:
	{
		m_patrolTypeButtonGroup->button(All_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_TOTAL_PATROL);
	}
	break;
	case Menu_Routine_Patrol:
	{
		m_patrolTypeButtonGroup->button(Routine_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_ROUTINE_PATROL);
	}
	break;
	case Menu_InfraredCalculateTmp:
	{
		m_patrolTypeButtonGroup->button(Single_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_INFRARED_THERMOMETRY);
	}
	break;
	case Menu_OilTableRecord:
	{
		m_patrolTypeButtonGroup->button(Single_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_OIL_LEVEL_THERMOMETRY);
	}
	break;
	case Menu_ArresterTableRead:
	{
		m_patrolTypeButtonGroup->button(Single_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_ARRESTER_METER_COPY);
	}
	break;
	case Menu_SF6PressureRecord:
	{
		m_patrolTypeButtonGroup->button(Single_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_SF6_PRESSURE_GAGE);
	}
	break;
	case Menu_HydraumaticTableRecord:
	{
		m_patrolTypeButtonGroup->button(Single_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_HYDRAULIC_PRESSURE_GAUGE);
	}
	break;
	case Menu_PosStateRecognition:
	{
		m_patrolTypeButtonGroup->button(Single_Patrol)->setChecked(true);
		m_taskNameInputWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_LOCATION_STATE_DISCERN);
	}
	break;
	case Menu_BadWeatherPatrol:
	{
		m_patrolTypeButtonGroup->button(Special_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_SCURVINESS_WEATHER_PAROL);
	}
	break;
	case Menu_DefectTrack:
	{
		m_patrolTypeButtonGroup->button(Special_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_FLAW_TAIL_AFTER);
	}
	break;
	case Menu_DistanceStateAffirm:
	{
		m_patrolTypeButtonGroup->button(Special_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_DISTANT_STATE_AFFIRM);
	}
	break;
	case Menu_DistanceAbnormalAlarmAffirm:
	{
		m_patrolTypeButtonGroup->button(Special_Patrol)->setChecked(true);
		m_taskNameInputWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_DISTANT_ABNORMAL_ALARM);
	}
	break;
	case Menu_SecurityLinkage:
	{
		m_patrolTypeButtonGroup->button(Special_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_SECURITY_LINKAGE);
	}
	break;
	case Menu_AssistAccidentDeal:
	{
		m_patrolTypeButtonGroup->button(Special_Patrol)->setChecked(true);
		m_lineEditBackWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		refreshCheckBox(WheelTaskAdminType::WHEEL_ASSIST_ACCIDENT_DISPOSE);
	}
	break;
	case Menu_CustomTask:
	{
		m_taskDescribeInputWidget->setVisible(false);
		m_specialModeBackWidget->setVisible(false);
		m_pButtonImportTask->setVisible(true);
		refreshCheckBox(WheelTaskAdminType::WHEEL_USER_DEFINED_TASK);
	}
	break;
	default:
		break;
	}

	onSearchButtonClicked();
}

void DLWheelTaskManage::onGetSaveOperationResult(bool isSuccess, QString strMsg)
{
	if (isSuccess)
	{
		initTableData();
	}
	else
	{
		DLMessageBox::showDLMessageBox(this, "错误", "保存任务编制失败\n错误码:" + strMsg, MessageButtonType::BUTTON_OK, true);
	}
}

void DLWheelTaskManage::onTaskOperateTypeButtonClicked(int buttonId, WheelTaskEditStruct data)
{
	TaskExcuteType excuteType = TaskExcuteType(buttonId);
	switch (excuteType)
	{
	case ImmediatelyExcute:
	{
		WheelRobotAssignTask task;
		task.task_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
		task.task_edit_uuid = data.task_edit_uuid;
		ButtonWidget* buttonWidget = static_cast<ButtonWidget*>(sender());
		if (buttonWidget != NULL)
		{
			int currentRow = buttonWidget->getRowIndex();
			QTableWidget* tableWidget = m_taskFormulateTable->getTableWidget();
			task.task_name = tableWidget->item(currentRow, 2)->text();
		}

		QStringList strDeviceIdList = WHEEL_TASK_ADMINISTRATION.getDeviceUUidFromTaskDevicesList(data.task_edit_uuid);
		for (int i = 0; i < strDeviceIdList.count(); i++)
		{
			task.devices.push_back(strDeviceIdList[i].toStdString());
		}

		DLWheelImmediatelyExcuteWindow* dLWheelImmediatelyExcuteWindow = new DLWheelImmediatelyExcuteWindow(this);
		dLWheelImmediatelyExcuteWindow->setImmediatelyTaskInfo(task);
		connect(dLWheelImmediatelyExcuteWindow, &DLWheelImmediatelyExcuteWindow::signalOKButtonClikced, this, [=](WheelRobotAssignTask task) {
			WHEEL_BACK_TO_CORE_SOCKET.robot_task_assign_req(task);
		});
		dLWheelImmediatelyExcuteWindow->exec();
	}
	break;
	case FixedTimeExcute:
	{
		DLWheelFixedTaskExcuteWindow* dLWheelFixedTaskExcuteWindow = new DLWheelFixedTaskExcuteWindow(this);
		connect(dLWheelFixedTaskExcuteWindow, &DLWheelFixedTaskExcuteWindow::signalOKButtonClikced, this, [=](QString strDate, QTime strTime) {
			WheelTaskTemplateStruct temp;
			temp.task_template_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
			temp.task_edit_uuid = data.task_edit_uuid;
			temp.task_type_id = WHEEL_ROBOT_TASK_TIMED_TASK;
			temp.task_end_action_id = WHEEL_ROBOT_TASK_END_TYPE_TO_CHARGE;
			temp.task_template_name = data.task_edit_name;
			temp.task_status_id = WHEEL_ROBOT_TASK_STATUS_WAIT;
			temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_NONE;
			temp.task_repeat_duration = 0;
			temp.task_start_date = strDate;
			temp.task_start_time = strTime;

			WHEEL_BACK_TO_CORE_SOCKET.robot_task_template_insert_req(temp);
			dLWheelFixedTaskExcuteWindow->close();

		});
		dLWheelFixedTaskExcuteWindow->exec();
	}
	break;
	case PeriodExcute:
	{
		DLWheelPeriodTaskExcuteWindow* dLWheelPeriodTaskExcuteWindow = new DLWheelPeriodTaskExcuteWindow(this);
		connect(dLWheelPeriodTaskExcuteWindow, &DLWheelPeriodTaskExcuteWindow::signalOKButtonClikced, this, [=](PeriodTaskType taskType) {
			WheelTaskTemplateStruct temp;
			temp.task_template_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
			temp.task_edit_uuid = data.task_edit_uuid;
			temp.task_type_id = WHEEL_ROBOT_TASK_LOOP_TASK;
			temp.task_end_action_id = WHEEL_ROBOT_TASK_END_TYPE_TO_CHARGE;
			temp.task_template_name = data.task_edit_name;
			temp.task_status_id = WHEEL_ROBOT_TASK_STATUS_WAIT;
			temp.task_repeat_duration = 0;
			switch (taskType)
			{
			case EveryDay:
			{
				temp.task_start_time = dLWheelPeriodTaskExcuteWindow->getEveryDayTime();
				temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_DAY;
			}
			break;
			case EveryWeek:
			{
				dLWheelPeriodTaskExcuteWindow->getEveryWeekTime(temp.task_start_date, temp.task_start_time);
				temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_WEEK;
			}
			break;
			case EveryMonth:
			{
				dLWheelPeriodTaskExcuteWindow->getEveryMonthTime(temp.task_start_date, temp.task_start_time);
				temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_EVERY_MONTH;
			}
			break;
			case FixedInterval:
			{
				dLWheelPeriodTaskExcuteWindow->getFixTime(temp.task_start_date, temp.task_repeat_duration, temp.task_start_time);
				temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_FIXED_INTERVAL_DAYS;
			}
			break;
			case MultiDate:
			{
				dLWheelPeriodTaskExcuteWindow->getMultiDateChoose(temp.task_start_date, temp.task_start_time);
				temp.task_loop_type_id = WHEEL_ROBOT_TASK_LOOP_TYPE_CALENDAR;
			}
			break;
			default:
				break;
			}

			WHEEL_BACK_TO_CORE_SOCKET.robot_task_template_insert_req(temp);
			dLWheelPeriodTaskExcuteWindow->close();
		});
		dLWheelPeriodTaskExcuteWindow->exec();
	}
	break;
	default:
		break;
	}
}

void DLWheelTaskManage::onDeleteTaskCallBack(bool isSuccess, QString strMsg)
{
	if (isSuccess)
	{
		initTableData();
	}
	else
	{
		DLMessageBox::showDLMessageBox(NULL, "错误", "删除失败:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
	}
}
