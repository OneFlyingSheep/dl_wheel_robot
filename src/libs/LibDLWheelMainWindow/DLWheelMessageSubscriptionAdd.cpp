#include "DLWheelMessageSubscriptionAdd.h"
#include <QHBoxLayout>
#include <QRadioButton>
#include <QPainter>
#include <QButtonGroup>
#include <QCheckBox>
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotStationConfig.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelCustomWidget/DefData.h"

#pragma execution_character_set("utf-8")

#define TITLE_WIDGET_HEIGHT 150

DLWheelMessageSubscriptionAdd::DLWheelMessageSubscriptionAdd()
{
	initWidget();
	this->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
	this->setFixedSize(QSize(1000, 750));
	this->setWindowModality(Qt::ApplicationModal);
    this->setAttribute(Qt::WA_DeleteOnClose);
	this->setStyleSheet("QWidget#GrayBackWidget{background:rgb(240, 240, 240);}\
							QWidget#LeftBackWidget{background-color:rgb(159,168,218);}\
							QWidget#SearchLineEditWidget{background:rgb(175, 191, 255);border:none;}\
							QTableWidget{border:none;}\
							QTabWidget::pane{border:2px solid rgb(159,168,218);}\
							QTabWidget::tab-bar{alignment:left; left:3px; right:3px; }\
							QTabBar::tab{min-height:20px;min-width:85px;background-color:rgb(175, 191, 255);\
							border:1px solid rgb(121,134,203);border-top-left-radius:3px;border-top-right-radius:3px;}\
							QTabBar::tab:selected{font-weight:bold;}\
							QTreeView{border: 1px solid lightgray;}\
							QTreeView::item{height: 25px;border:none;background:transparent;color: black;outline:none;}\
							QTreeView::item:hover{background: rgba(33, 150, 243, 80);}\
							QTreeView::branch:open:has-children{image: url(:/Resources/Common/image/branch_Open.png);}\
							QTreeView::branch:closed:has-children{image: url(:/Resources/Common/image/branch_Close.png);}"); 
}

void DLWheelMessageSubscriptionAdd::closeWidget()
{
    m_timeoutTime.stop();
    this->close();
}

void DLWheelMessageSubscriptionAdd::initTopWidget()
{
	m_messageSubscriptionAddWidget = new TitleWidget;
	m_messageSubscriptionAddWidget->setTitleText("消息订阅添加");
	m_messageSubscriptionAddWidget->setFixedHeight(180);

	QLabel* alarmLevelLabel = new QLabel("告警等级");
    QHBoxLayout* hAlarmLayout = new QHBoxLayout;
    hAlarmLayout->addWidget(alarmLevelLabel);
    QButtonGroup* alarmButtonGroup = new QButtonGroup(this);
    for (int i = 0; i < 3; i ++)
    {
        QCheckBox* alarmCheckBox = new QCheckBox();
        hAlarmLayout->addWidget(alarmCheckBox);
        alarmButtonGroup->addButton(alarmCheckBox, i);
        m_alarmLevelChecoBoxList.append(alarmCheckBox);
    }

    hAlarmLayout->addStretch();
    hAlarmLayout->setSpacing(60);
    hAlarmLayout->setMargin(0);

    m_alarmLevelChecoBoxList[0]->setText("一般告警");
    m_alarmLevelChecoBoxList[0]->setChecked(true);
    m_alarmLevelChecoBoxList[1]->setText("严重告警");
    m_alarmLevelChecoBoxList[2]->setText("危急告警");

	QLabel* sendTimeLabel = new QLabel("发送时间");
    QHBoxLayout* hSendTimeLayout = new QHBoxLayout;
    hSendTimeLayout->addWidget(sendTimeLabel);
    for (int i = 0; i < 3; i++)
    {
        QRadioButton* sendTimeRadioButton = new QRadioButton();
        hSendTimeLayout->addWidget(sendTimeRadioButton);
        m_sendTimeRadioButtonList.append(sendTimeRadioButton);
    }

    hSendTimeLayout->addStretch();
    hSendTimeLayout->setSpacing(60);
    hSendTimeLayout->setMargin(0);

    m_sendTimeRadioButtonList[0]->setText("实时发送");
    m_sendTimeRadioButtonList[0]->setChecked(true);
    m_sendTimeRadioButtonList[1]->setText("巡检完毕");
    m_sendTimeRadioButtonList[2]->setText("定时发送");

	QLabel* sendFrequencyLabel = new QLabel("发送频率");
    QHBoxLayout* hSendFrequencyLayout = new QHBoxLayout;
    hSendFrequencyLayout->addWidget(sendFrequencyLabel);
    for (int i = 0; i < 2; i++)
    {
        QRadioButton* sendFreqRadioButton = new QRadioButton();
        hSendFrequencyLayout->addWidget(sendFreqRadioButton);
        m_sendFreqRadioButtonList.append(sendFreqRadioButton);
    }

    hSendFrequencyLayout->addStretch();
    hSendFrequencyLayout->setSpacing(60);
    hSendFrequencyLayout->setMargin(0);

    m_sendFreqRadioButtonList[0]->setText("首次告警");
    m_sendFreqRadioButtonList[0]->setChecked(true);
    m_sendFreqRadioButtonList[1]->setText("重复告警");
	
	initButtonListWidget();

	QWidget* centerWidget = m_messageSubscriptionAddWidget->getCenterWidget();
	centerWidget->setObjectName("GrayBackWidget");
	QVBoxLayout* vTopLayout = new QVBoxLayout(centerWidget);
	vTopLayout->addLayout(hAlarmLayout);
	vTopLayout->addLayout(hSendTimeLayout);
	vTopLayout->addLayout(hSendFrequencyLayout);
	vTopLayout->addWidget(m_buttonListWidget);
	vTopLayout->setSpacing(10);
	vTopLayout->setContentsMargins(10, 15, 10, 0);
}

void DLWheelMessageSubscriptionAdd::initButtonListWidget()
{
	m_buttonListWidget = new CustomButtonListWidget;
	m_buttonListWidget->addToolButton(0, "配置", ":/Resources/Common/image/Button_Add.png");
	m_buttonListWidget->addWidgetFinished();

    connect(m_buttonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelMessageSubscriptionAdd::onConfigButtonClicked);
}

void DLWheelMessageSubscriptionAdd::initSystemAlarmWidget()
{
	m_systemAlarmBackWidget = new QWidget;

	QWidget* searchBackWidget = new QWidget;
	searchBackWidget->setObjectName("SearchLineEditWidget");
	searchBackWidget->setFixedHeight(30);

	QLineEdit* searchLineEdit = new QLineEdit;
	searchLineEdit->setFixedSize(QSize(230, 20));

	QToolButton* pButtonDeviceTreeSearch = new QToolButton;
	pButtonDeviceTreeSearch->setText("查询");
	pButtonDeviceTreeSearch->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	pButtonDeviceTreeSearch->setFixedSize(QSize(60, 20));
	pButtonDeviceTreeSearch->setIcon(QIcon(":/Resources/Common/image/Search.png"));
	pButtonDeviceTreeSearch->setIconSize(QSize(16, 16));
	pButtonDeviceTreeSearch->setStyleSheet("border:none;");

	QHBoxLayout* hSearchLayout = new QHBoxLayout(searchBackWidget);
	hSearchLayout->addWidget(searchLineEdit);
	hSearchLayout->addWidget(pButtonDeviceTreeSearch);
	hSearchLayout->addStretch();
	hSearchLayout->setMargin(0);
	hSearchLayout->setSpacing(10);

	m_systemAlarmTreeWidget = new QTreeWidget;
	m_systemAlarmTreeWidget->setHeaderHidden(true);
    m_systemAlarmTreeWidget->setStyleSheet("QTreeWidget::item:hover{background:rgb(159,168,218);}QTreeWidget::item:selected{background:rgb(180, 220, 200);}");

	QTreeWidgetItem* rootItem = new QTreeWidgetItem(m_systemAlarmTreeWidget, QStringList() << "系统警告");
	rootItem->setIcon(0, QIcon(":/Resources/Common/image/Folder.png"));

    QList<WeelSystemAlarm> userDataList = WHEEL_DEVICE_CONFIG.getSystemNameAndUUid();
    for (int i = 0; i < userDataList.count(); i++)
    {
        QTreeWidgetItem* childItem = new QTreeWidgetItem(rootItem, QStringList() << userDataList[i].SysAlarmName);
        childItem->setIcon(0, QIcon(":/Resources/Common/image/Folder.png"));
        childItem->setData(0, Qt::UserRole, userDataList[i].SysAlarmUUid);
    }

	QVBoxLayout* vSystemAlarmLayout = new QVBoxLayout(m_systemAlarmBackWidget);
	vSystemAlarmLayout->addWidget(searchBackWidget);
	vSystemAlarmLayout->addWidget(m_systemAlarmTreeWidget);
	vSystemAlarmLayout->setSpacing(0);
	vSystemAlarmLayout->setMargin(0);
}

void DLWheelMessageSubscriptionAdd::initLeftWidget()
{
	initSystemAlarmWidget();

	m_leftBackWidget = new QWidget;
	m_leftBackWidget->setObjectName("LeftBackWidget");
	m_tabWidget = new QTabWidget;
	m_tabWidget->setFixedWidth(500);

	m_deviceAlarmTreeWidget = new CustomTreeWidget;
	m_deviceAlarmTreeWidget->setTitleWidgetVisible(false);
	m_deviceAlarmTreeWidget->setFixedWidth(500);
	m_deviceAlarmTreeWidget->setTreeWidgetType(TreeItemWidgetType::ColorRect_CheckBox_Without);
	m_deviceAlarmTreeWidget->refreshTreeItemList();

	m_tabWidget->insertTab(0, m_deviceAlarmTreeWidget, "设备告警");
	m_tabWidget->insertTab(1, m_systemAlarmBackWidget, "系统告警");

	QHBoxLayout* hLeftLayout = new QHBoxLayout(m_leftBackWidget);
	hLeftLayout->addWidget(m_tabWidget);
	hLeftLayout->setContentsMargins(0, 6, 0, 0);
}

void DLWheelMessageSubscriptionAdd::initMessageReceiverTreeWidget()
{
    m_messageReceiverTreeWidget = new QTreeWidget;
    m_messageReceiverTreeWidget->setHeaderHidden(true);
    m_messageReceiverTreeWidget->setStyleSheet("QTreeWidget::item:hover{background:rgb(240, 250, 240);}QTreeWidget::item:selected{background:rgb(159,168,218);}");

    QTreeWidgetItem* rootItem = new QTreeWidgetItem(m_messageReceiverTreeWidget, QStringList() << "短信接收人");
    rootItem->setIcon(0, QIcon(":/Resources/Common/image/Folder.png"));

    QList<WheelUserConfig> userDataList = WHEEL_STATION_CONFIG.getWheelAllUserConfigData();
    for (int i = 0; i < userDataList.count(); i++)
    {
        QTreeWidgetItem* childItem = new QTreeWidgetItem(rootItem, QStringList() << userDataList[i].user_name);
        childItem->setIcon(0, QIcon(":/Resources/Common/image/Folder.png"));
        childItem->setData(0, Qt::UserRole, userDataList[i].user_uuid);
    }
}

void DLWheelMessageSubscriptionAdd::initRightWidget()
{
    initMessageReceiverTreeWidget();

	m_rightBackWidget = new TitleWidget;
	m_rightBackWidget->setTitleText("短信接收人");

	QHBoxLayout* hRightLayout = new QHBoxLayout(m_rightBackWidget->getCenterWidget());
	hRightLayout->addWidget(m_messageReceiverTreeWidget);
	hRightLayout->setMargin(0);
}

void DLWheelMessageSubscriptionAdd::initWidget()
{
	initTopWidget();
	initLeftWidget();
	initRightWidget();

	QIcon icon = style()->standardIcon(QStyle::SP_TitleBarCloseButton);
	m_pButtonClose = new QToolButton;
	m_pButtonClose->setIcon(icon);
	m_pButtonClose->setIconSize(QSize(12, 12));
	m_pButtonClose->setFixedSize(QSize(14, 14));
	m_pButtonClose->setStyleSheet("border:none;background:rgb(220, 235, 235);");
	connect(m_pButtonClose, &QToolButton::clicked, this, [=] {
		close();
	});

	QHBoxLayout* hTitleLayout = new QHBoxLayout();
	hTitleLayout->addStretch();
	hTitleLayout->addWidget(m_pButtonClose);
	hTitleLayout->setMargin(5);

	QHBoxLayout* hCenterLayout = new QHBoxLayout;
	hCenterLayout->addWidget(m_leftBackWidget);
	hCenterLayout->addWidget(m_rightBackWidget);
	hCenterLayout->setSpacing(0);
	hCenterLayout->setMargin(0);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addLayout(hTitleLayout);
	vMainLayout->addWidget(m_messageSubscriptionAddWidget);
	vMainLayout->addLayout(hCenterLayout);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(10);
}

void DLWheelMessageSubscriptionAdd::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.setPen(Qt::lightGray);
	painter.setBrush(QColor(200, 224, 221));
	painter.drawRoundedRect(QRect(0, 0, this->width() - 1, this->height() - 1), 3, 3);
}

void DLWheelMessageSubscriptionAdd::onConfigButtonClicked()
{
    WheelSubMsgInsert wheelSubMsgInsert;
    QString strReceiverId;
    QTreeWidgetItem* currentTreeWidget = m_messageReceiverTreeWidget->currentItem();
    if (currentTreeWidget == NULL && currentTreeWidget->text(0) != "短信接收人")
    {
        DLMessageBox::showDLMessageBox(NULL, "提示", "请选择接收人账号", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        return;
    }
    // 接收人ID;
    wheelSubMsgInsert.user_receive_uuid = currentTreeWidget->data(0, Qt::UserRole).toString();

    if (m_tabWidget->currentIndex() == 0)
    {
        // 系统告警tab页选择的设备;
        //ItemWidget* itemWidget = m_deviceAlarmTreeWidget->getCurrentSelectedItemWidget();
		QStandardItem *pCurItem = m_deviceAlarmTreeWidget->getCurrentSelectedItem();
        if (pCurItem != NULL)
        {
			RootNodeType nodeType = (RootNodeType)pCurItem->model()->data(pCurItem->index(), ITEM_TYPE).toInt();//itemWidget->getRootNodeType();
			wheelSubMsgInsert.fault_name_uuid = pCurItem->model()->data(pCurItem->index(), ITEM_UUID).toString();
			if (wheelSubMsgInsert.fault_name_uuid.isEmpty())
			{
				wheelSubMsgInsert.fault_name_uuid = m_deviceAlarmTreeWidget->getItemDeviceId(pCurItem->model()->data(pCurItem->index(), Qt::DisplayRole).toString(), nodeType);
			}

            switch (nodeType)
            {
            case Robot_Company:
            case Robot_Station:
                wheelSubMsgInsert.type = WheelDeviceTreePath::Company_And_Station;
                break;
            case RootNode_VoltageLevel:
                wheelSubMsgInsert.type = WheelDeviceTreePath::Voltage_Level;
                break;
            case RootNode_Interval:
                wheelSubMsgInsert.type = WheelDeviceTreePath::Equipment_Interval;
                break;
            case RootNode_DeviceType:
                wheelSubMsgInsert.type = WheelDeviceTreePath::Device_Type;
                break;
            case RootNode_Device:
                wheelSubMsgInsert.type = WheelDeviceTreePath::Device_Point;
                break;
            default:
                break;
            }
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "请选择设备", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return;
        }
    }
    else
    {
        // 系统告警页面;
        QTreeWidgetItem* currentTreeWidget = m_systemAlarmTreeWidget->currentItem();
        if (currentTreeWidget == NULL && currentTreeWidget->text(0) != "系统告警")
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "请选择接收人账号", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return;
        }
        wheelSubMsgInsert.fault_name_uuid = currentTreeWidget->data(0, Qt::UserRole).toString();
        wheelSubMsgInsert.type = WheelDeviceTreePath::Robot_System_Msg;
    }    

    // 告警等级;
    for (int i = 0; i < m_alarmLevelChecoBoxList.count(); i++)
    {
        if (m_alarmLevelChecoBoxList[i]->isChecked())
        {
            wheelSubMsgInsert.alarm_level_id = QString::number(Alarm_Common + i);
            break;
        }
    }
    // 发送时间;
    wheelSubMsgInsert.send_time = "1";
    // 发送频率;
    for (int i = 0; i < m_sendFreqRadioButtonList.count(); i++)
    {
        if (m_sendFreqRadioButtonList[i]->isChecked())
        {
            wheelSubMsgInsert.send_freq_id = QString::number(i + 1);
            break;
        }
    }

    WHEEL_BACK_TO_CORE_SOCKET.robot_insert_note_message_req(wheelSubMsgInsert);
    this->setDisabled(true);

    connect(&m_timeoutTime, &QTimer::timeout, this, [=] {
        DLMessageBox::showDLMessageBox(NULL, "提示", "操作超时", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        this->setDisabled(false);
    });
}