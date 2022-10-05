
/// 机器人控制界面程序

#include "firefightingrobotinformation.h"

FirefightingRobotInformation::FirefightingRobotInformation(QWidget *parent)
	: QWidget(parent)
{
	initWidget();
}

FirefightingRobotInformation::~FirefightingRobotInformation()
{

}

void FirefightingRobotInformation::initWidget()
{
	QWidget* nWidgetInfo = new QWidget;
	nWidgetInfo->setFixedHeight(500);
	nWidgetInfo->setStyleSheet(".QWidget{border: 2px solid rgb(0, 110, 110);border-radius:8px;padding-top:3px;}");
								//QLabel{padding-right:2px; width:200}}");

	int nRowNO = 0;
	QVBoxLayout* nWidgetLayout = new QVBoxLayout;

	/// 显示机器人列表
	QLabel* nlabelRobotNO     = new QLabel("消防机器人：");
	nlabelRobotNO->setAlignment(Qt::AlignRight);
	QComboBox* nComboBoxRobot = new QComboBox;
	QStringList nRobotList;
	nRobotList << "消防机器人1" << "消防机器人2";
	nComboBoxRobot->addItems(nRobotList);
	connect(nComboBoxRobot, &QComboBox::currentTextChanged, this, &FirefightingRobotInformation::slotRobotChangeFunction);
	QHBoxLayout* nLayoutHOne = new QHBoxLayout;
	nLayoutHOne->addWidget(nlabelRobotNO);
	nLayoutHOne->addWidget(nComboBoxRobot);

	/// 显示状态信息
	QLabel* nLabelQuantityOfElectricity = new QLabel("电量:");
	nLabelQuantityOfElectricity->setAlignment(Qt::AlignRight);
	BatteryItem* nProgressBarQuantityE  = new BatteryItem;
	nProgressBarQuantityE->setValue(50);

	QHBoxLayout* nLayoutHOne2 = new QHBoxLayout;
	nLayoutHOne2->addWidget(nLabelQuantityOfElectricity);
	nLayoutHOne2->addWidget(nProgressBarQuantityE);

	nWidgetLayout->addLayout(nLayoutHOne2);

	QLabel* nLabelSignalIntensity = new QLabel("信号强度:");
	nLabelSignalIntensity->setAlignment(Qt::AlignRight);
	QLabel* nLabelSignalIntensityImage = new QLabel;
	Signalsintensity nSignalsintensity;
	nLabelSignalIntensityImage->setPixmap(nSignalsintensity.GetPixmap(50));

	QHBoxLayout* nLayoutHTwo = new QHBoxLayout;
	nLayoutHTwo->addWidget(nLabelSignalIntensity);
	nLayoutHTwo->addWidget(nLabelSignalIntensityImage);

	nWidgetLayout->addLayout(nLayoutHTwo);

	///
	QLabel* nLabelDiskSapce = new QLabel("机器人磁盘空间:");
	nLabelDiskSapce->setAlignment(Qt::AlignRight);
	DiskInfoItem* nLineEditDistSpace = new DiskInfoItem;
	nLineEditDistSpace->setValue(100, 10);

	QHBoxLayout* nLayoutHThree = new QHBoxLayout;
	nLayoutHThree->addWidget(nLabelDiskSapce);
	nLayoutHThree->addWidget(nLineEditDistSpace);

	nWidgetLayout->addLayout(nLayoutHThree);

	QLabel* nLabelVelocity = new QLabel("机器人速度信息:");
	nLabelVelocity->setAlignment(Qt::AlignRight);
	QLabel* nLineEditVelocity = new QLabel("0");

	QHBoxLayout* nLayoutHFor = new QHBoxLayout;
	nLayoutHFor->addWidget(nLabelVelocity);
	nLayoutHFor->addWidget(nLineEditVelocity);

	nWidgetLayout->addLayout(nLayoutHFor);

	///
	QLabel* nLabelHorPositionOfPanTilt = new QLabel("水炮云台水平位置:");
	nLabelHorPositionOfPanTilt->setAlignment(Qt::AlignRight);
	QLabel* nLineEditHorPosition = new QLabel("-1");

	QHBoxLayout* nLayoutHFi = new QHBoxLayout;
	nLayoutHFi->addWidget(nLabelHorPositionOfPanTilt);
	nLayoutHFi->addWidget(nLineEditHorPosition);

	nWidgetLayout->addLayout(nLayoutHFi);

	QLabel* nLabelVelPositionOfPanTilt = new QLabel("水炮云台垂直位置:");
	nLabelVelPositionOfPanTilt->setAlignment(Qt::AlignRight);
	QLabel* nLineEditVelPositionOf = new QLabel("-1");

	QHBoxLayout* nLayoutHSix = new QHBoxLayout;
	nLayoutHSix->addWidget(nLabelVelPositionOfPanTilt);
	nLayoutHSix->addWidget(nLineEditVelPositionOf);

	nWidgetLayout->addLayout(nLayoutHSix);

	///
	QLabel* nLabelCurrentTaskRunTime = new QLabel("当前任务运行时间(s):");
	nLabelCurrentTaskRunTime->setAlignment(Qt::AlignRight);
	QLabel* nLineEditCurrentTaskRunTime = new QLabel("0");

	QHBoxLayout* nLayoutHFiv = new QHBoxLayout;
	nLayoutHFiv->addWidget(nLabelCurrentTaskRunTime);
	nLayoutHFiv->addWidget(nLineEditCurrentTaskRunTime);

	nWidgetLayout->addLayout(nLayoutHFiv);

	QLabel* nLabelSpacer = new QLabel;
	nLabelSpacer->setFixedHeight(250);
	nWidgetLayout->addWidget(nLabelSpacer);

	nWidgetInfo->setLayout(nWidgetLayout);

	QLabel* nLabelFireInfo = new QLabel("消防机器人信息");
	nLabelFireInfo->setAlignment(Qt::AlignCenter);

	QGridLayout* nGridLayout = new QGridLayout;
	nGridLayout->addWidget(nLabelFireInfo);
	nGridLayout->addWidget(nWidgetInfo);

	this->setLayout(nGridLayout);
#if 0
	NControlBackWidget * m_bodyControlWidget = new NControlBackWidget;
	m_bodyControlWidget->setTitleText("消防机器人车体控制");
	m_bodyControlWidget->setFixedHeight(250);

	NCustomButton* m_bodyControlButton = new NCustomButton(NWheel_RobotControl_BodyMove);
	m_bodyControlButton->setRadiusValue(65);
	m_bodyControlButton->setArcLength(40);
	m_bodyControlButton->setArcAngle(45, 89.8);

	connect(m_bodyControlButton, &NCustomButton::signalButtonPressed, this, &FirefightingRobotInformation::onRobotBodyControlPressed);
	connect(m_bodyControlButton, &NCustomButton::signalButtonRelease, this, &FirefightingRobotInformation::onRobotBodyControlReleased);

	m_bodyControlWidget->setCenterWidget(m_bodyControlButton);


	NControlBackWidget* m_ptzControlWidget = new NControlBackWidget;
	m_ptzControlWidget->setTitleText("消防机器人水炮云台控制");
	m_ptzControlWidget->setFixedHeight(250);

	NCustomButton* m_ptzControlButton = new NCustomButton(NWheel_RobotControl_PTZMove);
	m_ptzControlButton->setRadiusValue(65);
	m_ptzControlButton->setArcLength(40);
	m_ptzControlButton->setArcAngle(45, 89.8);

	connect(m_ptzControlButton, &NCustomButton::signalButtonPressed, this, &FirefightingRobotInformation::onRobotPTZControlPressed);
	connect(m_ptzControlButton, &NCustomButton::signalButtonRelease, this, &FirefightingRobotInformation::onRobotPTZControlReleased);

	m_ptzControlWidget->setCenterWidget(m_ptzControlButton);

	nGridLayout->addWidget(m_bodyControlWidget, iCurrRowNO, 0, 1, 2);
	nGridLayout->addWidget(m_ptzControlWidget, iCurrRowNO, 2, 1, 2);
	iCurrRowNO ++;
#endif
}

/// 切换控制消防机器人
void FirefightingRobotInformation::slotRobotChangeFunction(QString text)
{
	qDebug() << "text is:" << text;
}

/// 消防机器人控制区域鼠标左键点击事件
void FirefightingRobotInformation::onRobotBodyControlPressed(int buttonId)
{
	switch (buttonId)
	{
	case 0:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Up, false);
		break;
	case 1:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Left, false);
		break;
	case 2:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Down, false);
		break;
	case 3:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Right, false);
		break;
	case 4:
		//m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomOut);
		break;
	case 5:
		//m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomIn);
		break;
	default:
		break;
	}
}

/// 消防机器人控制区域鼠标左键放开事件
void FirefightingRobotInformation::onRobotBodyControlReleased(int buttonId)
{
	switch (buttonId)
	{
	case 0:
		//m_robotControlThread->setCurrentOperationType(Robot_Move_Head, false);
		break;
	case 1:
		//m_robotControlThread->setCurrentOperationType(Robot_Move_Left, false);
		break;
	case 2:
		//m_robotControlThread->setCurrentOperationType(Robot_Move_Tail, false);
		break;
	case 3:
		//m_robotControlThread->setCurrentOperationType(Robot_Move_Right, false);
		break;
	case 4:
		break;
	default:
		break;
	}
}

/// 消防机器人云台控制区域鼠标左键点击事件
void FirefightingRobotInformation::onRobotPTZControlPressed(int buttonId)
{
	switch (buttonId)
	{
	case 0:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Up, false);
		break;
	case 1:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Left, false);
		break;
	case 2:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Down, false);
		break;
	case 3:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Right, false);
		break;
	case 4:
		//m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomOut);
		break;
	case 5:
		//m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomIn);
		break;
	default:
		break;
	}
}

/// 消防机器人云台控制区域鼠标左键放开事件
void FirefightingRobotInformation::onRobotPTZControlReleased(int buttonId)
{
	qDebug() << "buttonId = " << buttonId;
	switch (buttonId)
	{
	case 0:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Up, true);
		break;
	case 1:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Left, true);
		break;
	case 2:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Down, true);
		break;
	case 3:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Right, true);
		break;
	case 4:
		//m_visibleVideoWidget->onButtonReleased(VideoButtonType::ZoomOut);
		break;
	case 5:
		//m_visibleVideoWidget->onButtonReleased(VideoButtonType::ZoomIn);
		break;
	default:
		break;
	}
}


void FirefightingRobotInformation::updateParameInformation()
{

}