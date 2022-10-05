#include "InspectionRobotControlClass.h"

InspectionRobotControlClass::InspectionRobotControlClass(QWidget* parent)
	:QWidget(parent), m_ptzControlWidgetFire(NULL),m_ptzControlWidget(NULL), m_bodyControlWidget(NULL)
{
	initWodget();
}

InspectionRobotControlClass::~InspectionRobotControlClass()
{
}

void InspectionRobotControlClass::initWodget()
{
	int nRowCount = 0;
	QVBoxLayout* nWidgetLayout = new QVBoxLayout;

	QLabel* nLabelRobotInfo = new QLabel("��������Ϣ");
	nLabelRobotInfo->setAlignment(Qt::AlignCenter);

	QWidget* nWidgetInfo = new QWidget;
	nWidgetInfo->setFixedHeight(500);
	nWidgetInfo->setStyleSheet(".QWidget{border: 2px solid rgb(0, 110, 110);border-radius:8px;padding-top:3px;}");
								//QLabel{padding-right:2px; width:200}");

	/// ��ʾ״̬��Ϣ
	QLabel* nLabelQuantityOfElectricity = new QLabel("����:");
	nLabelQuantityOfElectricity->setAlignment(Qt::AlignRight);
	BatteryItem* nProgressBarQuantityE = new BatteryItem;
	nProgressBarQuantityE->setValue(10);
	QHBoxLayout* nLayoutHOne = new QHBoxLayout;
	nLayoutHOne->addWidget(nLabelQuantityOfElectricity);
	nLayoutHOne->addWidget(nProgressBarQuantityE);

	nWidgetLayout->addLayout(nLayoutHOne);

	QLabel* nLabelSignalIntensity = new QLabel("�ź�ǿ��:");
	nLabelSignalIntensity->setAlignment(Qt::AlignRight);
	QLabel* nLabelSignalIntensityImage = new QLabel;
	Signalsintensity nSignalsIntensity;
	nLabelSignalIntensityImage->setPixmap(nSignalsIntensity.GetPixmap(70));
	QHBoxLayout* nLayoutHTwo = new QHBoxLayout;
	nLayoutHTwo->addWidget(nLabelSignalIntensity);
	nLayoutHTwo->addWidget(nLabelSignalIntensityImage);

	nWidgetLayout->addLayout(nLayoutHTwo);

	///
	QLabel* nLabelDiskSapce = new QLabel("�����˴��̿ռ�:");
	nLabelDiskSapce->setAlignment(Qt::AlignRight);
	DiskInfoItem* nLineEditDistSpace = new DiskInfoItem;
	nLineEditDistSpace->setValue(100, 92);
	QHBoxLayout* nLayoutHThree = new QHBoxLayout;
	nLayoutHThree->addWidget(nLabelDiskSapce);
	nLayoutHThree->addWidget(nLineEditDistSpace);

	nWidgetLayout->addLayout(nLayoutHThree);

	QLabel* nLabelVelocity = new QLabel("�������ٶ���Ϣ(s):");
	nLabelVelocity->setAlignment(Qt::AlignRight);
	QLabel* nLineEditVelocity = new QLabel("0");
	QHBoxLayout* nLayoutHFo = new QHBoxLayout;
	nLayoutHFo->addWidget(nLabelVelocity);
	nLayoutHFo->addWidget(nLineEditVelocity);

	nWidgetLayout->addLayout(nLayoutHFo);

	///
	QLabel* nLabelHorPositionOfPanTilt = new QLabel("��̨ˮƽλ��:");
	nLabelHorPositionOfPanTilt->setAlignment(Qt::AlignRight);
	QLabel* nLineEditHorPosition = new QLabel("-1");
	QHBoxLayout* nLayoutHFi = new QHBoxLayout;
	nLayoutHFi->addWidget(nLabelHorPositionOfPanTilt);
	nLayoutHFi->addWidget(nLineEditHorPosition);

	nWidgetLayout->addLayout(nLayoutHFi);

	QLabel* nLabelVelPositionOfPanTilt = new QLabel("��̨��ֱλ��:");
	nLabelVelPositionOfPanTilt->setAlignment(Qt::AlignRight);
	QLabel* nLineEditVelPositionOf = new QLabel("-1");
	QHBoxLayout* nLayoutHSix = new QHBoxLayout;
	nLayoutHSix->addWidget(nLabelVelPositionOfPanTilt);
	nLayoutHSix->addWidget(nLineEditVelPositionOf);

	nWidgetLayout->addLayout(nLayoutHSix);

	///
	m_infraredFunction = new NSwitchWidget(true);
	m_infraredFunction->setSwitchName("���޿���");
	m_infraredFunction->setSwitchState(true);
	//connect(m_infraredFunction, &NSwitchWidget::toggled, this, [=](bool isCheck) {
	//	emit signalInfraredSwitch(isCheck);
	//});
	nWidgetLayout->addWidget(m_infraredFunction);

	QLabel* nLabelFireValvePressure = new QLabel("���޷���ѹǿ(Pa):");
	nLabelFireValvePressure->setAlignment(Qt::AlignRight);
	QLabel* nLineEditFireValvePressure = new QLabel("0");
	QHBoxLayout* nLayoutHSev = new QHBoxLayout;
	nLayoutHSev->addWidget(nLabelFireValvePressure);
	nLayoutHSev->addWidget(nLineEditFireValvePressure);

	nWidgetLayout->addLayout(nLayoutHSev);

	///
	QLabel* nLabelCurrentTaskRunTime = new QLabel("��ǰ��������ʱ��(s):");
	nLabelCurrentTaskRunTime->setAlignment(Qt::AlignRight);
	QLabel* nLineEditCurrentTaskRunTime = new QLabel("0");
	QHBoxLayout* nLayoutHEt = new QHBoxLayout;
	nLayoutHEt->addWidget(nLabelCurrentTaskRunTime);
	nLayoutHEt->addWidget(nLineEditCurrentTaskRunTime);

	nWidgetLayout->addLayout(nLayoutHEt);

	QLabel* nLabelSpacer = new QLabel;
	nLabelSpacer->setFixedHeight(200);
	nWidgetLayout->addWidget(nLabelSpacer);

	//nGridLayout->addWidget(nLabelCurrentTaskRunTime, nRowCount, 0);
	//nGridLayout->addWidget(nLineEditCurrentTaskRunTime, nRowCount, 1);
	//nRowCount++;

	nWidgetInfo->setLayout(nWidgetLayout);

	
	QGridLayout* nGridLayout = new QGridLayout;
	nGridLayout->addWidget(nLabelRobotInfo);
	nGridLayout->addWidget(nWidgetInfo);

	this->setLayout(nGridLayout);

#if 0
	/// ��ӿ��ƽ���
	/// ���һ������˻����˿���
	m_bodyControlWidget = new NControlBackWidget;
	m_bodyControlWidget->setTitleText("���һ������˿���");

	m_bodyControlButton = new NCustomButton(NWheel_RobotControl_BodyMove);
	m_bodyControlButton->setRadiusValue(65);
	m_bodyControlButton->setArcLength(40);
	m_bodyControlButton->setArcAngle(45, 89.8); 

	connect(m_bodyControlButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotBodyControlPressed(int)));
	connect(m_bodyControlButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotBodyControlReleased(int)));

	m_bodyControlWidget->setCenterWidget(m_bodyControlButton);

	/// ˫����̨����
	m_ptzControlWidget = new NControlBackWidget;
	m_ptzControlWidget->setTitleText("˫����̨����");
	m_ptzControlWidget->setFixedHeight(250);

	m_ptzControlButton = new NCustomButton(NWheel_RobotControl_PTZMove);
	m_ptzControlButton->setRadiusValue(65);
	m_ptzControlButton->setArcLength(40);
	m_ptzControlButton->setArcAngle(45, 89.8);

	connect(m_ptzControlButton, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotPTZControlPressed(int)));
	connect(m_ptzControlButton, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotPTZControlReleased(int)));

	m_ptzControlWidget->setCenterWidget(m_ptzControlButton);

	/// �����̨����
	m_ptzControlWidgetFire = new NControlBackWidget;
	m_ptzControlWidgetFire->setTitleText("�����̨����");
	m_ptzControlWidgetFire->setFixedHeight(250);

	
	m_ptzControlButtonFire = new NCustomButton(NWheel_RobotControl_PTZMove);
	m_ptzControlButtonFire->setRadiusValue(65);
	m_ptzControlButtonFire->setArcLength(40);
	m_ptzControlButtonFire->setArcAngle(45, 89.8);

	//connect(m_ptzControlButtonFire, SIGNAL(signalButtonPressed(int)), this, SLOT(onRobotFirePTZControlPressed(int)));
	//connect(m_ptzControlButtonFire, SIGNAL(signalButtonRelease(int)), this, SLOT(onRobotFirePTZControlReleased(int)));

	m_ptzControlWidgetFire->setCenterWidget(m_ptzControlButtonFire);

	/// ���������ƿؼ�,��������һ��
	QHBoxLayout* nLayoutControl = new QHBoxLayout;
	nLayoutControl->addWidget(m_bodyControlWidget);
	nLayoutControl->addWidget(m_ptzControlWidget);
	nLayoutControl->addWidget(m_ptzControlWidgetFire);

	nGridLayout->addLayout(nLayoutControl, nRowCount, 0, 1, 4);
	nRowCount++;
#endif
}

/// ���һ������˿�����������������¼�
void InspectionRobotControlClass::onRobotBodyControlPressed(int buttonId)
{
	switch (buttonId)
	{
	case 0:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Up, false);	///<��������ǰ��
		break;
	case 1:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Left, false);	///<��ת
		break;
	case 2:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Down, false);	///<����
		break;
	case 3:
		//m_robotControlThread->setCurrentOperationType(PTZ_Move_Right, false);	///<��ת
		break;
	case 4:
		//m_visibleVideoWidget->onButtonPressed(VideoButtonType::ZoomOut);		///<��ͣ
		break;
	default:
		break;
	}
}

/// ���һ������˿��������������ſ��¼�
void InspectionRobotControlClass::onRobotBodyControlReleased(int buttonId)
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

/// ���һ���������̨������������������¼�
void InspectionRobotControlClass::onRobotPTZControlPressed(int buttonId)
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

/// ���һ���������̨���������������ſ��¼�
void InspectionRobotControlClass::onRobotPTZControlReleased(int buttonId)
{
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

/// ���һ������������̨������������������¼�
void InspectionRobotControlClass::onRobotFirePTZControlPressed(int buttonId)
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

/// ���һ������������̨���������������ſ��¼�
void InspectionRobotControlClass::onRobotFirePTZControlReleased(int buttonId)
{
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

void InspectionRobotControlClass::updateParameInformation()
{
}

/// ��ȡ��̨λ����Ϣ
void InspectionRobotControlClass::setInfraredFocus(int)
{
}

/// ��ȡ������Ϣ
void InspectionRobotControlClass::setQuantityOfExectricity(int pQuantityOfExectricity)
{
	m_quantityOfExectricity = pQuantityOfExectricity;
}