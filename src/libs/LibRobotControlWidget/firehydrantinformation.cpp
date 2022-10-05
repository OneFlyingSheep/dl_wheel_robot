
/// 机器人控制界面程序

#include "firehydrantinformation.h"

FireHydrantInformation::FireHydrantInformation(QWidget *parent)
	: QWidget(parent), m_infraredFunction(NULL), nLineEditNetWork(NULL), m_valveSwitchInfo(true), m_valvePressureInfo(0)
{
	initWidget();
}

FireHydrantInformation::~FireHydrantInformation()
{

}

void FireHydrantInformation::initWidget()
{
	QWidget* nWidgetInfo = new QWidget;
	nWidgetInfo->setStyleSheet(".QWidget{border: 2px solid rgb(0, 110, 110);border-radius:8px;padding-top:3px;}");

	QVBoxLayout* nWidgetLayout = new QVBoxLayout;

	///add 对接装置信号强度
	QLabel* nLabelSignalIntensity = new QLabel("对接装置信号强度");
	nLabelSignalIntensity->setAlignment(Qt::AlignRight);
	QLabel* nLineEditNetWorkInt = new QLabel;
	Signalsintensity nSignalsIntensity;
	nLineEditNetWorkInt->setPixmap(nSignalsIntensity.GetPixmap(10));
	QHBoxLayout* nLayoutOne = new QHBoxLayout;
	nLayoutOne->addWidget(nLabelSignalIntensity);
	nLayoutOne->addWidget(nLineEditNetWorkInt);

	QLabel* nLabelNetWork = new QLabel("阀门压强(Pa):");
	nLabelNetWork->setAlignment(Qt::AlignRight);
	nLineEditNetWork = new QLabel("0");

	QHBoxLayout* nLayoutTwo = new QHBoxLayout;
	nLayoutTwo->addWidget(nLabelNetWork);
	nLayoutTwo->addWidget(nLineEditNetWork);

	m_infraredFunction = new NSwitchWidget(true); 
	m_infraredFunction->setSwitchName("阀门开关"); 
	m_infraredFunction->setSwitchState(m_valveSwitchInfo);
	//connect(m_infraredFunction, &NSwitchWidget::toggled, this, [=](bool isCheck) {
	//	emit signalInfraredSwitch(isCheck);
	//});

	nWidgetLayout->addLayout(nLayoutOne);
	nWidgetLayout->addLayout(nLayoutTwo);
	nWidgetLayout->addWidget(m_infraredFunction);

	nWidgetInfo->setLayout(nWidgetLayout);

	QLabel* nLabelFireHydrantInfo = new QLabel("消防栓信息");
	nLabelFireHydrantInfo->setAlignment(Qt::AlignCenter);

	QGridLayout* nMainLayout = new QGridLayout;
	nMainLayout->addWidget(nLabelFireHydrantInfo);
	nMainLayout->addWidget(nWidgetInfo);

	this->setLayout(nMainLayout);
}

/// 修改界面参数信息
void FireHydrantInformation::updateParameInformation()
{
	if (NULL != m_infraredFunction)
	{
		m_infraredFunction->setSwitchState(m_valveSwitchInfo);
	}
	if (NULL != nLineEditNetWork)
	{
		nLineEditNetWork->setText(QString::number(m_valvePressureInfo));
	}
}


void FireHydrantInformation::setValveSwitchInformation(bool pSwitch)	///< 阀门开关状态
{	
	m_valveSwitchInfo = pSwitch;
}

void FireHydrantInformation::setValvePressureInformation(int Pressure) ///< 阀门压强
{
	m_valvePressureInfo = Pressure;
}

