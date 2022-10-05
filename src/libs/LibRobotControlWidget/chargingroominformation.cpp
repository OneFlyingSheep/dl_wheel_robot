
/// 机器人控制界面程序

#include "chargingroominformation.h"

ChargingRoomInformation::ChargingRoomInformation(QWidget *parent)
	: QWidget(parent), m_switchControl(NULL), m_signalStrength(10), m_chargingSwitchState(true), m_workingState(0), nLabelSignalIntensityInfo(NULL), nLineEditWeatherInfo(NULL)
{
	initWidget();
}

ChargingRoomInformation::~ChargingRoomInformation()
{
}

void ChargingRoomInformation::initWidget() 
{
	QWidget* nWidgetInfo = new QWidget;
	nWidgetInfo->setStyleSheet(".QWidget{border: 2px solid rgb(0, 110, 110);border-radius:8px;padding-top:3px;}");
								//QLabel{padding-right:2px; width:200}");

	///状态
	int nRowNO = 0;

	QGridLayout* nWidgetLayout = new QGridLayout;

	QLabel* nLabelSignalIntensity = new QLabel("信号强度:");
	nLabelSignalIntensity->setAlignment(Qt::AlignRight);
	nLabelSignalIntensityInfo = new QLabel;
	Signalsintensity nSignalInt;
	nLabelSignalIntensityInfo->setPixmap(nSignalInt.GetPixmap(10));

	nWidgetLayout->addWidget(nLabelSignalIntensity, nRowNO, 0, 1, 1);
	nWidgetLayout->addWidget(nLabelSignalIntensityInfo, nRowNO, 1, 1, 1);
	nRowNO++;

	///
	QLabel*	nLabelWeatherInfo = new QLabel("状态:");
	nLabelWeatherInfo->setAlignment(Qt::AlignRight);
	nLineEditWeatherInfo = new QLabel("空闲");

	nWidgetLayout->addWidget(nLabelWeatherInfo, nRowNO, 0, 1, 1);
	nWidgetLayout->addWidget(nLineEditWeatherInfo, nRowNO, 1, 1, 1);
	nRowNO++;

	///控制
	m_switchControl = new NSwitchWidget(true);
	m_switchControl->setSwitchName("充电开关");
	m_switchControl->setSwitchState(m_chargingSwitchState);
	//connect(m_switchControl, &NSwitchWidget::toggled, this, [=](bool isCheck) {
	//	emit signalInfraredSwitch(isCheck);
	//});
	nWidgetLayout->addWidget(m_switchControl, nRowNO, 0, 1, 2);

	nWidgetInfo->setLayout(nWidgetLayout);

	QLabel* nLayoutChargingRoomInfo = new QLabel("充电房状态信息");
	nLayoutChargingRoomInfo->setAlignment(Qt::AlignCenter);

	QGridLayout* nMainLayout = new QGridLayout;
	nMainLayout->addWidget(nLayoutChargingRoomInfo, 0, 0);
	nMainLayout->addWidget(nWidgetInfo, 1, 0);

	this->setLayout(nMainLayout);
}

/// 修改当前界面参数
void ChargingRoomInformation::updateParameInformation()
{
	if (NULL != nLabelSignalIntensityInfo)
	{
		Signalsintensity nSignalInt;
		nLabelSignalIntensityInfo->setPixmap(nSignalInt.GetPixmap(m_signalStrength));
	}
	if (0 == m_workingState)
	{
		nLineEditWeatherInfo->setText("空闲");
	}
	else
	{
		nLineEditWeatherInfo->setText("充电中...");
	}
	m_switchControl->setSwitchState(m_chargingSwitchState);
}

void ChargingRoomInformation::setSignalStrength(int nSignalStrength)			///< 充电房信号强度
{
	m_signalStrength = nSignalStrength;
}

void ChargingRoomInformation::setChargingSwitchState(bool nChargingSwitchState)	///< 充电开关状态
{
	m_chargingSwitchState = nChargingSwitchState;
}

void ChargingRoomInformation::setWorkingStateInformation(int nWorkingState)		///< 充电房工作状态
{
	m_workingState = nWorkingState;
}
