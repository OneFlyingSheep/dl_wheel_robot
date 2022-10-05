#pragma once
/// 充电放信息显示界面

#include <stdio.h>
#include <atlbase.h>
#include <atlcom.h>
#include <sapi.h>
#include <QWidget>
#include <QString>
#include <QStringList>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>

#include "Signalsintensity.h"
#include "nswitchcontrol.h"

#pragma execution_character_set("utf-8")


class ChargingRoomInformation 
	: public QWidget
{
	Q_OBJECT

public:
	ChargingRoomInformation(QWidget *parent = NULL);
	~ChargingRoomInformation();

	void initWidget();

	void updateParameInformation();
	void setSignalStrength(int nSignalStrength);			///< 充电房信号强度
	void setChargingSwitchState(bool nChargingSwitchState);	///< 充电开关状态
	void setWorkingStateInformation(int nWorkingState);		///< 充电房工作状态

private:
	NSwitchWidget* m_switchControl;
	int m_signalStrength;
	bool m_chargingSwitchState;
	int m_workingState;

	QLabel* nLabelSignalIntensityInfo;
	QLabel* nLineEditWeatherInfo;
};
