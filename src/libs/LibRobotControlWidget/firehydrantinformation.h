#pragma once
/// 消防栓信息展示界面

#include <stdio.h>
#include <atlbase.h>
#include <atlcom.h>
#include <sapi.h>
#include <QWidget>
#include <QString>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QGridLayout>
#include <QStringList>

#include "nswitchcontrol.h"
#include "Signalsintensity.h"

#pragma execution_character_set("utf-8")


class FireHydrantInformation : public QWidget
{
	Q_OBJECT

public:
	FireHydrantInformation(QWidget *parent = NULL);
	~FireHydrantInformation();

	void initWidget();
	void updateParameInformation();					///< 修改界面参数信息
	void setValveSwitchInformation(bool pSwitch);	///< 阀门开关状态
	void setValvePressureInformation(int Pressure); ///< 阀门压强

private:
	NSwitchWidget* m_infraredFunction;
	QLabel* nLineEditNetWork;
	bool m_valveSwitchInfo;
	int  m_valvePressureInfo;
};
