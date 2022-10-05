#pragma once

/// 查打一体机器人信息显示以及控制界面

#include "BatteryItem.h"
#include "nswitchcontrol.h"
#include "Signalsintensity.h"

#include <QWidget>
#include <QLabel>
#include <QDebug>
#include <QLineEdit>
#include <QProgressBar>
#include <QVBoxLayout>
#include <QHBoxLayout>

#pragma execution_character_set("utf-8")

#include "ncontrolBackWidget.h"
#include "ncustomButton.h"
#include "diskInfoItem.h"

/// 巡检机器人控制类

class InspectionRobotControlClass :
	public QWidget
{
	Q_OBJECT

public:
	InspectionRobotControlClass(QWidget *parent = NULL);
	~InspectionRobotControlClass();

	/// 设置界面信息函数
	void updateParameInformation();

	void setInfraredFocus(int);
	void setQuantityOfExectricity(int);

public slots:
	void onRobotBodyControlPressed(int);	///<车体控制
	void onRobotBodyControlReleased(int);
	void onRobotPTZControlPressed(int);		///<双光云台控制
	void onRobotPTZControlReleased(int);
	void onRobotFirePTZControlPressed(int);	///<灭火云台控制
	void onRobotFirePTZControlReleased(int);

private:
	void initWodget();

	int m_quantityOfExectricity;///< 电量信息


	NCustomButton*	   m_ptzControlButtonFire;
	NCustomButton*	   m_ptzControlButton;
	NCustomButton*	   m_bodyControlButton;

	NControlBackWidget* m_ptzControlWidgetFire;
	NControlBackWidget* m_ptzControlWidget;	///< 双光云台控制
	NControlBackWidget* m_bodyControlWidget; ///< 查打一体机器人控制

	NSwitchWidget* m_infraredFunction;
};

