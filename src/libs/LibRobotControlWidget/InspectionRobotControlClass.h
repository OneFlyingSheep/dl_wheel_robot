#pragma once

/// ���һ���������Ϣ��ʾ�Լ����ƽ���

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

/// Ѳ������˿�����

class InspectionRobotControlClass :
	public QWidget
{
	Q_OBJECT

public:
	InspectionRobotControlClass(QWidget *parent = NULL);
	~InspectionRobotControlClass();

	/// ���ý�����Ϣ����
	void updateParameInformation();

	void setInfraredFocus(int);
	void setQuantityOfExectricity(int);

public slots:
	void onRobotBodyControlPressed(int);	///<�������
	void onRobotBodyControlReleased(int);
	void onRobotPTZControlPressed(int);		///<˫����̨����
	void onRobotPTZControlReleased(int);
	void onRobotFirePTZControlPressed(int);	///<�����̨����
	void onRobotFirePTZControlReleased(int);

private:
	void initWodget();

	int m_quantityOfExectricity;///< ������Ϣ


	NCustomButton*	   m_ptzControlButtonFire;
	NCustomButton*	   m_ptzControlButton;
	NCustomButton*	   m_bodyControlButton;

	NControlBackWidget* m_ptzControlWidgetFire;
	NControlBackWidget* m_ptzControlWidget;	///< ˫����̨����
	NControlBackWidget* m_bodyControlWidget; ///< ���һ������˿���

	NSwitchWidget* m_infraredFunction;
};

