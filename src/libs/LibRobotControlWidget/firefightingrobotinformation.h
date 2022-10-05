#pragma once

/// 消防机器人状态显示以及控制界面

#include "Signalsintensity.h"
#include "BatteryItem.h"
#include "ncustomButton.h"
#include "ncontrolBackWidget.h"
#include "diskInfoItem.h"

#include <stdio.h>
#include <atlbase.h>
#include <atlcom.h>
#include <sapi.h>
#include <QDebug>
#include <QWidget>
#include <QString>
#include <QStringList>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QGridLayout>

#pragma execution_character_set("utf-8")


class FirefightingRobotInformation : public QWidget
{
	Q_OBJECT

public:
	FirefightingRobotInformation(QWidget *parent = NULL);
	~FirefightingRobotInformation();

	void initWidget();

	void updateParameInformation();

private slots:
	void slotRobotChangeFunction(QString text);		///< 切换消防机器人
	void onRobotBodyControlPressed(int buttonId);	///< 消防机器人控制界面鼠标左键点击事件
	void onRobotBodyControlReleased(int buttonId);	///< 消防机器人控制界面鼠标左键释放事件
	void onRobotPTZControlPressed(int buttonId);	///< 消防机器人云台控制界面鼠标左键点击事件
	void onRobotPTZControlReleased(int buttonId);	///< 消防机器人云台控制界面鼠标左键释放事件

private:
};
