#ifndef DL_WHEEL_ROBOT_SET_H
#define DL_WHEEL_ROBOT_SET_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>

class BorderWidget;
class InputWidget;
class SwitchWidget;

/********机器人设置页面********/

class DLWheelRobotSet : public QWidget
{
	Q_OBJECT

public:
	DLWheelRobotSet(QWidget* parent = NULL);
	
	// 窗口控件初始化;
	void initWidget();

signals:
    // 向主界面发送操作信息;
    void signalSendOperateMsg(QString);

    // 红外显示开关;
    void signalInfraredSwitch(bool isOpen);

    // 可见光显示开关;
    void signalVisibleSwitch(bool isOpen);

private:
    // 窗口各个控件初始化;
	void initRobotAlarmSet();
	void initRobotPtzControl();
	void initLeftWidget();
	void initRightWidget();

private:
	QWidget* m_leftBackWidget;
	BorderWidget* m_alarmSetWidget;
	BorderWidget* m_ptzControlWidget;

	InputWidget* m_alarmExecuteMechanism;
	InputWidget* m_abortExecuteMechanism;

	InputWidget* m_robotMoveSpeed;
	InputWidget* m_radarAlarmDistance;
    InputWidget* m_batteryUpperlimit;

	InputWidget* m_ptzXValye;
	InputWidget* m_ptzYValue;

	InputWidget* m_ptzHorizontalOffset;
	InputWidget* m_ptzVerticalOffset;


	QWidget* m_rightBackWidget;
	InputWidget* m_controlMode;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 轮子直径;
    InputWidget* m_wheelDiameter;
    // 轮子到车中心的距离;
    InputWidget* m_wheelToCeneterDistance;
};

#endif
