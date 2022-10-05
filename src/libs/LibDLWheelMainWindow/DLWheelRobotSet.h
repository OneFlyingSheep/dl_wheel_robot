#ifndef DL_WHEEL_ROBOT_SET_H
#define DL_WHEEL_ROBOT_SET_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>

class BorderWidget;
class InputWidget;
class SwitchWidget;

/********����������ҳ��********/

class DLWheelRobotSet : public QWidget
{
	Q_OBJECT

public:
	DLWheelRobotSet(QWidget* parent = NULL);
	
	// ���ڿؼ���ʼ��;
	void initWidget();

signals:
    // �������淢�Ͳ�����Ϣ;
    void signalSendOperateMsg(QString);

    // ������ʾ����;
    void signalInfraredSwitch(bool isOpen);

    // �ɼ�����ʾ����;
    void signalVisibleSwitch(bool isOpen);

private:
    // ���ڸ����ؼ���ʼ��;
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

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ����ֱ��;
    InputWidget* m_wheelDiameter;
    // ���ӵ������ĵľ���;
    InputWidget* m_wheelToCeneterDistance;
};

#endif
