#pragma once

#include <QtWidgets/QWidget>

enum CustomButtonType
{
	VideoControl = 0,							// ��Ƶ����;
	VideoParamSet,								// ��Ƶ��������;
	VideoParamSet_NoReset,						// ��Ƶ��������(�޸�λ��ť);
	VideoDirectionControl,						// ��Ƶ��������;
	RobotControl_BodyMove,						// �����������ƶ�;
	RobotControl_BodyMove_Big,					// �����������ƶ�_��Ƶ������;
	RobotControl_VideoMove,						// �ɼ����ƶ�;
	Wheel_RobotControl_BodyMove,				// ��ʽ�����˳����ƶ�;
	Wheel_RobotControl_PTZMove,					// ��ʽ������˫����̨�ƶ�;
	Wheel_RobotControl_FireFilht,				// ��ʽ�����������̨�ƶ�
    Wheel_RobotControl_SmallBodyMove,
};

class CustomButton : public QWidget
{
	Q_OBJECT

public:
	CustomButton(CustomButtonType cusButtonType, QWidget *parent = nullptr);

	// ����Բ�����뾶��С;
	void setRadiusValue(int radius);
	void setArcLength(int arcLength);
	// ����Բ���Ƕ�;
	void setArcAngle(qreal startAngle, qreal angleLength, int buttonCount = 4, QColor color = QColor(33, 150, 243));
private:
	// ��Ƶ���Ƴ�ʼ��;
	void initVideoControl();
	// ��Ƶ�������ó�ʼ��;
	void initVideoParamSet();
	// ��Ƶ�������ó�ʼ��(�޸�λ��ť);
	void initVideoParamSetNoReset();
	// ��Ƶ������Ƴ�ʼ��;
	void initVideoDirectionControl();
	// �������ƶ���ʼ��;
	void initRobotBodyMove();
	// �������ƶ���ʼ��;
	void initRobotBodyMove_Big();
	// �ɼ����ƶ�;
	void initRobotVidoeMove();
	// ��ʽ�����˳����ƶ�;
    void initWheelRobotBodyControl();
    void initWheelRobotBodyControlSmall();
	// ��ʽ��������̨�ƶ�;
	void initWheelRobotPTZControl();

	// �Ի�;
	void paintEvent(QPaintEvent *);
	// ���Բ��;
	void addArc(qreal startAngle, qreal angleLength, QColor color);
	// ����¼�;
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

	// ����뿪�¼�;
	void leaveEvent(QEvent *event);

signals:
	void signalButtonPressed(int buttonId);
	void signalButtonRelease(int buttonId);

private:
	// Բ�����뾶��С;
	int m_radius, m_arcLength;
	// Բ��·��;
	QList<QPainterPath> m_pathList;
	// Բ��������path;
	QList<QPainterPath> m_textPathList;
	// ��ɫ�б�;
	QList<QBrush> m_colorList;
	// ����Բ��ͳһ��ɫ;
	QColor m_arcColor;
	// ��ǰ����/���밴ť������;
	int m_pressIndex, m_enterIndex;
	// ��ǰ����Ƿ��°�ť;
	bool m_isMousePressed;
	// ��ǰ����Ƿ���밴ť;
	bool m_isMouseEntered;
	// ��ť����;
	CustomButtonType m_customButtonType;
};
