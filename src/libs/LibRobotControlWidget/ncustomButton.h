#pragma once

#include <QtWidgets/QWidget>
#include <QDebug>

enum NCustomButtonType
{
	NVideoControl = 0,							// ��Ƶ����;
	NVideoParamSet,								// ��Ƶ��������;
	NVideoParamSet_NoReset,						// ��Ƶ��������(�޸�λ��ť);
	NVideoDirectionControl,						// ��Ƶ��������;
	NRobotControl_BodyMove,						// �����������ƶ�;
	NRobotControl_BodyMove_Big,					// �����������ƶ�_��Ƶ������;
	NRobotControl_VideoMove,					// �ɼ����ƶ�;
	NWheel_RobotControl_BodyMove,				// ��ʽ�����˳����ƶ�;
	NWheel_RobotControl_PTZMove,				// ��ʽ��������̨�ƶ�;
};

class NCustomButton : public QWidget
{
	Q_OBJECT

public:
	NCustomButton(NCustomButtonType cusButtonType);

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
	NCustomButtonType m_customButtonType;
};
