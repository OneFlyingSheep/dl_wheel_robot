#pragma once

#include <QtWidgets/QWidget>
#include <QDebug>

enum NCustomButtonType
{
	NVideoControl = 0,							// 视频控制;
	NVideoParamSet,								// 视频参数设置;
	NVideoParamSet_NoReset,						// 视频参数设置(无复位按钮);
	NVideoDirectionControl,						// 视频方向设置;
	NRobotControl_BodyMove,						// 机器人身体移动;
	NRobotControl_BodyMove_Big,					// 机器人身体移动_视频窗口中;
	NRobotControl_VideoMove,					// 可见光移动;
	NWheel_RobotControl_BodyMove,				// 轮式机器人车体移动;
	NWheel_RobotControl_PTZMove,				// 轮式机器人云台移动;
};

class NCustomButton : public QWidget
{
	Q_OBJECT

public:
	NCustomButton(NCustomButtonType cusButtonType);

	// 设置圆弧及半径大小;
	void setRadiusValue(int radius);
	void setArcLength(int arcLength);
	// 设置圆弧角度;
	void setArcAngle(qreal startAngle, qreal angleLength, int buttonCount = 4, QColor color = QColor(33, 150, 243));

private:
	// 视频控制初始化;
	void initVideoControl();
	// 视频参数设置初始化;
	void initVideoParamSet();
	// 视频参数设置初始化(无复位按钮);
	void initVideoParamSetNoReset();
	// 视频方向控制初始化;
	void initVideoDirectionControl();
	// 机器人移动初始化;
	void initRobotBodyMove();
	// 机器人移动初始化;
	void initRobotBodyMove_Big();
	// 可见光移动;
	void initRobotVidoeMove();
	// 轮式机器人车体移动;
	void initWheelRobotBodyControl();
	// 轮式机器人云台移动;
	void initWheelRobotPTZControl();

	// 自绘;
	void paintEvent(QPaintEvent *);
	// 添加圆弧;
	void addArc(qreal startAngle, qreal angleLength, QColor color);
	// 鼠标事件;
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

	// 鼠标离开事件;
	void leaveEvent(QEvent *event);

signals:
	void signalButtonPressed(int buttonId);
	void signalButtonRelease(int buttonId);

private:
	// 圆弧及半径大小;
	int m_radius, m_arcLength;
	// 圆弧路径;
	QList<QPainterPath> m_pathList;
	// 圆弧上文字path;
	QList<QPainterPath> m_textPathList;
	// 颜色列表;
	QList<QBrush> m_colorList;
	// 设置圆盘统一颜色;
	QColor m_arcColor;
	// 当前按下/进入按钮的索引;
	int m_pressIndex, m_enterIndex;
	// 当前鼠标是否按下按钮;
	bool m_isMousePressed;
	// 当前鼠标是否进入按钮;
	bool m_isMouseEntered;
	// 按钮类型;
	NCustomButtonType m_customButtonType;
};
