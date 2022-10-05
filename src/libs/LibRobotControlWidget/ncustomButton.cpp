#include "ncustomButton.h"
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>

NCustomButton::NCustomButton(NCustomButtonType cusButtonType)
	: QWidget(NULL)
	, m_pressIndex(0)
	, m_enterIndex(0)
	, m_isMouseEntered(false)
	, m_isMousePressed(false)
	, m_radius(90)
	, m_arcLength(55)
	, m_customButtonType(cusButtonType)
{
	this->setMouseTracking(true);
	setRadiusValue(m_radius);
}

void NCustomButton::setRadiusValue(int radius)
{
	m_radius = radius;
	setFixedSize(QSize(m_radius * 2, m_radius * 2));
}

void NCustomButton::setArcLength(int arcLength)
{
	m_arcLength = arcLength;
}

void NCustomButton::setArcAngle(qreal startAngle, qreal angleLength, int buttonCount, QColor color)
{
	for (int i = 0; i < buttonCount; i++)
	{
		addArc(startAngle, angleLength, color);
		startAngle += 360 / buttonCount;
		if (startAngle > 360)
		{
			startAngle -= 360;
		}
	}

	m_arcColor = color;

	switch (m_customButtonType)
	{
	case NVideoControl:
		initVideoControl();
		break;
	case NVideoParamSet:
		initVideoParamSet();
		break;
	case NVideoParamSet_NoReset:
		initVideoParamSetNoReset();
		break;
	case NVideoDirectionControl:
		initVideoDirectionControl();
		break;
	case NRobotControl_BodyMove:
		initRobotBodyMove();
		break;
	case NRobotControl_BodyMove_Big:
		initRobotBodyMove_Big();
		break;
	case NRobotControl_VideoMove:
		initRobotVidoeMove();
		break;
	case NWheel_RobotControl_BodyMove:
		initWheelRobotBodyControl();
		break;
	case NWheel_RobotControl_PTZMove:
		initWheelRobotPTZControl();
		break;
	default:
		break;
	}
}

void NCustomButton::initVideoControl()
{
	QPainterPath ellipsePath;
	ellipsePath.addEllipse(QPoint(0, 0), m_radius - m_arcLength + 1, m_radius - m_arcLength + 1);
	m_pathList.append(ellipsePath);
	m_colorList.append(QColor(14, 150, 254));

	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(14);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(-10, -55), font, QStringLiteral("��"));
	m_textPathList[1].addText(QPoint(-70, 10), font, QStringLiteral("��"));
	m_textPathList[2].addText(QPoint(-10, 70), font, QStringLiteral("��"));
	m_textPathList[3].addText(QPoint(50, 10), font, QStringLiteral("��"));
}

void NCustomButton::initVideoParamSet()
{
	QPainterPath ellipsePath;
	ellipsePath.addEllipse(QPoint(0, 0), m_radius - m_arcLength + 1, m_radius - m_arcLength + 1);
	m_pathList.append(ellipsePath);
	m_colorList.append(QColor(14, 150, 254));

	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(14);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(18, -35), font, QStringLiteral("����+"));
	m_textPathList[1].addText(QPoint(-60, -35), font, QStringLiteral("����+"));
	m_textPathList[2].addText(QPoint(-60, 45), font, QStringLiteral("����-"));
	m_textPathList[3].addText(QPoint(25, 45), font, QStringLiteral("����-"));
}

void NCustomButton::initVideoParamSetNoReset()
{
	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(14);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(18, -35), font, QStringLiteral("����+"));
	m_textPathList[1].addText(QPoint(-60, -35), font, QStringLiteral("����+"));
	m_textPathList[2].addText(QPoint(-60, 45), font, QStringLiteral("����-"));
	m_textPathList[3].addText(QPoint(25, 45), font, QStringLiteral("����-"));
}

void NCustomButton::initVideoDirectionControl()
{
	QPainterPath ellipsePath;
	ellipsePath.addEllipse(QPoint(0, 0), m_radius - m_arcLength + 1, m_radius - m_arcLength + 1);
	m_pathList.append(ellipsePath);
	m_colorList.append(QColor(14, 150, 254));

	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(14);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(-10, -60), font, QStringLiteral("��"));
	m_textPathList[1].addText(QPoint(-65, -40), font, QStringLiteral("����"));
	m_textPathList[2].addText(QPoint(-75, 5), font, QStringLiteral("��"));
	m_textPathList[3].addText(QPoint(-65, 50), font, QStringLiteral("����"));
	m_textPathList[4].addText(QPoint(-10, 70), font, QStringLiteral("��"));
	m_textPathList[5].addText(QPoint(30, 50), font, QStringLiteral("����"));
	m_textPathList[6].addText(QPoint(55, 5), font, QStringLiteral("��"));
	m_textPathList[7].addText(QPoint(30, -40), font, QStringLiteral("����"));
}

void NCustomButton::initRobotBodyMove()
{
	int centerCircleRadius = m_radius - m_arcLength + 1;
	QRectF rect(-centerCircleRadius, -centerCircleRadius, centerCircleRadius * 2, centerCircleRadius * 2);

	// �����������·��;
	QPainterPath leftEsllipsePath, leftSubPath;
	leftEsllipsePath.arcTo(rect, 90, 180);

	leftSubPath.addRect(QRect(-1, -centerCircleRadius, 1, centerCircleRadius * 2));
	// �����μ�ȥС���εõ�Բ��;
	leftEsllipsePath -= leftSubPath;

	m_pathList.append(leftEsllipsePath);
	m_colorList.append(QColor(14, 150, 254));

	// �����ұ�����·��;
	QPainterPath rightEsllipsePath, rightSubPath;
	rightEsllipsePath.arcTo(rect, -90, 180);

	rightSubPath.addRect(QRect(0, -centerCircleRadius, 1, centerCircleRadius * 2));
	// �����μ�ȥС���εõ�Բ��;
	rightEsllipsePath -= rightSubPath;

	m_pathList.append(rightEsllipsePath);
	m_colorList.append(QColor(14, 150, 254));

	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(12);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(18, -25), font, QStringLiteral("ǰ��"));
	m_textPathList[1].addText(QPoint(-50, -25), font, QStringLiteral("����"));
	m_textPathList[2].addText(QPoint(-50, 36), font, QStringLiteral("�½�"));
	m_textPathList[3].addText(QPoint(18, 36), font, QStringLiteral("����"));
	m_textPathList[4].addText(QPoint(-20, 6), font, QStringLiteral("��"));
	m_textPathList[5].addText(QPoint(5, 6), font, QStringLiteral("��"));
}

void NCustomButton::initRobotBodyMove_Big()
{
	QPainterPath ellipsePath;
	ellipsePath.addEllipse(QPoint(0, 0), m_radius - m_arcLength + 1, m_radius - m_arcLength + 1);
	m_pathList.append(ellipsePath);
	m_colorList.append(QColor(14, 150, 254));

	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(14);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(18, -35), font, QStringLiteral("ǰ��"));
	m_textPathList[1].addText(QPoint(-60, -35), font, QStringLiteral("����"));
	m_textPathList[2].addText(QPoint(-60, 45), font, QStringLiteral("�½�"));
	m_textPathList[3].addText(QPoint(25, 45), font, QStringLiteral("����"));
}

void NCustomButton::initRobotVidoeMove()
{
	int centerCircleRadius = m_radius - m_arcLength + 1;
	QRectF rect(-centerCircleRadius, -centerCircleRadius, centerCircleRadius * 2, centerCircleRadius * 2);

	// �����������·��;
	QPainterPath leftEsllipsePath, leftSubPath;
	leftEsllipsePath.arcTo(rect, 0, 180);

	leftSubPath.addRect(QRect(-centerCircleRadius, -1, centerCircleRadius * 2, 1));
	// �����μ�ȥС���εõ�Բ��;
	leftEsllipsePath -= leftSubPath;

	m_pathList.append(leftEsllipsePath);
	m_colorList.append(QColor(14, 150, 254));

	// �����ұ�����·��;
	QPainterPath rightEsllipsePath, rightSubPath;
	rightEsllipsePath.arcTo(rect, 180, 180);

	rightSubPath.addRect(QRect(-centerCircleRadius, 0, centerCircleRadius * 2, 1));
	// �����μ�ȥС���εõ�Բ��;
	rightEsllipsePath -= rightSubPath;

	m_pathList.append(rightEsllipsePath);
	m_colorList.append(QColor(14, 150, 254));

	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(12);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(18, -25), font, QStringLiteral("����"));
	m_textPathList[1].addText(QPoint(-50, -25), font, QStringLiteral("����"));
	m_textPathList[2].addText(QPoint(-50, 36), font, QStringLiteral("����"));
	m_textPathList[3].addText(QPoint(18, 36), font, QStringLiteral("����"));
	m_textPathList[4].addText(QPoint(-8, -8), font, QStringLiteral("̧"));
	m_textPathList[5].addText(QPoint(-8, 16), font, QStringLiteral("��"));
}

void NCustomButton::initWheelRobotBodyControl()
{
	QPainterPath ellipsePath;
	ellipsePath.addEllipse(QPoint(0, 0), m_radius - m_arcLength + 1, m_radius - m_arcLength + 1);
	m_pathList.append(ellipsePath);
	m_colorList.append(QColor(220, 20, 60));

	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(14);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(-10, -35),font, QStringLiteral("W"));
	m_textPathList[1].addText(QPoint(-50, 10), font, QStringLiteral("A"));
	m_textPathList[2].addText(QPoint(-5, 50),  font, QStringLiteral("S"));
	m_textPathList[3].addText(QPoint(30, 10),  font, QStringLiteral("D"));
	m_textPathList[4].addText(QPoint(-21, 7), font, QStringLiteral("��ͣ"));
}

void NCustomButton::initWheelRobotPTZControl()
{
	int centerCircleRadius = m_radius - m_arcLength + 1;
	QRectF rect(-centerCircleRadius, -centerCircleRadius, centerCircleRadius * 2, centerCircleRadius * 2);

	// �����������·��;
	QPainterPath leftEsllipsePath, leftSubPath;
	leftEsllipsePath.arcTo(rect, 90, 180);

	leftSubPath.addRect(QRect(-1, -centerCircleRadius, 1, centerCircleRadius * 2));
	// �����μ�ȥС���εõ�Բ��;
	leftEsllipsePath -= leftSubPath;

	m_pathList.append(leftEsllipsePath);
	m_colorList.append(QColor(14, 150, 254));

	// �����ұ�����·��;
	QPainterPath rightEsllipsePath, rightSubPath;
	rightEsllipsePath.arcTo(rect, -90, 180);

	rightSubPath.addRect(QRect(0, -centerCircleRadius, 1, centerCircleRadius * 2));
	// �����μ�ȥС���εõ�Բ��;
	rightEsllipsePath -= rightSubPath;

	m_pathList.append(rightEsllipsePath);
	m_colorList.append(QColor(14, 150, 254));

	QFont font;
	font.setFamily("Microsoft YaHei");
	font.setPointSize(12);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		QPainterPath paintPath;
		m_textPathList.append(paintPath);
	}

	m_textPathList[0].addText(QPoint(-10, -35), font, QStringLiteral("W"));
	m_textPathList[1].addText(QPoint(-50, 10), font, QStringLiteral("A"));
	m_textPathList[2].addText(QPoint(-5, 50), font, QStringLiteral("S"));
	m_textPathList[3].addText(QPoint(30, 10), font, QStringLiteral("D"));
	m_textPathList[4].addText(QPoint(-20, 6), font, QStringLiteral("+"));
	m_textPathList[5].addText(QPoint(5, 6), font, QStringLiteral("-"));
}

void NCustomButton::paintEvent(QPaintEvent *)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing, true);
	painter.setPen(Qt::NoPen);
	painter.translate(width() >> 1, height() >> 1);

	painter.setBrush(QColor(60, 60, 60));
	painter.drawEllipse(QPoint(0, 0), m_radius - 1, m_radius - 1);

	for (int i = 0; i < m_pathList.count(); i++)
	{
		painter.setBrush(m_colorList[i]);
		painter.drawPath(m_pathList[i]);
		if (i == m_pressIndex && m_isMousePressed)
		{
			painter.setBrush(QColor(255, 255, 255, 120));
			painter.drawPath(m_pathList[i]);

		}
		else if (i == m_pressIndex && m_isMouseEntered)
		{
			painter.setBrush(QColor(255, 255, 255, 60));
			painter.drawPath(m_pathList[i]);
		}
		painter.setBrush(QColor(98, 98, 98));
		painter.drawPath(m_textPathList[i]);
	}
}

void NCustomButton::addArc(qreal startAngle, qreal angleLength, QColor color)
{
	QRectF rect(-m_radius, -m_radius, m_radius * 2, m_radius * 2);

	// ��������·��;
	QPainterPath path;
	path.arcTo(rect, startAngle, angleLength);
	if (m_customButtonType != NCustomButtonType::NVideoParamSet_NoReset)
	{
		QPainterPath subPath;
		// ����С����·��;
		subPath.addEllipse(rect.adjusted(m_arcLength, m_arcLength, -m_arcLength, -m_arcLength));
		// �����μ�ȥС���εõ�Բ��;
		path -= subPath;
	}	

	m_pathList.append(path);

	// ����Բ����ɫ;
	QRadialGradient radialGradient;
	radialGradient.setCenter(0, 0);
	radialGradient.setRadius(m_radius);
	radialGradient.setColorAt(0, QColor(130, 255, 245));
	radialGradient.setColorAt(1.0, color);
	m_colorList.append(radialGradient);
}

void NCustomButton::mousePressEvent(QMouseEvent *event)
{
	QPoint mousePressPoint = event->pos();
	QPoint translatePoint = mousePressPoint - QPoint(width() >> 1, height() >> 1);
	for (int i = 0; i < m_pathList.count(); i++)
	{
		QRectF pathRect = m_pathList[i].boundingRect();
		if (m_pathList[i].contains(translatePoint) || m_textPathList[i].contains(translatePoint))
		{
			m_pressIndex = i;
			m_isMousePressed = true;
			update();
			emit signalButtonPressed(m_pressIndex);
			break;
		}
	}
}

void NCustomButton::mouseReleaseEvent(QMouseEvent *event)
{
	if (m_isMousePressed)
	{
		m_isMousePressed = false;
		update();
		emit signalButtonRelease(m_pressIndex);
	}
}

void NCustomButton::mouseMoveEvent(QMouseEvent *event)
{
	m_isMouseEntered = false;
	QPoint mousePressPoint = event->pos();
	QPoint translatePoint = mousePressPoint - QPoint(width() >> 1, height() >> 1);
	for (int i = 0; i < m_pathList.count(); i++)
	{
		QRectF pathRect = m_pathList[i].boundingRect();
		if (m_pathList[i].contains(translatePoint) || m_textPathList[i].contains(translatePoint))
		{
			m_pressIndex = i;
			m_isMouseEntered = true;
			break;
		}
	}
	update();
}

void NCustomButton::leaveEvent(QEvent *event)
{
	m_isMouseEntered = false;
	update();
}