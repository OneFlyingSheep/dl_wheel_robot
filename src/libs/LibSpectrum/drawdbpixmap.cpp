

#include "drawdbpixmap.h"
#include "utils.h"
#include <QPainter>
#include <QResizeEvent>
#include <QTime>
#include <QDebug>


#define SPACING qreal(10.0)

DrawDBPixmap::DrawDBPixmap(QWidget *parent)
    :   QWidget(parent)
{
   // setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    //setMinimumHeight(50);
}

DrawDBPixmap::~DrawDBPixmap()
{

}

void DrawDBPixmap::SetData(qreal rTimes, QVector<FrequencySpectrum> vSpectrums)
{
	m_rTimes = rTimes;
	m_vSpectrums = vSpectrums;
	ResizePixmap();
}

void DrawDBPixmap::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);
	QPainter painter(this);

	QPoint ptPos = QPoint(0, height() / 2);
	painter.fillRect(rect(), Qt::black);

	QColor barColor(51, 204, 102);

	painter.save();
	const QColor gridColor = barColor.darker();
	QPen gridPen(gridColor);

	QVector<qreal> dashes;
	dashes << 2 << 2;
	gridPen.setDashPattern(dashes);
	painter.setPen(gridPen);
	painter.drawLine(QLineF(ptPos, QPointF(width(), height() /2)));
	painter.restore();

	painter.save();
	barColor = barColor.lighter();
	barColor.setAlphaF(0.75);
	QPen pen1;
	pen1.setColor(barColor);
	painter.setPen(pen1);
	painter.drawPath(m_painterPath);
	painter.restore();

	painter.save();
	QPen pen;
	pen.setColor(Qt::magenta);
	painter.setPen(pen);

	const int numVerticalSections = 10;
	//绘制纵向坐标轴
	if (m_rMaxVal != m_rMinVal)
	{
		QLineF line1(QPointF(SPACING, rect().top() + SPACING), QPointF(SPACING, rect().bottom() - SPACING));
		painter.drawLine(line1);

		qreal rHeight = rect().height() - SPACING * 2;
		qreal rCellHeight = rHeight / 10;

		for (int index = 1; index <= numVerticalSections/2; ++index)
		{
			QLineF lineCellUp(QPointF(SPACING, height() / 2 - index*rCellHeight), QPointF(SPACING + 3, height() / 2 - index * rCellHeight));
			QLineF lineCellDown(QPointF(SPACING, height() / 2 + index * rCellHeight), QPointF(SPACING+3, height() / 2 + index * rCellHeight));

			painter.drawLine(lineCellUp);
			painter.drawLine(lineCellDown);

			if (index == numVerticalSections / 2)
			{
				painter.drawText(QPointF(SPACING + 5, height() / 2 - index * rCellHeight + 4), QString::number((int)(index * rCellHeight / 2))+"(db)");
			}
			else
			{
				painter.drawText(QPointF(SPACING + 5, height() / 2 - index * rCellHeight + 4), QString::number((int)(index * rCellHeight / 2)));
			}
			painter.drawText(QPointF(SPACING + 5, height() / 2 + index * rCellHeight + 4), QString::number((int)(-index * rCellHeight/2)));
		}

	}

	//绘制时间
	QLineF lineX(QPointF(rect().left() + SPACING, rect().height() - 20), QPointF(rect().right() - SPACING, rect().height() - 20));
	painter.drawLine(lineX);

	QTime time(0, 0, 0);
	//painter.drawText(QPointF(rect().left(), rect().height() - 20), time.toString("hh:mm:ss"));
	painter.drawText(QPointF(rect().left() + SPACING, rect().height() - 5), time.toString("hh:mm:ss"));

	time = time.addSecs(m_rTimes);
	painter.drawText(QPointF(rect().right() - SPACING - 50, rect().height() - 5), time.toString("hh:mm:ss"));

	painter.restore();

	//绘制当前轴
	const qreal play = qreal(m_playPosition) / m_bufferLength;
	qreal rX = rect().left() + SPACING + play * (rect().width() - SPACING * 2);
	QLineF line(QPointF(rX, rect().top()), QPointF(rX, rect().bottom()));
	painter.save();
	pen.setColor(Qt::red);
	painter.setPen(pen);
	painter.drawLine(line);

	QTime currentTime(0, 0, 0);
	qreal rCurSec = m_rTimes * play;
	currentTime = currentTime.addSecs(rCurSec);
	painter.drawText(QPointF(rX + 5.0, rect().top() + 15), currentTime.toString("hh:mm:ss"));
	painter.restore();
}

void DrawDBPixmap::resizeEvent(QResizeEvent *event)
{
	Q_UNUSED(event);
	ResizePixmap();
}

void DrawDBPixmap::wheelEvent(QWheelEvent *event)
{
	Q_UNUSED(event);
	//int numberDegrees = event->delta() / 8;

	//int numberSteps = numberDegrees / 15;

	//if (event->orientation() == Qt::Vertical)

	//{   //实现的是横排移动，所以这里把滚轮的上下移动实现为
	//	m_rXCell *= numberSteps;
	//	if (m_rXCell <= 1.0)
	//	{
	//		m_rXCell = 1.0;
	//	}
	//}

	//event->accept();

	update();
}

void DrawDBPixmap::reset()
{
	m_bufferLength = 0;
//	m_recordPosition = 0;
	m_playPosition = 0;
	m_windowPosition = 0;
	m_windowLength = 0;
	update();
}

void DrawDBPixmap::CalcXCell()
{
	int nCount = CalcAllPoint();
	m_rXCell = 2;
	qreal rScale = (width() - SPACING*2) / m_rXCell / nCount;
	m_rXCell *= rScale;
}

int DrawDBPixmap::CalcAllPoint()
{
	int nCount = 0;
	QVector<FrequencySpectrum>::iterator it = m_vSpectrums.begin();
	while (it != m_vSpectrums.end())
	{
		FrequencySpectrum &stFrequencySpectrum = *it;

		nCount += stFrequencySpectrum.count()/100;
		++it;
	}

	return nCount;
}

void DrawDBPixmap::CreatePath()
{
	m_rMinVal = 0.0;
	m_rMaxVal = 0.0;

	m_painterPath = QPainterPath();
	QPoint ptPos = QPoint(SPACING, height() / 2);

	m_painterPath.moveTo(ptPos);

	qreal rX = SPACING;
	QVector<FrequencySpectrum>::iterator it = m_vSpectrums.begin();
	while (it != m_vSpectrums.end())
	{
		FrequencySpectrum &stFrequencySpectrum = *it;
		int nCount = stFrequencySpectrum.count();
		for (int index = 0; index < nCount; index += 100)
		{
			if (rX <= rect().right() - SPACING)
			{
				qreal rVal = stFrequencySpectrum[index].amplitude;
				if (index == 0)
				{
					m_rMinVal = rVal;
					m_rMaxVal = rVal;
				}
				else
				{
					if (m_rMinVal > rVal)
					{
						m_rMinVal = rVal;
					}

					if (m_rMaxVal < rVal)
					{
						m_rMaxVal = rVal;
					}
				}
				m_painterPath.lineTo(QPointF(rX, ptPos.y() - rVal * 2));
			}
			rX += m_rXCell;
		}
		it++;
	}
}

void DrawDBPixmap::ResizePixmap()
{
	CalcXCell();
	update();
	CreatePath();
}

void DrawDBPixmap::bufferLengthChanged(qint64 bufferSize)
{
	m_bufferLength = bufferSize;
	m_playPosition = 0;
	m_windowPosition = 0;
	m_windowLength = 0;
	repaint();
}

void DrawDBPixmap::playPositionChanged(qint64 playPosition)
{
	Q_ASSERT(playPosition >= 0);
	Q_ASSERT(playPosition <= m_bufferLength);
	m_playPosition = playPosition;
	repaint();
}

void DrawDBPixmap::windowChanged(qint64 position, qint64 length)
{
	Q_ASSERT(position >= 0);
	Q_ASSERT(position <= m_bufferLength);
	Q_ASSERT(position + length <= m_bufferLength);
	m_windowPosition = position;
	m_windowLength = length;
	repaint();
}




