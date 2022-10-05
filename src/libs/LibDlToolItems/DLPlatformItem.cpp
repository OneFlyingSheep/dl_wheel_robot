#include "DLPlatformItem.h"
#include <QRadialGradient>


DLPlatformItem::DLPlatformItem(QObject *parent)
	: QObject(parent)
{
	m_colorFlag = true;
	m_radius = 50;
	setFlags(QGraphicsItem::ItemIsMovable);
	setFlags(QGraphicsItem::ItemIsSelectable);
	startTimer(500);
}

QRectF DLPlatformItem::boundingRect() const
{
	return QRectF(-m_radius, -m_radius, 2*m_radius, 2*m_radius);
}


void DLPlatformItem::paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget *widget)
{
	//QRadialGradient gradientDark(QPointF(0,0), m_radius, QPointF(0,m_radius));
	QRadialGradient gradientDark(0, 0, m_radius);
	gradientDark.setColorAt(1.0, Qt::white);
	gradientDark.setColorAt(0, qRgb(90, 190, 20));

	//QRadialGradient gradientLight(QPointF(0,0), m_radius, QPointF(0,m_radius));
	QRadialGradient gradientLight(0, 0, m_radius);
	gradientLight.setColorAt(1.0, Qt::white);
	gradientLight.setColorAt(0, qRgb(181, 230, 121));
	painter->setOpacity(0.7);
	
	if(m_colorFlag){
		painter->setPen(Qt::NoPen);
		painter->setBrush(gradientDark);
		m_colorFlag = false;
	}
	else{
		painter->setPen(Qt::NoPen);
		painter->setBrush(gradientLight);
		m_colorFlag = true;
	}

	QPainterPath path;
	path.arcTo( boundingRect(), -45, 90);
	path.closeSubpath();
	
	painter->drawPath(path);
}


void DLPlatformItem::timerEvent(QTimerEvent *)
{
	update();
}


