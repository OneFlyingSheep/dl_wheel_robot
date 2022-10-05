#include <QDebug>
#include <QStyleOptionGraphicsItem>
#include <QPainter>
#include <QPixmapCache>
#include "LoadPixmapItem.h"

LoadPixmapItem::LoadPixmapItem(QGraphicsItem *parent)
    :QGraphicsItem(parent)
	, m_ptDrawStartPoint(0.0, 0.0)
	, m_ptDrawEndPoint(0.0, 0.0)
{
	setZValue(0);
}

LoadPixmapItem::~LoadPixmapItem()
{
	for (int i = 0; i < LEVEL_NUM; ++i)
	{
		m_pixmap[i].load("");
		//m_pixmap[i] = QPixmap();
	}
	QPixmapCache::clear();
}

void LoadPixmapItem::SetStartEndPoint(QPointF ptStartPoint, QPointF ptEndPoint)
{
    m_ptStartPoint = ptStartPoint;
    m_ptEndPoint = ptEndPoint;
}


void LoadPixmapItem::SetHighDefinitionPixmap(QPixmap &pixmap)
{
	m_pixmap[HIGH_DEFINTION_LEVEL] = pixmap;
}

void LoadPixmapItem::SetMediumPixmap(QPixmap &pixmap)
{
	m_pixmap[MEDIUM_LEVEL] = pixmap;
}

void LoadPixmapItem::SetLowPixmap(QPixmap &pixmap)
{
	m_pixmap[LOW_LEVEL] = pixmap;
}

void LoadPixmapItem::UpdateArea(QPointF ptStartPoint, QPointF ptEndPoint)
{
	m_ptDrawStartPoint = ptStartPoint;
	m_ptDrawEndPoint = ptEndPoint;

	if (m_ptDrawStartPoint.x() < m_ptStartPoint.x())
	{
		m_ptDrawStartPoint.setX(m_ptStartPoint.x());
	}
	if (m_ptDrawStartPoint.y() < m_ptStartPoint.y())
	{
		m_ptDrawStartPoint.setY(m_ptStartPoint.y());
	}

	if (m_ptDrawEndPoint.x() > m_ptEndPoint.x())
	{
		m_ptDrawEndPoint.setX(m_ptEndPoint.x());
	}
	if (m_ptDrawEndPoint.y() > m_ptEndPoint.y())
	{
		m_ptDrawEndPoint.setY(m_ptEndPoint.y());
	}
}

QRectF LoadPixmapItem::boundingRect()const
{
    return QRectF(m_ptStartPoint, m_ptEndPoint);
}

void LoadPixmapItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget)

// 	painter->save();
// 	painter->setPen(QPen(Qt::red, 2));
// 	painter->drawRect(boundingRect());
// 	painter->restore();

	const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());

	QPixmap pixmap;
	QRectF source;
// 	if (lod <= 0.5)
// 	{
// 		pixmap = m_pixmap[LOW_LEVEL];
// 		source = QRectF((m_ptDrawStartPoint - m_ptStartPoint)/2, (m_ptDrawEndPoint - m_ptStartPoint)/2);
// 	}
// 	else if (lod <= 1.0)
// 	{
		pixmap = m_pixmap[MEDIUM_LEVEL];
		source = QRectF(m_ptDrawStartPoint - m_ptStartPoint, m_ptDrawEndPoint - m_ptStartPoint);
// 	}
// 	else// if (lod <= 2.0)
// 	{
// 		pixmap = m_pixmap[HIGH_DEFINTION_LEVEL];
// 		source = QRectF((m_ptDrawStartPoint - m_ptStartPoint)*2, (m_ptDrawEndPoint - m_ptStartPoint)*2);
// 	}
	QRectF target(m_ptDrawStartPoint, m_ptDrawEndPoint);
	painter->save();
	painter->drawPixmap(target, pixmap, source);
	painter->restore();
	pixmap.load("");
}








