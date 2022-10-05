#include <QtWidgets>
#include "DLDeviceItem.h"


DLDeviceItem::DLDeviceItem(HShape::Rectangle pro, QString name)
	: pro_(pro), name_(name)
{
	state_ = 3;
	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);

}

DLDeviceItem::~DLDeviceItem()
{

}


void DLDeviceItem::setState(int state)
{
	state_ = state;
	update();
}


QRectF DLDeviceItem::boundingRect() const
{
	return QRectF(0-pro_.width_/2.0, 0-pro_.height_/2.0, pro_.width_, pro_.height_);
}

QPainterPath DLDeviceItem::shape() const
{

	QPainterPath path;
	path.addRect(boundingRect());
	return path;
	
}

void DLDeviceItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{

	QFont ft(QStringLiteral("ËÎÌו"), 8, QFont::Bold);
	painter->setFont(ft);
	painter->setTransform(QTransform(QMatrix(1.0, 0.0, 0.0, -1.0, 0, 0.0)), true);
	painter->setPen(get_pen());
	painter->setOpacity(0.5);
	if (state_ == 0) {
		painter->setBrush(Qt::green);
	}
	else if (state_ == 1) {
		painter->setBrush(Qt::red);
	}
	else if (state_ == 2) {
		painter->setBrush(Qt::yellow);
	}
	else{
		painter->setBrush(Qt::lightGray);
	}
	painter->drawRoundedRect(boundingRect(), 5, 5);
	painter->drawText(boundingRect(), Qt::AlignCenter, name_);

}

void DLDeviceItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mousePressEvent(event);
}

void DLDeviceItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mouseMoveEvent(event);
}

void DLDeviceItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mouseReleaseEvent(event);
}




