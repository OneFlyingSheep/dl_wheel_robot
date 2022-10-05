#include <QtWidgets>
#include "DLPixmapItem.h"


DLPixmapItem::DLPixmapItem(HShape::Rectangle pro, QString file_path)
	: pro_(pro), file_path_(file_path)
{
	//setFlag(ItemIsSelectable,true);
	//setFlag(ItemIsMovable,true);
	//setFlag(ItemSendsGeometryChanges);
	//setAcceptedMouseButtons(Qt::LeftButton);
	//setCursor(Qt::PointingHandCursor);
}

DLPixmapItem::~DLPixmapItem()
{



}

QRectF DLPixmapItem::boundingRect() const
{
	return QRectF( -pro_.width_/2.0, -pro_.height_/2.0, pro_.width_, pro_.height_);
	//return QRectF(-10000, -10000,20000, 20000);
}

QPainterPath DLPixmapItem::shape() const
{
	QPainterPath path;
	path.addRect(boundingRect());
	return path;
}

void DLPixmapItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	painter->save();
	QPixmap pixmap;
	pixmap.load(file_path_);
	pixmap.scaled(pro_.width_, pro_.height_, Qt::IgnoreAspectRatio);

	QRect rect( - pro_.width_ / 2.0, - pro_.height_ / 2.0, pro_.width_, pro_.height_);
//	QTransform transform(QMatrix(1.0, 0.0, 0.0, -1.0, 0.0, 0.0));
	//transform.rotate(180);
	//painter->setTransform(transform, true);
	painter->drawPixmap(rect, pixmap);
	painter->restore();
}

void DLPixmapItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	setCursor(Qt::ClosedHandCursor);
	HQShapeItem::mousePressEvent(event);
}


void DLPixmapItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	setCursor(Qt::PointingHandCursor);
	HQShapeItem::mouseReleaseEvent(event);
}




