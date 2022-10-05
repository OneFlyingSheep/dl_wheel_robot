#include "DLCoordinateItem.h"

DLCoordinateItem::DLCoordinateItem()
{
	setFlag(ItemIsSelectable, false);
	setFlag(ItemIsMovable, false);
	setAcceptedMouseButtons(Qt::NoButton);
}

DLCoordinateItem::~DLCoordinateItem()
{

}


QPainterPath DLCoordinateItem::shape() const
{

	QPainterPath path;
	path.addEllipse(boundingRect());
	QPainterPathStroker stroker;
	stroker.setWidth(1);
	return stroker.createStroke(path);
}


QRectF DLCoordinateItem::boundingRect() const
{
	return QRectF(-11, -11, 56, 56);
	//return QRectF(0, 0, 0, 0);
}


void DLCoordinateItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	
	painter->setPen(Qt::red);
	painter->setBrush(Qt::red);

	painter->drawEllipse(QPointF(0, 0), 2, 2);
	painter->drawLine(QPointF(0, 0), QPointF(0, 40));
	painter->drawLine(QPointF(0, 0), QPointF(40, 0));
	
	QPolygonF Polygon_1, Polygon_2;
	Polygon_1 << QPointF(-3, 40) << QPointF(3, 40) << QPointF(0, 45) << QPointF(-3, 40);
    Polygon_2 << QPointF(40, -3) << QPointF(40, 3) << QPointF(45, 0) << QPointF(40, -3);

	QPainterPath Path_1;
	QPainterPath Path_2;
	Path_1.addPolygon(Polygon_1);
	Path_2.addPolygon(Polygon_2);
	painter->drawPath(Path_1);
	painter->drawPath(Path_2);


	//painter->setTransform(QTransform(QMatrix(1, 0, 0, -1, 0, 0.0)),true);
	painter->setFont(QFont("Times", 5, QFont::Bold));
	painter->drawText(QRectF(-11, 40, 8, 8), Qt::AlignCenter, QObject::tr("Y"));
	painter->drawText(QRectF(37, 6, 8, 8), Qt::AlignCenter, QObject::tr("X"));
	painter->drawText(QRectF(-10, 0, 10, 9), Qt::AlignCenter, QObject::tr("O"));

}


