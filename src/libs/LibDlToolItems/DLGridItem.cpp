#include "DLGridItem.h"
#include <iostream>
#include <QPainter>

#define HLINECOUNT (500+1)
#define VLINECOUNT (500+1)

DLGridItem::DLGridItem(QRect rect)
	: rect_(rect)
{

	setZValue(0);
	setFlag(ItemIsSelectable, false);
	setFlag(ItemIsMovable, false);
	setFlag(ItemSendsGeometryChanges);
	setFlag(ItemSendsGeometryChanges);
}

DLGridItem::~DLGridItem()
{

}


QRectF DLGridItem::boundingRect() const
{
	return rect_;
}


QPainterPath DLGridItem::shape() const
{
	
	QPainterPath path;
	path.addRect(rect_);

	QPainterPathStroker stroker;
	stroker.setWidth(1);
	return stroker.createStroke(path);

}


void DLGridItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget )
{
	
	QPointF center_pos = rect_.center();
	QPointF top_left = rect_.topLeft();
	QPointF bottom_left = rect_.bottomLeft();
	QPointF top_right = rect_.topRight();
	QPointF bottom_right = rect_.bottomRight();

	double h_offset = abs((top_left - top_right).x()) / (HLINECOUNT - 1);
	double v_offset = abs((bottom_left - top_left).y()) / (VLINECOUNT - 1);

	QPen pen_1(Qt::lightGray, 4, Qt::SolidLine);
	QPen pen_2(Qt::lightGray, 2, Qt::SolidLine);


	//绘制水平线
	for (int i = 0; i < HLINECOUNT; ++i)
	{
		double temp = top_left.y() + i * v_offset;
		QPointF left_pos(top_left.x(), temp);
		QPointF right_pos(top_right.x(), temp);
		QLineF line(left_pos, right_pos);

		painter->setPen(pen_2);
		if ((i % 5) == 0) {
			painter->save();
			painter->setPen(pen_1);
			painter->drawLine(line);
			painter->restore();

		}
		else {
			painter->drawLine(line);
		}

	}

	//绘制垂直线
	for (int i = 0; i < VLINECOUNT; ++i)
	{
		double temp = top_left.x() + i * h_offset;
		QPointF top_pos(temp, top_left.y());
		QPointF bottom_pos(temp, bottom_left.y());
		QLineF line(top_pos, bottom_pos);

		painter->setPen(pen_2);
		if ((i % 5) == 0) {
			painter->save();
			painter->setPen(pen_1);
			painter->drawLine(line);
			painter->restore();
		}
		else {
			painter->drawLine(line);
		}
	}

}

void DLGridItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{

	prepareGeometryChange();
	HQShapeItem::mouseMoveEvent(event);

}

