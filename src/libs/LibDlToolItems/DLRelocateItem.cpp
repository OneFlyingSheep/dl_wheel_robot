#include "DLRelocateItem.h"
#include <QPainter>


DLRelocateItem::DLRelocateItem()
{
	setFlag(ItemSendsGeometryChanges);
	top_left_ = -20;
	top_right_ = -20;
	width_ = 40;
	height_ = 4400;
	is_over_ = false;
}

DLRelocateItem::~DLRelocateItem()
{

}


QRectF DLRelocateItem::boundingRect() const
{
	return QRectF(top_left_, top_right_, width_, height_);
}


QPainterPath DLRelocateItem::shape() const
{
	QPainterPath path, path_out, path_inner, rect_path, arrow_path;
	path_out.addEllipse(QRectF(-15, -15, 30, 30));
	path_inner.addEllipse(QRectF(-8, -8, 16, 16));
	path = path_out - path_inner;
	
	if (is_over_) {
		double top_left_x = -6;
		double top_left_y = 6;
		double width = 12;
		double height = rect_height_;
		rect_path.addRect(QRectF(top_left_x, top_left_y, width, height));
	}
	path += rect_path;

	QVector<QPointF> points;
	points.push_back(QPointF(-10, rect_height_));
	points.push_back(QPointF(0, rect_height_ + 15));
	points.push_back(QPointF(10, rect_height_));
	points.push_back(QPointF(-10, rect_height_));
	QPolygonF polygon(points);
	arrow_path.addPolygon(polygon);
	path += arrow_path;

	return path;
}


void DLRelocateItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	painter->setPen(Qt::NoPen);
	painter->setBrush(Qt::red);
	painter->drawPath(shape());

}

void DLRelocateItem::updateItem(QPointF start_point, QPointF end_point)
{
	QLineF line(start_point, end_point);

	if (line.length() < 8) {
		is_over_ = false;
	}
	else {
		is_over_ = true;
		rect_height_ = line.length() - 8;
	}
	
		
	prepareGeometryChange();
}

