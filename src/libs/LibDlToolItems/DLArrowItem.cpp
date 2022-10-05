#include <QtWidgets>
#include "DLArrowItem.h"
#include "DLControlTransformItem.h"
#include <iostream>
#define Pi (3.1415926535898)


DLArrowItem::DLArrowItem(HShape::Arrow pro ):pro_(pro)
{

	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);
	setCursor(Qt::PointingHandCursor);


	start_item_=new DLControlTransformItem(2);
	end_item_=new DLControlTransformItem(2);
	width_item_=new DLControlTransformItem(2);

	start_item_->setParentItem(this);
	end_item_->setParentItem(this);
	width_item_->setParentItem(this);

	set_arrow_pro(pro);
	set_control_pro(pro);

}

DLArrowItem::~DLArrowItem()
{

}


QRectF DLArrowItem::boundingRect() const
{
	QPolygonF arrow=get_arrow();
	QPolygonF stick=get_stick();
	QPolygonF mix=arrow+stick;
	return mix.boundingRect();
}


QPainterPath DLArrowItem::shape() const
{
	QPolygonF arrow=get_arrow();
	QPolygonF stick=get_stick();
	QPolygonF mix=arrow+stick;
	QPainterPath path;
	path.addPolygon(mix);

	if(is_solid_==true)
	{
		return path;
	}
	else
	{
		QPainterPathStroker stroker;
		stroker.setWidth(1);
		return stroker.createStroke(path);
	}
}


void DLArrowItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	painter->setPen(get_pen());

	QPolygonF arrow=get_arrow();
	QPolygonF stick=get_stick();

	painter->drawPolygon(stick);
	painter->drawPolygon(arrow);

}

void DLArrowItem::slot_pro(HShape::Arrow pro)
{
	set_arrow_pro(pro);
	set_control_pro(pro);
}

void DLArrowItem::set_arrow_pro(HShape::Arrow pro)
{
	pro_=pro;

	//通知形状更新
	prepareGeometryChange();

}

void DLArrowItem::set_control_pro(HShape::Arrow pro)
{

	QPointF start(mapFromScene(QPointF(pro_.start_.x_,pro_.start_.y_)));
	QPointF end(mapFromScene(QPointF(pro_.end_.x_,pro_.end_.y_)));

	start_item_->setPos(start);
	end_item_->setPos(end);

	QPointF center((start+end)/2.0);
	QLineF line(start,end);
	line.setP1(center);
	QLineF line_normal=line.normalVector();
	line_normal.setLength(pro_.width_/2.0);
	QPointF offset(line_normal.p2()-line_normal.p1());
	QLineF line_p=line.translated(offset);
	width_item_->setPos(line_p.p1());

}

void DLArrowItem::update_shape()
{

	HShape::Arrow pro=cal_pro();
	set_arrow_pro(pro);
	set_control_pro(pro);
	emit sig_pro(pro_);

}


HShape::Arrow DLArrowItem::cal_pro()
{

	HShape::Arrow pro;
	pro.start_.x_=start_item_->scenePos().x();
	pro.start_.y_=start_item_->scenePos().y();
	pro.end_.x_=end_item_->scenePos().x();
	pro.end_.y_=end_item_->scenePos().y();
	pro.width_=pro_.width_;

	if(width_item_->isSelected()){
		QLineF line(start_item_->pos(),end_item_->pos());
		QPointF pos=width_item_->pos();
		double width=cal_dis(pos,line);
		pro.width_=width;
	}

	pro.arrow_size_=pro_.width_*2.5;
	return pro;

}

double DLArrowItem::cal_dis( QPointF pos,QLineF line )
{

	QLineF cross_line(pos,line.p1());
	cross_line.setAngle(line.angle()-90);

	QPointF cross_point;
	cross_line.intersect(line,&cross_point);

	double vx=cross_point.x()-pos.x();
	double vy=cross_point.y()-pos.y();
	double height=sqrt(vx*vx+vy*vy);

	return height;

}


void DLArrowItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	setCursor(Qt::ClosedHandCursor);
	HQShapeItem::mousePressEvent(event);
}

void DLArrowItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{

	update_shape();
	HQShapeItem::mouseMoveEvent(event);

}

void DLArrowItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	setCursor(Qt::PointingHandCursor);
	HQShapeItem::mouseReleaseEvent(event);
}

QPolygonF DLArrowItem::get_arrow() const
{
	QPointF start_pos=mapFromScene(QPointF(pro_.start_.x_,pro_.start_.y_));
	QPointF end_pos=mapFromScene(QPointF(pro_.end_.x_,pro_.end_.y_));
	QLineF line(start_pos,end_pos);

	QPolygonF arrow;
	int arrow_size=pro_.arrow_size_;
	double angle=::acos(line.dx()/line.length());
	if (line.dy() >= 0)
		angle = (Pi * 2) - angle;

	QPointF arrow_p1 = line.p2() - QPointF(sin(angle + Pi / 3) * arrow_size,
		cos(angle + Pi / 3) * arrow_size);
	QPointF arrow_p2 = line.p2() - QPointF(sin(angle + Pi - Pi / 3) * arrow_size,
		cos(angle + Pi - Pi / 3) * arrow_size);

	arrow << arrow_p2 << line.p2() << arrow_p1;

	return arrow;
}


QPolygonF DLArrowItem::get_stick() const
{

	QPolygonF stick;

	QPointF start_pos=mapFromScene(QPointF(pro_.start_.x_,pro_.start_.y_));
	QPointF end_pos=mapFromScene(QPointF(pro_.end_.x_,pro_.end_.y_));
	QLineF line(start_pos,end_pos);
	QLineF line_normal=line.normalVector();
	line_normal.setLength(pro_.width_/2.0);
	QPointF offset=line_normal.p2()-line_normal.p1();
	QLineF line_p=line.translated(offset);
	QLineF line_n=line.translated(-offset);

	line_p.setLength(line.length()-pro_.arrow_size_*cos(Pi/6));
	line_n.setLength(line.length()-pro_.arrow_size_*cos(Pi/6));

	stick<<line_p.p2()<<line_p.p1()<<line_n.p1()<<line_n.p2();

	return stick;
}

HShape::Arrow DLArrowItem::get_pro()
{
	return pro_;
}
