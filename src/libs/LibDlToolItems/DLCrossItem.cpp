#include "DLCrossItem.h"
#include <QtWidgets>

DLCrossItem::DLCrossItem()
{
	setFlag(ItemIsSelectable, true);
	setFlag(ItemIsMovable, true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);
	setCursor(Qt::SizeAllCursor);

	pro_.center_.x_ = 0;
	pro_.center_.y_ = 0;
	pro_.height_ = 8;
	pro_.width_ = 8;

	set_cross_pro(pro_);
	set_control_pro(pro_);
}

DLCrossItem::DLCrossItem(HShape::Cross pro) 
	: pro_(pro)
{
	setFlag(ItemIsSelectable, true);
	setFlag(ItemIsMovable, true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);
	setCursor(Qt::SizeAllCursor);

	set_cross_pro(pro);
	set_control_pro(pro);
}

DLCrossItem::~DLCrossItem()
{

}



QRectF DLCrossItem::boundingRect() const
{
	QPointF top_left;
	top_left.setX(-pro_.width_);
	top_left.setY(-pro_.height_);
	QRectF rect(top_left, QSizeF(2 * pro_.width_, 2 * pro_.height_));
	return rect;
}

QPainterPath DLCrossItem::shape() const
{
	QPainterPath path;

	QPainterPath path_x, path_y;
	QPainterPath path_xi, path_yi;
	path_x.moveTo(QPointF(0, 0));
	path_x.lineTo(QPointF(pro_.width_, 0));

	path_xi.moveTo(QPointF(0, 0));
	path_xi.moveTo(QPointF(-pro_.width_, 0));

	path_y.moveTo(QPointF(0, 0));
	path_y.lineTo(QPointF(0, pro_.height_));

	path_yi.moveTo(QPointF(0, 0));
	path_yi.lineTo(QPointF(0, -pro_.height_));

	QPainterPathStroker stroker;
	stroker.setWidth(8); //设置宽度为3
	return stroker.createStroke(path_x | path_y | path_xi | path_yi);
}

void DLCrossItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * widget)
{
	painter->setPen(get_pen());
	painter->setBrush(get_brush());

	QLineF linex = QLineF(QPointF(0, 0), QPointF(pro_.width_, 0));
	QLineF linex1 = QLineF(QPointF(0, 0), QPointF(-pro_.width_, 0));
	QLineF liney = QLineF(QPointF(0, 0), QPointF(0, pro_.height_));
	QLineF liney1 = QLineF(QPointF(0, 0), QPointF(0, -pro_.height_));
	painter->drawLine(linex);
	painter->drawLine(linex1);
	painter->drawLine(liney);
	painter->drawLine(liney1);
}

void DLCrossItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
{
	HQShapeItem::mousePressEvent(event);
}

void DLCrossItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{
	update_shape();
	HQShapeItem::mouseMoveEvent(event);
}

void DLCrossItem::mouseReleaseEvent(QGraphicsSceneMouseEvent * event)
{
	HQShapeItem::mouseReleaseEvent(event);
}

void DLCrossItem::slot_pro(HShape::Cross pro)
{
	set_cross_pro(pro);
	set_control_pro(pro);
}

void DLCrossItem::update_shape()
{
	HShape::Cross pro = cal_pro();
	set_cross_pro(pro);
	emit sig_pro(pro_);
}


void DLCrossItem::set_cross_pro(HShape::Cross pro)
{
	pro_ = pro;
	setPos(pro.center_.x_, pro.center_.y_);
	//通知形状更新
	prepareGeometryChange();
}

void DLCrossItem::set_control_pro(HShape::Cross pro)
{
	//no control point
}


HShape::Cross DLCrossItem::get_pro()
{
	return pro_;
}

HShape::Cross DLCrossItem::cal_pro()
{
	HShape::Cross pro;
	pro.center_.x_ = pos().x();
	pro.center_.y_ = pos().y();
	pro.height_ = pro_.height_;
	pro.width_ = pro_.width_;
	return pro;
}
