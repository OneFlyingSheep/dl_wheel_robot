#include <QtWidgets>
#include "DLSegmentItem.h"
#include "DLControlTransformItem.h"


DLSegmentItem::DLSegmentItem(HShape::Segment pro, int id, std::string className , std::string instanceName , LineProperty lineClassName)
	: pro_(pro), id_(id), className_(className), instanceName_(instanceName), lineClassName_(lineClassName)
{
	set_solid(true);

	start_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	start_item_->setParentItem(this);
	start_item_->setVisible(true);

	end_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	end_item_->setParentItem(this);
	end_item_->setVisible(true);

	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);
	setFocus(Qt::MouseFocusReason);

	set_segment_pro(pro);
	set_control_pro(pro);

}

DLSegmentItem::DLSegmentItem(HShape::Segment pro, int id, LineProperty lineClassName)
	: pro_(pro), id_(id), lineClassName_(lineClassName)
{
	set_solid(true);

	start_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	start_item_->setParentItem(this);
	start_item_->setVisible(true);

	end_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	end_item_->setParentItem(this);
	end_item_->setVisible(true);

	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);

	set_segment_pro(pro);
	set_control_pro(pro);

}

DLSegmentItem::~DLSegmentItem()
{


	if(start_item_!=NULL)
	{
		delete start_item_;
		start_item_=NULL;
	}


	if(end_item_!=NULL)
	{
		delete end_item_;
		end_item_=NULL;
	}

}


QRectF DLSegmentItem::boundingRect() const
{

	QPainterPath path = shape();
	return path.boundingRect();

}

QPainterPath DLSegmentItem::shape() const
{
	QLineF line;
	line.setP1(QPointF(pro_.start_.x_, pro_.start_.y_));
	line.setP2(QPointF(pro_.end_.x_, pro_.end_.y_));
	QPainterPath path;
	QGraphicsLineItem *line_item = new QGraphicsLineItem(line);
	QRectF rect = line_item->boundingRect();
	delete line_item;
	line_item = NULL;

	path.addRect(rect);
	
	if(is_solid_==true)
	{
		return path;
	}
	else
	{
		QPainterPathStroker stroker;
		stroker.setWidth(2);
		return stroker.createStroke(path);
	}


}

void DLSegmentItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	switch(lineClassName_)
	{
	case NormalLine:
		painter->setPen(QPen(Qt::black, 2, Qt::SolidLine));
		break;
	case VirtualLine:
		painter->setPen(QPen(Qt::blue, 2, Qt::DotLine));
		break;
	case ForbiddenLine:
		painter->setPen(QPen(Qt::red, 2, Qt::DotLine));
		break;
	default:
		break;

	}
	//painter->setBrush(get_brush());
	//painter->drawPath(shape());
	painter->drawLine(pro_.start_.x_, pro_.start_.y_, pro_.end_.x_, pro_.end_.y_);

}

void DLSegmentItem::slot_pro(HShape::Segment pro )
{
	set_segment_pro(pro);
	set_control_pro(pro);
}

void DLSegmentItem::set_segment_pro(HShape::Segment pro )
{

	QPointF start = mapFromScene(QPointF(pro.start_.x_, pro.start_.y_));
	QPointF end = mapFromScene(QPointF(pro.end_.x_, pro.end_.y_));
	pro_.start_.x_ = start.x();
	pro_.start_.y_ = start.y();
	pro_.end_.x_ = end.x();
	pro_.end_.y_ = end.y();
	prepareGeometryChange();

}

void DLSegmentItem::set_control_pro(HShape::Segment pro)
{
	start_item_->setPos(pro_.start_.x_,pro_.start_.y_);
	end_item_->setPos(pro_.end_.x_,pro_.end_.y_);
}

void DLSegmentItem::update_shape()
{

	HShape::Segment pro=cal_pro();
	set_segment_pro(pro);

	emit sig_pro(pro_);

}

void DLSegmentItem::set_control_point_visible(bool visible)
{
	start_item_->setVisible(visible);
	end_item_->setVisible(visible);
}

HShape::Segment DLSegmentItem::cal_pro()
{

	HShape::Segment pro;
	pro.start_.x_=start_item_->scenePos().x();
	pro.start_.y_=start_item_->scenePos().y();
	pro.end_.x_=end_item_->scenePos().x();
	pro.end_.y_=end_item_->scenePos().y();

	return pro;

}

void DLSegmentItem::set_start_item(HQShapeItem *item)
{

	if (start_item_ != NULL)
	{
		start_item_->setParentItem(0);
		delete start_item_;
		start_item_ = NULL;
	}
	start_item_ = item;
	start_item_->setParentItem(this);

}

std::string DLSegmentItem::className() 
{
	return className_;
}

std::string DLSegmentItem::instanceName() 
{
	return instanceName_;
}


HShape::Segment DLSegmentItem::get_pro() 
{
	HShape::Segment pro;
	QPointF start = mapToScene(QPointF(pro_.start_.x_, pro_.start_.y_));
	QPointF end = mapToScene(QPointF(pro_.end_.x_, pro_.end_.y_));
	pro.start_.x_ = start.x();
	pro.start_.y_ = start.y();
	pro.end_.x_ = end.x();
	pro.end_.y_ = end.y();
	return pro;
}

void DLSegmentItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	set_control_point_visible(true);
	HQShapeItem::mousePressEvent(event);
}

void DLSegmentItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
	HQShapeItem::mouseDoubleClickEvent(event);
}

void DLSegmentItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	update_shape();
	HQShapeItem::mouseMoveEvent(event);
}

void DLSegmentItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mouseReleaseEvent(event);
}


