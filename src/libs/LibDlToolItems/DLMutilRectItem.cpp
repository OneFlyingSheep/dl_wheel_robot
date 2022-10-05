#include <QtGui>
#include "DLMutilRectItem.h"
#include "DLControlTransformItem.h"
#include <iostream>


DLMutilRectItem::DLMutilRectItem(HShape::Rectangle pro, int count)
	: pro_(pro), count_(count)
{
	is_solid_ = false;

	top_left_trans_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	top_right_trans_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	bottom_left_trans_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	bottom_right_trans_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);

	top_left_trans_item_->setParentItem(this);
	top_right_trans_item_->setParentItem(this);
	bottom_left_trans_item_->setParentItem(this);
	bottom_right_trans_item_->setParentItem(this);

	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);
	setCursor(Qt::SizeAllCursor);

	set_rectangle_pro(pro);
	set_control_pro(pro);

}

DLMutilRectItem::~DLMutilRectItem()
{

	if (top_left_trans_item_!=NULL)
	{
		delete top_left_trans_item_;
		top_left_trans_item_=NULL;
	}

}

QRectF DLMutilRectItem::boundingRect() const
{
	return QRectF(0-pro_.width_/2.0,0-pro_.height_/2.0,pro_.width_,pro_.height_);
}

QPainterPath DLMutilRectItem::shape() const
{

	QPainterPath path;
	path.addEllipse(boundingRect());//


	if(is_solid_==true)
	{
		return path;
	}
	else
	{
		QPainterPathStroker stroker;
		stroker.setWidth(40);//
		return stroker.createStroke(path);	
	}


}

void DLMutilRectItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	painter->setPen(get_pen());
	painter->setBrush(get_brush());
	painter->drawRect(boundingRect());
	QVector<QLineF> lines;
	double single_step = pro_.width_ / count_;
	double start_x = 0 - pro_.width_ / 2.0;
	double start_y = 0 - pro_.height_ / 2.0;
	double end_y = pro_.height_ / 2.0;
	for(int i = 1; i < count_; ++i)
	{
		QPointF start_pos, end_pos;
		start_pos.setX(start_x + i * single_step);
		start_pos.setY(start_y);
		end_pos.setX(start_x + i * single_step);
		end_pos.setY(end_y);
		painter->drawLine(start_pos, end_pos);
	}


}

void DLMutilRectItem::update_shape()
{

	HShape::Rectangle pro=cal_pro();
	set_rectangle_pro(pro);
	set_control_pro(pro);

	emit sig_pro(pro_);

}

void DLMutilRectItem::set_rectangle_pro(HShape::Rectangle pro )
{
	
	pro_=pro;
	setPos(QPointF(pro.center_.x_,pro_.center_.y_));
	prepareGeometryChange();

}

void DLMutilRectItem::set_control_pro(HShape::Rectangle pro )
{
	
	if (top_left_trans_item_->isSelected())
	{

		QPointF center=QPointF(pro_.center_.x_,pro_.center_.y_);
		QPointF top_left=last_top_left_scene_pos_-center;
		QPointF top_right=QPointF(last_bottom_right_scene_pos_.x(),last_top_left_scene_pos_.y())-center;
		QPointF bottom_right=last_bottom_right_scene_pos_-center;
		QPointF bottom_left=QPointF(last_top_left_scene_pos_.x(),last_bottom_right_scene_pos_.y())-center;

		top_left_trans_item_->setPos(top_left);
		top_right_trans_item_->setPos(top_right);
		bottom_right_trans_item_->setPos(bottom_right);
		bottom_left_trans_item_->setPos(bottom_left);


	}
	else if (top_right_trans_item_->isSelected())
	{

		QPointF center=QPointF(pro_.center_.x_,pro_.center_.y_);
		QPointF top_left=QPointF(last_bottom_left_scene_pos_.x(),last_top_right_scene_pos_.y())-center;
		QPointF top_right=last_top_right_scene_pos_-center;
		QPointF bottom_right=QPointF(last_top_right_scene_pos_.x(),last_bottom_left_scene_pos_.y())-center;
		QPointF bottom_left=last_bottom_left_scene_pos_-center;

		top_left_trans_item_->setPos(top_left);
		top_right_trans_item_->setPos(top_right);
		bottom_right_trans_item_->setPos(bottom_right);
		bottom_left_trans_item_->setPos(bottom_left);

	}
	else if (bottom_right_trans_item_->isSelected())
	{


		QPointF center=QPointF(pro_.center_.x_,pro_.center_.y_);
		QPointF top_left=last_top_left_scene_pos_-center;
		QPointF top_right=QPointF(last_bottom_right_scene_pos_.x(),last_top_left_scene_pos_.y())-center;
		QPointF bottom_right=last_bottom_right_scene_pos_-center;
		QPointF bottom_left=QPointF(last_top_left_scene_pos_.x(),last_bottom_right_scene_pos_.y())-center;

		top_left_trans_item_->setPos(top_left);
		top_right_trans_item_->setPos(top_right);
		bottom_right_trans_item_->setPos(bottom_right);
		bottom_left_trans_item_->setPos(bottom_left);


	}
	else if (bottom_left_trans_item_->isSelected())
	{
		QPointF center=QPointF(pro_.center_.x_,pro_.center_.y_);
		QPointF top_left=QPointF(last_bottom_left_scene_pos_.x(),last_top_right_scene_pos_.y())-center;
		QPointF top_right=last_top_right_scene_pos_-center;
		QPointF bottom_right=QPointF(last_top_right_scene_pos_.x(),last_bottom_left_scene_pos_.y())-center;
		QPointF bottom_left=last_bottom_left_scene_pos_-center;

		top_left_trans_item_->setPos(top_left);
		top_right_trans_item_->setPos(top_right);
		bottom_right_trans_item_->setPos(bottom_right);
		bottom_left_trans_item_->setPos(bottom_left);

	}
	else{

		//这个函数可以用下面这段
		QPointF top_left=QPointF(-pro_.width_/2.0,-pro_.height_/2.0);
		QPointF top_right=QPointF(pro_.width_/2.0,-pro_.height_/2.0);
		QPointF bottom_left=QPointF(-pro_.width_/2.0,pro_.height_/2.0);
		QPointF bottom_right=QPointF(pro_.width_/2.0,pro_.height_/2.0);

		top_left_trans_item_->setPos(top_left);
		top_right_trans_item_->setPos(top_right);
		bottom_right_trans_item_->setPos(bottom_right);
		bottom_left_trans_item_->setPos(bottom_left);

	}


}

HShape::Rectangle DLMutilRectItem::cal_pro()
{

	HShape::Rectangle pro;

	{

		QPointF center=QPointF(0,0);
		double width=0;
		double height=0;

		QPointF p1=QPointF(0,0);
		QPointF p2=QPointF(0,0);

		if (top_left_trans_item_->isSelected())
		{
			p1=top_left_trans_item_->scenePos();
			p2=bottom_right_trans_item_->scenePos();
		}
		else if (top_right_trans_item_->isSelected())
		{
			p1=top_right_trans_item_->scenePos();
			p2=bottom_left_trans_item_->scenePos();
		}
		else if (bottom_right_trans_item_->isSelected())
		{
			p1=bottom_right_trans_item_->scenePos();
			p2=top_left_trans_item_->scenePos();

		}
		else if (bottom_left_trans_item_->isSelected())
		{

			p1=bottom_left_trans_item_->scenePos();
			p2=top_right_trans_item_->scenePos();
		}
		else{
			p1=top_left_trans_item_->scenePos();
			p2=bottom_right_trans_item_->scenePos();
		}


		width=fabs(p2.x()-p1.x());
		height=fabs(p2.y()-p1.y());
		center=QPointF((p1.x()+p2.x())/2.0,(p1.y()+p2.y())/2.0);


		pro.center_.x_=center.x();
		pro.center_.y_=center.y();
		pro.width_=width;
		pro.height_=height;


	}


	{

		last_top_left_scene_pos_=top_left_trans_item_->scenePos();
		last_top_right_scene_pos_=top_right_trans_item_->scenePos();
		last_bottom_left_scene_pos_=bottom_left_trans_item_->scenePos();
		last_bottom_right_scene_pos_=bottom_right_trans_item_->scenePos();
	
	}

	return pro;

}


void DLMutilRectItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mousePressEvent(event);

}

void DLMutilRectItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mouseMoveEvent(event);
}

void DLMutilRectItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mouseReleaseEvent(event);
}

void DLMutilRectItem::slot_pro(HShape::Rectangle pro )
{
	set_rectangle_pro(pro);
	set_control_pro(pro);
}

HShape::Rectangle DLMutilRectItem::get_pro()
{
	return pro_;
}

int DLMutilRectItem::get_count()
{
	return count_;
}
