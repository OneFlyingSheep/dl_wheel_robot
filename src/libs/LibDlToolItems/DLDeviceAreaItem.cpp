#include <QtWidgets>
#include "DLDeviceAreaItem.h"
#include "DLControlTransformItem.h"
#include <iostream>



DeviceAreaPropertyWidget::DeviceAreaPropertyWidget()
{

	area_id_label_ = new QLabel(QStringLiteral("区域ID"));
	area_id_lineEdit_ = new QLineEdit();
	area_name_label_ = new QLabel(QStringLiteral("区域名"));
	area_name_lineEdit_ = new QLineEdit();
	QHBoxLayout *id_layout = new QHBoxLayout;
	QHBoxLayout *name_layout = new QHBoxLayout;
	QVBoxLayout *main_layout = new QVBoxLayout;
	id_layout->addWidget(area_id_label_);
	id_layout->addWidget(area_id_lineEdit_);
	name_layout->addWidget(area_name_label_);
	name_layout->addWidget(area_name_lineEdit_);
	main_layout->addLayout(id_layout);
	main_layout->addLayout(name_layout);
	this->setFixedSize(200, 80);
	this->setLayout(main_layout);
	setWindowFlags(Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);
	setWindowModality(Qt::ApplicationModal);						//设置窗口模态
}

DeviceAreaPropertyWidget::~DeviceAreaPropertyWidget()
{
}


void DeviceAreaPropertyWidget::setParent(DLDeviceAreaItem *item)
{
	parent_ = item;
}


void DeviceAreaPropertyWidget::slot_on_set_deviceArea_info()
{
	QString id = area_id_lineEdit_->text();
	QString instance_name = area_name_lineEdit_->text();
	parent_->set_instance_name(instance_name.toLocal8Bit().constData());
	parent_->set_id(id.toInt());
	this->hide();
}


void DeviceAreaPropertyWidget::keyPressEvent(QKeyEvent *event)
{
	if (Qt::Key_Return == event->key())
	{
		parent_->set_id(area_id_lineEdit_->text().toInt());
		parent_->set_instance_name(area_name_lineEdit_->text().toLocal8Bit().data());
		this->hide();
	}
}

void DeviceAreaPropertyWidget::setDeviceAreaProperty()
{
	int id = parent_->id();
	std::string instance_name = parent_->instanceName();
	area_id_lineEdit_->setText(QString::number(id));
	area_name_lineEdit_->setText(QString::fromLocal8Bit(instance_name.c_str()));
}



/////////////////////////////////////////////////////////////




DLDeviceAreaItem::DLDeviceAreaItem(HShape::Rectangle pro, int id, AreaProperty areaProperty)
	:pro_(pro), id_(id), areaProperty_(areaProperty)
{
	is_solid_ = false;
	setPos(pro.center_.x_, pro.center_.y_);

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
	setCursor(Qt::PointingHandCursor);

	set_rectangle_pro(pro);
	set_control_pro(pro);
	setZValue(10);

	if (areaProperty == DeviceArea) {
		className_ = "DEVICE";
	}
	else if (areaProperty == Station) {
		className_ = "STATION";
	}
}

DLDeviceAreaItem::~DLDeviceAreaItem()
{

	if (top_left_trans_item_!=NULL)
	{
		delete top_left_trans_item_;
		top_left_trans_item_=NULL;
	}

}

QRectF DLDeviceAreaItem::boundingRect() const
{
	return QRectF(0-pro_.width_/2.0,0-pro_.height_/2.0,pro_.width_,pro_.height_);
}

QPainterPath DLDeviceAreaItem::shape() const
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


void DLDeviceAreaItem::loadPixMap(QString file)
{
	std::string patj = file.toLocal8Bit();
	bool re = area_pixmap_.load(file);
	if (area_pixmap_.load(file)) {
		update();
	}
}



void DLDeviceAreaItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	painter->setRenderHint(QPainter::Antialiasing, true);
	painter->setPen(Qt::NoPen);
	switch (areaProperty_)
	{
	case DeviceArea:
		painter->setOpacity(0.5);
		painter->setBrush(QBrush(QColor(190, 190, 190)));
		break;
	case Station:
		painter->setPen(QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		break;
	default:
		break;
	}

	if (area_pixmap_.isNull()) {
		painter->drawRect(boundingRect());
	}
	else {
		painter->drawRect(boundingRect());
		painter->setTransform(QTransform(QMatrix(1.0, 0.0, 0.0, -1.0, 0, 0.0)), true);
		painter->drawPixmap(boundingRect().x(), boundingRect().y(), boundingRect().width(), boundingRect().height(), area_pixmap_);
	}
	
}

void DLDeviceAreaItem::set_control_point_visible(bool visible)
{
	top_left_trans_item_->setVisible(visible);
	top_right_trans_item_->setVisible(visible);
	bottom_left_trans_item_->setVisible(visible);
	bottom_right_trans_item_->setVisible(visible);
}


void DLDeviceAreaItem::set_deviceArea_moveable(bool moveable)
{
	setFlag(ItemIsMovable, moveable);
}


void DLDeviceAreaItem::update_shape()
{

	HShape::Rectangle pro=cal_pro();
	set_rectangle_pro(pro);
	set_control_pro(pro);

	emit sig_pro(pro_);

}

void DLDeviceAreaItem::set_rectangle_pro(HShape::Rectangle pro )
{
	
	pro_=pro;
	setPos(QPointF(pro.center_.x_,pro_.center_.y_));
	prepareGeometryChange();

}

void DLDeviceAreaItem::set_control_pro(HShape::Rectangle pro )
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

HShape::Rectangle DLDeviceAreaItem::cal_pro()
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


void DLDeviceAreaItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	setCursor(Qt::ClosedHandCursor);
	set_control_point_visible(true);
	HQShapeItem::mousePressEvent(event);
}

void DLDeviceAreaItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mouseMoveEvent(event);
}

void DLDeviceAreaItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	setCursor(Qt::PointingHandCursor);
	HQShapeItem::mouseReleaseEvent(event);
}


void DLDeviceAreaItem::slot_pro(HShape::Rectangle pro )
{
	set_rectangle_pro(pro);
	set_control_pro(pro);
}

HShape::Rectangle DLDeviceAreaItem::get_pro()
{
	return pro_;
}
