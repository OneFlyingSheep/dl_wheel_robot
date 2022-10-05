#include <QtWidgets>
#include "DLAdvancedAreaItem.h"
#include "DLControlTransformItem.h"


AdvancedAreaPropertyWidget::AdvancedAreaPropertyWidget()
{
	advancedArea_groupBox_ = new QGroupBox(QStringLiteral("高级区域设置"));

	advancedArea_base_groupBox_ = new QGroupBox(QStringLiteral("基础属性设置"));
	QGridLayout *advanceArea_base_layout = new QGridLayout;
	className_label_ = new QLabel(QStringLiteral("区域名"));
	className_lineEdit_ = new QLineEdit;
	intanceId_label_ = new QLabel(QStringLiteral("区域ID"));
	instanceId_lineEdit_ = new QLineEdit;
	instanceId_lineEdit_->setFocusPolicy(Qt::NoFocus);
	advanceArea_base_layout->addWidget(className_label_, 0, 0);
	advanceArea_base_layout->addWidget(className_lineEdit_, 0, 1);
	advanceArea_base_layout->addWidget(intanceId_label_, 1, 0);
	advanceArea_base_layout->addWidget(instanceId_lineEdit_, 1, 1);
	advancedArea_base_groupBox_->setLayout(advanceArea_base_layout);

	advancedArea_advanced_groupBox_ = new QGroupBox(QStringLiteral("高级属性设置"));
	QVBoxLayout *advanceArea_advanced_layout = new QVBoxLayout;
	ultrasonic_checkBox_ = new QCheckBox(QStringLiteral("是否开启超声"));
	fallingdown_checkBox_ = new QCheckBox(QStringLiteral("是否开启防跌落"));
	laserDevice_checkBox_ = new QCheckBox(QStringLiteral("是否开启激光"));
	gyroCaliLine_checkBox_ = new QCheckBox(QStringLiteral("是否开启陀螺仪"));
	collisionDevice_checkBox_ = new QCheckBox(QStringLiteral("是否开启碰撞传感器")); 
	ultrasonic_checkBox_->setCheckable(true);
	fallingdown_checkBox_->setCheckable(true);
	gyroCaliLine_checkBox_->setCheckable(true);
	laserDevice_checkBox_->setCheckable(true);
	collisionDevice_checkBox_->setCheckable(true);
	advanceArea_advanced_layout->addWidget(ultrasonic_checkBox_);
	advanceArea_advanced_layout->addWidget(fallingdown_checkBox_);
	advanceArea_advanced_layout->addWidget(gyroCaliLine_checkBox_);
	advanceArea_advanced_layout->addWidget(laserDevice_checkBox_);
	advanceArea_advanced_layout->addWidget(collisionDevice_checkBox_);
	advancedArea_advanced_groupBox_->setLayout(advanceArea_advanced_layout);

	QHBoxLayout* save_button_layout = new QHBoxLayout;
	save_info_buttom_ = new QPushButton(QStringLiteral("确认"));
	save_button_layout->addStretch();
	save_button_layout->addWidget(save_info_buttom_);


	QVBoxLayout *main_layout = new QVBoxLayout;
	main_layout->addWidget(advancedArea_base_groupBox_);
	main_layout->addWidget(advancedArea_advanced_groupBox_);
	main_layout->addLayout(save_button_layout);
	this->setLayout(main_layout);
	this->setFixedWidth(250);
	setWindowFlags(Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);
	setWindowModality(Qt::ApplicationModal);				//设置窗口模态


	connect(save_info_buttom_, SIGNAL(clicked()), this, SLOT(slot_on_set_advancedArea_info()));

}


AdvancedAreaPropertyWidget::~AdvancedAreaPropertyWidget()
{

}

void AdvancedAreaPropertyWidget::keyPressEvent(QKeyEvent *event)
{
	if (Qt::Key_Return == event->key())
	{
		slot_on_set_advancedArea_info();
		this->hide();
	}
}


void AdvancedAreaPropertyWidget::setAdvancedAreaProperty()
{
	instanceId_lineEdit_->setText(QString::fromLocal8Bit(parent_->instanceName().c_str()));
	className_lineEdit_->setText(QString::fromLocal8Bit(parent_->className().c_str()));

	std::vector<Property> PropertyVev = parent_->getPropertyList();
	if (PropertyVev.size() == 5)
	{
		if (PropertyVev[0].value == "true")
			ultrasonic_checkBox_->setCheckState(Qt::Checked);
		else
			ultrasonic_checkBox_->setCheckState(Qt::Unchecked);


		if (PropertyVev[1].value == "true")
			fallingdown_checkBox_->setCheckState(Qt::Checked);
		else
			fallingdown_checkBox_->setCheckState(Qt::Unchecked);


		if (PropertyVev[2].value == "true")
			gyroCaliLine_checkBox_->setCheckState(Qt::Checked);
		else
			gyroCaliLine_checkBox_->setCheckState(Qt::Unchecked);



		if (PropertyVev[3].value == "true")
			laserDevice_checkBox_->setCheckState(Qt::Checked);
		else
			laserDevice_checkBox_->setCheckState(Qt::Unchecked);


		if (PropertyVev[4].value == "true")
			collisionDevice_checkBox_->setCheckState(Qt::Checked);
		else
			collisionDevice_checkBox_->setCheckState(Qt::Unchecked);
	}
}


void AdvancedAreaPropertyWidget::setParent(DLAdvancedAreaItem *item)
{
	parent_ = item;
}


void AdvancedAreaPropertyWidget::slot_on_set_advancedArea_info()
{
	parent_->set_class_name(className_lineEdit_->text().toLocal8Bit().data());
	parent_->set_instance_name(instanceId_lineEdit_->text().toLocal8Bit().data());
	parent_->set_id(instanceId_lineEdit_->text().toInt());

	Property pro_Ultrasonic("Ultrasonic", "bool");
	Property pro_Fallingdown("Fallingdown", "bool");
	Property pro_GyroCaliLine("GyroCaliLine", "bool");
	Property pro_LaserDevice("LaserDevice", "bool");
	Property pro_CollisionDevice("CollisionDevice", "bool");


	if (ultrasonic_checkBox_->checkState() != Qt::Unchecked)
		pro_Ultrasonic.value = "true";
	else
		pro_Ultrasonic.value = "false";

	if (fallingdown_checkBox_->checkState() != Qt::Unchecked)
		pro_Fallingdown.value = "true";
	else
		pro_Fallingdown.value = "false";

	if (gyroCaliLine_checkBox_->checkState() != Qt::Unchecked)
		pro_GyroCaliLine.value = "true";
	else
		pro_GyroCaliLine.value = "false";

	if (laserDevice_checkBox_->checkState() != Qt::Unchecked)
		pro_LaserDevice.value = "true";
	else
		pro_LaserDevice.value = "false";

	if (collisionDevice_checkBox_->checkState() != Qt::Unchecked)
		pro_CollisionDevice.value = "true";
	else
		pro_CollisionDevice.value = "false";

	std::vector<Property> porpertyvec;
	porpertyvec.push_back(pro_Ultrasonic);
	porpertyvec.push_back(pro_Fallingdown);
	porpertyvec.push_back(pro_GyroCaliLine);
	porpertyvec.push_back(pro_LaserDevice);
	porpertyvec.push_back(pro_CollisionDevice);

	parent_->setPropertyList(porpertyvec);
	this->hide();
}




/////////////////////////////////////////////////////////////

DLAdvancedAreaItem::DLAdvancedAreaItem(HShape::Rectangle pro, int id, std::string className, std::string instanceName, AdvancedAreaProperty areatype)
	: pro_(pro), id_(id), className_(className), instanceName_(std::to_string(id)), areaProperty_(areatype)
{
	set_solid(true);

	setPos(pro.center_.x_, pro.center_.y_);

// 	top_left_trans_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
// 	top_right_trans_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
// 	bottom_left_trans_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
// 	bottom_right_trans_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
// 
// 	top_left_trans_item_->setParentItem(this);
// 	top_right_trans_item_->setParentItem(this);
// 	bottom_left_trans_item_->setParentItem(this);
// 	bottom_right_trans_item_->setParentItem(this);

 	setFlag(ItemIsSelectable, true);
// 	setFlag(ItemIsMovable, true);
// 	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);
	setCursor(Qt::PointingHandCursor);

	set_rectangle_pro(pro);
	set_control_pro(pro);
	setZValue(21);

	className_ = "AdvancedArea";

	Property pro_Ultrasonic("Ultrasonic", "bool", "true");
	Property pro_Fallingdown("Fallingdown", "bool", "true");
	Property pro_GyroCaliLine("GyroCaliLine", "bool", "true");
	Property pro_LaserDevice("LaserDevice", "bool", "true");
	Property pro_CollisionDevice("CollisionDevice", "bool", "true");

	PorpertyList_.push_back(pro_Ultrasonic);
	PorpertyList_.push_back(pro_Fallingdown);
	PorpertyList_.push_back(pro_GyroCaliLine);
	PorpertyList_.push_back(pro_LaserDevice);
	PorpertyList_.push_back(pro_CollisionDevice);

}


DLAdvancedAreaItem::DLAdvancedAreaItem(HShape::Rectangle pro, int id, AdvancedAreaProperty areaProperty)
	:pro_(pro), id_(id), areaProperty_(areaProperty)
{
	set_solid(true);

	setPos(pro.center_.x_, pro.center_.y_);
// 
// 	top_left_trans_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
// 	top_right_trans_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
// 	bottom_left_trans_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
// 	bottom_right_trans_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
// 
// 	top_left_trans_item_->setParentItem(this);
// 	top_right_trans_item_->setParentItem(this);
// 	bottom_left_trans_item_->setParentItem(this);
// 	bottom_right_trans_item_->setParentItem(this);

	setFlag(ItemIsSelectable, true);
	setFlag(ItemIsMovable, false);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);
	setCursor(Qt::PointingHandCursor);

	set_rectangle_pro(pro);
	set_control_pro(pro);
	setZValue(21);

	className_ = "AdvancedArea";
	instanceName_ = std::to_string(id);

	Property pro_Ultrasonic("Ultrasonic", "bool", "true");
	Property pro_Fallingdown("Fallingdown", "bool", "true");
	Property pro_GyroCaliLine("GyroCaliLine", "bool", "true");
	Property pro_LaserDevice("LaserDevice", "bool", "true");
	Property pro_CollisionDevice("CollisionDevice", "bool", "true");

	PorpertyList_.push_back(pro_Ultrasonic);
	PorpertyList_.push_back(pro_Fallingdown);
	PorpertyList_.push_back(pro_GyroCaliLine);
	PorpertyList_.push_back(pro_LaserDevice);
	PorpertyList_.push_back(pro_CollisionDevice);
}


DLAdvancedAreaItem::~DLAdvancedAreaItem()
{

}


QRectF DLAdvancedAreaItem::boundingRect() const
{
	return QRectF(0 - pro_.width_ / 2.0, 0 - pro_.height_ / 2.0, pro_.width_, pro_.height_);
}


QPainterPath DLAdvancedAreaItem::shape() const
{

	QPainterPath path;
	path.addRect(boundingRect());//

	if (is_solid_ == true)
	{
		return path;
	}
	else
	{
		QPainterPathStroker stroker;
		stroker.setWidth(40);
		return stroker.createStroke(path);
	}


}


void DLAdvancedAreaItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * widget /*= 0*/)
{
	painter->setRenderHint(QPainter::Antialiasing, true);
	//switch (areaProperty_)
	//{
	//case Normal:
	//	painter->setPen(QPen(Qt::black, 2, Qt::SolidLine));
	//	break;
	//case Virtual:
	//	painter->setPen(QPen(Qt::blue, 2, Qt::SolidLine));
	//	break;
	//case Forbidden:
	//	painter->setPen(QPen(Qt::red, 2, Qt::SolidLine));
	//	break;
	//default:
	//	break;

	//}
	if (option->state & QStyle::State_Selected)
	{//如果是选中线
		painter->save();
		QPen pen;
		pen.setColor(Qt::green);
		pen.setStyle(Qt::DotLine);
		painter->setPen(pen);
		QPainterPathStroker path_stroker;
		path_stroker.setWidth(3);
		painter->drawPath(shape());
		painter->restore();
	}

	painter->setOpacity(0.7);
	painter->setPen(Qt::NoPen);
	painter->setBrush(QBrush(QColor(255, 165, 157)));
	painter->drawPath(shape());
}


void DLAdvancedAreaItem::set_control_point_visible(bool visible)
{
// 	top_left_trans_item_->setVisible(visible);
// 	top_right_trans_item_->setVisible(visible);
// 	bottom_left_trans_item_->setVisible(visible);
// 	bottom_right_trans_item_->setVisible(visible);
}


void DLAdvancedAreaItem::update_shape()
{

	//HShape::Rectangle pro = cal_pro();
// 	set_rectangle_pro(pro);
// 	set_control_pro(pro);
// 
// 	emit sig_pro(pro_);

}


void DLAdvancedAreaItem::set_rectangle_pro(HShape::Rectangle pro)
{

	pro_ = pro;
	setPos(QPointF(pro.center_.x_, pro_.center_.y_));
	prepareGeometryChange();

}


void DLAdvancedAreaItem::set_control_pro(HShape::Rectangle pro)
{

	//if (top_left_trans_item_->isSelected())
	//	{
	//
	//		QPointF center = QPointF(pro_.center_.x_, pro_.center_.y_);
	//		QPointF top_left = last_top_left_scene_pos_ - center;
	//		QPointF top_right = QPointF(last_bottom_right_scene_pos_.x(), last_top_left_scene_pos_.y()) - center;
	//		QPointF bottom_right = last_bottom_right_scene_pos_ - center;
	//		QPointF bottom_left = QPointF(last_top_left_scene_pos_.x(), last_bottom_right_scene_pos_.y()) - center;
	//
	//		top_left_trans_item_->setPos(top_left);
	//		top_right_trans_item_->setPos(top_right);
	//		bottom_right_trans_item_->setPos(bottom_right);
	//		bottom_left_trans_item_->setPos(bottom_left);
	//
	//
	//	}
	//	//else if (top_right_trans_item_->isSelected())
	//	{
	//
	//		QPointF center = QPointF(pro_.center_.x_, pro_.center_.y_);
	//		QPointF top_left = QPointF(last_bottom_left_scene_pos_.x(), last_top_right_scene_pos_.y()) - center;
	//		QPointF top_right = last_top_right_scene_pos_ - center;
	//		QPointF bottom_right = QPointF(last_top_right_scene_pos_.x(), last_bottom_left_scene_pos_.y()) - center;
	//		QPointF bottom_left = last_bottom_left_scene_pos_ - center;
	//
	//		top_left_trans_item_->setPos(top_left);
	//		top_right_trans_item_->setPos(top_right);
	//		bottom_right_trans_item_->setPos(bottom_right);
	//		bottom_left_trans_item_->setPos(bottom_left);
	//
	//	}
	//	//else if (bottom_right_trans_item_->isSelected())
	//	{
	//
	//
	//		QPointF center = QPointF(pro_.center_.x_, pro_.center_.y_);
	//		QPointF top_left = last_top_left_scene_pos_ - center;
	//		QPointF top_right = QPointF(last_bottom_right_scene_pos_.x(), last_top_left_scene_pos_.y()) - center;
	//		QPointF bottom_right = last_bottom_right_scene_pos_ - center;
	//		QPointF bottom_left = QPointF(last_top_left_scene_pos_.x(), last_bottom_right_scene_pos_.y()) - center;
	//
	//		top_left_trans_item_->setPos(top_left);
	//		top_right_trans_item_->setPos(top_right);
	//		bottom_right_trans_item_->setPos(bottom_right);
	//		bottom_left_trans_item_->setPos(bottom_left);
	//
	//
	//	}
	//	//else if (bottom_left_trans_item_->isSelected())
	//	{
	//		QPointF center = QPointF(pro_.center_.x_, pro_.center_.y_);
	//		QPointF top_left = QPointF(last_bottom_left_scene_pos_.x(), last_top_right_scene_pos_.y()) - center;
	//		QPointF top_right = last_top_right_scene_pos_ - center;
	//		QPointF bottom_right = QPointF(last_top_right_scene_pos_.x(), last_bottom_left_scene_pos_.y()) - center;
	//		QPointF bottom_left = last_bottom_left_scene_pos_ - center;
	//
	//		top_left_trans_item_->setPos(top_left);
	//		top_right_trans_item_->setPos(top_right);
	//		bottom_right_trans_item_->setPos(bottom_right);
	//		bottom_left_trans_item_->setPos(bottom_left);
	//
	//	}
	//	//else 
	//	{
	//
	//		//这个函数可以用下面这段
	//		QPointF top_left = QPointF(-pro_.width_ / 2.0, -pro_.height_ / 2.0);
	//		QPointF top_right = QPointF(pro_.width_ / 2.0, -pro_.height_ / 2.0);
	//		QPointF bottom_left = QPointF(-pro_.width_ / 2.0, pro_.height_ / 2.0);
	//		QPointF bottom_right = QPointF(pro_.width_ / 2.0, pro_.height_ / 2.0);
	//
	//		top_left_trans_item_->setPos(top_left);
	//		top_right_trans_item_->setPos(top_right);
	//		bottom_right_trans_item_->setPos(bottom_right);
	//		bottom_left_trans_item_->setPos(bottom_left);
	//
	//	}


}


HShape::Rectangle DLAdvancedAreaItem::cal_pro()
{

	HShape::Rectangle pro;

	{
		QPointF center = QPointF(0, 0);
		double width = 0;
		double height = 0;

		QPointF p1 = QPointF(0, 0);
		QPointF p2 = QPointF(0, 0);

// 		if (top_left_trans_item_->isSelected())
// 		{
// 			p1 = top_left_trans_item_->scenePos();
// 			p2 = bottom_right_trans_item_->scenePos();
// 		}
// 		else if (top_right_trans_item_->isSelected())
// 		{
// 			p1 = top_right_trans_item_->scenePos();
// 			p2 = bottom_left_trans_item_->scenePos();
// 		}
// 		else if (bottom_right_trans_item_->isSelected())
// 		{
// 			p1 = bottom_right_trans_item_->scenePos();
// 			p2 = top_left_trans_item_->scenePos();
// 
// 		}
// 		else if (bottom_left_trans_item_->isSelected())
// 		{
// 
// 			p1 = bottom_left_trans_item_->scenePos();
// 			p2 = top_right_trans_item_->scenePos();
// 		}
// 		else {
// 			p1 = top_left_trans_item_->scenePos();
// 			p2 = bottom_right_trans_item_->scenePos();
// 		}


		width = fabs(p2.x() - p1.x());
		height = fabs(p2.y() - p1.y());
		center = QPointF((p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0);


		pro.center_.x_ = center.x();
		pro.center_.y_ = center.y();
		pro.width_ = width;
		pro.height_ = height;


	}

	{

// 		last_top_left_scene_pos_ = top_left_trans_item_->scenePos();
// 		last_top_right_scene_pos_ = top_right_trans_item_->scenePos();
// 		last_bottom_left_scene_pos_ = bottom_left_trans_item_->scenePos();
// 		last_bottom_right_scene_pos_ = bottom_right_trans_item_->scenePos();

	}

	return pro;

}


// void DLAdvancedAreaItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
// {
// 	setCursor(Qt::ClosedHandCursor);
// 	set_control_point_visible(true);
// 	HQShapeItem::mousePressEvent(event);
// }
// 
// 
// void DLAdvancedAreaItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
// {
// 	HQShapeItem::mouseMoveEvent(event);
// }
// 
// 
// void DLAdvancedAreaItem::mouseReleaseEvent(QGraphicsSceneMouseEvent * event)
// {
// 	setCursor(Qt::PointingHandCursor);
// 	HQShapeItem::mouseReleaseEvent(event);
// }


void DLAdvancedAreaItem::slot_pro(HShape::Rectangle pro)
{
	set_rectangle_pro(pro);
	set_control_pro(pro);
}


HShape::Rectangle DLAdvancedAreaItem::get_pro()
{
	return pro_;
}
