#include <QtWidgets>
#include "DLPolygonItem.h"
#include "DLControlTransformItem.h"


PolygonPropertyWidget::PolygonPropertyWidget()
{
	advancedArea_groupBox_ = new QGroupBox(QStringLiteral("高级区域设置"));

	advancedArea_base_groupBox_ = new QGroupBox(QStringLiteral("基础属性设置"));
	QGridLayout *advanceArea_base_layout = new QGridLayout;
	className_label_ = new QLabel(QStringLiteral("区域名"));
	className_lineEdit_ = new QLineEdit;
	intanceId_label_ = new QLabel(QStringLiteral("区域ID"));
	instanceId_lineEdit_ = new QLineEdit;
	//className_lineEdit_->setFocusPolicy(Qt::NoFocus);
	advanceArea_base_layout->addWidget(className_label_, 0, 0);
	advanceArea_base_layout->addWidget(className_lineEdit_, 0, 1);
	advanceArea_base_layout->addWidget(intanceId_label_, 1, 0);
	advanceArea_base_layout->addWidget(instanceId_lineEdit_, 1, 1);
	advancedArea_base_groupBox_->setLayout(advanceArea_base_layout);

	advancedArea_advanced_groupBox_ = new QGroupBox(QStringLiteral("高级属性设置"));
	QVBoxLayout *advanceArea_advanced_layout = new QVBoxLayout;
	ultrasonic_checkBox_ = new QCheckBox(QStringLiteral("是否开启超声"));
	fallingdown_checkBox_ = new QCheckBox(QStringLiteral("是否开启防跌落"));
	gyroCaliLine_checkBox_ = new QCheckBox(QStringLiteral("是否开启激光"));
	laserDevice_checkBox_ = new QCheckBox(QStringLiteral("是否开启陀螺仪"));
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

	connect(save_info_buttom_, SIGNAL(clicked()), this, SLOT(slot_on_set_advancedArea_info()));

}
	

PolygonPropertyWidget::~PolygonPropertyWidget()
{

}

void PolygonPropertyWidget::keyPressEvent(QKeyEvent *event)
{
	if (Qt::Key_Return == event->key())
	{
		slot_on_set_advancedArea_info();
		this->hide();
	}
}


void PolygonPropertyWidget::setAdvancedAreaProperty()
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


void PolygonPropertyWidget::setParent(DLPolygonItem *item)
{
	parent_ = item;
}


void PolygonPropertyWidget::slot_on_set_advancedArea_info()
{
	parent_->set_className(className_lineEdit_->text().toLocal8Bit().data());

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





/////////////////////////////////////////////



DLPolygonItem::DLPolygonItem(QVector<QPointF> points, int id, std::string className , std::string instanceName ,LineProperty lineClassName)
	: points_(points), id_(id), className_(className), instanceName_(instanceName), lineClassName_(lineClassName)
{
	set_solid(true);

	for (int i = 0; i < points.size(); ++i)
	{
		DLControlTransformItem *control_item = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
		control_item->setParentItem(this);
		control_item->setVisible(true);
		control_item_vec_.push_back(control_item);
	}
	
	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);

	set_polygon_pro(points_);
	set_control_pro(points_);

	setCursor(Qt::PointingHandCursor);

}

DLPolygonItem::DLPolygonItem(QVector<QPointF> points, int id, LineProperty lineClassName)
	: points_(points), id_(id), lineClassName_(lineClassName)
{
	set_solid(true);

	for (int i = 0; i < points.size(); ++i)
	{
		DLControlTransformItem *control_item = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
		control_item->setParentItem(this);
		control_item->setVisible(false);
		control_item_vec_.push_back(control_item);
	}

	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);

	set_polygon_pro(points_);
	set_control_pro(points_);

	setCursor(Qt::PointingHandCursor);


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

DLPolygonItem::~DLPolygonItem()
{

}

QRectF DLPolygonItem::boundingRect() const
{

	QPainterPath path=shape();
	return path.boundingRect();

}

QPainterPath DLPolygonItem::shape() const
{

	QPainterPath path;

	QVector<QPointF> temp_points(points_);
	if (points_.size() != 0) {
		temp_points.push_back(points_.first());
	}
	QPolygonF polygon(temp_points);
	path.addPolygon(polygon);
	

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

void DLPolygonItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	switch(lineClassName_)
	{
	case NormalLine:
		painter->setPen(QPen(Qt::black, 2, Qt::SolidLine));
		break;
	case VirtualLine:
		painter->setPen(QPen(Qt::blue, 2, Qt::SolidLine));
		break;
	case ForbiddenLine:
		painter->setPen(QPen(Qt::red, 2, Qt::SolidLine));
		break;
	default:
		break;

	}
	painter->setOpacity(0.7);
	painter->setPen(Qt::NoPen);
	painter->setBrush(QBrush(QColor(255, 165, 157)));
	painter->drawPath(shape());
}


std::string DLPolygonItem::className() {
	return className_;
}

std::string DLPolygonItem::instanceName() {
	return instanceName_;
}

int DLPolygonItem::id() 
{
	return id_;
}

QVector<QPointF> DLPolygonItem::get_pro() {
	return points_;
}

void DLPolygonItem::slot_pro(QVector<QPointF> pro )
{
	set_polygon_pro(pro);
	set_control_pro(pro);
}

void DLPolygonItem::set_polygon_pro(QVector<QPointF> points)
{

	points_= points;
	//通知形状更新
	prepareGeometryChange();

}

void DLPolygonItem::set_control_pro(QVector<QPointF> points)
{
	for (int i = 0; i < points.size(); ++i)
	{
		//if (control_item_vec_[i]->isSelected()) {
		//	control_item_vec_[i]->setPos(last_points_[i]);
		//}
		//else {
			control_item_vec_[i]->setPos(points_[i]);
		//}
	}
	
}

void DLPolygonItem::update_shape()
{

	QVector<QPointF> pro=cal_pro();
	set_polygon_pro(pro);

	//emit sig_pro(pro_);

}

void DLPolygonItem::set_control_point_visible(bool visible)
{
	for (int i = 0; i < control_item_vec_.size(); ++i) 
	{
		control_item_vec_[i]->setVisible(visible);
	}
}

QVector<QPointF> DLPolygonItem::cal_pro()
{

	QVector<QPointF> pro;
	for (int i = 0; i < control_item_vec_.size(); ++i)
	{
		pro.push_back(control_item_vec_[i]->scenePos());
		//last_points_[i] = control_item_vec_[i]->scenePos();
	}
	return pro;

}

void DLPolygonItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	set_control_point_visible(true);
	setCursor(Qt::ClosedHandCursor);
	HQShapeItem::mousePressEvent(event);
}

void DLPolygonItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	HQShapeItem::mouseMoveEvent(event);
}

void DLPolygonItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	setCursor(Qt::PointingHandCursor);
	HQShapeItem::mouseReleaseEvent(event);
}

void DLPolygonItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event)
{
	HQShapeItem::mouseDoubleClickEvent(event);

}

