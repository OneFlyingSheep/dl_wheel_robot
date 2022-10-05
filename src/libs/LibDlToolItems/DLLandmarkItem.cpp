#include <QtWidgets>
#include <iostream>
#include <QFont>
#include "DLLandmarkItem.h"
#include "DLControlRotateItem.h"
#include "DLBezierItem.h"
#include "math.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLSceneView/DLCustomScene.h"

#define DIRLABEL_SIZE (5)

LandMarkPropertyWidget::LandMarkPropertyWidget()
{

	landmark_base_goupBox_ = new QGroupBox("巡检点设置");

	landmark_id_label_ = new QLabel("巡检点ID");
	landmark_id_lineEdit_ = new QLineEdit();
	landmark_instancename_label_ = new QLabel("巡检点名");
	landmark_instancename_lineEdit_ = new QLineEdit();
	landmark_instancename_lineEdit_->setFocusPolicy(Qt::NoFocus);
	landmark_type_label_ = new QLabel("巡检点类型");
	is_assist_landmark_comboBox_ = new QComboBox;
	is_assist_landmark_comboBox_->addItem(TASK_LANDMARK);
	is_assist_landmark_comboBox_->addItem(ORIGIN_LANDMARK);
	is_assist_landmark_comboBox_->addItem(NODE_LANDMARK);
	is_assist_landmark_comboBox_->addItem(CALIBRATE_LANDMARK);
	is_assist_landmark_comboBox_->addItem(CHARGE_LADMARK);
	is_assist_landmark_comboBox_->addItem(CHARGEAUX_LANDMARK);

	rotate_checkBox_ = new QCheckBox("是否可以旋转");
	rotate_checkBox_->setCheckable(true);
	rotate_checkBox_->setCheckState(Qt::Checked);
	fixed_checkBox = new QCheckBox("是否是固定点");
	fixed_checkBox->setCheckable(true);
	fixed_checkBox->setCheckState(Qt::Checked);

	QHBoxLayout* nCheckBoxLayout = new QHBoxLayout;
	nCheckBoxLayout->addWidget(rotate_checkBox_);
	nCheckBoxLayout->addWidget(fixed_checkBox);

	landmark_x_label_ = new QLabel("X");
	landmark_x_lineEdit_ = new QLineEdit;
	landmark_y_label_ = new QLabel("Y");
	landmark_y_lineEdit_ = new QLineEdit;
	landmark_angel_label_ = new QLabel("方向");
	landmark_angel_lineEdit_ = new QLineEdit;
	landmark_angel_lineEdit_->setValidator(new QDoubleValidator(-180.0,180.0,4,this));

	set_location_button_ = new QPushButton("取消");
	save_info_buttom_ = new QPushButton("确认");

	QHBoxLayout *id_layout = new QHBoxLayout;
	QHBoxLayout *name_layout = new QHBoxLayout;
	QHBoxLayout *property_layout = new QHBoxLayout;
	QGridLayout *location_layout = new QGridLayout;
	QHBoxLayout *set_layout = new QHBoxLayout;

	QVBoxLayout *base_layout = new QVBoxLayout;
	id_layout->addWidget(landmark_id_label_);
	id_layout->addWidget(landmark_id_lineEdit_);
	name_layout->addWidget(landmark_instancename_label_);
	name_layout->addWidget(landmark_instancename_lineEdit_);
	property_layout->addWidget(landmark_type_label_);
	property_layout->addWidget(is_assist_landmark_comboBox_);
	location_layout->addWidget(landmark_angel_label_, 0, 0);
	location_layout->addWidget(landmark_angel_lineEdit_, 0, 1);
	location_layout->addWidget(landmark_x_label_, 1, 0);
	location_layout->addWidget(landmark_x_lineEdit_, 1, 1);
	location_layout->addWidget(landmark_y_label_, 2, 0);
	location_layout->addWidget(landmark_y_lineEdit_, 2, 1);
	set_layout->addWidget(set_location_button_);
	set_layout->addStretch();
	set_layout->addWidget(save_info_buttom_);

	base_layout->addLayout(property_layout);
	base_layout->addLayout(nCheckBoxLayout);
	base_layout->addLayout(id_layout);
	base_layout->addLayout(name_layout);
	base_layout->addLayout(location_layout);
	base_layout->addLayout(set_layout);
	landmark_base_goupBox_->setFixedSize(300, 300);
	landmark_base_goupBox_->setLayout(base_layout);

	QHBoxLayout *main_layout = new QHBoxLayout;
	main_layout->addWidget(landmark_base_goupBox_);
	this->setLayout(main_layout);
	setWindowFlags(Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);

	connect(set_location_button_, SIGNAL(clicked()), this, SLOT(slot_on_cancel()));
	connect(save_info_buttom_, SIGNAL(clicked()), this, SLOT(slot_on_set_landmark_info()));

	setWindowModality(Qt::ApplicationModal);				//设置窗口模态
}


LandMarkPropertyWidget::~LandMarkPropertyWidget()
{
}


void LandMarkPropertyWidget::slot_on_cancel()
{
	this->hide();
}


void LandMarkPropertyWidget::slot_on_set_landmark_info()
{
	//初始化pro
	HShape::Ellipse pro = parent_->get_pro();

	double angle = 0;
	double temp_angle = landmark_angel_lineEdit_->text().toDouble();
	if (temp_angle > 0 && temp_angle < 180)
	{
		angle = 360 - temp_angle;
	}
	else if(temp_angle > -180 && temp_angle < 0)
	{
		angle = 0 - temp_angle;
	}
	else if (abs(temp_angle - 180) < 0.000001 || abs(temp_angle + 180) < 0.000001) {
		angle = 180;
	}

	pro.angle_ = angle;
	pro.center_.x_ = landmark_x_lineEdit_->text().toDouble();
	pro.center_.y_ = landmark_y_lineEdit_->text().toDouble();

	std::vector<Property> propertyVec;
	Property landmark_pro;
	landmark_pro.key = "allowspin";
	landmark_pro.type = "bool";
	if (rotate_checkBox_->checkState() != Qt::Unchecked) 
	{
		parent_->set_rotate_pro(true);
		landmark_pro.value = "true";
	}
	else {
		parent_->set_rotate_pro(false);
		landmark_pro.value = "false";
	}
	propertyVec.push_back(landmark_pro);
	parent_->setPropertyList(propertyVec);

	std::vector<Add_IsFixed> isFixedVec;
	Add_IsFixed nIsFixed_pro;
	nIsFixed_pro.key = "fixed";
	nIsFixed_pro.type = "bool";
	if (fixed_checkBox->checkState() != Qt::Unchecked)
	{
		nIsFixed_pro.value = "true";
	}
	else
	{
		nIsFixed_pro.value = "false";
	}
	isFixedVec.push_back(nIsFixed_pro);
	parent_->setIsFixedList(isFixedVec);

	parent_->set_className(is_assist_landmark_comboBox_->currentText().toLocal8Bit().constData());
	parent_->set_id(landmark_id_lineEdit_->text().toInt());
	parent_->set_instanceName(landmark_instancename_lineEdit_->text().toLocal8Bit().data());


	parent_->slot_pro(pro);
	this->hide();
}


void LandMarkPropertyWidget::keyPressEvent(QKeyEvent *event)
{
	if (Qt::Key_Return == event->key())
	{
		slot_on_set_landmark_info();
	}
	else if (Qt::Key_Escape == event->key())
	{
		slot_on_cancel();
	}
}

void LandMarkPropertyWidget::setLandMarkProperty()
{
	int id = parent_->id();
	std::string className = parent_->className();
	QString instanceName = QString::number(id);

	double angle = 0;
	double temp_angle = parent_->get_pro().angle_;
	if (temp_angle > 0 && temp_angle < 180) {
		angle = 0 - temp_angle;
	}
	else if(temp_angle > 180 && temp_angle < 360){
		angle = 360 - temp_angle;
	}
	else if(abs(temp_angle-180) < 0.00001){
		angle = 180;
	}

	double x = parent_->get_pro().center_.x_;
	double y = parent_->get_pro().center_.y_;
	bool rotate_pro = parent_->rotate_pro();
	
	is_assist_landmark_comboBox_->setCurrentText(QString::fromLocal8Bit(className.c_str()));
	landmark_id_lineEdit_->setText(QString::number(id));
	landmark_instancename_lineEdit_->setText(instanceName);
	landmark_angel_lineEdit_->setText(QString::number(angle));
	landmark_x_lineEdit_->setText(QString::number(x));
	landmark_y_lineEdit_->setText(QString::number(y));
	if (rotate_pro) 
	{
		rotate_checkBox_->setCheckState(Qt::Checked);
	}
	else {
		rotate_checkBox_->setCheckState(Qt::Unchecked);
	}
}


void LandMarkPropertyWidget::setParent(DLLandmarkItem *item)
{
	parent_ = item;
}




/////////////////////////////////////////////////////////////



DLLandmarkItem::DLLandmarkItem(QPointF pos,int id,double angle,std::string instanceName, std::string className) 
	: id_(id), angle_(angle), instanceName_(instanceName), className_(className)
{	
	is_rotate_ = true;
	is_fixed_ = true;
	HShape::Ellipse pro;
	pro.center_.x_=pos.x();
	pro.center_.y_=pos.y();
	pro.angle_ = angle_;
	pro.height_=20;
	pro.width_=20;
	pro_ = pro;
	create_item(pro, id, instanceName);
}


DLLandmarkItem::DLLandmarkItem(QPointF pos,int id,std::string instanceName)
	:id_(id),instanceName_(instanceName)
{
	is_rotate_ = true;
	is_fixed_ = true;
	className_ = TASK_LANDMARK;
	HShape::Ellipse pro;
	pro.center_.x_ = pos.x();
	pro.center_.y_ = pos.y();
	pro.angle_ = angle_;
	pro.height_ = 20;
	pro.width_ = 20;
	pro_ = pro;
	create_item(pro, id, instanceName);
}


DLLandmarkItem::DLLandmarkItem(HShape::Ellipse pro,int id,std::string instanceName)
	:pro_(pro),id_(id),instanceName_(instanceName)
{
	is_rotate_ = true;
	className_ = TASK_LANDMARK;
	create_item(pro, id, instanceName);

}


void DLLandmarkItem::create_item(HShape::Ellipse pro,int id,std::string name)
{
	m_bIsDrawStateSelected = false;
	is_selected_ = false;

	Property landmark_pro;
	landmark_pro.key = "allowspin";
	landmark_pro.type = "bool";
	landmark_pro.value = "true";
	PorpertyList_.push_back(landmark_pro);

	rotate_item_=new DLControlRotateItem();
	rotate_item_->setParentItem(this);
	rotate_item_->setVisible(true);
	setZValue(100);
	
	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons( Qt::LeftButton);
	

	set_ellipse_pro(pro);
	set_control_pro(pro);

	QBrush brush(Qt::red);
	QPen pen(Qt::NoPen);

	rotate_item_->set_brush(brush);
	rotate_item_->set_pen(pen);

	init_landmark_info();

}


void DLLandmarkItem::SetDrawState(bool bIsSelected)
{
	m_bIsDrawStateSelected = bIsSelected;
	update();
}

void DLLandmarkItem::init_landmark_info()
{

}


DLLandmarkItem::~DLLandmarkItem()
{

	if(rotate_item_!=NULL)
	{
		delete rotate_item_;
		rotate_item_=NULL;
	}

}


QRectF DLLandmarkItem::boundingRect() const
{
	double center_x=0;
	double center_y=0;
	double width=pro_.width_;
	double height=pro_.height_;
	return QRectF(center_x-width/2.0,center_y-height/2.0,width,height);

}


QPainterPath DLLandmarkItem::shape() const
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
		stroker.setWidth(1);//
		return stroker.createStroke(path);
	}


}


void DLLandmarkItem::setSelecteState(bool selected)
{
	is_selected_ = selected;
	update();
}


bool DLLandmarkItem::isSlelectState()
{
	return is_selected_;
}

void DLLandmarkItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{

	//画圆
	painter->save();
	painter->setPen(Qt::NoPen);
	if (className_ == TASK_LANDMARK)
	{
		painter->setOpacity(0.9);
		painter->setBrush(QColor(245, 160, 105));		//橘黄色
	}
	else if (className_ == CHARGE_LADMARK || className_ == CHARGEAUX_LANDMARK)
	{
		painter->setOpacity(0.8);
		painter->setBrush(QColor(40, 170, 70));		//绿色
	}
	else if (className_ == ORIGIN_LANDMARK || className_ == NODE_LANDMARK) 
	{
		painter->setOpacity(0.8);
		painter->setBrush(Qt::gray);		//灰色
	}
	else
	{
		painter->setOpacity(0.8);
		painter->setBrush(QColor(255, 200, 10));		//黄色
	}
	if (m_bIsDrawStateSelected /*|| option->state & QStyle::State_Selected*/)
	{
		QBrush brush = painter->brush();
		brush.setColor(brush.color().light(150));
		painter->setOpacity(1);
		painter->setBrush(brush);
	}

	painter->drawEllipse(QRectF(-pro_.width_ / 2.0 + 2, -pro_.height_ / 2.0 + 2, pro_.width_ - 4, pro_.height_ - 4));
	painter->restore();

	//更新选中效果
// 	if(isSelected())
// 	{
// 		painter->save();
// 		painter->setPen(QPen(Qt::green, 1, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin));
// 		painter->drawEllipse(QRectF(-pro_.width_ / 2.0 + 2, -pro_.height_ / 2.0 + 2, pro_.width_ - 4, pro_.height_ - 4));
// 		painter->restore();
// 	}

	if (is_selected_ || isSelected()) 
	{
		painter->save();
		painter->setPen(QPen(Qt::green, 1, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin));
		painter->drawEllipse(QRectF(-pro_.width_ / 2.0 + 2, -pro_.height_ / 2.0 + 2, pro_.width_ - 4, pro_.height_ - 4));
		painter->restore();
	}

	QFont ft;
	ft.setPointSize(6);
	painter->setFont(ft);


	painter->save();
	painter->rotate(-pro_.angle_);
	//painter->setTransform(QTransform(QMatrix(1.0, 0.0, 0.0, -1.0, 0, 0.0)), true);
	painter->drawText(boundingRect(), Qt::AlignCenter, QString::fromStdString(instanceName_));
	painter->restore();

}


void DLLandmarkItem::set_size(int size)
{
	pro_.width_ = size;
	pro_.height_ = size;
	update();
}


int DLLandmarkItem::id() 
{
	return id_;
}


void DLLandmarkItem::set_id(int id) 
{
	id_ = id;
	update();
}


std::string DLLandmarkItem::className() {
	return className_;
}


std::string DLLandmarkItem::instanceName() {
	return instanceName_;
}


void DLLandmarkItem::set_className(std::string className)
{
	className_ = className;
	update();
}


void DLLandmarkItem::set_instanceName(std::string instanceName) {

	instanceName_ = instanceName;
	update();
}



void DLLandmarkItem::set_rotate_pro(bool state)
{
	is_rotate_ = state;
	Property pro;
	pro.key = "allowspin";
	pro.type = "bool";
	for (int i = 0; i < PorpertyList_.size(); i++) {
		if (PorpertyList_[i].key == "allowspin") {
			if (state) {
				pro.value = "true";
				PorpertyList_[0] = pro;
			}
			else {
				pro.value = "false";
				PorpertyList_[0] = pro;
			}
		}
	}

}


bool DLLandmarkItem::rotate_pro()
{
	return is_rotate_;
}


double DLLandmarkItem::angle() 
{
	return pro_.angle_;
}


HShape::Ellipse DLLandmarkItem::get_pro() 
{
	return pro_;
}


void DLLandmarkItem::setPropertyList(std::vector<Property> propertyVec) 
{
	if (propertyVec.size() == 0) {
		return;
	}
	PorpertyList_.clear();
	std::vector<Property>::iterator it;
	for (it = propertyVec.begin(); it != propertyVec.end(); ++it)
	{
		Property tempPro;
		tempPro.key = it->key;
		tempPro.type = it->type;
		tempPro.value = it->value;
		PorpertyList_.push_back(tempPro);
		if (tempPro.key == "allowspin" && tempPro.value == "true") {
			is_rotate_ = true;
		}
		else if (tempPro.key == "allowspin" && tempPro.value == "false") {
			is_rotate_ = false;
		}
	}

	DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
	if (NULL != pScene)
	{
		emit pScene->SMAPChangedSignal();
	}
}

void DLLandmarkItem::setIsFixedList(std::vector<Add_IsFixed> isfixedValues)
{
	if (isfixedValues.size() == 0)
	{
		return;
	}
	isFixedList_.clear();
	std::vector<Add_IsFixed>::iterator it;
	for (it = isfixedValues.begin(); it != isfixedValues.end(); ++it)
	{
		Add_IsFixed tempPro;
		tempPro.key = it->key;
		tempPro.type = it->type;
		tempPro.value = it->value;
		isFixedList_.push_back(tempPro);
		if (tempPro.key == "fixed" && tempPro.value == "true")
		{
			is_fixed_ = true;
		}
		else if (tempPro.key == "fixed" && tempPro.value == "false") 
		{
			is_fixed_ = false;
		}
	}
	DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
	if (NULL != pScene)
	{
		emit pScene->SMAPChangedSignal();
	}
}


std::vector<Property>& DLLandmarkItem::getPropertyList() 
{
	return PorpertyList_;
}

std::vector<Add_IsFixed>& DLLandmarkItem::getIsFixedList()
{
	return isFixedList_;
}

void DLLandmarkItem::slot_pro(HShape::Ellipse pro)
{
	set_ellipse_pro(pro);
	set_control_pro(pro);
	update_edge();
	emit sig_pro(pro_);
}


void DLLandmarkItem::set_ellipse_pro(HShape::Ellipse pro)
{
	pro_ = pro;
	setPos(pro.center_.x_,pro_.center_.y_);
	setRotation(pro_.angle_);//item 坐标系
	//通知形状更新
	prepareGeometryChange();
}


void DLLandmarkItem::set_control_pro(HShape::Ellipse pro)
{

	double delta=rotate_item_->get_length()*sqrt(3.0)/6.0;
	rotate_item_->setPos(pro_.width_/2.0+delta/2,0);

}


void DLLandmarkItem::update_edge()
{

	//printf("DLLandmarkItem::update_edge\n");
	QList<DLBezierItem*>::iterator it=edge_list_.begin();
	for(;it!=edge_list_.end();++it){
		(*it)->update_bezier();
	}
}


Landmark DLLandmarkItem::getLandmarkInfo() const
{

	return landmark_info_;

}


void DLLandmarkItem::update_shape()
{
	
	HShape::Ellipse pro=cal_pro();
	set_ellipse_pro(pro);
	set_control_pro(pro);//更新控制点

	update_edge();
	

	emit sig_pro(pro_);

}


void DLLandmarkItem::set_control_point_visible(bool visible)
{
	rotate_item_->setVisible(visible);

}


HShape::Ellipse DLLandmarkItem::cal_pro()
{

	HShape::Ellipse pro;
	//更新圆心
	pro.center_.x_=cal_center().x();
	pro.center_.y_=cal_center().y();
	
	//更新半径
	pro.angle_=cal_angle();
	pro.width_=pro_.width_;
	pro.height_=pro_.height_;

	return pro;

}


void DLLandmarkItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	set_control_point_visible(true);
	DLShapeItem::mousePressEvent(event);
}


void DLLandmarkItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{

	update_edge();
	DLShapeItem::mouseMoveEvent(event);

}


void DLLandmarkItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	DLShapeItem::mouseReleaseEvent(event);
}


QPointF DLLandmarkItem::cal_center()
{
	//中心是相对于父坐标系的
	return pos();
}


double DLLandmarkItem::cal_angle()
{
	
	//QPointF pos=rotate_item_->scenePos();//旋转是相对于scene坐标系的，一定是scenePos()而不是pos()
	QPointF item_pos=rotate_item_->pos();
	QPointF pos=mapToParent(item_pos);//这种写法比上一种写法更好，没有假设父的坐标系是scene
	QPointF center=cal_center();

	QLineF line(center,pos);
	//The return value will be in the range of values from 0.0 up to but not including 360.0.
	//The angles are measured counter-clockwise from a point on the x-axis to the right of the origin (x > 0).
	return line.angle();//degree


}


QVariant DLLandmarkItem::itemChange( GraphicsItemChange change,const QVariant & value )
{
	if (change == ItemPositionChange && scene()) {
		update_edge();

		DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
		if (NULL != pScene)
		{
			emit pScene->SMAPChangedSignal();
		}
	}
	return QGraphicsItem::itemChange(change, value);
}


