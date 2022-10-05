#ifndef DLLANDMARKITEM_ALEXWEI_20180420_H
#define DLLANDMARKITEM_ALEXWEI_20180420_H

#include "DLShapeItem.h"
#include <string>
#include <QWidget>

class DLControlRotateItem;
class DLBezierItem;
class DLLandmarkItem;
class QLabel;
class QPushButton;
class QLineEdit;
class QComboBox;
class QCheckBox;
class QGroupBox;

Q_DECLARE_METATYPE(Landmark)

#define TASK_LANDMARK			("Task")
#define ORIGIN_LANDMARK			("Origin")
#define NODE_LANDMARK			("Node")
#define CALIBRATE_LANDMARK		("Calibrate")
#define CHARGE_LADMARK			("Charge")
#define CHARGEAUX_LANDMARK		("ChargeAux")


class LandMarkPropertyWidget : public QWidget
{
	Q_OBJECT
public:
	LandMarkPropertyWidget();
	~LandMarkPropertyWidget();

	void keyPressEvent(QKeyEvent *event);
	void setLandMarkProperty();
	void setParent(DLLandmarkItem *item);

private slots:
	void slot_on_cancel();
	void slot_on_set_landmark_info();

private:
	QGroupBox * landmark_base_goupBox_;

	QLabel * landmark_id_label_;
	QLineEdit *landmark_id_lineEdit_;
	QLabel *landmark_instancename_label_;
	QLineEdit *landmark_instancename_lineEdit_;
	QLabel *landmark_type_label_;
	QComboBox *is_assist_landmark_comboBox_;
	QCheckBox *rotate_checkBox_;
	QCheckBox* fixed_checkBox;

	QLabel *landmark_angel_label_;
	QLineEdit *landmark_angel_lineEdit_;
	QLabel *landmark_x_label_;
	QLineEdit *landmark_x_lineEdit_;
	QLabel *landmark_y_label_;
	QLineEdit *landmark_y_lineEdit_;
	QPushButton *set_location_button_;
	QPushButton *save_info_buttom_;

private:
	DLLandmarkItem * parent_;
};


class DLLandmarkItem:public DLShapeItem
{
	Q_OBJECT
	
public:
	enum { 
		Type = HShape::LANDMARKITEM 
	};
	int type() const
	{
		return Type;
	}
	DLLandmarkItem(QPointF pos,int id,double angle,std::string instanceName,std::string className);
	DLLandmarkItem(QPointF pos,int id,std::string name);
	DLLandmarkItem(HShape::Ellipse pro,int id=0,std::string name="");
	~DLLandmarkItem();
	
	void create_item(HShape::Ellipse pro,int id,std::string name);

	void SetDrawState(bool bIsSelected);
	void set_ellipse_pro(HShape::Ellipse pro);//

public:

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);
	void setSelecteState(bool selected);
	bool isSlelectState();

	void set_size(int size = 20);
	int id();
	void set_id(int id);
	std::string className();
	std::string instanceName();
	void set_className(std::string className = TASK_LANDMARK);
	void set_instanceName(std::string instanceName);
	void set_rotate_pro(bool state);
	bool rotate_pro();
	double angle();
	HShape::Ellipse get_pro();
	
	void setPropertyList(std::vector<Property> propertyVec);
	void setIsFixedList(std::vector<Add_IsFixed> isfixedValues);

	std::vector<Property>& getPropertyList();
	std::vector<Add_IsFixed>& getIsFixedList();

	void init_landmark_info();
	Landmark getLandmarkInfo() const;

public slots:
	void slot_pro(HShape::Ellipse pro);//外部设置属性

signals:
	void sig_pro(HShape::Ellipse pro);//属性变化通知外部


public:

	void update_shape();//供子item调用
	void update_edge();
	void set_control_point_visible(bool visible);

	void add_edge(DLBezierItem* item){
		edge_list_.push_back(item);
	}

	void remove_edge(DLBezierItem *item){
		QList<DLBezierItem*>::iterator it=edge_list_.begin();
		for(;it!=edge_list_.end();++it){
			if((*it)==item){
				it=edge_list_.erase(it);
			}
		}
	}

	

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);
	QVariant itemChange(GraphicsItemChange change,const QVariant & value);

private:
	//接口不开放
	void set_control_pro(HShape::Ellipse pro);
	void set_edge();

	//HShape::Ellipse get_pro();

private:
	HShape::Ellipse pro_;
	HShape::Ellipse cal_pro();//控制点的移动导致形状的变化
	QPointF cal_center();//计算中心点
	double cal_angle();

private:
	DLControlRotateItem *rotate_item_;
	QList<DLBezierItem*> edge_list_;
	Landmark landmark_info_;

private:
	int id_;
	double angle_;
	bool is_selected_;		//自定义的选中
	bool is_rotate_;
	bool is_fixed_;
	std::string instanceName_;
	std::string className_;
	std::vector<Property> PorpertyList_;
	std::vector<Add_IsFixed> isFixedList_;
	bool m_bIsDrawStateSelected;								//在绘制状态下的选中状态

};


#endif