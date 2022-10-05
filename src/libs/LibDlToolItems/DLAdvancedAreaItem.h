#ifndef DLADVANCEDARE_ALEXWEI_20180504_H
#define DLADVANCEDARE_ALEXWEI_20180504_H

#include "HQShapeItem.h"
#include <QPainterPath>
#include <QWidget>

class DLControlTransformItem;
class DLSegmentItem;

class DLAdvancedAreaItem;
class QLabel;
class QLineEdit;
class QCheckBox;
class QGroupBox;
class QPushButton;


class AdvancedAreaPropertyWidget : public QWidget
{
	Q_OBJECT
public:
	AdvancedAreaPropertyWidget();
	~AdvancedAreaPropertyWidget();

	void keyPressEvent(QKeyEvent *event);
	void setAdvancedAreaProperty();
	void setParent(DLAdvancedAreaItem *item);

	private slots:
	void slot_on_set_advancedArea_info();

private:
	QGroupBox * advancedArea_groupBox_;

	QGroupBox *advancedArea_base_groupBox_;
	QLabel *className_label_;
	QLineEdit *className_lineEdit_;
	QLabel *intanceId_label_;
	QLineEdit *instanceId_lineEdit_;

	QGroupBox *advancedArea_advanced_groupBox_;
	QCheckBox *ultrasonic_checkBox_;
	QCheckBox *fallingdown_checkBox_;
	QCheckBox *gyroCaliLine_checkBox_;
	QCheckBox *laserDevice_checkBox_;
	QCheckBox *collisionDevice_checkBox_;

	QPushButton *save_info_buttom_;

private:
	DLAdvancedAreaItem * parent_;


};

class DLAdvancedAreaItem :public HQShapeItem
{
	Q_OBJECT

public:
	enum AdvancedAreaProperty
	{
		Normal = -1,
		Forbidden = 0,
		Virtual = 1
	};
	enum { Type = HShape::ADVANCEDAREA };
	int type() const
	{
		return Type;
	}
	DLAdvancedAreaItem(HShape::Rectangle pro, int id, AdvancedAreaProperty areatype = Forbidden);
	DLAdvancedAreaItem(HShape::Rectangle pro, int id, std::string className, std::string instanceName, AdvancedAreaProperty areatype);
	~DLAdvancedAreaItem();

public:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * widget = 0);


// public:
// 	void mousePressEvent(QGraphicsSceneMouseEvent * event);
// 	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
// 	void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);


public:
	int id() const {
		return id_;
	}

	void set_id(int id) {
		id_ = id;
	}

	void set_class_name(std::string class_name) {
		className_ = class_name;
	}

	void set_instance_name(std::string instance_name) {
		instanceName_ = instance_name;
	}

	std::string instanceName() {
		return instanceName_;
	}
	std::string className() {
		return className_;
	}

	void setPropertyList(std::vector<Property> PropertyVec) {
		if (PropertyVec.size() == 0) {
			return;
		}
		PorpertyList_.clear();
		std::vector<Property>::iterator it;
		for (it = PropertyVec.begin(); it != PropertyVec.end(); ++it)
		{
			Property tempPro;
			tempPro.key = it->key;
			tempPro.type = it->type;
			tempPro.value = it->value;
			PorpertyList_.push_back(tempPro);
		}
	}

	std::vector<Property>& getPropertyList() {
		return PorpertyList_;
	}

public slots:
	void slot_pro(HShape::Rectangle pro);//外部设置属性

signals:
	void sig_pro(HShape::Rectangle pro);//属性变化通知外部

public:
	void set_control_point_visible(bool visible);
	void update_shape();//供子item调用
	

private:
	//接口不开放
	void set_rectangle_pro(HShape::Rectangle pro);//
	void set_control_pro(HShape::Rectangle pro);
	HShape::Rectangle cal_pro();//控制点的移动导致形状的变化

public:
	HShape::Rectangle get_pro();

private:
	HShape::Rectangle pro_;
	int id_;
	AdvancedAreaProperty areaProperty_;
	std::string area_name_;
	std::string className_;
	std::string instanceName_;
	std::vector<Property> PorpertyList_;
	std::vector<Add_IsFixed> m_IsFixedInfoList;

private:

// 	DLControlTransformItem * top_left_trans_item_;
// 	DLControlTransformItem *top_right_trans_item_;
// 	DLControlTransformItem *bottom_left_trans_item_;
// 	DLControlTransformItem *bottom_right_trans_item_;


	QPointF last_top_left_scene_pos_;
	QPointF last_top_right_scene_pos_;
	QPointF last_bottom_left_scene_pos_;
	QPointF last_bottom_right_scene_pos_;

};


#endif