#ifndef DLDEVICEAREAITEM_ALEXWEI_20180504_H
#define DLDEVICEAREAITEM_ALEXWEI_20180504_H

#include "HQShapeItem.h"
#include <QWidget>

class DLControlTransformItem;
class QLabel;
class QLineEdit;
class DLDeviceAreaItem;

class DeviceAreaPropertyWidget : public QWidget
{
	Q_OBJECT
public:
	DeviceAreaPropertyWidget();
	~DeviceAreaPropertyWidget();

	void setDeviceAreaProperty();
	void setParent(DLDeviceAreaItem *item);

protected:
	virtual void keyPressEvent(QKeyEvent *event);

private slots:
	void slot_on_set_deviceArea_info();

private:
	QLabel * area_id_label_;
	QLineEdit *area_id_lineEdit_;
	QLabel *area_name_label_;
	QLineEdit *area_name_lineEdit_;

private:
	DLDeviceAreaItem *parent_;

};

class DLDeviceAreaItem :public HQShapeItem
{
	Q_OBJECT
	
public:
	enum AreaProperty 
	{
		DeviceArea,
		Station
	};
	enum { Type = HShape::DEVICEAREAITEM};
	int type() const
	{
		return Type;
	}
	DLDeviceAreaItem(HShape::Rectangle pro, int id, AreaProperty lineClassName = DeviceArea);
	~DLDeviceAreaItem();

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
		area_name_ = instance_name;
		instanceName_ = instance_name;
	}

	void loadPixMap(QString file);

	void set_control_point_visible(bool visible);
	void set_deviceArea_moveable(bool moveable);

protected:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

public slots:
	void slot_pro(HShape::Rectangle pro);//外部设置属性

signals:
	void sig_pro(HShape::Rectangle pro);//属性变化通知外部

public:

	void update_shape();//供子item调用
	std::string instanceName() {
		return instanceName_;
	}
	std::string className() {
		return className_;
	}


public:

	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);

private:
	//接口不开放
	void set_rectangle_pro(HShape::Rectangle pro);//
	void set_control_pro(HShape::Rectangle pro);

public:
	HShape::Rectangle get_pro();

private:
	QPixmap area_pixmap_;
	HShape::Rectangle pro_;
	HShape::Rectangle cal_pro();//控制点的移动导致形状的变化
	int id_;
	std::string area_name_;
	AreaProperty areaProperty_;
	std::string className_;
	std::string instanceName_;
private:

	DLControlTransformItem *top_left_trans_item_;
	DLControlTransformItem *top_right_trans_item_;
	DLControlTransformItem *bottom_left_trans_item_;
	DLControlTransformItem *bottom_right_trans_item_;


	QPointF last_top_left_scene_pos_;
	QPointF last_top_right_scene_pos_;
	QPointF last_bottom_left_scene_pos_;
	QPointF last_bottom_right_scene_pos_;

};


#endif