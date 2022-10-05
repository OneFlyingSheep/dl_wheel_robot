#ifndef DLMUTILRECTITEM_ALEXWEI_20180420_H
#define DLMUTILRECTITEM_ALEXWEI_20180420_H

#include "HQShapeItem.h"

class DLControlTransformItem;


class DLMutilRectItem:public HQShapeItem
{
	Q_OBJECT
	
public:
	enum { Type = HShape::MUTILRECTITEM };
	int type() const
	{
		return Type;
	}
	DLMutilRectItem(HShape::Rectangle pro, int count);
	~DLMutilRectItem();

public:

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

public slots:
	void slot_pro(HShape::Rectangle pro);//外部设置属性

signals:
	void sig_pro(HShape::Rectangle pro);//属性变化通知外部

public:

	void update_shape();//供子item调用


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
	int get_count();

private:
	int count_;
	HShape::Rectangle pro_;
	HShape::Rectangle cal_pro();//控制点的移动导致形状的变化


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