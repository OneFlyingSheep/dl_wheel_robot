#ifndef DLARROWITEM_ALEXWEI_20180404_H
#define DLARROWITEM_ALEXWEI_20180404_H

#include "HQShapeItem.h"
#include <QLineF>

class DLControlTransformItem;

class DLArrowItem:public HQShapeItem
{
	Q_OBJECT

public:
	enum { Type = HShape::ARROWITEM };
	int type() const
	{
		return Type;
	}
	DLArrowItem(HShape::Arrow pro);
	~DLArrowItem();

public:

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

public slots:
	void slot_pro(HShape::Arrow pro);//外部设置属性

signals:
	void sig_pro(HShape::Arrow pro);//属性变化通知外部

public:

	void update_shape();//供子item调用

public:

	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);


private:
	//接口不开放
	void set_arrow_pro(HShape::Arrow pro);//
	void set_control_pro(HShape::Arrow pro);

	HShape::Arrow cal_pro();
public:
	HShape::Arrow get_pro();

private:

	HShape::Arrow pro_;

private:

	QPolygonF get_arrow() const;
	QPolygonF get_stick() const;
private:

	DLControlTransformItem *start_item_;
	DLControlTransformItem *end_item_;
	DLControlTransformItem *width_item_;
	double cal_dis( QPointF pos,QLineF line );

};


#endif  //DLARROWITEM_ALEXWEI_20180404_H