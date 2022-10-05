#ifndef DLCONTROLROTATEITEM_ALEXWEI_20180420_H
#define DLCONTROLROTATEITEM_ALEXWEI_20180420_H

#include "DLShapeItem.h"


class DLControlRotateItem : public DLShapeItem
{
	Q_OBJECT
	
public:

	DLControlRotateItem();
	~DLControlRotateItem();

public:
	

	void set_length(double length){
		length_=length;
	}

	double get_length(){
		return length_;
	}

public:

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

	
protected:

	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);


private:
	QPointF pre_pos_;//���Ƶ��ƶ�ǰ�ڸ�item����ϵ�µ�����

private:
	double length_;

	QPolygonF cal_shape() const; 


};


#endif