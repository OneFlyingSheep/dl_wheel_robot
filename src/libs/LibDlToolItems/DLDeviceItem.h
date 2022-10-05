#ifndef DLDEVICEITEM_ALEXWEI_20180420_H
#define DLDEVICEITEM_ALEXWEI_20180420_H

#include "HQShapeItem.h"


class DLDeviceItem : public HQShapeItem
{
	Q_OBJECT
	
public:
	enum { Type = HShape::DEVICEITEM };
	int type() const
	{
		return Type;
	}

	DLDeviceItem(HShape::Rectangle pro_, QString name);
	~DLDeviceItem();

public:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void setState(int state);
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

public:
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);

private:
	HShape::Rectangle pro_;
	QString name_;
	int state_;
};


#endif