#ifndef DLCOORDINATEITEM_ALEXWEI_20180420_H
#define DLCOORDINATEITEM_ALEXWEI_20180420_H

#include <QGraphicsItem>
#include <QPainter>
#include "HShape.h"

class DLCoordinateItem : public QGraphicsItem
{
public:
	enum { Type = HShape::COORDINATEITEM};
	int type() const
	{
		return Type;
	}
	DLCoordinateItem();
	~DLCoordinateItem();

	QRectF boundingRect() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget *widget);
	QPainterPath shape() const;

private:
	

};


#endif