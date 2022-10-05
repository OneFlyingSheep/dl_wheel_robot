#ifndef DLITEMITEM_ALEXWEI_20180404_H
#define DLITEMITEM_ALEXWEI_20180404_H

#include "HQShapeItem.h"

class DLControlTransformItem;

class DLGridItem : public HQShapeItem
{
	Q_OBJECT

public:
	enum { Type = HShape::ARROWITEM };
	int type() const
	{
		return Type;
	}
	DLGridItem(QRect rect);
	~DLGridItem();

public:

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);


private:
	QRect rect_;	//记录item栅格大小

};


#endif  // DLITEMITEM_ALEXWEI_20180404_H