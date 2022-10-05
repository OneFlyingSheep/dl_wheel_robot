#ifndef DLPIXMAPITEM_ALEXWEI_20180420_H
#define DLPIXMAPITEM_ALEXWEI_20180420_H

#include "HQShapeItem.h"


class DLPixmapItem : public HQShapeItem
{
	Q_OBJECT
	
public:
	enum { Type = HShape::PIXMAPITEM };
	int type() const
	{
		return Type;
	}
	DLPixmapItem(HShape::Rectangle pro_, QString file_path);
	~DLPixmapItem();

protected:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);

private:
	HShape::Rectangle pro_;
	QString file_path_;
};


#endif