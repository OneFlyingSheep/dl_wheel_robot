#ifndef DLRELOCATEITEM_ALEXWEI_20180404_H
#define DLRELOCATEITEM_ALEXWEI_20180404_H

#include <QGraphicsObject>


class DLRelocateItem : public QGraphicsObject
{
	Q_OBJECT

public:
	DLRelocateItem();
	~DLRelocateItem();

public:

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

public:
	void updateItem(QPointF start_point, QPointF end_point);

private:
	bool is_over_;
	double rect_height_;

	double top_left_;
	double top_right_;
	double width_;
	double height_;
};


#endif  //DLARROWITEM_ALEXWEI_20180404_H