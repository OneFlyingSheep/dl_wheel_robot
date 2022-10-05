#ifndef DLROBOTITEM_ALEXWEI_2018_05_18_H
#define DLROBOTITEM_ALEXWEI_2018_05_18_H
#pragma execution_character_set("utf-8")

#include <QGraphicsItem>
#include <QPainter>
#include "HShape.h"


class DLRobotItem : public QGraphicsItem
{
public:
	enum { Type = HShape::ROBOTITEM };
	int type() const
	{
		return Type;
	}

	DLRobotItem();
	~DLRobotItem();

public:
	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	QPainterPath shape() const;
	void flushRobot(double x, double y, double angle);
	void setSize(int size);

private:
	void loadPixmap();

private:
	int robot_size_;
	QPixmap pixmap_;
};


#endif

