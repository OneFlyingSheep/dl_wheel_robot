#ifndef DLLASERITEM_ALEXWEI_2018_05_17_H
#define DLLASERITEM_ALEXWEI_2018_05_17_H

#include <QGraphicsItem>
#include <QVector>

class QPainter;

class DLLaserItem : public QGraphicsItem
{

public:
	DLLaserItem();
	~DLLaserItem(){}
	void flushData(std::vector<QPointF> dataList, qreal rResolution);

public:
	QRectF boundingRect() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);
	QPainterPath shape() const;

private:
	QVector<QPointF> dataList_;
    QMutex dataMutex_;
    //QRect last_rect_;

};

#endif