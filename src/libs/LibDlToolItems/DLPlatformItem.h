#ifndef DLPLATFORMITEM_HOUWEI_H
#define DLPLATFORMITEM_HOUWEI_H


#include <QObject>
#include <QGraphicsItem>
#include <QPainter>

class DLPlatformItem : public QObject, public QGraphicsItem
{
	Q_OBJECT
	
public:
	
	DLPlatformItem(QObject *parent = 0);
	~DLPlatformItem(){}

protected:
	QRectF boundingRect() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget *widget);
	void timerEvent(QTimerEvent *);

private:
	bool m_colorFlag;
	double m_radius;

};



#endif