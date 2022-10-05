#ifndef LOAD_PIXMAP_ITEM_H
#define LOAD_PIXMAP_ITEM_H

#pragma execution_character_set("utf-8")

#include <QGraphicsItem>

class LoadPixmapItem : public QGraphicsItem
{
	enum PIXMAP_LEVEL 
	{
		HIGH_DEFINTION_LEVEL,		//高清
		MEDIUM_LEVEL,				//中等水平
		LOW_LEVEL,					//低级
		LEVEL_NUM
	};
public:
    LoadPixmapItem(QGraphicsItem *parent = NULL);
    ~LoadPixmapItem();
    void SetStartEndPoint(QPointF ptStartPoint, QPointF ptEndPoint);
	void SetHighDefinitionPixmap(QPixmap &pixmap);
	void SetMediumPixmap(QPixmap &pixmap);
	void SetLowPixmap(QPixmap &pixmap);

	void UpdateArea(QPointF ptStartPoint, QPointF ptEndPoint);

protected:
    virtual QRectF boundingRect() const ;
    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = Q_NULLPTR);

private:
    QPointF m_ptStartPoint;
    QPointF m_ptEndPoint;

	QPixmap m_pixmap[LEVEL_NUM];

	QPointF m_ptDrawStartPoint;
	QPointF m_ptDrawEndPoint;
};

#endif // LOAD_PIXMAP_ITEM_H
