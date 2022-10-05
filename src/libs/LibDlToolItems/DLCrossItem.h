#pragma once
#ifndef DLCROSSITEM_ALEXWEI_20180404
#define DLCROSSITEM_ALEXWEI_20180404

#include "HQShapeItem.h"


class DLCrossItem : public HQShapeItem 
{
	Q_OBJECT

public:
	enum { Type = HShape::CROSSITEM };
	int type() const
	{
		return Type;
	}
	DLCrossItem();
	DLCrossItem(HShape::Cross pro);
	~DLCrossItem();

public:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * widget = 0);

	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);

public slots:
	void slot_pro(HShape::Cross pro);//�ⲿ��������

signals:
	void sig_pro(HShape::Cross pro);//���Ա仯֪ͨ�ⲿ

public:
	void update_shape();//����item����

private:
	//�ӿڲ�����
	void set_cross_pro(HShape::Cross pro);//
	void set_control_pro(HShape::Cross pro);

public:
	HShape::Cross get_pro();

private:
	HShape::Cross cal_pro();

private:
	HShape::Cross pro_;

};


#endif // !DLCROSSITEM_ALEXWEI_20180404



