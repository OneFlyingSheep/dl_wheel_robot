#ifndef DLCONTROLTRANSFORMITEM_ALEXWEI_20180404_H
#define DLCONTROLTRANSFORMITEM_ALEXWEI_20180404_H

#include "DLShapeItem.h"


class DLControlTransformItem:public DLShapeItem
{
public:
	enum {
		Type = HShape::BEZIERCONTROLPTITEM
	};
	int type() const
	{
		return Type;
	}
	DLControlTransformItem(int align=2, QGraphicsItem *parent = NULL);
	~DLControlTransformItem();

//public:
//	void set_align(int align);
//	void set_axis_line(QLineF line);//���Ҫ����Ϊ ALIGN_AXIS�����������ƶ�����


public:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);



protected:
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);

public:
	const static int ALIGN_HORIZON=0;//ˮƽ�ƶ�
	const static int ALIGN_VERTICAL=1;//��ֱ�ƶ�
	const static int ALIGN_FREE=2;//�����ƶ�
	const static int ALIGN_FATHERCENTER_CHILDPOS_AXIS=3;//���Ÿ������ĵ�ͺ���λ������������� �ƶ�
	const static int ALIGN_FATHERCENTER_CHILDPOS_RADIUS=4;//���Ÿ������ĵ���ת

private:
	int align_;
	QLineF axis_line_;//

private:

	QPointF pre_pos_;//���Ƶ��ƶ�ǰ�ڸ�item����ϵ�µ�����


};


#endif //DLCONTROLTRANSFORMITEM_ALEXWEI_20180404_H