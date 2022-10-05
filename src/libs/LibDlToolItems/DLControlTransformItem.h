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
//	void set_axis_line(QLineF line);//如果要设置为 ALIGN_AXIS，必须设置移动轴线


public:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);



protected:
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);

public:
	const static int ALIGN_HORIZON=0;//水平移动
	const static int ALIGN_VERTICAL=1;//垂直移动
	const static int ALIGN_FREE=2;//自由移动
	const static int ALIGN_FATHERCENTER_CHILDPOS_AXIS=3;//按着父的中心点和孩子位置坐标的连线轴 移动
	const static int ALIGN_FATHERCENTER_CHILDPOS_RADIUS=4;//按着父的中心点旋转

private:
	int align_;
	QLineF axis_line_;//

private:

	QPointF pre_pos_;//控制点移动前在父item坐标系下的坐标


};


#endif //DLCONTROLTRANSFORMITEM_ALEXWEI_20180404_H