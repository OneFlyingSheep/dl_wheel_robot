#ifndef _HQSHAPEITEM_YELLOW_H_
#define _HQSHAPEITEM_YELLOW_H_

#include <QGraphicsObject>
#include "HShape.h"
#include <QPen>
#include <QBrush>


class HQShapeItem:public QGraphicsObject{

	Q_OBJECT

public:

	HQShapeItem(QGraphicsItem *parent = NULL);
	virtual ~HQShapeItem();
	bool PointIsChanged();

protected:
	virtual QVariant itemChange(GraphicsItemChange change, const QVariant &value);
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);

public:
	virtual QPointF center();//形状的中心点
	
public:
	virtual void update_shape();//更新父节点形状,供子item调用
	void set_childitems_visible(bool visible);
	
	bool get_solid(){
		return is_solid_;
	}
	
	void set_solid(bool value){
		is_solid_=value;
	}

	void set_pen(QPen pen){
		pen_=pen;
	}
	void set_brush(QBrush brush){
		brush_=brush;
	}

	QPen get_pen(){
		return pen_;
	}

	QBrush get_brush(){
		return brush_;
	}
	
public:
	bool is_solid_;//形状是实心还是空心


private:
	//set pen
	QColor pen_color_;
	double pen_width_;
	Qt::PenStyle pen_style_;
	Qt::PenCapStyle pen_cap_;
	Qt::PenJoinStyle pen_join_;
	QPen pen_;

	//set brush
	QColor brush_color_;
	Qt::BrushStyle brush_style_;
	QBrush brush_;
	
	//press position
	QPointF pre_pos_;

	bool m_bPointIsChanged;							//坐标是否改变

};


#endif