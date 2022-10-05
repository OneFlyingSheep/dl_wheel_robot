#ifndef _FQSHAPEITEM_YELLOW_H_
#define _FQSHAPEITEM_YELLOW_H_

#include <QGraphicsObject>
#include "HShape.h"
#include "HQShapeItem.h"
#include <QPen>
#include <QBrush>

class DLShapeItem:public HQShapeItem{

	Q_OBJECT
public:

	DLShapeItem(QGraphicsItem *parent = NULL);
	~DLShapeItem();

public:
	void update_shape();

public:
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);

	virtual int id(){
		return 0;
	}
	virtual std::string name(){
		return "";
	}

	void add_parent(DLShapeItem *item){
		parent_list_.push_back(item);

	}
	
	void add_children(DLShapeItem *item){
		children_list_.push_back(item);
	}

private:
	QPointF pre_pos_;


	QList<DLShapeItem*> parent_list_;
	QList<DLShapeItem*> children_list_;

public:
	static const int TYPE_LANDMARK=UserType+1;
	static const int TYPE_BEZIER=UserType+2;
	static const int TYPE_ARC=UserType+3;
	static const int TYPE_MUTILSEGMENT=UserType+4;
	static const int TYPE_POLYGON=UserType+5;
	static const int TYPE_POINT=UserType+6;
	static const int TYPE_MULTI_POINT=UserType+7;
	static const int TYPE_PICTURE=UserType+8;

};


#endif