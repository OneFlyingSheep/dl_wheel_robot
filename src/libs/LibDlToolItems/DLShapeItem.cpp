#include <QtWidgets>
#include "DLShapeItem.h"

DLShapeItem::DLShapeItem(QGraphicsItem *parent)
	:HQShapeItem(parent)
{

}

DLShapeItem::~DLShapeItem()
{

}





void DLShapeItem::update_shape()
{
	//printf("DLShapeItem::update_shape\n");
}


void DLShapeItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
{

	if (event->button() == Qt::LeftButton) {

		pre_pos_=pos();
		QGraphicsItem::mousePressEvent(event);

	}
	
}

void DLShapeItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{

	//更新自己
	update_shape();
	
	//更新父item
	HQShapeItem *parent=(HQShapeItem*)parentItem();
	if (parent!=NULL)
	{
		
		QPointF e_pos=mapToItem(parent,event->pos());//鼠标事件的坐标映射到item坐标系下
		double distance = (e_pos-pre_pos_).manhattanLength();

		if ((event->buttons() & Qt::LeftButton)&&(distance >= QApplication::startDragDistance()))
		{

			setPos(e_pos.x(),e_pos.y());
			//通知父亲item去更新视图
			parent->update_shape();

		}
	}
	else{


		QGraphicsItem::mouseMoveEvent(event);
		
	}


		
	

}

void DLShapeItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event)
{

	//HQShapeItem::mouseReleaseEvent(event);
	
	if (event->button() == Qt::LeftButton) {
		pre_pos_=pos();//当前位置和先前位置相等
		QGraphicsItem::mouseReleaseEvent(event);

	}
	

}