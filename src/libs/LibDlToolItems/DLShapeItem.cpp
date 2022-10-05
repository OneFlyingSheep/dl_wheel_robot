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

	//�����Լ�
	update_shape();
	
	//���¸�item
	HQShapeItem *parent=(HQShapeItem*)parentItem();
	if (parent!=NULL)
	{
		
		QPointF e_pos=mapToItem(parent,event->pos());//����¼�������ӳ�䵽item����ϵ��
		double distance = (e_pos-pre_pos_).manhattanLength();

		if ((event->buttons() & Qt::LeftButton)&&(distance >= QApplication::startDragDistance()))
		{

			setPos(e_pos.x(),e_pos.y());
			//֪ͨ����itemȥ������ͼ
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
		pre_pos_=pos();//��ǰλ�ú���ǰλ�����
		QGraphicsItem::mouseReleaseEvent(event);

	}
	

}