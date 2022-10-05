#include <QtWidgets>
#include "HQShapeItem.h"

HQShapeItem::HQShapeItem(QGraphicsItem *parent)
	:QGraphicsObject(parent)
	, m_bPointIsChanged(false)
{
	is_solid_=true;
	pen_=QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	brush_=QBrush(Qt::transparent,Qt::SolidPattern);

}

HQShapeItem::~HQShapeItem()
{

}

bool HQShapeItem::PointIsChanged()
{
	return m_bPointIsChanged;
}

QVariant HQShapeItem::itemChange(GraphicsItemChange change, const QVariant &value)
{
	if (change == QGraphicsItem::ItemPositionChange && scene())
	{
		//qDebug() << "========================================";
		//qDebug() << "item move";
		//qDebug() << "========================================";

		if (!m_bPointIsChanged)
		{
			m_bPointIsChanged = true;
		}
	}

	return QGraphicsObject::itemChange(change, value);
}

void HQShapeItem::set_childitems_visible(bool visible)
{
	
}

QPointF HQShapeItem::center()
{
	return QPointF(0,0);
}

void HQShapeItem::update_shape()
{
	//printf("HQShapeItem::update_shape\n");
}


void HQShapeItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
{

	if (event->button() == Qt::LeftButton) {
		pre_pos_=pos();
		QGraphicsItem::mousePressEvent(event);
	}

	
}

void HQShapeItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
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

void HQShapeItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event)
{
	
	if (event->button() == Qt::LeftButton) {
		pre_pos_=pos();//��ǰλ�ú���ǰλ�����
		QGraphicsItem::mouseReleaseEvent(event);

	}

}