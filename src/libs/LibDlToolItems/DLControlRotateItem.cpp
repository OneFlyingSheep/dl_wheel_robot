#include <QtWidgets>
#include "DLControlRotateItem.h"
#include "math.h"

DLControlRotateItem::DLControlRotateItem()
{
	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);

	length_=8;

	
}

DLControlRotateItem::~DLControlRotateItem()
{

}

QRectF DLControlRotateItem::boundingRect() const
{
	QPolygonF shape=cal_shape() ;
	return shape.boundingRect();

}

QPainterPath DLControlRotateItem::shape() const
{
	
	QPainterPath path;
	path.addPolygon(cal_shape());
	return path;


}



void DLControlRotateItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	painter->setPen(get_pen());
	painter->setBrush(get_brush());
	painter->drawPolygon(cal_shape());
}

void DLControlRotateItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{

#if 0
	if (event->button() == Qt::LeftButton) {
		pre_pos_=pos();//记录移动前控制点的坐标
		QGraphicsItem::mousePressEvent(event);
	}
#endif

	DLShapeItem::mousePressEvent(event);

}

void DLControlRotateItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	
#if 0 
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
	else
	{

		//QGraphicsItem::mouseMoveEvent(event);
	
	}
#endif

	DLShapeItem::mouseMoveEvent(event);


}

void DLControlRotateItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{

#if 0

	if (event->button() == Qt::LeftButton) {

		pre_pos_=pos();//当前位置和先前位置相等
		QGraphicsItem::mouseReleaseEvent(event);

	}

#endif

	DLShapeItem::mouseReleaseEvent(event);

}


QPolygonF DLControlRotateItem::cal_shape() const
{

	double x=1/6.0*sqrt(3.0)*length_;
	QPointF p1(2*x,0);
	QPointF p2(-x,sqrt(3.0)*x);
	QPointF p3(-x,-sqrt(3.0)*x);

	QVector<QPointF> point_list;
	point_list.push_back(p1);
	point_list.push_back(p2);
	point_list.push_back(p3);
	return QPolygonF(point_list);


}