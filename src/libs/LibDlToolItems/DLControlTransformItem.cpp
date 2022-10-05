#include <QtWidgets>
#include "DLControlTransformItem.h"


DLControlTransformItem::DLControlTransformItem( int align, QGraphicsItem *parent)
	: DLShapeItem(parent)
	,align_(align)
{
	setFlag(ItemIsSelectable,true);
	setFlag(ItemIsMovable,true);
	setFlag(ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);

}

DLControlTransformItem::~DLControlTransformItem()
{

}

QRectF DLControlTransformItem::boundingRect() const
{
	QRectF rect(-3,-3,6,6);
	return rect;
}

QPainterPath DLControlTransformItem::shape() const
{
	QPainterPath path;
	path.addRect(boundingRect());
	return path;
}

void DLControlTransformItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	painter->setBrush(Qt::green);
	painter->drawEllipse(boundingRect());
	//painter->drawRect(boundingRect());
}

void DLControlTransformItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	if (event->button() == Qt::LeftButton) {
		pre_pos_=pos();//记录移动前控制点的坐标
		QGraphicsItem::mousePressEvent(event);
	}
}

void DLControlTransformItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{

	HQShapeItem *parent=(HQShapeItem*)parentItem();
	if (parent!=NULL)
	{

		QPointF e_pos=mapToItem(parent,event->pos());//鼠标事件的坐标映射到item坐标系下
		double distance = (e_pos-pre_pos_).manhattanLength();

		if ((event->buttons() & Qt::LeftButton)&&(distance >= QApplication::startDragDistance()))
		{

			switch (align_)
			{
			case ALIGN_HORIZON://x axis
				setPos(e_pos.x(),pos().y());
				break;
			case ALIGN_VERTICAL://y axis
				setPos(pos().x(),e_pos.y());
				break;
			case ALIGN_FREE://free
				setPos(e_pos.x(),e_pos.y());
				break;
			case ALIGN_FATHERCENTER_CHILDPOS_AXIS://
				{
					QLineF base_line(QPointF(0,0),pre_pos_);//基线
					QLineF mouse_line(QPointF(0,0),e_pos);//鼠标线
					double angle=mouse_line.angleTo(base_line);//夹角
					double prj_length=mouse_line.length()*cos(angle*3.1415926535898/180.0);//投影长度
					base_line.setLength(prj_length);//
					setPos(base_line.p2());//
					break;
				}
			case ALIGN_FATHERCENTER_CHILDPOS_RADIUS:
				{
					double radius=sqrt(pre_pos_.x()*pre_pos_.x()+pre_pos_.y()*pre_pos_.y());
					QLineF line(QPointF(0,0),e_pos);
					line.setLength(radius);
					setPos(line.p2());
					break;
				}
			default:
				setPos(e_pos.x(),e_pos.y());
				break;

			}

			//通知父亲item去更新视图
			parent->update_shape();
		}
	}
	else
	{
		//QGraphicsItem::mouseMoveEvent(event);
	}


}

void DLControlTransformItem::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{

	if (event->button() == Qt::LeftButton) {
		pre_pos_=pos();//当前位置和先前位置相等
		QGraphicsItem::mouseReleaseEvent(event);
	}


}
