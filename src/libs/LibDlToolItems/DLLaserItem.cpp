#include <QtWidgets>
#include "DLLaserItem.h"


DLLaserItem::DLLaserItem()
{
	setZValue(13);
	setFlag(ItemSendsGeometryChanges);
}


QPainterPath DLLaserItem::shape() const
{

	QPainterPath path;
	path.addEllipse(boundingRect());
	QPainterPathStroker stroker;
	stroker.setWidth(1);
	return stroker.createStroke(path);
}


QRectF DLLaserItem::boundingRect() const
{
    return QRectF(-20000, -20000, 40000, 40000);
}


void DLLaserItem::paint(QPainter *painter,const QStyleOptionGraphicsItem *, QWidget *)
{
    //painter->eraseRect(last_rect_);
    dataMutex_.lock();
	painter->setOpacity(0.5);
	painter->setBrush(QBrush(QColor(221, 240, 255)));
	painter->drawPolygon(QPolygonF(dataList_));						//ªÊ÷∆∂‡±ﬂ–Œ
    dataMutex_.unlock();
}


void DLLaserItem::flushData(std::vector<QPointF> dataList, qreal rResolution)
{
    dataMutex_.lock();
	dataList_.clear();
	for (int i = 0; i < dataList.size(); ++i) {
        QPointF point(dataList[i].x() / rResolution, -dataList[i].y() / rResolution);
    //    QPointF point(dataList[i].x() / rResolution, dataList[i].y() / rResolution);
		dataList_.append(point);
	}
    dataMutex_.unlock();
	update();
}

