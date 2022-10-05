#include <QDir>
#include <QDebug>
#include "DLRobotItem.h"

DLRobotItem::DLRobotItem()
{
	robot_size_ = 50;
	loadPixmap();
	setFlag(ItemSendsGeometryChanges);
}


DLRobotItem::~DLRobotItem()
{

}

void DLRobotItem::loadPixmap()
{
	bool ret = pixmap_.load(":/Resources/Common/image/robot.png");
}



void DLRobotItem::setSize(int size)
{
	robot_size_ = size;
	update();
}


QRectF DLRobotItem::boundingRect() const
{
	QRectF rect = pixmap_.rect();
	return QRectF(-robot_size_/2, -robot_size_/2, robot_size_, robot_size_);
}

void DLRobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	painter->drawPixmap(boundingRect(), pixmap_, pixmap_.rect());
}

QPainterPath DLRobotItem::shape() const
{
	QPainterPath path;
	path.addRect(boundingRect());
	return path;
}


void DLRobotItem::flushRobot(double x, double y, double angle)
{
	this->setPos(QPointF(x, y));
	this->setRotation(angle);
	//prepareGeometryChange();
	update();
}
