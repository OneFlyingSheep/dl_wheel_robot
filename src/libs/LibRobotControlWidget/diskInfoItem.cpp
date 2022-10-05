
/// 机器人控制界面程序

#include "DiskInfoItem.h"


//做这个不是很难，希望给大家有些启发，希望大家喜欢
DiskInfoItem::DiskInfoItem(QWidget *parent)
	: QLabel(parent)
{
	//设置大小
	//setFixedSize(200, 20);
}

DiskInfoItem::~DiskInfoItem()
{

}

///每次设置值，刷新电量值
///参数说明:total:总的磁盘容量,value:当前使用的磁盘容量
void DiskInfoItem::setValue(int total, int value)
{
	float nData = (float)value / total;
	m_dispInfo = QString::number(value) + "/" + QString::number(total);

	if (nData >= 0.9)
	{
		m_color.setRgb(220, 20, 60);	///< 使用超过90%,显示颜色为红色
	}
	else
	{
		m_color.setRgb(100, 149, 237);	///< 使用不超过90%,显示颜色为蓝色
	}

	m_value = nData * this->width();
	update();
}

void DiskInfoItem::paintEvent(QPaintEvent *event)
{
	QPainter paint(this);

	paint.setPen(QColor(0, 0, 0));
	paint.drawRect(0, 0, this->width()-1, this->height()-1);

	paint.setBrush(m_color);
	paint.drawRect(1, 1, m_value, this->height() - 2);

	QFont font;
	font.setPointSize(8);
	font.setFamily("Microsoft YaHei");

	paint.setFont(font);
	paint.drawText(QPoint(this->width()/2 - m_dispInfo.length(), 14), m_dispInfo);
}

