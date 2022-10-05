#include "BatteryItem.h"


//做这个不是很难，希望给大家有些启发，希望大家喜欢
BatteryItem::BatteryItem(QWidget *parent)
	: QLabel(parent) 
{
	//设置电池大小
	setFixedSize(40, 16);
}

BatteryItem::~BatteryItem()
{

}

//每次设置值，刷新电量值
void BatteryItem::setValue(int value)
{
	m_color.setRgb(0,0,0);
	//0~20
	m_value = value / 5;
	if(m_value <= 2)
	{
		m_colorR.setRgb(220, 20, 60);
	}
	else if (m_value > 2 && m_value <= 10)
	{
		m_colorR.setRgb(255, 255, 0);
	}
	else
	{
		m_colorR.setRgb(136, 205, 112);
	}
	update();
}

void BatteryItem::paintEvent(QPaintEvent *event)
{
	QPainter paint(this);
	paint.setPen(QColor(136, 205, 112));
	paint.drawRoundedRect(0, 0, 23, 14, 2, 2);
	paint.setBrush(QColor(137, 249, 83));
	paint.drawRect(23, 4, 1, 6);
	paint.drawRect(2, 2, m_value, 10); 
}

