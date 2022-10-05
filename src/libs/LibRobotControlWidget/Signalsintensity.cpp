#include "Signalsintensity.h"


Signalsintensity::Signalsintensity()
{

}

QPixmap Signalsintensity::GetPixmap(const int ping)
{
	return GetSignalPixmap(GetColor(ping), GetLineNum(ping));
}

QColor Signalsintensity::GetColor(const int ping)//获取颜色
{
	if (ping <= 10)
		return QColor(0x6a, 0x6a, 0xff);
	else if (ping <= 20)
		return QColor(0x28, 0x94, 0xff);
	else if (ping <= 30)
		return QColor(0x00, 0xff, 0xff);
	else if (ping <= 40)
		return QColor(0x9a, 0xff, 0x02);
	else if (ping <= 50)
		return QColor(0xe1, 0xe1, 0x00);
	else if (ping <= 60)
		return QColor(0xea, 0xc1, 0x00);
	else if (ping <= 70)
		return QColor(0xe8, 0x00, 0xe8);
	else if (ping <= 80)
		return QColor(0xff, 0x00, 0x80);
	else
		return QColor(0xea, 0x00, 0x00);
}

int Signalsintensity::GetLineNum(const int ping)//获取线条数量
{
	if (ping <= 20)
		return 5;
	else if (ping <= 40)
		return 4;
	else if (ping <= 60)
		return 3;
	else if (ping <= 80)
		return 2;
	else
		return 1;
}

QPixmap Signalsintensity::GetSignalPixmap(const QColor &color, const int linenum)//获取信号位图
{
	QPixmap pixmap(20, 30);
	pixmap.fill(QColor(255, 255, 255, 0));
	QPainter painter(&pixmap);
	painter.setPen(QPen(color, 3)); 

	for (int i = 1, xpos = 0; i <= linenum; ++i, ++xpos)
	{
		painter.drawLine(xpos * 4, 30, xpos * 4, 33 - (i * 6));
	}
	return pixmap;
}
