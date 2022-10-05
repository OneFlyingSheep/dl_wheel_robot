#pragma once

#include <QPixmap>
#include <QColor>
#include <QPainter>
//显示信号强度的图标
//模拟类似手机的信号强度图标的绘制,比较简单，欢迎参考

class Signalsintensity : public QPixmap
{
public:
	explicit Signalsintensity();
	QPixmap GetPixmap(const int ping);
private:
	QColor GetColor(const int ping);//获取颜色
	int GetLineNum(const int ping);//获取线条数量
	QPixmap GetSignalPixmap(const QColor &color, const int lineNum);//获取信号位图
};

