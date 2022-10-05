#pragma once

/// 系统信息显示界面

#include <QLabel>
#include <QPainter>
#include <QColor>

#pragma execution_character_set("utf-8")

class BatteryItem :
	public QLabel
{
	Q_OBJECT
public:
	BatteryItem(QWidget *parent = NULL);
	~BatteryItem();

	void setValue(int value);

protected:
	void paintEvent(QPaintEvent *event);

private:
	int m_value;
	QColor m_color;
	QColor m_colorR;
};

