#pragma once


#pragma once

/// 系统信息显示界面
#include <QFont>
#include <QLabel>
#include <QPainter>
#include <QColor>
#include <QDebug>
#include <QString>

#pragma execution_character_set("utf-8")

class DiskInfoItem :
	public QLabel
{
	Q_OBJECT
public:
	DiskInfoItem(QWidget *parent = NULL);
	~DiskInfoItem();

	void setValue(int , int);

protected:
	void paintEvent(QPaintEvent *event);

private:
	int m_value;
	QString m_dispInfo;
	QColor m_color;
};

