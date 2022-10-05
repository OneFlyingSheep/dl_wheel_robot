#pragma once

/// 系统信息显示界面

#include <QWidget>
#include <QPainter>
#include <QHBoxLayout>
#include <QPushButton>

class NControlBackWidget : public QWidget
{
public:
	NControlBackWidget(QWidget* parent = NULL);

	// 设置标题;
	void setTitleText(QString text);

	// 设置中心Widget;
	void setCenterWidget(QWidget* widget);

private:
	// 绘制边框;
	void paintEvent(QPaintEvent *event);

private:
	QPushButton * m_pButtonText;
	QWidget* m_centerWidget;
};

